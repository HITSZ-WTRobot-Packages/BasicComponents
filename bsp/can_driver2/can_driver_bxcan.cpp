/**
 * @file can_driver_bxcan.cpp
 * @brief Classic bxCAN backend of the lightweight CAN wrapper.
 *
 * Compiled only when the firmware enables the classic CAN HAL
 * (HAL_CAN_MODULE_ENABLED -> CAN_DRIVER_FDCAN == 0). The whole translation
 * unit is empty otherwise, so no Can member is defined twice.
 *
 * Supported hardware configuration:
 * - Classic CAN 2.0A/B frames only; FD format (and its BRS/ESI flags) is
 *   rejected with Status::Unsupported instead of being downgraded.
 * - Payloads up to 8 bytes. HAL_CAN_AddTxMessage()/HAL_CAN_GetRxMessage()
 *   always move eight bytes, so remote frames and short caller spans are
 *   staged through a zeroed 8-byte buffer while padded_storage frames are
 *   handed to the HAL directly.
 * - Three Tx mailboxes; a full mailbox set yields Status::Busy and the frame
 *   belongs to the common software queue. Nothing is retried on its own: the
 *   queue is only pushed forward by mailbox-empty/abort/error notifications.
 * - Bus-off is latched in CAN_ESR.BOFF and reported as Status::BusOff until
 *   the node recovers (recovery stays the board's responsibility).
 * - Filters are the raw bxCAN banks. IdFilter::Mask maps to one 32-bit mask
 *   bank (identifier bits compared, IDE pinned to the requested identifier
 *   type, RTR ignored so data and remote frames both match); a standard-ID
 *   IdFilter::List maps to the two 16-bit mask entries of one bank with the
 *   same IDE/RTR semantics. Two different extended IDs cannot be expressed by
 *   one bank with those semantics and return Status::Unsupported. BankFilter
 *   is written through verbatim as the four 16/32-bit halfwords, so every
 *   IDE/RTR combination the hardware can express stays reachable.
 *   Range/FD-only actions (Priority, RxBuffer, ...), GlobalFilter and
 *   ExtendedIdMask have no bxCAN equivalent and return Status::Unsupported.
 * - On parts whose CAN1/CAN2 pair shares 28 banks, the filter registers are
 *   always reached through CAN1 (the master), including for a CAN2 handle;
 *   the master instance must therefore be initialized (clocked) as well.
 *   HAL_CAN_ConfigFilter() reprograms FMR.CAN2SB from
 *   CAN_FilterTypeDef::SlaveStartFilterBank on every call, so every bank
 *   written here passes the boundary currently held in FMR - the value
 *   configured through FilterBankSplit() is never silently reset. Bank
 *   ownership (bank < CAN2SB belongs to CAN1, the rest to CAN2) is the
 *   caller's explicit choice: the index is written as given.
 *   Every shared-domain bank and the boundary itself must be configured
 *   before either bus of the CAN1/CAN2 pair is started: a shared-domain write
 *   while the other registered bus is running is refused (Status::InvalidState)
 *   so this instance cannot enter the shared filter initialization mode
 *   underneath a live bus.
 */

#include "can_driver.hpp"

#if !CAN_DRIVER_FDCAN

namespace bsp::can
{

/**
 * bxCAN exposes one shared filter domain per register block: 14 banks on a
 * single-controller part and 28 banks behind CAN1 when a CAN2 (and, where
 * present, a separate 14-bank CAN3) exists. The macros below are the only
 * chip-specific knowledge of this backend and can be overridden from the
 * build system for a part whose vendor headers describe the domain
 * differently.
 */
#ifndef CAN_DRIVER_BXCAN_SHARED_FILTER_BANKS
#    if defined(CAN2) || defined(CAN3)
#        define CAN_DRIVER_BXCAN_SHARED_FILTER_BANKS 1
#    else
#        define CAN_DRIVER_BXCAN_SHARED_FILTER_BANKS 0
#    endif
#endif

#ifndef CAN_DRIVER_BXCAN_SINGLE_FILTER_BANKS
#    define CAN_DRIVER_BXCAN_SINGLE_FILTER_BANKS 14U
#endif

#ifndef CAN_DRIVER_BXCAN_SHARED_FILTER_BANK_COUNT
#    define CAN_DRIVER_BXCAN_SHARED_FILTER_BANK_COUNT 28U
#endif

namespace
{

/**
 * Filter key field positions.
 *
 * 32-bit scale (RM0090 32.7.2): the standard identifier occupies bits 31:21
 * (STID[10:0]), the 29 bits of an extended identifier bits 31:3
 * (STID[10:0] + EXID[17:0]), IDE bit 2, RTR bit 1.
 * 16-bit scale: STID[10:0] (or EXID[28:18]) bits 15:5, RTR bit 4, IDE bit 3,
 * EXID[17:15] bits 2:0 - RTR and IDE are swapped compared to the 32-bit scale,
 * and only the upper 11 bits of an extended ID take part in the comparison.
 */
constexpr std::uint32_t filter_32_id_shift  = 21U;    ///< STID[10:0] position.
constexpr std::uint32_t filter_32_ext_shift = 3U;     ///< 29-bit extended ID position.
constexpr std::uint32_t filter_32_ide       = 0x0004U; ///< IDE bit of the 32-bit key.
constexpr std::uint32_t filter_16_id_shift  = 5U;     ///< STID[10:0] / EXID[28:18] position.
constexpr std::uint32_t filter_16_ide       = 0x0008U; ///< IDE bit of the 16-bit key.

constexpr std::uint32_t standard_id_limit = 0x7FFU;
constexpr std::uint32_t extended_id_limit = 0x1FFFFFFFU;

/// Full 16-bit identifier mask: all STID[10:0] bits plus the IDE bit.
constexpr std::uint32_t filter_16_standard_mask =
    (standard_id_limit << filter_16_id_shift) | filter_16_ide;

/// Full 32-bit identifier mask for an exact extended-ID match (IDE=1 compared,
/// RTR and the reserved bit ignored).
constexpr std::uint32_t filter_32_exact_extended_mask =
    (extended_id_limit << filter_32_ext_shift) | filter_32_ide;

/// Either the shared CAN1/CAN2 domain or one controller's own 14-bank domain.
[[nodiscard]] bool has_shared_filter_banks(const CAN_HandleTypeDef& handle) noexcept
{
#if CAN_DRIVER_BXCAN_SHARED_FILTER_BANKS
#    if defined(CAN3)
    // CAN3 keeps its own filter banks and is not part of the CAN1/CAN2 domain.
    return handle.Instance != CAN3;
#    else
    (void)handle;
    return true;
#    endif
#else
    (void)handle;
    return false;
#endif
}

/// Uppermost bank index (exclusive) usable by this handle's filter domain.
[[nodiscard]] std::uint32_t filter_bank_count(const CAN_HandleTypeDef& handle) noexcept
{
    return has_shared_filter_banks(handle) ? CAN_DRIVER_BXCAN_SHARED_FILTER_BANK_COUNT
                                           : CAN_DRIVER_BXCAN_SINGLE_FILTER_BANKS;
}

/// Register block holding the filter banks: CAN1 for a shared domain, the
/// handle's own instance otherwise.
[[nodiscard]] CAN_TypeDef* filter_master(const CAN_HandleTypeDef& handle) noexcept
{
#if CAN_DRIVER_BXCAN_SHARED_FILTER_BANKS
#    if defined(CAN3)
    return (handle.Instance == CAN3) ? handle.Instance : CAN1;
#    else
    (void)handle;
    return CAN1;
#    endif
#else
    return handle.Instance;
#endif
}

/// Boundary currently held in FMR.CAN2SB, read from the hardware.
[[nodiscard]] std::uint32_t can2_start_bank(const CAN_HandleTypeDef& handle) noexcept
{
#if CAN_DRIVER_BXCAN_SHARED_FILTER_BANKS
    if (has_shared_filter_banks(handle))
        return (filter_master(handle)->FMR & CAN_FMR_CAN2SB) >> CAN_FMR_CAN2SB_Pos;
#else
    (void)handle;
#endif
    return 0U;
}

/// Bank template with the domain's current split, so HAL_CAN_ConfigFilter()
/// rewrites FMR.CAN2SB with the value it already holds.
[[nodiscard]] CAN_FilterTypeDef make_bank(const CAN_HandleTypeDef& handle) noexcept
{
    CAN_FilterTypeDef filter{};
    filter.FilterIdHigh         = 0U;
    filter.FilterIdLow          = 0U;
    filter.FilterMaskIdHigh     = 0U;
    filter.FilterMaskIdLow      = 0U;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterBank           = 0U;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterActivation     = CAN_FILTER_DISABLE;
    filter.SlaveStartFilterBank = can2_start_bank(handle);
    return filter;
}

[[nodiscard]] Status apply_bank(CAN_HandleTypeDef& handle, const CAN_FilterTypeDef& filter) noexcept
{
    return HAL_CAN_ConfigFilter(&handle, &filter) == HAL_OK ? Status::Ok : Status::HardwareError;
}

/// IdFilter: identifier match by ID only, never by frame type.
[[nodiscard]] Status configure_id_bank(CAN_HandleTypeDef& handle, const IdFilter& request) noexcept
{
    const std::uint32_t bank_count = filter_bank_count(handle);
    if (request.index >= bank_count)
        return Status::OutOfRange;

    // calibration_message/rx_buffer_index only describe an FDCAN RxBuffer and
    // cannot be honoured by a bxCAN FIFO bank.
    if (request.calibration_message || request.rx_buffer_index != 0U)
        return Status::Unsupported;

    bool extended = false;
    switch (request.id_type)
    {
        case IdType::Standard:
            break;
        case IdType::Extended:
            extended = true;
            break;
        default:
            return Status::InvalidArgument;
    }

    CAN_FilterTypeDef filter = make_bank(handle);
    filter.FilterBank        = request.index;

    switch (request.action)
    {
        case FilterAction::Disable:
            filter.FilterActivation = CAN_FILTER_DISABLE;
            return apply_bank(handle, filter);
        case FilterAction::Fifo0:
            filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
            break;
        case FilterAction::Fifo1:
            filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
            break;
        // No bxCAN bank can reject, prioritise or act as an RxBuffer.
        case FilterAction::Reject:
        case FilterAction::Priority:
        case FilterAction::PriorityFifo0:
        case FilterAction::PriorityFifo1:
        case FilterAction::RxBuffer:
            return Status::Unsupported;
        default:
            return Status::InvalidArgument;
    }
    filter.FilterActivation = CAN_FILTER_ENABLE;

    const std::uint32_t id_limit = extended ? extended_id_limit : standard_id_limit;
    if (request.id1 > id_limit || request.id2 > id_limit)
        return Status::InvalidConfiguration;

    switch (request.mode)
    {
        case FilterMode::Mask:
        {
            // One 32-bit mask bank. The key hashes the identifier only: IDE is
            // compared against the requested type so a standard filter cannot
            // capture extended frames, and RTR stays unmasked so data and
            // remote frames of that identifier both match.
            const std::uint32_t id_key = extended
                ? (request.id1 << filter_32_ext_shift) | filter_32_ide
                : request.id1 << filter_32_id_shift;
            const std::uint32_t id_mask = extended
                ? (request.id2 << filter_32_ext_shift) | filter_32_ide
                : request.id2 << filter_32_id_shift;

            filter.FilterScale      = CAN_FILTERSCALE_32BIT;
            filter.FilterMode       = CAN_FILTERMODE_IDMASK;
            filter.FilterIdHigh     = static_cast<std::uint16_t>(id_key >> 16U);
            filter.FilterIdLow      = static_cast<std::uint16_t>(id_key);
            filter.FilterMaskIdHigh = static_cast<std::uint16_t>(id_mask >> 16U);
            filter.FilterMaskIdLow  = static_cast<std::uint16_t>(id_mask) | filter_32_ide;
            break;
        }
        case FilterMode::List:
        {
            if (!extended)
            {
                // Two exact standard identifiers in the two 16-bit mask entries
                // of one bank. The 16-bit key is STID<<5 | RTR<<4 | IDE<<3, so
                // comparing the identifier bits plus IDE keeps extended frames
                // out while data and remote frames both match.
                filter.FilterScale      = CAN_FILTERSCALE_16BIT;
                filter.FilterMode       = CAN_FILTERMODE_IDMASK;
                filter.FilterIdLow      = static_cast<std::uint16_t>(request.id1
                                                                    << filter_16_id_shift);
                filter.FilterMaskIdLow  = static_cast<std::uint16_t>(filter_16_standard_mask);
                filter.FilterIdHigh     = static_cast<std::uint16_t>(request.id2
                                                                    << filter_16_id_shift);
                filter.FilterMaskIdHigh = static_cast<std::uint16_t>(filter_16_standard_mask);
            }
            else
            {
                // A 16-bit bank compares only EXID[28:15] (14 bits) of an
                // extended ID, so two different 29-bit IDs cannot both be
                // matched exactly: that request is refused instead of being
                // widened.
                if (request.id1 != request.id2)
                    return Status::Unsupported;

                const std::uint32_t id_key =
                    (request.id1 << filter_32_ext_shift) | filter_32_ide;
                filter.FilterScale      = CAN_FILTERSCALE_32BIT;
                filter.FilterMode       = CAN_FILTERMODE_IDMASK;
                filter.FilterIdHigh     = static_cast<std::uint16_t>(id_key >> 16U);
                filter.FilterIdLow      = static_cast<std::uint16_t>(id_key);
                filter.FilterMaskIdHigh =
                    static_cast<std::uint16_t>(filter_32_exact_extended_mask >> 16U);
                filter.FilterMaskIdLow =
                    static_cast<std::uint16_t>(filter_32_exact_extended_mask);
            }
            break;
        }
        case FilterMode::Range:
        case FilterMode::RangeWithoutExtendedMask:
            return Status::Unsupported; // Inclusion ranges are an FDCAN filter mode.
        default:
            return Status::InvalidArgument;
    }

    return apply_bank(handle, filter);
}

/// BankFilter: the four hardware halfwords are written verbatim.
[[nodiscard]] Status configure_raw_bank(CAN_HandleTypeDef& handle,
                                        const BankFilter&   request) noexcept
{
    if (request.index >= filter_bank_count(handle))
        return Status::OutOfRange;

    std::uint32_t scale = 0U;
    switch (request.scale)
    {
        case FilterScale::Bits32:
            scale = CAN_FILTERSCALE_32BIT;
            break;
        case FilterScale::Bits16:
            scale = CAN_FILTERSCALE_16BIT;
            break;
        default:
            return Status::InvalidArgument;
    }

    std::uint32_t mode = 0U;
    switch (request.mode)
    {
        case BankFilterMode::Mask:
            mode = CAN_FILTERMODE_IDMASK;
            break;
        case BankFilterMode::List:
            mode = CAN_FILTERMODE_IDLIST;
            break;
        default:
            return Status::InvalidArgument;
    }

    std::uint32_t fifo = 0U;
    switch (request.fifo)
    {
        case RxFifo::Fifo0:
            fifo = CAN_FILTER_FIFO0;
            break;
        case RxFifo::Fifo1:
            fifo = CAN_FILTER_FIFO1;
            break;
        default:
            return Status::InvalidArgument;
    }

    CAN_FilterTypeDef filter = make_bank(handle);
    filter.FilterBank        = request.index;
    filter.FilterScale       = scale;
    filter.FilterMode        = mode;
    filter.FilterFIFOAssignment = fifo;
    filter.FilterActivation  = request.enabled ? CAN_FILTER_ENABLE : CAN_FILTER_DISABLE;
    filter.FilterIdLow       = request.id_low;
    filter.FilterMaskIdLow   = request.mask_low;
    filter.FilterIdHigh      = request.id_high;
    filter.FilterMaskIdHigh  = request.mask_high;
    return apply_bank(handle, filter);
}

} // namespace

Status Can::start_hardware() noexcept
{
    if (handle_.Instance == nullptr || !IS_CAN_ALL_INSTANCE(handle_.Instance))
        return Status::InvalidState;

    // Only an initialized but not yet started peripheral may be claimed. A
    // handle that is already listening belongs to somebody else's start; it is
    // refused here so rollback_start() can never stop it.
    if (handle_.State != HAL_CAN_STATE_READY)
        return Status::InvalidState;

    // HAL allows callbacks to be installed in the READY state only.
    if (HAL_CAN_RegisterCallback(&handle_, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, rx_fifo0_irq) !=
            HAL_OK ||
        HAL_CAN_RegisterCallback(&handle_, HAL_CAN_RX_FIFO1_MSG_PENDING_CB_ID, rx_fifo1_irq) !=
            HAL_OK)
        return Status::HardwareError;

    if (Can::tx_queue_capacity > 0U)
    {
        // A completion, an abort or an error (bus-off) frees a mailbox and may
        // push the software queue forward.
        constexpr HAL_CAN_CallbackIDTypeDef tx_callbacks[] = {
            HAL_CAN_TX_MAILBOX0_COMPLETE_CB_ID, HAL_CAN_TX_MAILBOX1_COMPLETE_CB_ID,
            HAL_CAN_TX_MAILBOX2_COMPLETE_CB_ID, HAL_CAN_TX_MAILBOX0_ABORT_CB_ID,
            HAL_CAN_TX_MAILBOX1_ABORT_CB_ID,    HAL_CAN_TX_MAILBOX2_ABORT_CB_ID,
            HAL_CAN_ERROR_CB_ID};
        for (const HAL_CAN_CallbackIDTypeDef id : tx_callbacks)
            if (HAL_CAN_RegisterCallback(&handle_, id, tx_irq) != HAL_OK)
                return Status::HardwareError;
    }

    // Enter the normal mode. Timing, GPIO and MSP init stay the board's job;
    // the INRQ/INAK handshake may wait, so interrupts must be available here.
    if (HAL_CAN_Start(&handle_) != HAL_OK)
        return Status::HardwareError;

    // From here on the peripheral was started by this instance (and only now
    // may rollback_start() stop it).
    hardware_started_ = true;
    return Status::Ok;
}

Status Can::enable_notifications() noexcept
{
    // Called with interrupts masked: every call below is a plain register
    // write and never waits.
    std::uint32_t interrupts = CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING;
    if (Can::tx_queue_capacity > 0U)
    {
        // Mailbox-empty pushes the software queue; bus-off and the error state
        // transitions abort pending mailboxes, which frees them as well.
        interrupts |= CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_ERROR | CAN_IT_BUSOFF |
                      CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE;
    }

    if (HAL_CAN_ActivateNotification(&handle_, interrupts) != HAL_OK)
        return Status::HardwareError;
    return Status::Ok;
}

void Can::rollback_start() noexcept
{
    // Nothing was started by this instance: leave the peripheral alone.
    if (!hardware_started_)
        return;
    hardware_started_ = false;

    (void)HAL_CAN_DeactivateNotification(&handle_,
                                        CAN_IT_RX_FIFO0_MSG_PENDING |
                                            CAN_IT_RX_FIFO1_MSG_PENDING |
                                            CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_ERROR |
                                            CAN_IT_BUSOFF | CAN_IT_ERROR_WARNING |
                                            CAN_IT_ERROR_PASSIVE);
    (void)HAL_CAN_Stop(&handle_);
}

Status Can::validate_hardware_frame(const FrameView& frame) const noexcept
{
    // send() runs this capability gate after the common frame validation; FD
    // only features are refused here instead of being silently dropped.
    if (frame.header.format != FrameFormat::Classic)
        return Status::Unsupported;
    if (frame.header.bit_rate_switch || frame.header.error_state_indicator)
        return Status::Unsupported;

    if (!is_valid_id(frame.header.id_type, frame.header.id))
        return Status::InvalidArgument;

    if (frame.header.type == FrameType::Remote)
    {
        // A remote frame requests up to 8 bytes; it carries no payload.
        if (frame.header.remote_length > max_data_length || !frame.data.empty())
            return Status::InvalidArgument;
    }
    else if (frame.data.size() > max_data_length)
    {
        return Status::InvalidArgument;
    }

    return Status::Ok;
}

Status Can::write(const FrameView& frame, const bool padded_storage) noexcept
{
    // The common layer only hands validated frames to the backend (send() runs
    // validate_frame + validate_hardware_frame, flush_tx replays frames that
    // were validated when they were queued), so what remains here is the
    // hardware state and capability of the moment.

    // A READY peripheral is still in initialization mode: the HAL would accept
    // the frame and report success without a running bus, so only LISTENING is
    // writable.
    if (handle_.State != HAL_CAN_STATE_LISTENING)
        return Status::InvalidState;

    // Bus-off is latched until the node recovers; a frame submitted now would
    // never leave the mailbox.
    if ((handle_.Instance->ESR & CAN_ESR_BOFF) != 0U)
        return Status::BusOff;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&handle_) == 0U)
        return Status::Busy;

    CAN_TxHeaderTypeDef header{};
    header.TransmitGlobalTime = DISABLE;
    header.RTR                = frame.header.type == FrameType::Remote ? CAN_RTR_REMOTE
                                                                       : CAN_RTR_DATA;
    if (frame.header.id_type == IdType::Extended)
    {
        header.IDE   = CAN_ID_EXT;
        header.ExtId = frame.header.id;
    }
    else
    {
        header.IDE   = CAN_ID_STD;
        header.StdId = frame.header.id;
    }
    header.DLC = header.RTR == CAN_RTR_REMOTE ? frame.header.remote_length
                                              : static_cast<std::uint32_t>(frame.data.size());

    // HAL_CAN_AddTxMessage() reads eight data bytes unconditionally. A full
    // 8-byte payload is handed over without a copy, and so is padded storage
    // (backed by a StoredFrame whose readable tail is initialized). Short
    // spans and remote frames, which carry no payload at all, are staged in a
    // zeroed buffer.
    std::uint8_t        staging[max_data_length] = {};
    const std::uint8_t* data                     = staging;
    if (header.RTR == CAN_RTR_DATA && frame.data.data() != nullptr &&
        (padded_storage || frame.data.size() == max_data_length))
    {
        data = frame.data.data();
    }
    else
    {
        // Bounded by the validated payload length; a longer span could only
        // come from a caller that bypassed the common validation.
        const std::size_t payload = frame.data.size() < max_data_length ? frame.data.size()
                                                                       : max_data_length;
        for (std::size_t i = 0; i < payload; ++i)
            staging[i] = frame.data[i];
    }

    std::uint32_t mailbox = 0U;
    if (HAL_CAN_AddTxMessage(&handle_, &header, data, &mailbox) != HAL_OK)
        return HAL_CAN_GetTxMailboxesFreeLevel(&handle_) == 0U ? Status::Busy
                                                               : Status::HardwareError;
    return Status::Ok;
}

Status Can::read(const std::uint32_t location, StoredFrame& frame) noexcept
{
    // location is the bxCAN Rx FIFO index (CAN_RX_FIFO0 / CAN_RX_FIFO1).
    if (location != static_cast<std::uint32_t>(CAN_RX_FIFO0) &&
        location != static_cast<std::uint32_t>(CAN_RX_FIFO1))
        return Status::InvalidArgument;

    if (HAL_CAN_GetRxFifoFillLevel(&handle_, location) == 0U)
        return Status::Empty;

    CAN_RxHeaderTypeDef header{};
    if (HAL_CAN_GetRxMessage(&handle_, location, &header, frame.data.data()) != HAL_OK)
        return Status::HardwareError;

    FrameHeader& out = frame.header;
    out              = FrameHeader{};
    out.format       = FrameFormat::Classic;
    out.id_type      = header.IDE == CAN_ID_EXT ? IdType::Extended : IdType::Standard;
    out.id           = out.id_type == IdType::Extended ? header.ExtId : header.StdId;
    out.type         = header.RTR == CAN_RTR_REMOTE ? FrameType::Remote : FrameType::Data;

    // bxCAN reports the raw DLC nibble, which is only representable up to 8.
    const std::uint8_t length = header.DLC > max_data_length
                                    ? static_cast<std::uint8_t>(max_data_length)
                                    : static_cast<std::uint8_t>(header.DLC);
    if (out.type == FrameType::Remote)
    {
        // Remote frames request length; they carry no payload bytes.
        out.remote_length = length;
        frame.length      = 0U;
    }
    else
    {
        out.remote_length = 0U;
        frame.length      = length;
    }

    return Status::Ok;
}

Status Can::configure_filter_hardware(const FilterConfig& config) noexcept
{
    if (handle_.Instance == nullptr || !IS_CAN_ALL_INSTANCE(handle_.Instance))
        return Status::InvalidConfiguration;

    // Filter banks may only be programmed before the bus is started: a
    // LISTENING handle - even one started by another owner - must not be
    // touched, and a RESET handle means HAL_CAN_Init() has not run yet.
    if (handle_.State != HAL_CAN_STATE_READY)
        return Status::InvalidState;

#if CAN_DRIVER_BXCAN_SHARED_FILTER_BANKS
    // Banks and the CAN1/CAN2 boundary sit in one shared register block, so
    // every shared-domain write must happen before either bus of the pair is
    // started. shared_filter_bus_started() only reports the *other* registered
    // Can; this instance is still pre-start here.
    if (has_shared_filter_banks(handle_) && shared_filter_bus_started(handle_))
        return Status::InvalidState;
#endif

    if (const auto* split = std::get_if<FilterBankSplit>(&config))
    {
#if CAN_DRIVER_BXCAN_SHARED_FILTER_BANKS
        // A single-controller part (and CAN3) owns a fixed 14-bank domain, so
        // there is no boundary to move.
        if (!has_shared_filter_banks(handle_))
            return Status::Unsupported;
        // CAN2SB = 0 is legal: the whole domain then belongs to CAN2. The upper
        // bound is the number of banks the shared domain has.
        if (split->first_can2_bank >= CAN_DRIVER_BXCAN_SHARED_FILTER_BANK_COUNT)
            return Status::OutOfRange;

        // Short register sequence, no HAL waiting; the caller (common
        // configure_filter()) already runs this under the configuration guard.
        CAN_TypeDef* const master = filter_master(handle_);
        master->FMR |= CAN_FMR_FINIT;
        MODIFY_REG(master->FMR,
                   CAN_FMR_CAN2SB,
                   split->first_can2_bank << CAN_FMR_CAN2SB_Pos);
        master->FMR &= ~CAN_FMR_FINIT;
        return Status::Ok;
#else
        (void)split;
        return Status::Unsupported;
#endif
    }

    if (const auto* id_filter = std::get_if<IdFilter>(&config))
        return configure_id_bank(handle_, *id_filter);

    if (const auto* bank_filter = std::get_if<BankFilter>(&config))
        return configure_raw_bank(handle_, *bank_filter);

    // GlobalFilter and ExtendedIdMask describe FDCAN acceptance rules.
    return Status::Unsupported;
}

void Can::rx_fifo0_irq(NativeHandle* handle)
{
    Can* const bus = find(handle);
    if (bus != nullptr)
        bus->on_receive(0U); // CAN_RX_FIFO0
}

void Can::rx_fifo1_irq(NativeHandle* handle)
{
    Can* const bus = find(handle);
    if (bus != nullptr)
        bus->on_receive(1U); // CAN_RX_FIFO1
}

void Can::tx_irq(NativeHandle* handle)
{
    Can* const bus = find(handle);
    if (bus != nullptr)
        bus->on_tx_available();
}

} // namespace bsp::can

#endif // !CAN_DRIVER_FDCAN

/**
 * @file can_driver_fdcan.cpp
 * @brief FDCAN backend for the lightweight CAN driver.
 *
 * Selected at compile time through CAN_DRIVER_FDCAN (see can_driver.hpp).
 * The whole translation unit is conditional so that a bxCAN firmware never
 * links a second definition of the shared private members.
 *
 * The backend owns every HAL callback: all of them are private static members
 * of Can that are registered with USE_HAL_FDCAN_REGISTER_CALLBACKS, so no weak
 * global HAL callback is overridden and several Can instances stay independent.
 */
#include "can_driver.hpp"

#if CAN_DRIVER_FDCAN

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>
#include <variant>

namespace bsp::can
{
namespace
{
/// Sentinel for a byte length that has no FDCAN DLC encoding.
constexpr std::uint32_t invalid_dlc = 0xFFFFFFFFU;

// Rx element words. These mirror the HAL-internal FDCAN_ELEMENT_MASK_* values;
// the HAL does not export them, but the message RAM layout is architectural.
constexpr std::uint32_t element_mask_esi{0x80000000U};
constexpr std::uint32_t element_mask_xtd{0x40000000U};
constexpr std::uint32_t element_mask_rtr{0x20000000U};
constexpr std::uint32_t element_mask_stdid{0x1FFC0000U};
constexpr std::uint32_t element_mask_extid{0x1FFFFFFFU};
constexpr std::uint32_t element_mask_dlc{0x000F0000U};
constexpr std::uint32_t element_mask_brs{0x00100000U};
constexpr std::uint32_t element_mask_fdf{0x00200000U};

constexpr std::uint32_t extended_id_limit{0x1FFFFFFFU};
constexpr std::uint32_t standard_id_limit{0x7FFU};

// Silicon limits, independent of whatever the handle's Init claims.
constexpr std::uint32_t standard_filter_hardware_limit{128U};
constexpr std::uint32_t extended_filter_hardware_limit{64U};
constexpr std::uint32_t rx_element_hardware_limit{64U};

/// DLC code that stores exactly @p length data bytes; HAL macros, no shifting.
constexpr std::uint32_t dlc_from_length(const std::size_t length) noexcept
{
    switch (length)
    {
    case 0U: return FDCAN_DLC_BYTES_0;
    case 1U: return FDCAN_DLC_BYTES_1;
    case 2U: return FDCAN_DLC_BYTES_2;
    case 3U: return FDCAN_DLC_BYTES_3;
    case 4U: return FDCAN_DLC_BYTES_4;
    case 5U: return FDCAN_DLC_BYTES_5;
    case 6U: return FDCAN_DLC_BYTES_6;
    case 7U: return FDCAN_DLC_BYTES_7;
    case 8U: return FDCAN_DLC_BYTES_8;
    case 12U: return FDCAN_DLC_BYTES_12;
    case 16U: return FDCAN_DLC_BYTES_16;
    case 20U: return FDCAN_DLC_BYTES_20;
    case 24U: return FDCAN_DLC_BYTES_24;
    case 32U: return FDCAN_DLC_BYTES_32;
    case 48U: return FDCAN_DLC_BYTES_48;
    case 64U: return FDCAN_DLC_BYTES_64;
    default: return invalid_dlc;
    }
}

/// Byte count stored by a DLC code; codes 9..15 are not linear.
constexpr std::size_t bytes_from_dlc(const std::uint32_t dlc) noexcept
{
    constexpr std::size_t table[16] = {0U,  1U,  2U,  3U,  4U,  5U,  6U,  7U,
                                       8U,  12U, 16U, 20U, 24U, 32U, 48U, 64U};
    return dlc < 16U ? table[dlc] : 0U;
}

/// Payload bytes an element of @p words 32-bit words can hold after its
/// two-word header; zero for a malformed element size.
constexpr std::size_t element_payload_capacity(const std::uint32_t words) noexcept
{
    return words >= 2U ? words * 4U - 8U : 0U;
}

/// Translate one non-matching action; rejects corrupted discriminators and an
/// accept destination whose FIFO has no element storage allocated.
Status map_non_matching(const FDCAN_HandleTypeDef& handle, const NonMatchingAction action,
                        std::uint32_t& native) noexcept
{
    switch (action)
    {
    case NonMatchingAction::Reject:
        native = FDCAN_REJECT;
        return Status::Ok;
    case NonMatchingAction::Fifo0:
        if (handle.Init.RxFifo0ElmtsNbr == 0U)
            return Status::NoCapacity;
        native = FDCAN_ACCEPT_IN_RX_FIFO0;
        return Status::Ok;
    case NonMatchingAction::Fifo1:
        if (handle.Init.RxFifo1ElmtsNbr == 0U)
            return Status::NoCapacity;
        native = FDCAN_ACCEPT_IN_RX_FIFO1;
        return Status::Ok;
    default:
        return Status::InvalidArgument;
    }
}

Status apply_global_filter(FDCAN_HandleTypeDef& handle, const GlobalFilter& filter) noexcept
{
    std::uint32_t non_matching_standard = 0U;
    std::uint32_t non_matching_extended = 0U;

    const Status standard = map_non_matching(handle, filter.standard, non_matching_standard);
    if (standard != Status::Ok)
        return standard;
    const Status extended = map_non_matching(handle, filter.extended, non_matching_extended);
    if (extended != Status::Ok)
        return extended;

    const std::uint32_t reject_standard =
            filter.reject_standard_remote ? FDCAN_REJECT_REMOTE : FDCAN_FILTER_REMOTE;
    const std::uint32_t reject_extended =
            filter.reject_extended_remote ? FDCAN_REJECT_REMOTE : FDCAN_FILTER_REMOTE;

    return HAL_FDCAN_ConfigGlobalFilter(&handle, non_matching_standard, non_matching_extended,
                                        reject_standard, reject_extended) == HAL_OK
                   ? Status::Ok
                   : Status::HardwareError;
}

Status apply_id_filter(FDCAN_HandleTypeDef& handle, const IdFilter& filter) noexcept
{
    if (filter.id_type != IdType::Standard && filter.id_type != IdType::Extended)
        return Status::InvalidArgument; // corrupted discriminator

    const bool          standard     = filter.id_type == IdType::Standard;
    const std::uint32_t id_limit     = standard ? standard_id_limit : extended_id_limit;
    const std::uint32_t allocated    = standard ? handle.Init.StdFiltersNbr
                                                : handle.Init.ExtFiltersNbr;
    const std::uint32_t hardware_max = standard ? standard_filter_hardware_limit
                                                : extended_filter_hardware_limit;
    // The HAL only asserts these bounds (assert_param is compiled out here), so
    // apply both the configured count and the silicon limit; an oversized Init
    // count must never let an index escape the real filter element array.
    const std::uint32_t capacity = allocated < hardware_max ? allocated : hardware_max;

    if (capacity == 0U)
        return Status::NoCapacity;
    if (filter.index >= capacity)
        return Status::OutOfRange;
    if (filter.id1 > id_limit)
        return Status::InvalidArgument;

    FDCAN_FilterTypeDef native{};
    native.IdType      = standard ? FDCAN_STANDARD_ID : FDCAN_EXTENDED_ID;
    native.FilterIndex = filter.index;
    native.FilterID1   = filter.id1;

    switch (filter.mode)
    {
    case FilterMode::Mask:
        native.FilterType = FDCAN_FILTER_MASK;
        break;
    case FilterMode::List:
        native.FilterType = FDCAN_FILTER_DUAL;
        break;
    case FilterMode::Range:
        native.FilterType = FDCAN_FILTER_RANGE;
        break;
    case FilterMode::RangeWithoutExtendedMask:
        // The EIDM bypass only exists for the extended filter list.
        if (standard)
            return Status::Unsupported;
        native.FilterType = FDCAN_FILTER_RANGE_NO_EIDM;
        break;
    default:
        return Status::Unsupported;
    }

    switch (filter.action)
    {
    case FilterAction::Disable:
        native.FilterConfig = FDCAN_FILTER_DISABLE;
        break;
    case FilterAction::Fifo0:
        native.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        break;
    case FilterAction::Fifo1:
        native.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
        break;
    case FilterAction::Reject:
        native.FilterConfig = FDCAN_FILTER_REJECT;
        break;
    case FilterAction::Priority:
        native.FilterConfig = FDCAN_FILTER_HP;
        break;
    case FilterAction::PriorityFifo0:
        native.FilterConfig = FDCAN_FILTER_TO_RXFIFO0_HP;
        break;
    case FilterAction::PriorityFifo1:
        native.FilterConfig = FDCAN_FILTER_TO_RXFIFO1_HP;
        break;
    case FilterAction::RxBuffer:
        native.FilterConfig = FDCAN_FILTER_TO_RXBUFFER;
        break;
    default:
        return Status::Unsupported;
    }

    if (filter.id2 > id_limit)
        return Status::InvalidArgument;
    // A range is an inclusive interval; reversed bounds can never match.
    if ((filter.mode == FilterMode::Range || filter.mode == FilterMode::RangeWithoutExtendedMask) &&
        filter.id2 < filter.id1)
        return Status::InvalidArgument;

    if (filter.action == FilterAction::RxBuffer)
    {
        // The peripheral ignores the filter type in Rx Buffer mode and stores
        // an exact match into one dedicated buffer; anything else cannot be
        // represented, so refuse instead of silently dropping the request.
        if (filter.mode != FilterMode::Mask || filter.id2 != 0U)
            return Status::InvalidArgument;
        if (filter.calibration_message && !standard)
            return Status::InvalidArgument; // calibration messages are standard-ID only

        const std::uint32_t buffer_max = handle.Init.RxBuffersNbr < rx_element_hardware_limit
                                                 ? handle.Init.RxBuffersNbr
                                                 : rx_element_hardware_limit;
        if (buffer_max == 0U)
            return Status::NoCapacity;
        if (filter.rx_buffer_index >= buffer_max)
            return Status::OutOfRange;

        native.FilterID2        = 0U;
        native.RxBufferIndex    = filter.rx_buffer_index;
        native.IsCalibrationMsg = filter.calibration_message ? 1U : 0U;
    }
    else
    {
        // Fields that only exist in Rx Buffer mode must not be silently
        // dropped when another action was requested.
        if (filter.rx_buffer_index != 0U)
            return Status::InvalidArgument;
        if (filter.calibration_message)
            return Status::Unsupported;

        // Directing a filter at an unallocated FIFO would silently lose every
        // matching frame, so report the missing element storage instead.
        if (filter.action == FilterAction::Fifo0 || filter.action == FilterAction::PriorityFifo0)
        {
            if (handle.Init.RxFifo0ElmtsNbr == 0U)
                return Status::NoCapacity;
        }
        else if (filter.action == FilterAction::Fifo1 ||
                 filter.action == FilterAction::PriorityFifo1)
        {
            if (handle.Init.RxFifo1ElmtsNbr == 0U)
                return Status::NoCapacity;
        }

        native.FilterID2 = filter.id2;
    }

    return HAL_FDCAN_ConfigFilter(&handle, &native) == HAL_OK ? Status::Ok
                                                              : Status::HardwareError;
}
} // namespace

Status Can::start_hardware() noexcept
{
    // A handle that does not point at a real FDCAN peripheral can never be
    // programmed; USE_FULL_ASSERT is off, so the HAL's assert_param gives no
    // runtime protection and the check has to happen here.
    if (!IS_FDCAN_ALL_INSTANCE(handle_.Instance))
        return Status::InvalidConfiguration;

    // The peripheral must already be HAL-initialised and stopped, i.e. ready to
    // leave INIT mode. A BUSY handle belongs to another owner (for example a
    // legacy driver) and is deliberately left untouched by this instance.
    if (handle_.State != HAL_FDCAN_STATE_READY)
        return Status::InvalidState;

    // Private static callbacks are installed before the peripheral leaves INIT
    // mode because every HAL_FDCAN_Register*Callback() requires State == READY.
    const bool callbacks_installed =
            HAL_FDCAN_RegisterRxFifo0Callback(&handle_, rx_fifo0_irq) == HAL_OK &&
            HAL_FDCAN_RegisterRxFifo1Callback(&handle_, rx_fifo1_irq) == HAL_OK &&
            HAL_FDCAN_RegisterCallback(&handle_, HAL_FDCAN_RX_BUFFER_NEW_MSG_CB_ID,
                                       rx_buffers_irq) == HAL_OK &&
            HAL_FDCAN_RegisterTxBufferCompleteCallback(&handle_, tx_buffers_irq) == HAL_OK &&
            HAL_FDCAN_RegisterTxBufferAbortCallback(&handle_, tx_buffers_irq) == HAL_OK &&
            HAL_FDCAN_RegisterCallback(&handle_, HAL_FDCAN_TX_FIFO_EMPTY_CB_ID, tx_irq) == HAL_OK;
    if (!callbacks_installed)
        return Status::HardwareError;

    // Runs with interrupts enabled on purpose: HAL_FDCAN_Start() also resets the
    // error code, and keeping the call out of a PRIMASK section preserves the
    // HAL timing semantics used by the (potentially waiting) HAL helpers.
    if (HAL_FDCAN_Start(&handle_) != HAL_OK)
        return Status::HardwareError;

    // Only a start performed here may be undone by rollback_start().
    hardware_started_ = true;
    return Status::Ok;
}

Status Can::enable_notifications() noexcept
{
    std::uint32_t active_its  = 0U;
    if (handle_.Init.RxFifo0ElmtsNbr > 0U)
        active_its |= FDCAN_IT_RX_FIFO0_NEW_MESSAGE;
    if (handle_.Init.RxFifo1ElmtsNbr > 0U)
        active_its |= FDCAN_IT_RX_FIFO1_NEW_MESSAGE;
    if (handle_.Init.RxBuffersNbr > 0U)
        active_its |= FDCAN_IT_RX_BUFFER_NEW_MESSAGE;

    // The software Tx queue is advanced by completion/cancellation and by the
    // Tx FIFO-empty notification, never by a later send() call. Without a
    // software queue no Tx interrupt is required at all.
    std::uint32_t tx_buffer_mask = 0U;
    if (tx_queue_capacity > 0U)
    {
        active_its |= FDCAN_IT_TX_FIFO_EMPTY | FDCAN_IT_TX_COMPLETE |
                      FDCAN_IT_TX_ABORT_COMPLETE;
        // Every buffer index; bits of non-existent buffers are never set in
        // TXBTO/TXBCF, so a full mask only monitors what actually transmits.
        tx_buffer_mask = 0xFFFFFFFFU;
    }

    if (active_its == 0U)
        return Status::Ok;

    // Register-only programming that must not wait; the caller already masks
    // interrupts around this call.
    return HAL_FDCAN_ActivateNotification(&handle_, active_its, tx_buffer_mask) == HAL_OK
                   ? Status::Ok
                   : Status::HardwareError;
}

void Can::rollback_start() noexcept
{
    // Undo only a start performed by this instance; a handle that was already
    // BUSY when start() ran belongs to another owner and keeps running.
    // HAL_FDCAN_Stop() busy-waits on CCCR.INIT/CSR, so the common start() calls
    // this after releasing the critical section (never inside PRIMASK).
    if (!hardware_started_)
        return;

    (void)HAL_FDCAN_Stop(&handle_);
    hardware_started_ = false;
}

Status Can::validate_hardware_frame(const FrameView& frame) const noexcept
{
    // The common layer already checked identifiers, frame composition and the
    // FD length encoding; this hook only rejects what the configured
    // peripheral cannot represent, without silently downgrading the request.
    const FrameHeader& header  = frame.header;
    const std::size_t  payload = header.type == FrameType::Remote ? header.remote_length
                                                                  : frame.data.size();

    if (header.format == FrameFormat::Fd)
    {
        if (handle_.Init.FrameFormat == FDCAN_FRAME_CLASSIC)
            return Status::Unsupported; // a classic-only peripheral cannot emit FD
        if (header.type == FrameType::Remote)
            return Status::Unsupported; // CAN FD has no remote frames
        if (header.bit_rate_switch && handle_.Init.FrameFormat != FDCAN_FRAME_FD_BRS)
            return Status::Unsupported; // cannot switch bit rate on this peripheral
    }
    else
    {
        if (header.bit_rate_switch)
            return Status::InvalidConfiguration; // BRS only exists in FD format
        if (payload > 8U)
            return Status::InvalidConfiguration; // classic frames carry at most 8 bytes
    }

    if (header.type == FrameType::Remote && payload > 8U)
        return Status::InvalidConfiguration; // a remote request length is a classic DLC

    if (payload > max_data_length)
        return Status::Unsupported;

    // Transmission always goes through the Tx FIFO/queue.
    if (handle_.Init.TxFifoQueueElmtsNbr == 0U)
        return Status::Unsupported;

    // The HAL writes the payload words into the Tx element without checking the
    // element size, so a frame that does not fit would overwrite adjacent
    // Message RAM. Require the configured element to hold ceil(payload/4)
    // words behind the two-word header.
    const std::uint32_t tx_capacity =
            handle_.Init.TxElmtSize > 2U ? handle_.Init.TxElmtSize - 2U : 0U;
    if ((payload + 3U) / 4U > tx_capacity)
        return Status::Unsupported;

    return Status::Ok;
}

Status Can::write(const FrameView& frame, bool padded_storage) noexcept
{
    // The common layer calls validate_hardware_frame() before submitting, and a
    // software queue entry was validated when it was enqueued, so the hardware
    // capability checks are not repeated here.

    // write() only reports the statuses send() understands; a peripheral that is
    // not running is a hardware-side failure here.
    if (handle_.State != HAL_FDCAN_STATE_BUSY)
        return Status::HardwareError;

    // Bus off is reported, never recovered: the application decides whether to
    // restart the bus.
    if ((handle_.Instance->PSR & FDCAN_PSR_BO) != 0U)
        return Status::BusOff;

    if (HAL_FDCAN_GetTxFifoFreeLevel(&handle_) == 0U)
        return Status::Busy;

    const bool        remote        = frame.header.type == FrameType::Remote;
    const std::size_t payload_bytes = remote ? frame.header.remote_length : frame.data.size();

    const std::uint32_t dlc = dlc_from_length(payload_bytes);
    if (dlc == invalid_dlc)
        return Status::InvalidConfiguration;

    FDCAN_TxHeaderTypeDef tx{};
    tx.Identifier          = frame.header.id;
    tx.IdType              = frame.header.id_type == IdType::Extended ? FDCAN_EXTENDED_ID
                                                                     : FDCAN_STANDARD_ID;
    tx.TxFrameType         = remote ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
    tx.DataLength          = dlc;
    tx.ErrorStateIndicator = frame.header.error_state_indicator ? FDCAN_ESI_PASSIVE
                                                                : FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = frame.header.bit_rate_switch ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
    tx.FDFormat            = frame.header.format == FrameFormat::Fd ? FDCAN_FD_CAN
                                                                    : FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0U;

    // The HAL copies the payload in 4-byte words up to DLCtoBytes[dlc], so a
    // span whose length is a multiple of four is read exactly as-is while
    // lengths 1/2/3/5/6/7 read up to three bytes beyond it. Remote frames carry
    // no payload at all and must never touch the caller's empty span. A padded
    // frame already has its round-up word tail initialised by the common layer
    // and needs no copy here.
    const std::size_t read_words = (payload_bytes + 3U) / 4U;
    const std::size_t copy_bytes = remote ? 0U : frame.data.size();

    alignas(std::uint32_t) std::uint8_t staging[8]; // only [0, read_words * 4) is used
    const std::uint8_t*                 payload = staging;

    if (padded_storage && !remote && copy_bytes != 0U)
    {
        payload = frame.data.data();
    }
    else if (copy_bytes != 0U && copy_bytes % 4U == 0U)
    {
        payload = frame.data.data();
    }
    else
    {
        // Short or remote frame: stage only the bytes the HAL actually reads,
        // so nothing outside the caller's storage is ever touched.
        if (read_words * 4U > sizeof(staging))
            return Status::InvalidConfiguration; // unreachable for a valid DLC
        if (copy_bytes != 0U)
            std::memcpy(staging, frame.data.data(), copy_bytes);
        for (std::size_t index = copy_bytes; index < read_words * 4U; ++index)
            staging[index] = 0U;
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(&handle_, &tx, payload) != HAL_OK)
    {
        // The free level was re-checked above, so classify the residual causes.
        if ((handle_.Instance->TXFQS & FDCAN_TXFQS_TFQF) != 0U)
            return Status::Busy;
        if ((handle_.Instance->PSR & FDCAN_PSR_BO) != 0U)
            return Status::BusOff;
        return Status::HardwareError;
    }

    return Status::Ok;
}

Status Can::read(std::uint32_t location, StoredFrame& frame) noexcept
{
    if (handle_.State != HAL_FDCAN_STATE_BUSY)
        return Status::HardwareError;

    enum class Acknowledge : std::uint8_t
    {
        None,
        Fifo0,
        Fifo1,
        Buffer,
    };

    FDCAN_GlobalTypeDef* instance          = handle_.Instance;
    const volatile std::uint32_t* element  = nullptr;
    std::size_t          element_payload   = 0U;
    std::uint32_t        acknowledge_index = 0U;
    Acknowledge          acknowledge       = Acknowledge::None;

    if (location == FDCAN_RX_FIFO0)
    {
        const std::uint32_t allocated = handle_.Init.RxFifo0ElmtsNbr < rx_element_hardware_limit
                                                ? handle_.Init.RxFifo0ElmtsNbr
                                                : rx_element_hardware_limit;
        if (allocated == 0U)
            return Status::Empty;

        const std::uint32_t status = instance->RXF0S;
        if ((status & FDCAN_RXF0S_F0FL) == 0U)
            return Status::Empty;

        std::uint32_t get_index = (status & FDCAN_RXF0S_F0GI) >> FDCAN_RXF0S_F0GI_Pos;
        // A full FIFO in overwrite mode has already discarded the element the
        // get index points at. Advance by one and wrap at the allocated count;
        // masking with the count (as the HAL does) is wrong for a non-power-of
        // two count and would leave the index out of range.
        if (((status & FDCAN_RXF0S_F0F) >> FDCAN_RXF0S_F0F_Pos) == 1U &&
            ((instance->RXF0C & FDCAN_RXF0C_F0OM) >> FDCAN_RXF0C_F0OM_Pos) ==
                    FDCAN_RX_FIFO_OVERWRITE)
        {
            ++get_index;
            if (get_index >= allocated)
                get_index = 0U;
        }

        // The status register must never hand out an index outside the range
        // that was actually allocated.
        if (get_index >= allocated)
            return Status::HardwareError;

        element = reinterpret_cast<const volatile std::uint32_t*>(static_cast<std::uintptr_t>(
                handle_.msgRam.RxFIFO0SA + get_index * handle_.Init.RxFifo0ElmtSize * 4U));
        element_payload   = element_payload_capacity(handle_.Init.RxFifo0ElmtSize);
        acknowledge_index = get_index;
        acknowledge       = Acknowledge::Fifo0;
    }
    else if (location == FDCAN_RX_FIFO1)
    {
        const std::uint32_t allocated = handle_.Init.RxFifo1ElmtsNbr < rx_element_hardware_limit
                                                ? handle_.Init.RxFifo1ElmtsNbr
                                                : rx_element_hardware_limit;
        if (allocated == 0U)
            return Status::Empty;

        const std::uint32_t status = instance->RXF1S;
        if ((status & FDCAN_RXF1S_F1FL) == 0U)
            return Status::Empty;

        std::uint32_t get_index = (status & FDCAN_RXF1S_F1GI) >> FDCAN_RXF1S_F1GI_Pos;
        if (((status & FDCAN_RXF1S_F1F) >> FDCAN_RXF1S_F1F_Pos) == 1U &&
            ((instance->RXF1C & FDCAN_RXF1C_F1OM) >> FDCAN_RXF1C_F1OM_Pos) ==
                    FDCAN_RX_FIFO_OVERWRITE)
        {
            ++get_index;
            if (get_index >= allocated)
                get_index = 0U;
        }

        if (get_index >= allocated)
            return Status::HardwareError;

        element = reinterpret_cast<const volatile std::uint32_t*>(static_cast<std::uintptr_t>(
                handle_.msgRam.RxFIFO1SA + get_index * handle_.Init.RxFifo1ElmtSize * 4U));
        element_payload   = element_payload_capacity(handle_.Init.RxFifo1ElmtSize);
        acknowledge_index = get_index;
        acknowledge       = Acknowledge::Fifo1;
    }
    else
    {
        const std::uint32_t buffer_max = handle_.Init.RxBuffersNbr < rx_element_hardware_limit
                                                 ? handle_.Init.RxBuffersNbr
                                                 : rx_element_hardware_limit;
        // Dedicated Rx buffer; its new-data flag stays set until read() clears
        // it, so an exhausted location reports Empty and stops the caller loop.
        if (location >= buffer_max)
            return Status::Empty;

        const std::uint32_t pending =
                location < FDCAN_RX_BUFFER32
                        ? ((instance->NDAT1 >> location) & 1U)
                        : ((instance->NDAT2 >> (location & 0x1FU)) & 1U);
        if (pending == 0U)
            return Status::Empty;

        element = reinterpret_cast<const volatile std::uint32_t*>(static_cast<std::uintptr_t>(
                handle_.msgRam.RxBufferSA + location * handle_.Init.RxBufferSize * 4U));
        element_payload   = element_payload_capacity(handle_.Init.RxBufferSize);
        acknowledge_index = location;
        acknowledge       = Acknowledge::Buffer;
    }

    const std::uint32_t word1 = element[0];
    const std::uint32_t word2 = element[1];

    const bool          extended = (word1 & element_mask_xtd) != 0U;
    const bool          remote   = (word1 & element_mask_rtr) != 0U;
    const bool          fd       = (word2 & element_mask_fdf) != 0U;
    const std::uint32_t dlc      = (word2 & element_mask_dlc) >> 16U;

    FrameHeader& header = frame.header;
    header.id                   = extended ? (word1 & element_mask_extid)
                                           : ((word1 & element_mask_stdid) >> 18U);
    header.id_type              = extended ? IdType::Extended : IdType::Standard;
    header.format               = fd ? FrameFormat::Fd : FrameFormat::Classic;
    header.type                 = remote ? FrameType::Remote : FrameType::Data;
    // BRS and ESI only exist in FD format; a classic element never defines them.
    header.bit_rate_switch      = fd && ((word2 & element_mask_brs) != 0U);
    header.error_state_indicator = fd && ((word1 & element_mask_esi) != 0U);
    header.remote_length        = 0U;

    std::size_t length = bytes_from_dlc(dlc);
    // A classic frame never carries more than 8 bytes, whatever the raw DLC
    // code says; do not turn codes 9..15 into 12..64 bytes here.
    if (!fd && length > 8U)
        length = 8U;

    bool deliverable = true;
    if (remote && fd)
    {
        // CAN FD has no remote frames, so such an element is invalid: consume
        // and acknowledge it below, but never deliver it.
        deliverable  = false;
        frame.length = 0U;
    }
    else if (remote)
    {
        // No payload is stored for a remote frame: the length field is a
        // protocol request, not data, so it is never truncated.
        header.remote_length = static_cast<std::uint8_t>(length);
        frame.length         = 0U;
    }
    else if (length > element_payload || length > max_data_length)
    {
        // The configured element is smaller than the received payload, so the
        // hardware stored a truncated frame. Consume and acknowledge it below,
        // but report it as undeliverable instead of returning a short frame,
        // and never read past the element's Message RAM.
        deliverable  = false;
        frame.length = 0U;
    }
    else
    {
        // Read the element word by word instead of memcpy()ing the payload: the
        // element lives in hardware-updated memory, so the compiler must not
        // cache it, and only the valid bytes of the last word are extracted.
        // ceil(length/4) words never leave the element because the configured
        // payload capacity above is always a multiple of four bytes.
        const std::size_t words = (length + 3U) / 4U;
        for (std::size_t word_index = 0U; word_index < words; ++word_index)
        {
            const std::uint32_t word = element[2U + word_index];
            const std::size_t   base = word_index * 4U;
            for (std::size_t byte = 0U; byte < 4U && base + byte < length; ++byte)
                frame.data[base + byte] = static_cast<std::uint8_t>((word >> (8U * byte)) & 0xFFU);
        }
        frame.length = static_cast<std::uint8_t>(length);
    }

    switch (acknowledge)
    {
    case Acknowledge::Fifo0:
        instance->RXF0A = acknowledge_index;
        break;
    case Acknowledge::Fifo1:
        instance->RXF1A = acknowledge_index;
        break;
    case Acknowledge::Buffer:
        if (acknowledge_index < FDCAN_RX_BUFFER32)
            instance->NDAT1 = std::uint32_t{1} << acknowledge_index;
        else
            instance->NDAT2 = std::uint32_t{1} << (acknowledge_index & 0x1FU);
        break;
    case Acknowledge::None:
        break;
    }

    // InvalidFrame means "consumed and acknowledged above, but not deliverable";
    // the common layer keeps draining the location instead of dispatching it.
    return deliverable ? Status::Ok : Status::InvalidFrame;
}

Status Can::configure_filter_hardware(const FilterConfig& config) noexcept
{
    // A handle that does not point at a real FDCAN peripheral can never be
    // programmed (the common layer only rejects nullptr), and the HAL refuses
    // global-filter changes once the peripheral is running.
    if (!IS_FDCAN_ALL_INSTANCE(handle_.Instance))
        return Status::InvalidConfiguration;
    if (handle_.State != HAL_FDCAN_STATE_READY)
        return Status::InvalidState;

    return std::visit(
            [this](const auto& filter) noexcept -> Status
            {
                using Filter = std::decay_t<decltype(filter)>;
                if constexpr (std::is_same_v<Filter, IdFilter>)
                {
                    return apply_id_filter(handle_, filter);
                }
                else if constexpr (std::is_same_v<Filter, GlobalFilter>)
                {
                    return apply_global_filter(handle_, filter);
                }
                else if constexpr (std::is_same_v<Filter, ExtendedIdMask>)
                {
                    if (filter.mask > extended_id_limit)
                        return Status::InvalidArgument;
                    return HAL_FDCAN_ConfigExtendedIdMask(&handle_, filter.mask) == HAL_OK
                                   ? Status::Ok
                                   : Status::HardwareError;
                }
                else
                {
                    // BankFilter and FilterBankSplit describe the bxCAN bank
                    // layout, which the FDCAN filter list cannot represent.
                    return Status::Unsupported;
                }
            },
            config);
}

// ---------------------------------------------------------------------------
// HAL callback bridges. Every callback runs in interrupt context on the FDCAN
// interrupt line; it only looks up the owning instance and defers to the
// shared on_receive()/on_tx_available() entry points, which re-enter the HAL
// through distinct registers and use no retained state.
// ---------------------------------------------------------------------------

void Can::rx_fifo0_irq(NativeHandle* handle, std::uint32_t)
{
    if (Can* instance = find(handle))
        instance->on_receive(FDCAN_RX_FIFO0);
}

void Can::rx_fifo1_irq(NativeHandle* handle, std::uint32_t)
{
    if (Can* instance = find(handle))
        instance->on_receive(FDCAN_RX_FIFO1);
}

void Can::rx_buffers_irq(NativeHandle* handle)
{
    Can* instance = find(handle);
    if (instance == nullptr)
        return;

    // The HAL only clears the shared interrupt flag, so the per-buffer new-data
    // flags are still valid here; read() clears them one buffer at a time.
    const std::uint32_t limit =
            instance->handle_.Init.RxBuffersNbr < 64U ? instance->handle_.Init.RxBuffersNbr : 64U;
    const std::uint32_t pending_low  = instance->handle_.Instance->NDAT1;
    const std::uint32_t pending_high = instance->handle_.Instance->NDAT2;

    for (std::uint32_t index = 0U; index < limit; ++index)
    {
        const std::uint32_t pending = index < FDCAN_RX_BUFFER32
                                              ? ((pending_low >> index) & 1U)
                                              : ((pending_high >> (index & 0x1FU)) & 1U);
        if (pending != 0U)
            instance->on_receive(index);
    }
}

void Can::tx_buffers_irq(NativeHandle* handle, std::uint32_t)
{
    // A completed or cancelled buffer frees room in the Tx FIFO/queue, so the
    // software queue can move on without waiting for another send().
    if (Can* instance = find(handle))
        instance->on_tx_available();
}

void Can::tx_irq(NativeHandle* handle)
{
    if (Can* instance = find(handle))
        instance->on_tx_available();
}

} // namespace bsp::can

#endif // CAN_DRIVER_FDCAN

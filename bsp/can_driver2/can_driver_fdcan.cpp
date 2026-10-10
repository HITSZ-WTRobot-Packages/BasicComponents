/**
 * @file can_driver_fdcan.cpp
 * @brief 轻量 CAN 驱动的 FDCAN 后端实现。
 *
 * 通过 CAN_DRIVER_FDCAN 在编译期选择（见 can_driver.hpp）。整个翻译单元被
 * 条件编译包裹，使 bxCAN 固件不会链接到共享私有成员的重复定义。
 *
 * 此后端独占所有 HAL 回调：它们都是 Can 的私有静态成员，通过
 * USE_HAL_FDCAN_REGISTER_CALLBACKS 注册，因此不会覆盖任何弱符号的全局 HAL
 * 回调，且多个 Can 实例可以彼此独立。
 */
#include "can_driver.hpp"

#if CAN_DRIVER_FDCAN

#    include <algorithm>
#    include <cstddef>
#    include <cstdint>
#    include <cstring>
#    include <type_traits>
#    include <variant>

namespace bsp::can
{
namespace
{
/// 无对应 FDCAN DLC 编码的字节长度的哨兵值。
constexpr std::uint32_t invalid_dlc = 0xFFFFFFFFU;

constexpr std::uint32_t extended_id_limit{ 0x1FFFFFFFU };
constexpr std::uint32_t standard_id_limit{ 0x7FFU };

// 硬件（硅）上限，与 handle 的 Init 所声明的数量无关。
constexpr std::uint32_t standard_filter_hardware_limit{ 128U };
constexpr std::uint32_t extended_filter_hardware_limit{ 64U };
constexpr std::uint32_t rx_element_hardware_limit{ 64U };

/// 返回恰好能存放 @p length 个数据字节的 DLC 编码；直接使用 HAL 宏，不做位移。
constexpr std::uint32_t dlc_from_length(const std::size_t length) noexcept
{
    switch (length)
    {
    case 0U:
        return FDCAN_DLC_BYTES_0;
    case 1U:
        return FDCAN_DLC_BYTES_1;
    case 2U:
        return FDCAN_DLC_BYTES_2;
    case 3U:
        return FDCAN_DLC_BYTES_3;
    case 4U:
        return FDCAN_DLC_BYTES_4;
    case 5U:
        return FDCAN_DLC_BYTES_5;
    case 6U:
        return FDCAN_DLC_BYTES_6;
    case 7U:
        return FDCAN_DLC_BYTES_7;
    case 8U:
        return FDCAN_DLC_BYTES_8;
    case 12U:
        return FDCAN_DLC_BYTES_12;
    case 16U:
        return FDCAN_DLC_BYTES_16;
    case 20U:
        return FDCAN_DLC_BYTES_20;
    case 24U:
        return FDCAN_DLC_BYTES_24;
    case 32U:
        return FDCAN_DLC_BYTES_32;
    case 48U:
        return FDCAN_DLC_BYTES_48;
    case 64U:
        return FDCAN_DLC_BYTES_64;
    default:
        return invalid_dlc;
    }
}

/// 放在静态只读存储中，避免每次取帧时把查表数据复制到栈；每项只需一个字节。
constexpr std::uint8_t dlc_byte_lengths[16] = { 0U, 1U,  2U,  3U,  4U,  5U,  6U,  7U,
                                                8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U };

/// 返回某个 DLC 编码所表示的字节数；编码 9..15 不是线性递增的。
constexpr std::size_t bytes_from_dlc(const std::uint32_t dlc) noexcept
{
    return dlc < 16U ? dlc_byte_lengths[dlc] : 0U;
}

/// 一个由 @p words 个 32-bit word 组成的 element，在扣除其 2 word 的头部之后
/// 可容纳的负载字节数；element 尺寸非法时返回 0。
constexpr std::size_t element_payload_capacity(const std::uint32_t words) noexcept
{
    return words >= 2U ? words * 4U - 8U : 0U;
}

/// 将单个非匹配动作翻译为 HAL 取值；拒绝损坏的判别符，以及目标 FIFO 未分配
/// element 存储空间的 accept 目标。
Status map_non_matching(const FDCAN_HandleTypeDef& handle,
                        const NonMatchingAction    action,
                        std::uint32_t&             native) noexcept
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

    const std::uint32_t reject_standard = filter.reject_standard_remote ? FDCAN_REJECT_REMOTE
                                                                        : FDCAN_FILTER_REMOTE;
    const std::uint32_t reject_extended = filter.reject_extended_remote ? FDCAN_REJECT_REMOTE
                                                                        : FDCAN_FILTER_REMOTE;

    return HAL_FDCAN_ConfigGlobalFilter(&handle,
                                        non_matching_standard,
                                        non_matching_extended,
                                        reject_standard,
                                        reject_extended) == HAL_OK
                   ? Status::Ok
                   : Status::HardwareError;
}

Status apply_id_filter(FDCAN_HandleTypeDef& handle, const IdFilter& filter) noexcept
{
    if (filter.id_type != IdType::Standard && filter.id_type != IdType::Extended)
        return Status::InvalidArgument; // 判别符已损坏

    const bool          standard     = filter.id_type == IdType::Standard;
    const std::uint32_t id_limit     = standard ? standard_id_limit : extended_id_limit;
    const std::uint32_t allocated    = standard ? handle.Init.StdFiltersNbr
                                                : handle.Init.ExtFiltersNbr;
    const std::uint32_t hardware_max = standard ? standard_filter_hardware_limit
                                                : extended_filter_hardware_limit;
    // HAL 仅以断言校验这些边界（此处 assert_param 已被编译移除），因此需同时
    // 施加配置数量和硬件上限；Init 中过大的数量绝不能导致索引越出真实的
    // filter element 数组。
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
        // EIDM 旁路仅存在于扩展 ID filter list 中。
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
    // range 是闭区间；上下界反序时永远不可能匹配。
    if ((filter.mode == FilterMode::Range || filter.mode == FilterMode::RangeWithoutExtendedMask) &&
        filter.id2 < filter.id1)
        return Status::InvalidArgument;

    if (filter.action == FilterAction::RxBuffer)
    {
        // 在 Rx Buffer 模式下硬件忽略 filter type，并将精确匹配存入一个
        // 专用 buffer；其他情形无法表达，因此直接拒绝而不是静默丢弃请求。
        if (filter.mode != FilterMode::Mask || filter.id2 != 0U)
            return Status::InvalidArgument;
        if (filter.calibration_message && !standard)
            return Status::InvalidArgument; // calibration 报文仅支持标准 ID

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
        // 当请求的是其他动作时，仅存在于 Rx Buffer 模式的字段不能被静默忽略。
        if (filter.rx_buffer_index != 0U)
            return Status::InvalidArgument;
        if (filter.calibration_message)
            return Status::Unsupported;

        // 将过滤器指向未分配的 FIFO 会静默丢失所有匹配帧，因此改为报告缺少
        // element 存储空间。
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

    return HAL_FDCAN_ConfigFilter(&handle, &native) == HAL_OK ? Status::Ok : Status::HardwareError;
}

} // namespace

/// 启动 FDCAN 外设：校验 Instance 与 READY 状态、注册全部私有回调、调用
/// HAL_FDCAN_Start()，并记录“本次启动”以便回滚。
///
/// 前置条件：外设已被 HAL 初始化且处于停止态（State == READY）。若 handle 为
/// BUSY，则视为其他所有者占用，返回 InvalidState 且不做任何改动。
Status Can::start_hardware() noexcept
{
    // 指向非真实 FDCAN 外设的 handle 永远无法被编程；由于 USE_FULL_ASSERT 关闭，
    // HAL 的 assert_param 不提供运行时保护，故必须在此处自行检查。
    if (!IS_FDCAN_ALL_INSTANCE(handle_.Instance))
        return Status::InvalidConfiguration;

    // 外设必须已被 HAL 初始化并处于停止态，即已具备退出 INIT 模式的条件。
    // BUSY 的 handle 归属于其他所有者（例如旧驱动），本实例刻意不去触碰它。
    if (handle_.State != HAL_FDCAN_STATE_READY)
        return Status::InvalidState;

    // 私有静态回调在外设退出 INIT 模式之前安装，因为每个
    // HAL_FDCAN_Register*Callback() 都要求 State == READY。
    const bool callbacks_installed =
            HAL_FDCAN_RegisterRxFifo0Callback(&handle_, rx_fifo0_irq) == HAL_OK &&
            HAL_FDCAN_RegisterRxFifo1Callback(&handle_, rx_fifo1_irq) == HAL_OK &&
            HAL_FDCAN_RegisterCallback(&handle_,
                                       HAL_FDCAN_RX_BUFFER_NEW_MSG_CB_ID,
                                       rx_buffers_irq) == HAL_OK &&
            HAL_FDCAN_RegisterTxBufferCompleteCallback(&handle_, tx_buffers_irq) == HAL_OK &&
            HAL_FDCAN_RegisterTxBufferAbortCallback(&handle_, tx_buffers_irq) == HAL_OK &&
            HAL_FDCAN_RegisterCallback(&handle_, HAL_FDCAN_TX_FIFO_EMPTY_CB_ID, tx_irq) == HAL_OK;
    if (!callbacks_installed)
        return Status::HardwareError;

    // 刻意在开中断状态下执行：HAL_FDCAN_Start() 会同时复位错误码，将其排除在
    // PRIMASK 临界区之外可保持（可能阻塞等待的）HAL 辅助函数所依赖的 HAL 时序
    // 语义。
    if (HAL_FDCAN_Start(&handle_) != HAL_OK)
        return Status::HardwareError;

    // 只有此处执行的 start 才允许被 rollback_start() 撤销。
    hardware_started_ = true;
    return Status::Ok;
}

/// 按 Init 中实际分配的资源启用 RX/TX 中断通知：只为已分配的 Rx FIFO/buffer 打开
/// 对应中断，且仅当存在软件 Tx 队列时才打开 Tx 中断。不涉及等待，调用方需自行
/// 屏蔽中断。
Status Can::enable_notifications() noexcept
{
    std::uint32_t active_its = 0U;
    if (handle_.Init.RxFifo0ElmtsNbr > 0U)
        active_its |= FDCAN_IT_RX_FIFO0_NEW_MESSAGE;
    if (handle_.Init.RxFifo1ElmtsNbr > 0U)
        active_its |= FDCAN_IT_RX_FIFO1_NEW_MESSAGE;
    if (handle_.Init.RxBuffersNbr > 0U)
        active_its |= FDCAN_IT_RX_BUFFER_NEW_MESSAGE;

    // 软件 Tx 队列由完成/取消通知以及 Tx FIFO 空通知来推进，绝不会由后续的
    // send() 调用推进。没有软件队列时完全不需要任何 Tx 中断。
    std::uint32_t tx_buffer_mask = 0U;
    if (Can::TxQueueCapacity > 0U)
    {
        active_its |= FDCAN_IT_TX_FIFO_EMPTY | FDCAN_IT_TX_COMPLETE | FDCAN_IT_TX_ABORT_COMPLETE;
        // 覆盖所有 buffer 索引；不存在的 buffer 对应位在 TXBTO/TXBCF 中永远不会
        // 置位，因此全 1 掩码只会监视那些实际参与发送的 buffer。
        tx_buffer_mask = 0xFFFFFFFFU;
    }

    if (active_its == 0U)
        return Status::Ok;

    // 仅涉及寄存器写、不得等待的编程；调用方已在本调用前后屏蔽中断。
    return HAL_FDCAN_ActivateNotification(&handle_, active_its, tx_buffer_mask) == HAL_OK
                   ? Status::Ok
                   : Status::HardwareError;
}

/// 撤销由本实例执行的启动；对启动前已是 BUSY（属于其他所有者）的 handle 不做任何
/// 处理。会忙等 CCCR.INIT/CSR，故不得在 PRIMASK 临界区内调用。
void Can::rollback_start() noexcept
{
    // 只撤销本实例自己执行的 start；若 start() 执行时 handle 已处于 BUSY，
    // 则它归属其他所有者，需保持继续运行。
    // HAL_FDCAN_Stop() 会忙等 CCCR.INIT/CSR，因此公共层的 start() 在释放临界区
    // 之后才调用本函数（绝不在 PRIMASK 区间内）。
    if (!hardware_started_)
        return;

    (void)HAL_FDCAN_Stop(&handle_);
    hardware_started_ = false;
}

/// 校验一帧是否可由当前配置的外设发送：检查 FD/Classic 格式与 BRS 的兼容性、
/// 远程帧约束、负载长度、Tx FIFO/queue 是否存在，以及 Tx element 能否容纳该负载。
/// 只做能力校验，不修改任何状态；不满足时返回 Unsupported/InvalidConfiguration，
/// 绝不静默降级或截断。
Status Can::validate_hardware_frame(const FrameView& frame) const noexcept
{
    // 公共层已检查标识符、帧构成和 FD 长度编码；本钩子只拒绝当前配置的外设无法
    // 表达的请求，绝不静默降级。
    const FrameHeader& header  = frame.header;
    const std::size_t  payload = header.type == FrameType::Remote ? header.remote_length
                                                                  : frame.data.size();

    if (header.format == FrameFormat::Fd)
    {
        if (handle_.Init.FrameFormat == FDCAN_FRAME_CLASSIC)
            return Status::Unsupported; // 仅支持 Classic 的外设无法发出 FD 帧
        if (header.type == FrameType::Remote)
            return Status::Unsupported; // CAN FD 没有远程帧
        if (header.bit_rate_switch && handle_.Init.FrameFormat != FDCAN_FRAME_FD_BRS)
            return Status::Unsupported; // 此外设无法切换位速率（BRS）
    }
    else
    {
        if (header.bit_rate_switch)
            return Status::InvalidConfiguration; // BRS 仅存在于 FD 格式
        if (payload > 8U)
            return Status::InvalidConfiguration; // classic 帧最多携带 8 字节
    }

    if (header.type == FrameType::Remote && payload > 8U)
        return Status::InvalidConfiguration; // 远程请求的长度属于 classic DLC

    if (payload > Can::MaxDataLength)
        return Status::Unsupported;

    // 发送始终经由 Tx FIFO/queue 进行。
    if (handle_.Init.TxFifoQueueElmtsNbr == 0U)
        return Status::Unsupported;

    // HAL 将负载 word 写入 Tx element 时不校验 element 尺寸，因此放不下的帧会
    // 覆写相邻的 Message RAM。要求所配置的 element 在 2 word 头部之后能容纳
    // ceil(payload/4) 个 word。
    const std::uint32_t tx_capacity = handle_.Init.TxElmtSize > 2U ? handle_.Init.TxElmtSize - 2U
                                                                   : 0U;
    if ((payload + 3U) / 4U > tx_capacity)
        return Status::Unsupported;

    return Status::Ok;
}

/// 将一帧投入 Tx FIFO/queue。要求外设处于 BUSY；BusOff 只上报不恢复。不重复硬件
/// 能力校验（调用方已做）。@p padded_storage 为 true 时表示负载已由公共层补齐到
/// 4 字节边界，可直接交给 HAL；否则短帧负载会被暂存到安全缓冲，避免 HAL 按 word
/// 读取时越出调用方 span。
Status Can::write(const FrameView& frame, bool padded_storage) noexcept
{
    // 公共层在提交前会调用 validate_hardware_frame()，而软件队列中的条目在入队
    // 时也已校验，因此此处不再重复硬件能力检查。

    // write() 只上报 send() 能理解的若干状态；外设未运行时在此视为硬件侧故障。
    if (handle_.State != HAL_FDCAN_STATE_BUSY)
        return Status::HardwareError;

    // BusOff 只上报、绝不自动恢复：是否重启总线由应用决定。经 HAL 读取协议状态，
    // 避免手写 PSR。
    FDCAN_ProtocolStatusTypeDef protocol_status{};
    (void)HAL_FDCAN_GetProtocolStatus(&handle_, &protocol_status);
    if (protocol_status.BusOff != 0U)
        return Status::BusOff;

    if (HAL_FDCAN_GetTxFifoFreeLevel(&handle_) == 0U)
        return Status::Busy;

    const bool        remote        = frame.header.type == FrameType::Remote;
    const std::size_t payload_bytes = remote ? frame.header.remote_length : frame.data.size();

    const std::uint32_t dlc = dlc_from_length(payload_bytes);
    if (dlc == invalid_dlc)
        return Status::InvalidConfiguration;

    FDCAN_TxHeaderTypeDef tx{};
    tx.Identifier = frame.header.id;
    tx.IdType = frame.header.id_type == IdType::Extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    tx.TxFrameType         = remote ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
    tx.DataLength          = dlc;
    tx.ErrorStateIndicator = frame.header.error_state_indicator ? FDCAN_ESI_PASSIVE
                                                                : FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = frame.header.bit_rate_switch ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
    tx.FDFormat = frame.header.format == FrameFormat::Fd ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker      = 0U;

    // HAL 按 4 字节 word 复制负载，直到 DLCtoBytes[dlc]，因此长度为 4 的整数倍的
    // span 会被原样读取，而长度 1/2/3/5/6/7 时会多读到其后的至多 3 字节。远程帧
    // 根本不携带负载，绝不能触碰调用方的空 span。填充帧的向上取整 word 尾部已由
    // 公共层初始化，此处无需再复制。
    const std::size_t read_words = (payload_bytes + 3U) / 4U;
    const std::size_t copy_bytes = remote ? 0U : frame.data.size();

    alignas(std::uint32_t) std::uint8_t staging[8]; // 仅使用 [0, read_words * 4)
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
        // 短帧或远程帧：只暂存 HAL 实际会读取的字节，从而绝不触碰调用方存储
        // 范围之外的任何数据。
        if (read_words * 4U > sizeof(staging))
            return Status::InvalidConfiguration; // 对合法 DLC 不可达
        if (copy_bytes != 0U)
            std::memcpy(staging, frame.data.data(), copy_bytes);
        for (std::size_t index = copy_bytes; index < read_words * 4U; ++index)
            staging[index] = 0U;
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(&handle_, &tx, payload) != HAL_OK)
    {
        // 空闲级别已在上面复查，故对剩余原因分类：TX 满与 BusOff 均经 HAL 读取，
        // 不再手写 TXFQS/PSR。
        if (HAL_FDCAN_GetTxFifoFreeLevel(&handle_) == 0U)
            return Status::Busy;
        (void)HAL_FDCAN_GetProtocolStatus(&handle_, &protocol_status);
        if (protocol_status.BusOff != 0U)
            return Status::BusOff;
        return Status::HardwareError;
    }

    return Status::Ok;
}

/// 从 @p location（FDCAN_RX_FIFO0/FIFO1，或专用 Rx buffer 索引）取出一个已接收
/// 帧。要求外设处于 BUSY 状态，否则返回 HardwareError。
///
/// 唯一的取帧路径是 HAL_FDCAN_GetRxMessage()：由 HAL 完成 Message RAM 的 header/
/// payload 复制以及出队/清标志（ack）。本驱动不再手工 peek Message RAM、解析 element、
/// 复制负载或写入确认，因此 overwrite 与非 overwrite 不再区分，取帧与确认行为完全由
/// HAL 决定。
///
/// 空判断在本函数中只做一次（先 query 再取帧），因此随后 HAL 取帧并自行确认，不会
/// 因重复查询清掉标志而丢消息：
///  - FIFO0/FIFO1：检查对应 Init 分配数量非 0，且 HAL_FDCAN_GetRxFifoFillLevel() 不为 0；
///  - 专用 Rx buffer：检查 location 在 min(RxBuffersNbr, 64) 之内，且
///    HAL_FDCAN_IsRxBufferMessageAvailable() 返回非 0。
/// 这些判断不读取任何直接寄存器或 Message RAM。
///
/// @warning HAL 已知缺陷（H7）：本驱动按用户决定不修复、不区分 overwrite 模式。
/// 当前 STM32H7 HAL 在 FIFO 已满且配置为 overwrite 时，把 get index 以
/// `(GetIndex + 1) & ((RXFxC & RxC_S) >> RxC_S_Pos)` 回绕，即把 FIFO 容量当作掩码
/// 而非“元素个数 - 1”。当容量为 2 的幂时该回绕会越界：例如容量 8、GetIndex 7 时
/// 得到 8，指向 element 范围之外，导致读取错误的 Message RAM 地址（甚至可能读到相邻
/// 区域）。本驱动不通过配置拒绝 overwrite、不做范围规避，overwrite 模式下的丢帧/
/// 越界读取属于 HAL 的既有行为。
///
/// @warning HAL_IsRxBufferMessageAvailable() 内部会清除该 buffer 的 NDAT 位（HAL 既有
/// 副作用）。本驱动遵循 HAL，不对此做任何补偿，也不手工读写 NDAT。
///
/// @warning HAL_GetRxMessage() 复制负载时不校验 Rx element 容量（按
/// DLCtoBytes[DataLength] 直接复制）。若收到的帧超出所配置 element 容量，HAL 内部可能
/// 已越界访问 Message RAM，取帧后的长度校验无法修正它，只能把该帧判为
/// InvalidFrame（frame.data 为空）。
///
/// 负载直接复制进调用方提供的 @p buffer，帧只借用其中有效长度：HAL 消费 element 后
/// 硬件即可复用该 element，不影响本次输出；取帧同时完成出队/清标志，因此同一 element
/// 不会被重复取出。buffer 必须比返回的 frame 视图更长寿。
///
/// @p buffer 须提供至少 MaxDataLength 字节且非空，否则在任何 HAL 写入或消费之前
/// 即返回 InvalidArgument。仅 Ok 时 frame 有效：frame.header 由后端直接填充，
/// frame.data 借用 buffer 中有效负载（远程帧有效长度为 0）。InvalidFrame 表示帧已
/// 消费并确认但不可交付，此时 frame.data 为空，调用方应继续排空该位置。
///
/// @retval Status::Ok             成功交付一帧有效数据
/// @retval Status::Empty          该位置已无待取数据
/// @retval Status::InvalidArgument buffer 为空或不足以容纳 MaxDataLength 字节
/// @retval Status::InvalidFrame   已消费并确认（acknowledge），但内容不可交付
///                                （如 FD 远程帧、超出 element 容量的截断帧），
///                                调用方应继续排空该位置
Status Can::read(std::uint32_t location, std::span<std::uint8_t> buffer, FrameView& frame) noexcept
{
    // 在任何 HAL 写入或消费（出队/清标志）之前校验输出容量，避免越界写或
    // 消费后才发现无法交付。
    if (buffer.data() == nullptr || buffer.size() < MaxDataLength)
        return Status::InvalidArgument;

    if (handle_.State != HAL_FDCAN_STATE_BUSY)
        return Status::HardwareError;

    // 一次性空判断，并按 Init 的 Rx element wordsize 计算容量，供取帧后检查长度。
    std::size_t capacity = 0U;
    if (location == FDCAN_RX_FIFO0 || location == FDCAN_RX_FIFO1)
    {
        const bool          fifo0      = location == FDCAN_RX_FIFO0;
        const std::uint32_t configured = fifo0 ? handle_.Init.RxFifo0ElmtsNbr
                                               : handle_.Init.RxFifo1ElmtsNbr;
        const std::uint32_t allocated  = std::min(configured, rx_element_hardware_limit);
        if (allocated == 0U)
            return Status::Empty;
        if (HAL_FDCAN_GetRxFifoFillLevel(&handle_, location) == 0U)
            return Status::Empty;

        capacity = element_payload_capacity(fifo0 ? handle_.Init.RxFifo0ElmtSize
                                                  : handle_.Init.RxFifo1ElmtSize);
    }
    else
    {
        // 越界位置与已读空位置一样视为无数据；HAL 按 Init.RxBufferSize 计算 buffer
        // 地址，故容量同样取自 RxBufferSize。
        const std::uint32_t allocated = std::min(handle_.Init.RxBuffersNbr,
                                                 rx_element_hardware_limit);
        if (location >= allocated)
            return Status::Empty;
        // 会清除该 buffer 的 NDAT 位（HAL 既有副作用）；只查询这一次，随后 HAL 取帧。
        if (HAL_FDCAN_IsRxBufferMessageAvailable(&handle_, location) == 0U)
            return Status::Empty;

        capacity = element_payload_capacity(handle_.Init.RxBufferSize);
    }

    // 唯一取帧路径：无条件交给 HAL 写 buffer 并出队/清标志（ack）；本驱动不自行确认。
    FDCAN_RxHeaderTypeDef native; // 仅在 HAL_OK 后读取，所需字段均由 HAL 填充。
    if (HAL_FDCAN_GetRxMessage(&handle_, location, &native, buffer.data()) != HAL_OK)
        return Status::HardwareError; // 保留 HAL 返回的失败，不自行消费或补偿。

    const bool  fd     = native.FDFormat == FDCAN_FD_CAN;
    const bool  remote = native.RxFrameType == FDCAN_REMOTE_FRAME;
    std::size_t length = bytes_from_dlc(native.DataLength);
    // Classic 的 DLC 9..15 仍只表示 8 字节，不能按 FD 的非线性长度交付。
    if (!fd && length > 8U)
        length = 8U;

    auto& header   = frame.header;
    header.id      = native.Identifier;
    header.id_type = native.IdType == FDCAN_EXTENDED_ID ? IdType::Extended : IdType::Standard;
    header.format  = fd ? FrameFormat::Fd : FrameFormat::Classic;
    header.type    = remote ? FrameType::Remote : FrameType::Data;
    header.bit_rate_switch       = fd && native.BitRateSwitch == FDCAN_BRS_ON;
    header.error_state_indicator = fd && native.ErrorStateIndicator == FDCAN_ESI_PASSIVE;
    header.remote_length         = 0U;

    if (fd && remote)
    {
        // CAN FD 不支持远程帧；HAL 已完成消费，此处只拒绝交付。
        frame.data = {};
        return Status::InvalidFrame;
    }
    if (remote)
    {
        // 远程帧的 DLC 表示请求长度，不是本帧携带的负载长度。
        header.remote_length = static_cast<std::uint8_t>(length);
        frame.data           = buffer.first(0);
        return Status::Ok;
    }
    if (length > capacity)
    {
        // 这是取帧后的交付检查，不修正 HAL 内部可能已经发生的越界访问。
        frame.data = {};
        return Status::InvalidFrame;
    }
    frame.data = buffer.first(length);
    return Status::Ok;
}

/// 应用一条过滤器配置：要求外设已初始化且处于 READY（HAL 拒绝在运行中修改全局
/// 过滤器）。按 @p config 的变体分派到 ID/全局/扩展掩码过滤器；bxCAN 专有的
/// bank 配置无法在 FDCAN 上表达，返回 Unsupported。
Status Can::configure_filter_hardware(const FilterConfig& config) noexcept
{
    // 指向非真实 FDCAN 外设的 handle 永远无法被编程（公共层只拒绝了 nullptr），
    // 且外设一旦运行，HAL 将拒绝更改全局过滤器配置。
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
                    // BankFilter 描述 bxCAN 的 bank 布局，FDCAN 的 filter list 无法表达。
                    return Status::Unsupported;
                }
            },
            config);
}

// ---------------------------------------------------------------------------
// HAL 回调桥接。每个回调都运行在 FDCAN 中断线上的中断上下文里；它只查找所属
// 实例并转交到共享的 on_receive()/on_tx_available() 入口，后者通过不同的寄存器
// 重新进入 HAL，且不保留任何状态。
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

    // 此处只遍历配置的下标，存在性判断统一留给 read()。
    // HAL_FDCAN_IsRxBufferMessageAvailable 会清 NDAT；若此处先查询，read() 再次
    // 查询将得到“无数据”，导致漏掉该消息。不能在两个位置重复查询。
    const std::uint32_t limit = instance->handle_.Init.RxBuffersNbr < rx_element_hardware_limit
                                        ? instance->handle_.Init.RxBuffersNbr
                                        : rx_element_hardware_limit;
    for (std::uint32_t index = 0U; index < limit; ++index)
        instance->on_receive(index);
}

void Can::tx_buffers_irq(NativeHandle* handle, std::uint32_t)
{
    // 完成或取消的 buffer 会释放 Tx FIFO/queue 的空间，因此软件队列无需等待
    // 下一次 send() 即可继续推进。
    if (Can* instance = find(handle))
        instance->on_tx_available();
}

void Can::tx_irq(NativeHandle* handle)
{
    if (Can* instance = find(handle))
        instance->on_tx_available();
}

} // namespace bsp::can

#endif // CAN_DRIVER_FDCAN：FDCAN 后端。

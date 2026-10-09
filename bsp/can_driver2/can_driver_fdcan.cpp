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

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>
#include <variant>

namespace bsp::can
{
namespace
{
/// 无对应 FDCAN DLC 编码的字节长度的哨兵值。
constexpr std::uint32_t invalid_dlc = 0xFFFFFFFFU;

// Rx element 内各字段的掩码。它们与 HAL 内部的 FDCAN_ELEMENT_MASK_* 值一致；
// HAL 未导出这些宏，但 message RAM 的布局属于架构规格。
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

// 硬件（硅）上限，与 handle 的 Init 所声明的数量无关。
constexpr std::uint32_t standard_filter_hardware_limit{128U};
constexpr std::uint32_t extended_filter_hardware_limit{64U};
constexpr std::uint32_t rx_element_hardware_limit{64U};

/// 返回恰好能存放 @p length 个数据字节的 DLC 编码；直接使用 HAL 宏，不做位移。
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

/// 返回某个 DLC 编码所表示的字节数；编码 9..15 不是线性递增的。
constexpr std::size_t bytes_from_dlc(const std::uint32_t dlc) noexcept
{
    constexpr std::size_t table[16] = {0U,  1U,  2U,  3U,  4U,  5U,  6U,  7U,
                                       8U,  12U, 16U, 20U, 24U, 32U, 48U, 64U};
    return dlc < 16U ? table[dlc] : 0U;
}

/// 一个由 @p words 个 32-bit word 组成的 element，在扣除其 2 word 的头部之后
/// 可容纳的负载字节数；element 尺寸非法时返回 0。
constexpr std::size_t element_payload_capacity(const std::uint32_t words) noexcept
{
    return words >= 2U ? words * 4U - 8U : 0U;
}

/// 将单个非匹配动作翻译为 HAL 取值；拒绝损坏的判别符，以及目标 FIFO 未分配
/// element 存储空间的 accept 目标。
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

    return HAL_FDCAN_ConfigFilter(&handle, &native) == HAL_OK ? Status::Ok
                                                              : Status::HardwareError;
}
} // 匿名命名空间

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
            HAL_FDCAN_RegisterCallback(&handle_, HAL_FDCAN_RX_BUFFER_NEW_MSG_CB_ID,
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
    std::uint32_t active_its  = 0U;
    if (handle_.Init.RxFifo0ElmtsNbr > 0U)
        active_its |= FDCAN_IT_RX_FIFO0_NEW_MESSAGE;
    if (handle_.Init.RxFifo1ElmtsNbr > 0U)
        active_its |= FDCAN_IT_RX_FIFO1_NEW_MESSAGE;
    if (handle_.Init.RxBuffersNbr > 0U)
        active_its |= FDCAN_IT_RX_BUFFER_NEW_MESSAGE;

    // 软件 Tx 队列由完成/取消通知以及 Tx FIFO 空通知来推进，绝不会由后续的
    // send() 调用推进。没有软件队列时完全不需要任何 Tx 中断。
    std::uint32_t tx_buffer_mask = 0U;
    if (tx_queue_capacity > 0U)
    {
        active_its |= FDCAN_IT_TX_FIFO_EMPTY | FDCAN_IT_TX_COMPLETE |
                      FDCAN_IT_TX_ABORT_COMPLETE;
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

    if (payload > max_data_length)
        return Status::Unsupported;

    // 发送始终经由 Tx FIFO/queue 进行。
    if (handle_.Init.TxFifoQueueElmtsNbr == 0U)
        return Status::Unsupported;

    // HAL 将负载 word 写入 Tx element 时不校验 element 尺寸，因此放不下的帧会
    // 覆写相邻的 Message RAM。要求所配置的 element 在 2 word 头部之后能容纳
    // ceil(payload/4) 个 word。
    const std::uint32_t tx_capacity =
            handle_.Init.TxElmtSize > 2U ? handle_.Init.TxElmtSize - 2U : 0U;
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

    // BusOff 只上报、绝不自动恢复：是否重启总线由应用决定。
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
        // 空闲级别已在上面复查，故对剩余原因进行分类。
        if ((handle_.Instance->TXFQS & FDCAN_TXFQS_TFQF) != 0U)
            return Status::Busy;
        if ((handle_.Instance->PSR & FDCAN_PSR_BO) != 0U)
            return Status::BusOff;
        return Status::HardwareError;
    }

    return Status::Ok;
}

/// 从 @p location（FDCAN_RX_FIFO0/FIFO1，或专用 Rx buffer 索引）取出一个已接收
/// 帧。要求外设处于 BUSY 状态，否则返回 HardwareError。
///
/// 直接读取硬件更新的 Message RAM（volatile），不经过任何中间缓冲；取到的数据
/// 在本次调用返回后即可被硬件覆盖，调用方必须在返回前完成消费。取帧时同步完成
/// 出队/清标志（FIFO 写 RXFx A 寄存器，buffer 清对应 NDAT 位），因此同一 element
/// 不会被重复取出。
///
/// @retval Status::Ok            成功交付一帧有效数据
/// @retval Status::Empty         该位置已无待取数据
/// @retval Status::InvalidFrame  已消费并确认（acknowledge），但内容不可交付
///                               （如 FD 远程帧、超出 element 容量的截断帧），
///                               调用方应继续排空该位置
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
        // overwrite 模式下 FIFO 已满时，get index 指向的 element 已被丢弃。
        // 需将其前移一位并按已分配数量回绕；若像 HAL 那样与数量做按位与，在
        // 非 2 的幂数量下是错误的，会使索引越界。
        if (((status & FDCAN_RXF0S_F0F) >> FDCAN_RXF0S_F0F_Pos) == 1U &&
            ((instance->RXF0C & FDCAN_RXF0C_F0OM) >> FDCAN_RXF0C_F0OM_Pos) ==
                    FDCAN_RX_FIFO_OVERWRITE)
        {
            ++get_index;
            if (get_index >= allocated)
                get_index = 0U;
        }

        // 状态寄存器给出的索引绝不能超出实际已分配的范围。
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
        // 专用 Rx buffer；其 new-data 标志会一直保持，直到 read() 将其清除，
        // 因此已读空的位置返回 Empty 以终止调用方的循环。
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
    // BRS 与 ESI 仅存在于 FD 格式；classic element 从不定其义。
    header.bit_rate_switch      = fd && ((word2 & element_mask_brs) != 0U);
    header.error_state_indicator = fd && ((word1 & element_mask_esi) != 0U);
    header.remote_length        = 0U;

    std::size_t length = bytes_from_dlc(dlc);
    // classic 帧最多携带 8 字节，无论原始 DLC 编码为何；不要在此把编码 9..15
    // 换算成 12..64 字节。
    if (!fd && length > 8U)
        length = 8U;

    bool deliverable = true;
    if (remote && fd)
    {
        // CAN FD 没有远程帧，故此类 element 非法：在下方消费并确认它，
        // 但绝不交付。
        deliverable  = false;
        frame.length = 0U;
    }
    else if (remote)
    {
        // 远程帧不存储负载：长度字段是协议请求而非数据，故绝不截断。
        header.remote_length = static_cast<std::uint8_t>(length);
        frame.length         = 0U;
    }
    else if (length > element_payload || length > max_data_length)
    {
        // 所配置的 element 小于收到的负载，说明硬件已存储了一个被截断的帧。
        // 在下方消费并确认它，但将其报告为不可交付，而不是返回一个短帧；
        // 且绝不读到该 element 的 Message RAM 之外。
        deliverable  = false;
        frame.length = 0U;
    }
    else
    {
        // 逐 word 读取 element，而非对负载 memcpy()：该 element 位于硬件随时
        // 更新的内存中，编译器不得缓存它，且只提取最后一个 word 中的有效字节。
        // ceil(length/4) 个 word 不会越出 element，因为上面确认的已配置负载容量
        // 始终是 4 字节的整数倍。
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

    // 数据已从 element 复制到调用方输出后，才执行确认（出队/清标志）：此时
    // element 即便随即被硬件复用，也不会影响本次已取到的数据。
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

    // InvalidFrame 表示“已在上方消费并确认，但不可交付”；公共层会继续排空该位置
    // 而不是派发这一帧。
    return deliverable ? Status::Ok : Status::InvalidFrame;
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
                    // BankFilter 与 FilterBankSplit 描述的是 bxCAN 的 bank 布局，
                    // FDCAN 的 filter list 无法表达。
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

    // HAL 只清除了共享中断标志，因此各 buffer 的 new-data 标志在此仍然有效；
    // read() 会逐个 buffer 地清除它们。
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

} // 命名空间 bsp::can

#endif // CAN_DRIVER_FDCAN：FDCAN 后端。

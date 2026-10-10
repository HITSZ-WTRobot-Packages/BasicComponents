/**
 * @file can_driver_bxcan.cpp
 * @brief 轻量 CAN 封装的经典 bxCAN 后端实现。
 *
 * 仅当固件启用经典 CAN HAL（HAL_CAN_MODULE_ENABLED -> CAN_DRIVER_FDCAN == 0）
 * 时才参与编译；否则整个翻译单元为空，避免同一个 Can 成员被重复定义。
 *
 * 支持的硬件配置：
 * - 只支持经典 CAN 2.0A/B 帧；FD 格式（以及其 BRS/ESI 标志）以
 *   Status::Unsupported 拒绝，而不是降级发送。
 * - 载荷上限 8 字节。TX 侧：HAL_CAN_AddTxMessage() 总是搬运 8 个字节，因此远程帧
 *   和调用方传入的短 span 会先经一个清零的 8 字节缓冲暂存，而 padded_storage 帧
 *   （背后是 StoredFrame，其可读尾部已初始化）直接交给 HAL。
 * - RX 侧：HAL_CAN_GetRxMessage() 同样无条件写满 8 个字节，因此 read() 直接写入
 *   调用方提供的、至少 MaxDataLength 字节的非空缓冲，并让 FrameView::data 借用其
 *   中的有效前缀（不拷贝、不清零）。该缓冲归调用方所有，视图只在缓冲被复用或销毁
 *   前有效；on_receive() 每轮复用它，故视图仅在本次回调期间有效。
 * - 三个 Tx 邮箱；邮箱全满返回 Status::Busy，该帧仍归属公共软件队列。本层不会
 *   自行重试：软件队列只由邮箱空/中止/错误通知推动前进。
 * - Bus-off 由 ESR.BOFF 锁存，节点恢复前 write() 一直返回 Status::BusOff（恢复
 *   动作由板级负责）。该判断用 __HAL_CAN_GET_FLAG(CAN_FLAG_BOF) 实时读回 ESR，
 *   而不是读 HAL_CAN_GetError() 的错误码缓存——后者只在 HAL 的中断/错误回调路径
 *   更新，可能落后于硬件；错误通知仅负责推动软件队列，二者互不替代。
 *
 * 滤波器即原始 bxCAN bank：
 * - IdFilter::Mask 映射到一个 32 位 mask bank（比较标识符位，IDE 固定为请求的
 *   标识符类型，RTR 不参与比较，因此数据帧与远程帧都能匹配）；
 * - 标准 ID 的 IdFilter::List 映射到同一 bank 的两个 16 位 mask 项，IDE/RTR
 *   语义与上相同；两个不同的扩展 ID 无法用具有该语义的单个 bank 表达，返回
 *   Status::Unsupported。
 * - BankFilter 以四个 16/32 位半字原样写入，因此硬件能表达的每种 IDE/RTR 组合
 *   都可达。
 * - Range/FD 专属动作（Priority、RxBuffer 等）、GlobalFilter 与 ExtendedIdMask
 *   在 bxCAN 上没有对应物，返回 Status::Unsupported。
 *
 * 滤波器编程完全由 HAL 主导：本后端不直接写任何滤波器寄存器（FMR/FM1R/FS1R/
 * FFA1R/FA1R/FR1/FR2），一律经 HAL_CAN_ConfigFilter() 完成；该函数每次都用
 * CAN_FilterTypeDef::SlaveStartFilterBank 重写 FMR.CAN2SB，因此本后端在每次配置
 * 时都直接把 CAN_DRIVER_BXCAN_CAN2_START_BANK 填入该字段：CAN1/CAN2 共用域的分
 * 界因此被编译期宏全局固定（取值范围 0..27，默认 14），不存在运行时可改的分界，
 * 也不保留任何分界 getter、bank 0 快照或回写。在 CAN1/CAN2 共用 28 个 bank 的
 * 型号上，滤波器寄存器始终通过 CAN1（主控）访问，即使是 CAN2 的 handle 也一样；
 * 因此主实例也必须被初始化（有时钟）。bank 归属（bank < CAN2SB 属于 CAN1，其余
 * 属于 CAN2）由该宏决定，而 bank 索引仍按调用方给定值写入。CAN3（存在独立 14
 * bank 域的型号）与单控制器型号的原生 HAL 本就忽略 SlaveStartFilterBank 参数，
 * 故本后端对其无条件填入该宏，既不改变其 bank 数量也不影响其保护。
 *
 * 共享域中的每个 bank 都必须在 CAN1/CAN2 这对总线的任一条启动之前配置完成：本
 * 实例须在两路 HAL init 完成、任一条总线启动之前完成配置（公共 configure_filter()
 * 已在配置互斥下保证该顺序）；若另一条已注册总线正在运行，则共享域写入会被拒绝
 * （Status::InvalidState），以保证本实例不会在一条活跃总线之下进入共享滤波器
 * 初始化模式。两路 HAL init 完成后再配共享 bank 时，禁止随后任何外部 HAL 以不同
 * 的 SlaveStartFilterBank 重置分界。CAN3 拥有自有的 14 个 bank，不受该约束。
 */

#include "can_driver.hpp"

#if !CAN_DRIVER_FDCAN

namespace bsp::can
{

/**
 * 每个 bxCAN 寄存器块内只有一个共享滤波器域：单控制器型号 14 个 bank；存在 CAN2
 * 时（以及部分型号上独立的 14 bank CAN3）CAN1 背后共有 28 个 bank。下面的宏是本
 * 后端唯一的芯片相关假设，可由构建系统覆盖，以适配厂商头文件以不同方式描述该域
 * 的型号。
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
 * 滤波器关键字段的位布局。
 *
 * 32 位布局（RM0090 32.7.2）：标准标识符位于第 31:21 位（STID[10:0]）；
 * 扩展标识符的 29 位位于第 31:3 位（STID[10:0] + EXID[17:0]），IDE 为
 * 第 2 位，RTR 为第 1 位。
 * 16 位布局：STID[10:0]（或 EXID[28:18]）位于第 15:5 位，RTR 为第 4 位，
 * IDE 为第 3 位，EXID[17:15] 位于第 2:0 位。RTR 和 IDE 的位置不同于
 * 32 位布局；扩展 ID 仅比较高 14 位，低 15 位不参与比较。
 */
constexpr std::uint32_t filter_32_id_shift  = 21U;    ///< STID[10:0] 的位置。
constexpr std::uint32_t filter_32_ext_shift = 3U;     ///< 29 位扩展 ID 的位置。
constexpr std::uint32_t filter_32_ide       = 0x0004U; ///< 32 位 key 的 IDE 位。
constexpr std::uint32_t filter_16_id_shift  = 5U;     ///< STID[10:0] / EXID[28:18] 的位置。
constexpr std::uint32_t filter_16_ide       = 0x0008U; ///< 16 位 key 的 IDE 位。

/// 标准标识符上界 (2^11 - 1)。
constexpr std::uint32_t standard_id_limit = 0x7FFU;
/// 扩展标识符上界 (2^29 - 1)。
constexpr std::uint32_t extended_id_limit = 0x1FFFFFFFU;

/// 完整的 16 位标识符掩码：STID[10:0] 全部位加上 IDE 位。
constexpr std::uint32_t filter_16_standard_mask =
    (standard_id_limit << filter_16_id_shift) | filter_16_ide;

/// 用于精确匹配单个扩展 ID 的完整 32 位标识符掩码（比较 IDE=1，忽略 RTR 与
/// 保留位）。
constexpr std::uint32_t filter_32_exact_extended_mask =
    (extended_id_limit << filter_32_ext_shift) | filter_32_ide;

/// 判断该 handle 使用的是共享的 CAN1/CAN2 域，还是某控制器自有的 14 bank 域。
[[nodiscard]] bool has_shared_filter_banks(const CAN_HandleTypeDef& handle) noexcept
{
#if CAN_DRIVER_BXCAN_SHARED_FILTER_BANKS
#    if defined(CAN3)
    // CAN3 拥有自己的滤波器 bank，不属于 CAN1/CAN2 共享域。
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

/// 该 handle 所属滤波器域可用的最大 bank 索引（开区间上界）。
[[nodiscard]] std::uint32_t filter_bank_count(const CAN_HandleTypeDef& handle) noexcept
{
    return has_shared_filter_banks(handle) ? CAN_DRIVER_BXCAN_SHARED_FILTER_BANK_COUNT
                                           : CAN_DRIVER_BXCAN_SINGLE_FILTER_BANKS;
}

/// 以编译期固定分界构造 bank 模板，使 HAL_CAN_ConfigFilter() 以
/// CAN_DRIVER_BXCAN_CAN2_START_BANK 重写 FMR.CAN2SB（全局固定分界）。其余字段给
/// 出确定的初值，避免使用未初始化的栈字段。
[[nodiscard]] CAN_FilterTypeDef make_bank() noexcept
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
    filter.SlaveStartFilterBank = CAN_DRIVER_BXCAN_CAN2_START_BANK;
    return filter;
}

/// 把 HAL 的返回值折叠为统一的 Status；HAL 只在参数/状态异常时失败。
[[nodiscard]] Status apply_bank(CAN_HandleTypeDef& handle, const CAN_FilterTypeDef& filter) noexcept
{
    return HAL_CAN_ConfigFilter(&handle, &filter) == HAL_OK ? Status::Ok : Status::HardwareError;
}

/// IdFilter：仅按 ID 匹配，绝不按帧类型匹配（见下方 RTR 处理）。
[[nodiscard]] Status configure_id_bank(CAN_HandleTypeDef& handle, const IdFilter& request) noexcept
{
    const std::uint32_t bank_count = filter_bank_count(handle);
    if (request.index >= bank_count)
        return Status::OutOfRange;

    // calibration_message/rx_buffer_index 只描述 FDCAN 的 RxBuffer，bxCAN 的
    // FIFO bank 无法实现，故拒绝而不是忽略。
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

    CAN_FilterTypeDef filter = make_bank();
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
        // bxCAN 的 bank 无法拒绝、无法设置优先级，也不能充当 RxBuffer。
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

    // id1/id2 按所选 ID 类型限幅：超出即配置非法，不截断。
    const std::uint32_t id_limit = extended ? extended_id_limit : standard_id_limit;
    if (request.id1 > id_limit || request.id2 > id_limit)
        return Status::InvalidConfiguration;

    switch (request.mode)
    {
        case FilterMode::Mask:
        {
            // 使用一个 32 位 mask bank。key 只对标识符做哈希：IDE 按请求的类型
            // 比较（标准滤波器因此无法捕获扩展帧），RTR 始终不参与屏蔽，所以同一
            // 标识符的数据帧与远程帧都会匹配。
            // 掩码低半字额外或上 filter_32_ide：IDE 位不在标识符位域内，必须由
            // 掩码显式选中才会参与比较（id2 本身可能未置该位）。
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
                // 在一个 bank 的两个 16 位 mask 项里放两个精确的标准标识符。16
                // 位 key 为 STID<<5 | RTR<<4 | IDE<<3，因此比较标识符位加 IDE 位
                // 既排除扩展帧，又让数据帧与远程帧都匹配（RTR 位被掩码忽略）。
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
                // 16 位 bank 只比较扩展 ID 的 EXID[28:15]（共 14 位），两个不同的
                // 29 位 ID 无法同时精确匹配，因此该请求被拒绝，而不是放宽匹配范围。
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
            return Status::Unsupported; // 区间过滤是 FDCAN 才有的滤波模式。
        default:
            return Status::InvalidArgument;
    }

    return apply_bank(handle, filter);
}

/// BankFilter：四个硬件半字原样写入，因此硬件可表达的任何 IDE/RTR/scale/mode
/// 组合都不会在本层被解释、丢弃或改写；index 仍受本域 bank 数量限制，scale/
/// mode/fifo/enabled 也只做合法性检查后直通。
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

    CAN_FilterTypeDef filter = make_bank();
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

} // 匿名命名空间

Status Can::start_hardware() noexcept
{
    if (handle_.Instance == nullptr || !IS_CAN_ALL_INSTANCE(handle_.Instance))
        return Status::InvalidState;

    // 只允许认领已初始化但尚未启动的外设。已处于监听状态的 handle 属于别人的
    // start()，此处拒绝，以保证 rollback_start() 绝不会把它停掉。
    if (handle_.State != HAL_CAN_STATE_READY)
        return Status::InvalidState;

    // HAL 只允许在 READY 状态安装回调。FIFO0/FIFO1 接收回调无条件注册；注册失败
    // 直接返回 HardwareError，此时 hardware_started_ 仍为 false，因此调用方的
    // rollback_start() 不会去动外设。
    if (HAL_CAN_RegisterCallback(&handle_, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, rx_fifo0_irq) !=
            HAL_OK ||
        HAL_CAN_RegisterCallback(&handle_, HAL_CAN_RX_FIFO1_MSG_PENDING_CB_ID, rx_fifo1_irq) !=
            HAL_OK)
        return Status::HardwareError;

    if (Can::TxQueueCapacity > 0U)
    {
        // 发送完成或发送中止会释放对应邮箱；错误/bus-off 状态迁移也可能让待发邮箱
        // 被中止而释放。任一邮箱被释放都可能推动软件队列前进，故三类通知一并注册。
        constexpr HAL_CAN_CallbackIDTypeDef tx_callbacks[] = {
            HAL_CAN_TX_MAILBOX0_COMPLETE_CB_ID, HAL_CAN_TX_MAILBOX1_COMPLETE_CB_ID,
            HAL_CAN_TX_MAILBOX2_COMPLETE_CB_ID, HAL_CAN_TX_MAILBOX0_ABORT_CB_ID,
            HAL_CAN_TX_MAILBOX1_ABORT_CB_ID,    HAL_CAN_TX_MAILBOX2_ABORT_CB_ID,
            HAL_CAN_ERROR_CB_ID};
        for (const HAL_CAN_CallbackIDTypeDef id : tx_callbacks)
            if (HAL_CAN_RegisterCallback(&handle_, id, tx_irq) != HAL_OK)
                return Status::HardwareError;
    }

    // 进入正常模式。时序、GPIO 与 MSP 初始化由板级负责；INRQ/INAK 握手需要等待，
    // 因此此处必须允许中断才能让 HAL 的超时推进。
    if (HAL_CAN_Start(&handle_) != HAL_OK)
        return Status::HardwareError;

    // 从这里起该外设由本实例启动，也只有此时 rollback_start() 才允许停止它。
    hardware_started_ = true;
    return Status::Ok;
}

Status Can::enable_notifications() noexcept
{
    // 调用时中断已被屏蔽：下面每个 HAL 调用都是纯寄存器写入，绝不等待。
    std::uint32_t interrupts = CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING;
    if (Can::TxQueueCapacity > 0U)
    {
        // 邮箱空会推动软件队列前进；bus-off 与错误状态迁移可能中止待发邮箱，
        // 同样释放邮箱并推动队列，因此一并使能相应中断源。
        interrupts |= CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_ERROR | CAN_IT_BUSOFF |
                      CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE;
    }

    if (HAL_CAN_ActivateNotification(&handle_, interrupts) != HAL_OK)
        return Status::HardwareError;
    return Status::Ok;
}

void Can::rollback_start() noexcept
{
    // 本实例没有启动过任何东西：不动外设（它可能属于其他 owner）。
    if (!hardware_started_)
        return;
    // 先清标志保证可重入/幂等；随后无条件关闭本后端可能开启的全部通知位（不依赖
    // 队列容量配置），并停止外设。返回值被忽略：失败时也没有可做的补偿动作。
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
    // send() 在公共帧校验之后调用此能力闸门；仅 FDCAN 才有的特性在这里明确拒绝，
    // 而不是被静默丢弃或降级。
    if (frame.header.format != FrameFormat::Classic)
        return Status::Unsupported;
    if (frame.header.bit_rate_switch || frame.header.error_state_indicator)
        return Status::Unsupported;

    // 标识符范围按 id_type 检查（标准 11 位 / 扩展 29 位）。
    if (!is_valid_id(frame.header.id_type, frame.header.id))
        return Status::InvalidArgument;

    if (frame.header.type == FrameType::Remote)
    {
        // 远程帧用 remote_length 请求最多 8 字节，本身不携带载荷，因此 data 必须为空。
        if (frame.header.remote_length > MaxDataLength || !frame.data.empty())
            return Status::InvalidArgument;
    }
    else if (frame.data.size() > MaxDataLength)
    {
        return Status::InvalidArgument;
    }

    return Status::Ok;
}

Status Can::write(const FrameView& frame, const bool padded_storage) noexcept
{
    // 公共层只把手校验过的帧交给后端（send() 会执行 validate_frame +
    // validate_hardware_frame；flush_tx 重放的是入队时即已校验的帧），因此这里只需
    // 检查当前时刻的硬件状态与能力。

    // READY 的外设仍处于初始化模式：此时 HAL 会接受该帧并报告成功，但总线上不会
    // 发出任何东西，因此只有 LISTENING 状态可写。
    if (handle_.State != HAL_CAN_STATE_LISTENING)
        return Status::InvalidState;

    // Bus-off 被锁存直到节点恢复；此时提交的帧永远出不了邮箱，故直接返回 BusOff。
    // 这里用 CAN_FLAG_BOF 实时读回 ESR.BOFF，而不是读 HAL_CAN_GetError() 的错误码
    // 缓存——该缓存只在 HAL 的中断/错误回调路径更新，可能落后于硬件。
    if (__HAL_CAN_GET_FLAG(&handle_, CAN_FLAG_BOF) != RESET)
        return Status::BusOff;

    // 邮箱全满则本层无法提交；帧仍归属公共软件队列，由后续通知推动重试。空闲邮箱
    // 由 HAL/硬件在三个邮箱中挑选，且帧真正上总线的先后由标识符仲裁决定，因此本层
    // 不保证实际发送顺序与软件提交顺序一致。
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

    // HAL_CAN_AddTxMessage() 无条件读取 8 个数据字节。完整 8 字节载荷直接零拷贝
    // 传入，padded_storage 帧也一样（其背后是 StoredFrame，可读尾部已初始化）。
    // 短 span 以及不携带任何载荷的远程帧则先暂存到清零缓冲，保证被读取的 8 字节
    // 全部已定义。
    std::uint8_t        staging[MaxDataLength] = {};
    const std::uint8_t* data                     = staging;
    if (header.RTR == CAN_RTR_DATA && frame.data.data() != nullptr &&
        (padded_storage || frame.data.size() == MaxDataLength))
    {
        data = frame.data.data();
    }
    else
    {
        // 拷贝量由已校验的载荷长度决定并受 8 字节上限约束；更长的 span 只可能来自
        // 绕过公共校验的调用方，此时多余部分不会被读入暂存缓冲。
        const std::size_t payload = frame.data.size() < MaxDataLength ? frame.data.size()
                                                                       : MaxDataLength;
        for (std::size_t i = 0; i < payload; ++i)
            staging[i] = frame.data[i];
    }

    std::uint32_t mailbox = 0U;
    if (HAL_CAN_AddTxMessage(&handle_, &header, data, &mailbox) != HAL_OK)
        return HAL_CAN_GetTxMailboxesFreeLevel(&handle_) == 0U ? Status::Busy
                                                               : Status::HardwareError;
    return Status::Ok;
}

Status Can::read(const std::uint32_t location, const std::span<std::uint8_t> buffer,
                 FrameView& frame) noexcept
{
    // location 是 bxCAN 的 Rx FIFO 索引（CAN_RX_FIFO0 / CAN_RX_FIFO1）。
    if (location != static_cast<std::uint32_t>(CAN_RX_FIFO0) &&
        location != static_cast<std::uint32_t>(CAN_RX_FIFO1))
        return Status::InvalidArgument;

    // HAL_CAN_GetRxMessage() 无条件向缓冲写入 8 个数据字节，因此必须在任何 HAL 访问
    // 或消费之前确认缓冲至少能容纳一个最大长度帧，否则拒绝而不是越界写。
    if (buffer.data() == nullptr || buffer.size() < MaxDataLength)
        return Status::InvalidArgument;

    // 非 Ok 返回一律不得让调用方看到可交付的数据视图。
    frame.data = {};

    if (HAL_CAN_GetRxFifoFillLevel(&handle_, location) == 0U)
        return Status::Empty;

    // HAL_CAN_GetRxMessage() 无条件向缓冲写入 8 个数据字节，因此直接以调用方提供的
    // 最大长度缓冲作为接收缓冲：其可写空间已在上方验证，无需额外暂存，也不经过
    // FrameView::data（后者是只读视图）。未被本帧使用的尾部会残留上一次内容，但不
    // 会通过 frame.data 的长度暴露出去。
    CAN_RxHeaderTypeDef header{};
    if (HAL_CAN_GetRxMessage(&handle_, location, &header, buffer.data()) != HAL_OK)
        return Status::HardwareError;

    FrameHeader& out = frame.header;
    out              = FrameHeader{};
    out.format       = FrameFormat::Classic;
    out.id_type      = header.IDE == CAN_ID_EXT ? IdType::Extended : IdType::Standard;
    out.id           = out.id_type == IdType::Extended ? header.ExtId : header.StdId;
    out.type         = header.RTR == CAN_RTR_REMOTE ? FrameType::Remote : FrameType::Data;

    // bxCAN 上报原始 DLC 半字节，只有 ≤8 的值有效。HAL 已把 ≥8 截断为 8，这里再
    // 做一次防御性截断，保证长度字段始终落在经典 CAN 可表达的范围内。
    const std::uint8_t length = header.DLC > MaxDataLength
                                    ? static_cast<std::uint8_t>(MaxDataLength)
                                    : static_cast<std::uint8_t>(header.DLC);
    if (out.type == FrameType::Remote)
    {
        // 远程帧只请求长度、不携带载荷字节：长度写入 remote_length，payload 长度为 0。
        out.remote_length = length;
        frame.data        = {};
    }
    else
    {
        out.remote_length = 0U;
        // 数据视图直接借用调用方缓冲的有效前缀，不做任何载荷拷贝或清零；该视图仅
        // 在 buffer 被复用或销毁前有效（回调期间由调用方保证）。
        frame.data = std::span<const std::uint8_t>{ buffer.data(), length };
    }

    return Status::Ok;
}

Status Can::configure_filter_hardware(const FilterConfig& config) noexcept
{
    if (handle_.Instance == nullptr || !IS_CAN_ALL_INSTANCE(handle_.Instance))
        return Status::InvalidConfiguration;

    // 滤波器 bank 只允许在总线启动之前编程：LISTENING 的 handle（即便由其他 owner
    // 启动）绝不能被触碰；RESET 的 handle 说明 HAL_CAN_Init() 尚未执行。注意 HAL
    // 自身也允许在 LISTENING 下改滤波器，这里的更严格限制是本驱动的策略。
    if (handle_.State != HAL_CAN_STATE_READY)
        return Status::InvalidState;

#if CAN_DRIVER_BXCAN_SHARED_FILTER_BANKS
    // bank 与 CAN1/CAN2 分界位于同一共享寄存器块中，因此共享域的每次写入都必须
    // 发生在该对总线任一条启动之前。shared_filter_bus_started() 只报告*另外*的已
    // 注册 Can；此处本实例仍处于启动前。
    if (has_shared_filter_banks(handle_) && shared_filter_bus_started(handle_))
        return Status::InvalidState;
#endif

    if (const auto* id_filter = std::get_if<IdFilter>(&config))
        return configure_id_bank(handle_, *id_filter);

    if (const auto* bank_filter = std::get_if<BankFilter>(&config))
        return configure_raw_bank(handle_, *bank_filter);

    // GlobalFilter 与 ExtendedIdMask 描述的是 FDCAN 的接收规则，bxCAN 无对应物。
    return Status::Unsupported;
}

/// HAL 接收回调（FIFO0）的桥接：按 HAL handle 反查已注册的 Can 实例，若该 handle
/// 尚未被任何实例认领则忽略。运行在中断上下文，实际取帧与用户回调派发由
/// on_receive() -> read() 完成。
void Can::rx_fifo0_irq(NativeHandle* handle)
{
    Can* const bus = find(handle);
    if (bus != nullptr)
        bus->on_receive(0U); // 对应 CAN_RX_FIFO0，即接收 FIFO0。
}

/// HAL 接收回调（FIFO1）的桥接，行为同 rx_fifo0_irq()。
void Can::rx_fifo1_irq(NativeHandle* handle)
{
    Can* const bus = find(handle);
    if (bus != nullptr)
        bus->on_receive(1U); // 对应 CAN_RX_FIFO1，即接收 FIFO1。
}

/// Tx 相关回调（邮箱完成/中止、错误）共用的桥接：在中断上下文中让实例推进软件
/// 发送队列；未认领的 handle 被忽略。
void Can::tx_irq(NativeHandle* handle)
{
    Can* const bus = find(handle);
    if (bus != nullptr)
        bus->on_tx_available();
}

} // 命名空间 bsp::can

#endif // !CAN_DRIVER_FDCAN：bxCAN 后端。

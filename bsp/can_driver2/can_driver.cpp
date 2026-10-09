/** @file can_driver.cpp @brief 实例注册表、回调派发与发送队列的共享实现。 */
#include "can_driver.hpp"
#include "isr_lock.h"

#include <cstring>

namespace bsp::can
{

constinit std::array<Can*, CAN_DRIVER_MAX_INSTANCES> Can::instances_{};

/// 析构只释放注册表槽位：不调用 stop()，也不注销 HAL 回调，硬件仍可能继续
/// 产生中断。调用方须在销毁前确保硬件已停止；此后的 IRQ 经 find() 返回
/// nullptr，不再访问本对象。
Can::~Can()
{
    ISRGuard guard;
    release_instance();
}

/// 按 HAL handle 的地址在注册表中查出对应实例。后端 IRQ 回调只有裸句柄，
/// 借此映射回 Can 再转发 on_receive()/on_tx_available()。未注册返回 nullptr，
/// 故已销毁对象不会再被中断解引用。ISR 上下文调用，不加锁。
Can* Can::find(NativeHandle* const handle) noexcept
{
    for (auto* const instance : instances_)
    {
        if (instance != nullptr && &instance->handle_ == handle)
            return instance;
    }
    return nullptr;
}

/// 仅 bxCAN 双外设（CAN1/CAN2 共享滤波 bank）场景有意义：检查同共享域的
/// 另一个已注册实例是否已启动（started_ 或 starting_）。返回 true 表示共享
/// 滤波域已被占用，当前实例在启动前不得再写共享滤波寄存器。非该场景恒为 false。
bool Can::shared_filter_bus_started(const NativeHandle& handle) noexcept
{
#if !CAN_DRIVER_FDCAN && defined(CAN1) && defined(CAN2)
    if (handle.Instance != CAN1 && handle.Instance != CAN2)
        return false;
    for (auto* const instance : instances_)
    {
        if (instance != nullptr && &instance->handle_ != &handle &&
            (instance->handle_.Instance == CAN1 || instance->handle_.Instance == CAN2) &&
            (instance->started_ || instance->starting_))
            return true;
    }
#else
    (void)handle;
#endif
    return false;
}

Status Can::claim_instance() noexcept
{
    // 独占的是外设本身，而非仅 HAL 包装结构体的地址：同一 handle_ 地址或
    // 同一 handle_.Instance 都判为冲突，防止另一包装共用同一 CAN 外设。
    for (auto* const instance : instances_)
    {
        if (instance == this)
            return Status::Ok;
        if (instance != nullptr &&
            (&instance->handle_ == &handle_ || instance->handle_.Instance == handle_.Instance))
            return Status::HandleInUse;
    }
    // 未冲突：占用首个空槽；槽位耗尽返回 NoCapacity。
    for (auto& instance : instances_)
    {
        if (instance == nullptr)
        {
            instance = this;
            return Status::Ok;
        }
    }
    return Status::NoCapacity;
}

/// 清空指向本实例的注册表槽位（ISRGuard 已由调用方持有）。析构与 start()
/// 失败路径都会调用，使 find() 之后不再命中本对象。
void Can::release_instance() noexcept
{
    for (auto& instance : instances_)
    {
        if (instance == this)
        {
            instance = nullptr;
            return;
        }
    }
}

/// 在临界区外做前置检查：ISR 上下文、PRIMASK 屏蔽或 BASEPRI 屏蔽（Cortex-M3+）
/// 一律拒绝，因为随后的 HAL 启动依赖可运行的中断/超时。started_ 或 starting_
/// 已置位则拒绝重复启动。抢占注册表槽位后置 starting_，再在中断开放状态下
/// 执行 start_hardware()；成功则在临界区内置 started_ 并 enable_notifications()
/// （该调用不得阻塞）。任一步失败都回滚：rollback_start() 只会停止由本实例启动
/// 的硬件（hardware_started_），绝不误停其他持有者的外设，随后清 starting_
/// 并释放槽位。不初始化 GPIO/时序/消息 RAM/TDC。
Status Can::start() noexcept
{
    if (__get_IPSR() != 0 || __get_PRIMASK() != 0)
        return Status::InvalidState;
#if defined(__CORTEX_M) && (__CORTEX_M >= 3U)
    if (__get_BASEPRI() != 0)
        return Status::InvalidState;
#endif
    {
        ISRGuard guard;
        if (started_ || starting_)
            return Status::InvalidState;
        if (handle_.Instance == nullptr)
            return Status::InvalidConfiguration;
        const auto result = claim_instance();
        if (result != Status::Ok)
            return result;
        starting_ = true;
    }

    // HAL_Start/Stop 可能基于 tick 超时，绝不能跨这两个调用屏蔽中断（否则
    // 超时无法推进）。starting_ 在启动过程中冻结配置，避免并发 configure/
    // add_callback 改动硬件。
    auto result = start_hardware();
    if (result == Status::Ok)
    {
        ISRGuard guard;
        started_ = true;
        result   = enable_notifications();
        if (result == Status::Ok)
        {
            starting_ = false;
            return Status::Ok;
        }
        started_ = false;
    }

    rollback_start();
    {
        ISRGuard guard;
        starting_ = false;
        release_instance();
    }
    return result;
}

/// 仅允许启动前配置滤波。若句柄/外设已被其他 Can 占用则拒绝——包括同一 CAN
/// 外设但使用不同 HAL handle 的另一个包装实例。资源索引由 FilterConfig 显式给出。
Status Can::configure_filter(const FilterConfig& config) noexcept
{
    ISRGuard guard;
    if (starting_ || started_)
        return Status::InvalidState;
    if (handle_.Instance == nullptr)
        return Status::InvalidConfiguration;
    // 拒绝配置已被其他 Can 占用的句柄/外设，包括同一 CAN 外设、但使用
    // 不同 HAL handle 的另一个包装实例。
    for (auto* const instance : instances_)
    {
        if (instance != nullptr && instance != this &&
            instance->handle_.Instance == handle_.Instance)
            return Status::HandleInUse;
    }
    return configure_filter_hardware(config);
}

/// 追加一个回调条目；仅允许启动前注册（starting_/started_ 为假），否则返回
/// InvalidState，保证运行期回调表不变。表满返回 NoCapacity。
Status Can::add_callback(const Callback& callback) noexcept
{
    ISRGuard guard;
    if (starting_ || started_)
        return Status::InvalidState;
    if (callback_count_ == callbacks_.size())
        return Status::NoCapacity;
    callbacks_[callback_count_++] = callback;
    return Status::Ok;
}

/// 纯软件侧的帧合法性检查（不触硬件）：
/// - ID 按 id_type（Standard/Extended）判断范围与扩展位；
/// - 格式仅 Classic/Fd，类型仅 Data/Remote，非空 data 必须带有效指针；
/// - Classic：BRS/ESI 必须为假；Remote 不得携带数据且 remote_length ≤ 8；
///   Data 的 data.size() ≤ 8 且 remote_length == 0；
/// - FD：必须是 Data 帧、remote_length == 0，长度需为合法 FD 长度
///   （is_fd_length，不可任意取值）；本编译单元不支持 FD 时返回 Unsupported。
Status Can::validate_frame(const FrameView& frame) noexcept
{
    const auto& header = frame.header;
    if (!is_valid_id(header.id_type, header.id) ||
        (header.format != FrameFormat::Classic && header.format != FrameFormat::Fd) ||
        (header.type != FrameType::Data && header.type != FrameType::Remote) ||
        (!frame.data.empty() && frame.data.data() == nullptr))
        return Status::InvalidArgument;

    if (header.format == FrameFormat::Classic)
    {
        if (header.bit_rate_switch || header.error_state_indicator)
            return Status::InvalidArgument;
        if (header.type == FrameType::Remote)
        {
            if (!frame.data.empty() || header.remote_length > 8)
                return Status::InvalidArgument;
        }
        else if (frame.data.size() > 8 || header.remote_length != 0)
            return Status::InvalidArgument;
    }
    else
    {
        if (header.type != FrameType::Data || header.remote_length != 0 ||
            !is_fd_length(frame.data.size()))
            return Status::InvalidArgument;
        if constexpr (!SupportsFD)
            return Status::Unsupported;
    }
    return Status::Ok;
}

/// 把底层 Status 归类为面向调用者的 SendResult：Ok→Submitted、Busy→Busy、
/// Unsupported→Unsupported、各类参数/范围错误→InvalidFrame、BusOff→BusOff、
/// InvalidState→NotStarted，其余默认 HardwareError。
SendResult Can::send_result(const Status status) noexcept
{
    switch (status)
    {
    case Status::Ok:
        return SendResult::Submitted;
    case Status::Busy:
        return SendResult::Busy;
    case Status::Unsupported:
        return SendResult::Unsupported;
    case Status::InvalidArgument:
    case Status::InvalidFrame:
    case Status::OutOfRange:
        return SendResult::InvalidFrame;
    case Status::BusOff:
        return SendResult::BusOff;
    case Status::InvalidState:
        return SendResult::NotStarted;
    default:
        return SendResult::HardwareError;
    }
}

/// 由 FrameView 拷入存储：复制 header、记录 length、memcpy 负载，并按后端可读
/// 范围对尾部补零，作为发送队列与接收暂存的统一载体。
void Can::StoredFrame::assign(const FrameView& frame) noexcept
{
    header = frame.header;
    length = static_cast<std::uint8_t>(frame.data.size());
    if (length != 0)
        std::memcpy(data.data(), frame.data.data(), length);

    // bxCAN HAL 始终读取固定 8 字节；FDCAN HAL 按完整 32 位字读取，即使负载
    // 只有 1/2/3/5/6/7 字节也要读到该字末尾。故只清零 HAL 可能读到的尾部
    // （Remote 帧按 8 字节计，其余向上取整到 4 的倍数），不做多余 memset。
    const std::size_t readable = !supports_fd || header.type == FrameType::Remote
                                         ? 8U
                                         : (static_cast<std::size_t>(length) + 3U) &
                                                   ~std::size_t{ 3 };
    if (readable > length)
        std::memset(data.data() + length, 0, readable - length);
}

/// 在临界区内（调用方持 ISRGuard）排空发送队列：只有 write() 返回 Ok 才弹出
/// 队首，否则原样返回状态、保留该帧以便下次重试。每帧以 padded_storage=true 写出。
Status Can::flush_tx() noexcept
{
#if CAN_DRIVER_TX_QUEUE_SIZE > 0
    while (const auto* const pending = tx_queue_.front())
    {
        const auto frame  = pending->view();
        const auto result = write(frame, true);
        if (result != Status::Ok)
            return result;
        (void)tx_queue_.pop();
    }
#endif
    return Status::Ok;
}

/// 提交一帧的完整流程：先做软件校验并映射结果；在临界区内确认已启动；再做
/// 硬件相关校验；随后排空历史队列，队列为空时尝试直接写硬件，若硬件忙则改为
/// 入队。入队时若队列已满，覆盖最旧一项并返回 ReplacedOldest，否则返回 Queued。
/// Submitted/Queued/ReplacedOldest 均不保证最终发送成功。
SendResult Can::send(const FrameView& frame) noexcept
{
    auto result = validate_frame(frame);
    if (result != Status::Ok)
        return send_result(result);

    ISRGuard guard;
    if (!started_)
        return SendResult::NotStarted;
    result = validate_hardware_frame(frame);
    if (result != Status::Ok)
        return send_result(result);

#if CAN_DRIVER_TX_QUEUE_SIZE > 0
    result = flush_tx();
    if (result != Status::Ok && result != Status::Busy)
        return send_result(result);

    if (tx_queue_.empty())
    {
        result = write(frame, false);
        if (result != Status::Busy)
            return send_result(result);
    }

    const bool replaced = tx_queue_.full();
    (void)tx_queue_.push([&frame](StoredFrame& entry) noexcept { entry.assign(frame); });
    return replaced ? SendResult::ReplacedOldest : SendResult::Queued;
#else
    return send_result(write(frame, false));
#endif
}

/// 发送完成中断入口：仅在已启动时排空软件队列；硬件忙时本轮停止，等待下一次
/// TX 中断再继续。
void Can::on_tx_available() noexcept
{
    ISRGuard guard;
    if (started_)
        (void)flush_tx();
}

/// 接收中断入口。持续从指定 location（FIFO/缓冲区索引）读出帧直到无数据或出错，
/// 使一次中断尽量排空该队列。InvalidFrame 表示后端已确认并释放该不可用帧，继续
/// 排空而非返回。回调在本 ISR 上下文同步执行，storage 每轮复用，故 view 仅在
/// 本次回调调用期间有效；回调表启动后不变，无需额外全局屏蔽即可安全遍历。
void Can::on_receive(const std::uint32_t location) noexcept
{
    if (!started_)
        return;
    StoredFrame storage;
    for (;;)
    {
        const auto result = read(location, storage);
        if (result == Status::InvalidFrame)
            continue; // 后端已确认并释放该不可用帧，继续排空。
        if (result != Status::Ok)
            return;
        const auto frame = storage.view();
        // 回调表启动后不变；用户代码在此不额外加全局中断屏蔽，FIFO IRQ 优先级
        // 由板级配置决定。frame 仅在本轮迭代内有效，下一轮 read 会覆盖 storage。
        for (std::size_t index = 0; index < callback_count_; ++index)
            callbacks_[index].invoke(callbacks_[index], frame);
    }
}

} // namespace bsp::can

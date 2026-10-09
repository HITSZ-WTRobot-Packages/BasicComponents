/**
 * @file can_driver.hpp
 * @brief 固定存储的 CAN/FDCAN 包装：无堆分配、无 stop API、无 RX 队列。
 * 需要 C++20 与 HAL 回调注册支持。
 *
 * 使用顺序：先初始化 HAL 句柄，再配置过滤器/注册回调，最后 start()。HAL 的中断
 * 处理函数仍由板级代码负责。一个 Can 独占一个句柄的回调；不得再把旧驱动挂到
 * 同一句柄上。句柄、Can 对象以及被借用的回调对象必须比外设的使用期更长。
 * 析构只解除 C++ 实例的登记，不会停止外设；start() 之后不得再改动 HAL 句柄或
 * 其 Init 资源布局。
 *
 * 接收在 ISR 上下文按注册顺序同步广播。回调不得阻塞、抛异常、销毁本总线或
 * 保存 FrameView。start() 之后的注册/过滤变更会被拒绝，且没有注销（unregister）
 * 接口。send() 为非阻塞，单核 Cortex-M 上可由任务或 ISR 调用。只保留硬件提交
 * 顺序，不保留 CAN 仲裁顺序。接收帧若大于其配置的硬件 element 会被丢弃，
 * 绝不作为看似合法但被截断的负载交付。
 *
 * 过滤变更只影响指定的 entry/策略。FDCAN 复位默认可能接受未匹配帧：需要拒收时
 * 显式应用 GlobalFilter{}。所有 bxCAN 共享 bank 及其分界必须在任一条总线启动前
 * 配置。
 *
 * CAN_DRIVER_TX_QUEUE_SIZE 选择可用的软件 TX 容量（允许为 0）。
 * CAN_DRIVER_MAX_CALLBACKS 与 CAN_DRIVER_MAX_INSTANCES 是固定资源上限。所有使用
 * 本套件的翻译单元必须给出这些宏的一致定义。TX 容量在 FDCAN 上默认 0，在
 * bxCAN 上默认 8。
 */
#pragma once

#include "main.h"
#include "can_driver_types.hpp"

#include <array>
#include <functional>
#include <memory>
#include <type_traits>

#if defined(HAL_FDCAN_MODULE_ENABLED) && defined(HAL_CAN_MODULE_ENABLED)
#    error "CANDriver requires exactly one HAL CAN backend per firmware"
#elif defined(HAL_FDCAN_MODULE_ENABLED)
#    define CAN_DRIVER_FDCAN 1
#    if !defined(USE_HAL_FDCAN_REGISTER_CALLBACKS) || !USE_HAL_FDCAN_REGISTER_CALLBACKS
#        error "CANDriver requires USE_HAL_FDCAN_REGISTER_CALLBACKS"
#    endif
#elif defined(HAL_CAN_MODULE_ENABLED)
#    define CAN_DRIVER_FDCAN 0
#    if !defined(USE_HAL_CAN_REGISTER_CALLBACKS) || !USE_HAL_CAN_REGISTER_CALLBACKS
#        error "CANDriver requires USE_HAL_CAN_REGISTER_CALLBACKS"
#    endif
#else
#    error "CANDriver requires HAL_CAN_MODULE_ENABLED or HAL_FDCAN_MODULE_ENABLED"
#endif

#ifndef CAN_DRIVER_TX_QUEUE_SIZE
#    if CAN_DRIVER_FDCAN
#        define CAN_DRIVER_TX_QUEUE_SIZE 0
#    else
#        define CAN_DRIVER_TX_QUEUE_SIZE 8
#    endif
#endif
#ifndef CAN_DRIVER_MAX_CALLBACKS
#    define CAN_DRIVER_MAX_CALLBACKS 14
#endif
#ifndef CAN_DRIVER_MAX_INSTANCES
#    if defined(FDCAN3) || defined(CAN3)
#        define CAN_DRIVER_MAX_INSTANCES 3
#    elif defined(FDCAN2) || defined(CAN2)
#        define CAN_DRIVER_MAX_INSTANCES 2
#    else
#        define CAN_DRIVER_MAX_INSTANCES 1
#    endif
#endif

#if CAN_DRIVER_TX_QUEUE_SIZE < 0 || CAN_DRIVER_MAX_CALLBACKS < 1 || CAN_DRIVER_MAX_INSTANCES < 1
#    error "CANDriver has invalid resource limits"
#endif
#if CAN_DRIVER_TX_QUEUE_SIZE > 0
#    include "RingBuffer.hpp"
#endif

namespace bsp::can
{

/// 由编译期 HAL 后端选择决定的 HAL 句柄类型。
#if CAN_DRIVER_FDCAN
using NativeHandle = FDCAN_HandleTypeDef;
#else
using NativeHandle = CAN_HandleTypeDef;
#endif

/**
 * 固定存储的 CAN/FDCAN 包装。一个实例独占一个 HAL 句柄——实际上是该句柄指向
 * 的外设（同一外设、不同句柄也会被视为冲突）。无堆分配、无 stop API、无 RX
 * 队列；生命周期与回调约束见文件头说明。
 */
class Can final
{
public:
    static constexpr bool SupportsFD = CAN_DRIVER_FDCAN != 0; ///< 编译期后端是否支持 CAN FD。
    static constexpr std::size_t MaxDataLength   = SupportsFD ? 64 : 8;      ///< 单帧负载字节上限。
    static constexpr std::size_t TxQueueCapacity = CAN_DRIVER_TX_QUEUE_SIZE; ///< 软件 TX
                                                                             ///< 队列容量（可为
                                                                             ///< 0）。
    static constexpr std::size_t CallbackCapacity =
            CAN_DRIVER_MAX_CALLBACKS; ///< 可注册接收回调数上限。

    using ReceiveFunction = void (*)(const FrameView&); ///< 无状态函数指针回调类型。

    /// 仅保存句柄引用，不触碰硬件、不占用实例槽；句柄须已完成 HAL Init。
    explicit Can(NativeHandle& handle) noexcept : handle_(handle) {}
    /// 释放在 registry 中占用的实例槽；不停止外设、不注销 HAL 回调。
    ~Can();
    /// 独占外设与回调槽，禁止复制与移动。
    Can(const Can&)            = delete;
    Can& operator=(const Can&) = delete;
    Can(Can&&)                 = delete;
    Can& operator=(Can&&)      = delete;

    /**
     * 安装该句柄的 HAL 回调、启动硬件并使能所需的 IRQ 通知。不初始化 GPIO、
     * 时序、message RAM 或 TDC，这些须由 HAL 句柄的 Init 阶段完成。
     *
     * HAL 启动期间中断保持可用，以便基于 tick 的 HAL 超时能够推进，因此必须
     * 在线程模式且中断未屏蔽时调用（IPSR/PRIMASK/BASEPRI 非零时返回
     * InvalidState），绝不可在 ISR 中调用。启动进行中以 starting_ 冻结配置变更。
     *
     * 成功返回 Ok；句柄 Instance 为空返回 InvalidConfiguration；外设已被其他
     * 实例占用返回 HandleInUse；实例槽耗尽返回 NoCapacity；使能通知失败会回滚
     * 启动并释放槽位后返回对应错误。重复 start() 返回 InvalidState。
     */
    [[nodiscard]] Status start() noexcept;

    /**
     * 校验并提交一帧，或在硬件无空位时把它复制进软件队列。负载为借用，函数
     * 返回后调用方即可复用其缓冲区。软件队列满时丢弃最旧一项并返回
     * ReplacedOldest。
     *
     * Submitted/Queued/ReplacedOldest 仅表示帧已被受理，不保证真正发送成功，也
     * 不含 CAN 仲裁顺序信息。FD 数据长度必须精确可表示（见 is_fd_length），
     * 不做填充或分片。总线尚未 start() 时返回 NotStarted。单核 Cortex-M 上
     * 可在任务或 ISR 中调用。
     */
    [[nodiscard]] SendResult send(const FrameView& frame) noexcept;

    /// 配置一个过滤器 entry 或全局策略，仅可在 start() 之前调用。资源 index 由
    /// 配置显式给出并做边界检查；越界/无容量返回 OutOfRange/NoCapacity，后端
    /// 不支持的备选项返回 Unsupported，句柄为空返回 InvalidConfiguration，外设
    /// 已被其他实例占用返回 HandleInUse。
    [[nodiscard]] Status configure_filter(const FilterConfig& config) noexcept;

    /// 注册无状态函数指针回调；function 为 nullptr 返回 InvalidArgument。回调
    /// 在 ISR 上下文按注册顺序同步广播，不得阻塞、抛异常或销毁本总线。仅可在
    /// start() 之前注册，否则返回 InvalidState；表满返回 NoCapacity。
    [[nodiscard]] Status register_callback(const ReceiveFunction function) noexcept
    {
        if (function == nullptr)
            return Status::InvalidArgument;
        Callback callback{};
        callback.target.function = function;
        callback.invoke          = [](const Callback& entry, const FrameView& frame) noexcept
        { entry.target.function(frame); };
        return add_callback(callback);
    }

    /// 借用左值可调用对象；不接受临时捕获 lambda。回调在 ISR 上下文同步触发，
    /// 注册后该对象必须存活至本总线停止使用，且不得阻塞或抛异常。
    /// 仅接受可用 callable(frame) 直接调用的对象；成员函数指针使用另一重载。
    template <class Callable>
    requires(std::is_object_v<Callable>&& requires(Callable& object, const FrameView& frame) {
        object(frame);
    }) [[nodiscard]] Status register_callback(Callable& callable) noexcept
    {
        Callback callback{};
        callback.target.object = std::addressof(callable);
        callback.invoke        = [](const Callback& entry, const FrameView& frame) noexcept
        {
            // Callable 保持其原有的 const 限定；可变可调用对象在注册时是以
            // 非 const 左值传入的。
            auto* object = static_cast<Callable*>(const_cast<void*>(entry.target.object));
            (*object)(frame);
        };
        return add_callback(callback);
    }

    /// 成员函数回调：用法见 bus.register_callback<&Motor::on_can>(motor)。
    /// Method 在注册时即固定，要求签名 void(Object&, const FrameView&)；
    /// object 必须存活至本总线停止使用。
    template <auto Method, class Object>
    requires std::
            is_invocable_r_v<void, decltype(Method), Object&, const FrameView&> [[nodiscard]] Status
            register_callback(Object& object) noexcept
    {
        Callback callback{};
        callback.target.object = std::addressof(object);
        callback.invoke        = [](const Callback& entry, const FrameView& frame) noexcept
        {
            auto* receiver = static_cast<Object*>(const_cast<void*>(entry.target.object));
            std::invoke(Method, *receiver, frame);
        };
        return add_callback(callback);
    }

private:
    /// 单个回调条目：函数指针或借用对象，加统一调用入口。
    struct Callback
    {
        /// 目标要么是裸函数指针，要么是借用对象指针，二者互斥。
        union Target
        {
            const void*     object{};
            ReceiveFunction function;
        } target{};
        void (*invoke)(const Callback&, const FrameView&) noexcept {}; ///< 由对应注册重载填充。
    };

    /**
     * 软件队列中的一帧（仅 CAN_DRIVER_TX_QUEUE_SIZE > 0 时使用）。data 按
     * MaxDataLength 固定分配并 4 字节对齐，length 为有效字节数。assign() 会把
     * HAL 会读取的向上取整字尾部清零，使其可直接交给后端复制；view() 借用自身
     * 存储，须在本条目存活期内使用。
     */
    struct StoredFrame
    {
        FrameHeader  header{};
        std::uint8_t length{};
        alignas(std::uint32_t) std::array<std::uint8_t, MaxDataLength> data;

        void                    assign(const FrameView& frame) noexcept;
        [[nodiscard]] FrameView view() const noexcept
        {
            return { header, std::span<const std::uint8_t>{ data.data(), length } };
        }
    };

    NativeHandle&                          handle_;           ///< 借用的 HAL 句柄，须比本对象长寿。
    std::array<Callback, CallbackCapacity> callbacks_{};      ///< 定长回调表，start() 后不再改变。
    std::size_t                            callback_count_{}; ///< 已注册回调数，受 ISRGuard 保护。
    bool starting_{};         ///< start() 进行中：冻结配置并阻止重复启动。
    bool started_{};          ///< 已启动：回调与过滤器冻结，可发送/接收。
    bool hardware_started_{}; ///< 后端回滚时不得停止其他 owner 的句柄。
#if CAN_DRIVER_TX_QUEUE_SIZE > 0
    libs::RingBuffer<StoredFrame, CAN_DRIVER_TX_QUEUE_SIZE + 1, true>
            tx_queue_; ///< 软件 TX 队列（多一个 slot 以区分满/空）。
#endif

    static std::array<Can*, CAN_DRIVER_MAX_INSTANCES>
                              instances_; ///< 实例 registry，用于句柄/外设唯一性与反查。
    [[nodiscard]] static Can* find(NativeHandle* handle) noexcept; ///< 由 HAL 句柄反查所属实例。
    [[nodiscard]] static bool shared_filter_bus_started(
            const NativeHandle& handle) noexcept;   ///< 同一共享 filter 域的另一条总线是否已启动。
    [[nodiscard]] Status claim_instance() noexcept; ///< Caller holds ISRGuard. 调用方须持有
                                                    ///< ISRGuard。
    void release_instance() noexcept; ///< Caller holds ISRGuard. 调用方须持有 ISRGuard。
    [[nodiscard]] Status add_callback(
            const Callback& callback) noexcept; ///< 启动前入表；已启动返回
                                                ///< InvalidState，满返回
                                                ///< NoCapacity。
    [[nodiscard]] static Status validate_frame(
            const FrameView& frame) noexcept; ///< 后端无关的帧结构与长度校验。
    [[nodiscard]] static SendResult send_result(Status status) noexcept; ///< Status 到 SendResult
                                                                         ///< 的映射。
    [[nodiscard]] Status flush_tx() noexcept; ///< Caller holds ISRGuard. 尽可能清空并重放软件队列。
    void                 on_tx_available() noexcept;  ///< TX 空间释放时的 ISR 入口。
    void on_receive(std::uint32_t location) noexcept; ///< RX ISR 入口：循环读空 location 并广播。

    // 由恰好一个后端实现。start_hardware() 可能等待 HAL；enable_notifications()
    // 在中断屏蔽下调用，绝不等待。
    [[nodiscard]] Status start_hardware() noexcept;       ///< 后端：启动硬件并安装回调。
    [[nodiscard]] Status enable_notifications() noexcept; ///< 后端：使能 IRQ 通知（不得等待）。
    void                 rollback_start() noexcept; ///< 后端：使能通知失败时回滚已启动的硬件。
    [[nodiscard]] Status validate_hardware_frame(
            const FrameView& frame) const noexcept; ///< 后端能力校验（FD 支持、element 容量）。
    /**
     * 后端写帧。padded_storage 为真表示 frame 来自软件队列，其向上取整字尾部
     * 已由 StoredFrame::assign() 初始化，后端可跳过临时缓冲直接引用；来自
     * send() 的裸视图须自行补零后再提交。
     */
    [[nodiscard]] Status write(const FrameView& frame, bool padded_storage) noexcept;
    /**
     * 后端读帧。location 为 Rx FIFO 索引或 Rx Buffer 下标（含义随后端而定）。
     * Empty 表示该位置已无数据；InvalidFrame 表示帧已被消费并确认但不可交付，
     * 调用方应继续排空同一位置；其余非 Ok 视为硬件错误并终止。
     */
    [[nodiscard]] Status read(std::uint32_t location, StoredFrame& frame) noexcept;
    [[nodiscard]] Status configure_filter_hardware(
            const FilterConfig& config) noexcept; ///< 后端：应用过滤器配置。

#if CAN_DRIVER_FDCAN
    static void rx_fifo0_irq(NativeHandle* handle, std::uint32_t events); ///< HAL 回调桥，运行于
                                                                          ///< ISR 上下文。
    static void rx_fifo1_irq(NativeHandle* handle, std::uint32_t events);
    static void rx_buffers_irq(NativeHandle* handle);
    static void tx_buffers_irq(NativeHandle* handle, std::uint32_t buffers);
#else
    static void rx_fifo0_irq(NativeHandle* handle);
    static void rx_fifo1_irq(NativeHandle* handle);
#endif
    static void tx_irq(NativeHandle* handle); ///< TX 完成 IRQ，运行于 ISR 上下文。
};

} // namespace bsp::can

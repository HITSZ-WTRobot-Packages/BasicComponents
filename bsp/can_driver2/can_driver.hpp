/**
 * @file can_driver.hpp
 * @brief 固定存储的 CAN/FDCAN 包装：无堆分配、无 stop API、无 RX 队列。
 * 面向带 CAN/FDCAN 的 STM32 MCU HAL1，需要 C++20 与 HAL 回调注册支持。
 * 不支持 STM32MP 或 HAL2（包括 STM32C5）；不包含没有 CAN/FDCAN 外设的型号。
 *
 * bxCAN 后端承载 Classic CAN；FDCAN 后端承载 Classic/FD，并按当前 HAL 的
 * FDCAN_DLC_BYTES_* 宏解码接收长度，兼容原始与移位 DLC 编码。
 * bxCAN 目标系列：F0/F1/F2/F3/F4/F7/L4（仅 Classic CAN）。
 * 固定 message RAM FDCAN 系列：G4/G0/C0/L5/H5/U5/U3/H7RS，Rx FIFO0/1 与
 * Tx FIFO/Queue 固定各 3 个 64 字节负载 element。标准/扩展过滤器容量取 Init
 * 启用数量与硬件上限 28/8 的较小值；需要过滤器时，须在 HAL Init 前设置
 * StdFiltersNbr/ExtFiltersNbr。这些系列不支持专用 RxBuffer 或 calibration_message，
 * 请求该功能返回 Unsupported。传统 H7（不含 H7RS）与 N6 保留可配置 RAM 布局。
 *
 * 兼容性验证记录（HAL1 driver 版本，不是 STM32Cube 固件包版本）：
 * C = 真实 HAL/CMSIS 下交叉编译三个驱动翻译单元；S = 主机 HAL 边界模拟收发。
 * 系列       代表型号   HAL1 版本          验证层级
 * F0         F072       1.7.9              C
 * F1         F105       1.1.10             C
 * F2         F207       1.2.9              C
 * F3         F303xC     1.5.9              C
 * F4         F405       1.8.5              C
 * F7         F767       1.3.3              C
 * L4         L476       1.13.6             C
 * G4         G431       1.2.7              C + S
 * G0         G0B1       1.4.7              C + S
 * C0         C092       1.4.1              C + S
 * L5         L552       1.0.7              C + S
 * H5         H563       1.7.0              C + S
 * U5         U575       1.6.3              C + S
 * U3         U375       1.4.0              C + S
 * H7RS       H7S7       1.2.1              C + S
 * 传统 H7    H723       1.11.0 / 1.11.6    C + S（移位 / 原始 DLC）
 * N6         N657       1.4.0              C + S
 * 代表型号覆盖 TX 校验 0/1 × 软件队列容量 0/4；所用 CMSIS 中带 CAN/FDCAN 的
 * 160 个芯片宏另完成交叉编译（TX 校验 1、软件队列 0）。双核 H7 的 CM4/CM7
 * 均编译验证，板级须选择 CORE_CM4/CORE_CM7，并确保同一外设仅由一个核独占；
 * 驱动不提供跨核同步。上述证据不等于所有 HAL 历史版本或订货号的板上收发验证。
 * 未进行板上验证；时钟、GPIO、时序、IRQ 与外设核归属仍由板级负责。
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
 * 顺序，不保留 CAN 仲裁顺序。FDCAN 的所有已分配 Rx FIFO/Buffer element 须可
 * 容纳 64 字节负载；容量不足时 start() 返回 InvalidConfiguration，不启动硬件。
 *
 * 过滤器配置写入指定 entry/策略，并应用后端全局设置。FDCAN 默认可能接受未匹配帧，需要拒收时
 * 显式应用 GlobalFilter{}。所有 bxCAN 共享 bank 必须在两路 HAL 初始化完成后、
 * 任一条总线启动前配置；共享分界由 CAN_DRIVER_BXCAN_CAN2_START_BANK 统一指定，
 * 随每次过滤器配置写入 HAL，不再提供运行时分界配置。
 *
 * CAN_DRIVER_TX_QUEUE_SIZE 选择可用的软件 TX 容量（允许为 0）。
 * CAN_DRIVER_MAX_CALLBACKS 与 CAN_DRIVER_MAX_INSTANCES 是固定资源上限。所有使用
 * 本套件的翻译单元必须给出这些宏的一致定义。TX 容量在 FDCAN 上默认 0，在
 * bxCAN 上默认 8。
 *
 * CAN_DRIVER_VALIDATE_TX_FRAME 默认 0，表示发送路径不执行帧结构或后端帧能力
 * 校验；非零时启用。所有使用本套件的翻译单元必须给出该宏的一致定义。关闭时
 * 调用方必须自行保证提交的帧合法且适配已配置的后端，尤其不得超出 TX element
 * 容量；此时 send() 不保证给出 InvalidFrame/Unsupported 诊断。
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

#if !CAN_DRIVER_FDCAN
// CAN1/CAN2 共享过滤器组中首个归属 CAN2 的 bank，默认各分配 14 个 bank。
// 合法范围为 0..27；0 表示全部归 CAN2。CAN3 与单 CAN 的独立过滤器组不受影响。
// 通过驱动目标的编译定义统一覆盖，不能只在业务源文件中定义此宏。
#    ifndef CAN_DRIVER_BXCAN_CAN2_START_BANK
#        define CAN_DRIVER_BXCAN_CAN2_START_BANK 14U
#    endif
#    if CAN_DRIVER_BXCAN_CAN2_START_BANK < 0 || CAN_DRIVER_BXCAN_CAN2_START_BANK > 27
#        error "CAN_DRIVER_BXCAN_CAN2_START_BANK must be in [0, 27]"
#    endif
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

// 是否在发送路径执行运行时帧校验：0（默认）不执行帧结构或后端帧能力校验，也
// 不产生 InvalidFrame/Unsupported 诊断；非零启用。所有使用本套件的翻译单元必须
// 通过统一的编译定义给出相同取值。关闭时调用方必须自行保证帧合法且适配已配置
// 的后端（尤其 TX element 容量）。
#ifndef CAN_DRIVER_VALIDATE_TX_FRAME
#    define CAN_DRIVER_VALIDATE_TX_FRAME 0
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
     * 时序或 message RAM；TDC 可在 HAL Init 后由板级或配置 API 显式设置，start 不改写。
     *
     * HAL 启动期间中断保持可用，以便基于 tick 的 HAL 超时能够推进，因此必须
     * 在线程模式且中断未屏蔽时调用（IPSR/PRIMASK/BASEPRI 非零时返回
     * InvalidState），绝不可在 ISR 中调用。启动进行中以 starting_ 冻结配置变更。
     *
     * 成功返回 Ok；句柄 Instance 为空返回 InvalidConfiguration；外设已被其他
     * 实例占用返回 HandleInUse；实例槽耗尽返回 NoCapacity；使能通知失败会回滚
     * 本实例启动的硬件并释放槽位后返回对应错误。重复 start() 返回 InvalidState。
     * FDCAN 已分配的 Rx FIFO/Buffer element 负载容量不足 64 字节时返回 InvalidConfiguration，
     * 未分配的资源不检查 element 尺寸；板级须在 HAL Init 前配置足够容量。
     *
     * HAL Callback 安装不是事务：任何启动失败都不会恢复或注销已成功安装的
     * Callback。调用方须自行保证 start() 成功，不得假定失败后句柄可直接交给
     * 其他 owner 使用。
     */
    [[nodiscard]] Status start() noexcept;

    /**
     * 校验并提交一帧，或在硬件无空位时把它复制进软件队列。负载为借用，函数
     * 返回后调用方即可复用其缓冲区。软件队列满时丢弃最旧一项并返回
     * ReplacedOldest。
     *
     * 仅当 CAN_DRIVER_VALIDATE_TX_FRAME 非零时才执行帧结构与后端帧能力校验
     * （默认 0 不执行）。关闭时调用方必须自行保证帧合法且适配已配置的后端，
     * 尤其不得超出 TX element 容量；此时不会给出 InvalidFrame/Unsupported 诊断。
     *
     * Submitted/Queued/ReplacedOldest 仅表示帧已被受理，不保证真正发送成功，也
     * 不含 CAN 仲裁顺序信息。FD 数据长度必须精确可表示（见 is_fd_length），
     * 不做填充或分片。总线尚未 start() 时返回 NotStarted。单核 Cortex-M 上
     * 可在任务或 ISR 中调用。
     */
    SendResult send(const FrameView& frame) noexcept;

    /// 配置一个过滤器 entry 或全局策略，仅可在 start() 之前调用。资源 index 由
    /// 配置显式给出并做边界检查；越界/无容量返回 OutOfRange/NoCapacity，后端
    /// 不支持的备选项返回 Unsupported，句柄为空返回 InvalidConfiguration，外设
    /// 已被其他实例占用返回 HandleInUse。
    [[nodiscard]] Status configure_filter(const FilterConfig& config) noexcept;

    /**
     * 在 HAL Init 后、start() 前立即配置发送延迟补偿，不保存配置。FDCAN 必须为
     * READY；启用要求 FD_BRS 模式、DataPrescaler 为 1/2，offset/filter 为 0..127 mtq。
     * 禁用仅清除 TDC 使能位，忽略其他字段。不调用则保持已有板级配置；重新 HAL Init
     * 后须重配。启动中/已启动或非 READY 返回 InvalidState，外设被占用返回 HandleInUse，
     * 句柄/时序不合法返回 InvalidConfiguration，参数越界返回 OutOfRange，HAL 失败返回
     * HardwareError；bxCAN 返回 Unsupported。
     */
    [[nodiscard]] Status configure_tx_delay_compensation(
            const TxDelayCompensation& config) noexcept;

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

#if CAN_DRIVER_TX_QUEUE_SIZE > 0
    /**
     * TX 软件队列持有的一帧；不用于 RX。data 按 MaxDataLength 固定分配并
     * 4 字节对齐，length 为有效字节数。assign() 初始化 HAL 会读取的尾部；
     * view() 借用本条目的数据，须在该条目存活期内使用。
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
#endif

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
#if CAN_DRIVER_VALIDATE_TX_FRAME
    [[nodiscard]] static Status validate_frame(
            const FrameView& frame) noexcept; ///< 后端无关的帧结构与长度校验。
#endif
    [[nodiscard]] static SendResult send_result(Status status) noexcept; ///< Status 到 SendResult
                                                                         ///< 的映射。
    [[nodiscard]] Status flush_tx() noexcept; ///< Caller holds ISRGuard. 尽可能清空并重放软件队列。
    void                 on_tx_available() noexcept;  ///< TX 空间释放时的 ISR 入口。
    void on_receive(std::uint32_t location) noexcept; ///< RX ISR 入口：循环读空 location 并广播。

    // 由恰好一个后端实现。start_hardware() 可能等待 HAL；enable_notifications()
    // 在中断屏蔽下调用，绝不等待。
    [[nodiscard]] Status start_hardware() noexcept;       ///< 后端：启动硬件并安装回调。
    [[nodiscard]] Status enable_notifications() noexcept; ///< 后端：使能 IRQ 通知（不得等待）。
    void rollback_start() noexcept; ///< 只回滚本实例启动的硬件，不恢复或注销 HAL Callback。
#if CAN_DRIVER_VALIDATE_TX_FRAME
    [[nodiscard]] Status validate_hardware_frame(
            const FrameView& frame) const noexcept; ///< 后端能力校验（FD 支持、element 容量）。
#endif
    /**
     * 后端写帧。padded_storage 为真表示 frame 来自软件队列，其向上取整字尾部
     * 已由 StoredFrame::assign() 初始化，后端可跳过临时缓冲直接引用；来自
     * send() 的裸视图须自行补零后再提交。
     */
    [[nodiscard]] Status write(const FrameView& frame, bool padded_storage) noexcept;
    /**
     * 后端读帧。location 为 Rx FIFO 索引或 Rx Buffer 下标（含义随后端而定）。
     * buffer 须提供至少 MaxDataLength 字节的可写空间；后端直接填写 frame.header，
     * 并使 frame.data 借用 buffer 中的有效负载。仅返回 Ok 时 frame 有效，远程帧
     * 的数据视图为空；调用方不得在 buffer 被复用或销毁后继续持有视图。
     * Empty 表示无数据；InvalidFrame 表示帧已消费并确认但不可交付，须继续排空。
     */
    [[nodiscard]] Status read(std::uint32_t           location,
                              std::span<std::uint8_t> buffer,
                              FrameView&              frame) noexcept;
    [[nodiscard]] Status configure_filter_hardware(
            const FilterConfig& config) noexcept; ///< 后端：应用过滤器配置。
    [[nodiscard]] Status configure_tx_delay_compensation_hardware(
            const TxDelayCompensation& config) noexcept; ///< 后端：立即应用 TDC 配置，不得等待。

#if CAN_DRIVER_FDCAN
    static void rx_fifo0_irq(NativeHandle* handle, std::uint32_t events); ///< HAL 回调桥，运行于
                                                                          ///< ISR 上下文。
    static void rx_fifo1_irq(NativeHandle* handle, std::uint32_t events);
#    if defined(FDCAN_FILTER_TO_RXBUFFER)
    static void rx_buffers_irq(NativeHandle* handle);
#    endif
    static void tx_buffers_irq(NativeHandle* handle, std::uint32_t buffers);
#else
    static void rx_fifo0_irq(NativeHandle* handle);
    static void rx_fifo1_irq(NativeHandle* handle);
#endif
    static void tx_irq(NativeHandle* handle); ///< TX 完成 IRQ，运行于 ISR 上下文。
};

} // namespace bsp::can

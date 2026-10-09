/**
 * @file can_driver.hpp
 * @brief Fixed-storage CAN/FDCAN wrapper. No heap, stop API or RX queue.
 * Requires C++20 and HAL callback registration support.
 *
 * Initialize the HAL handle first, configure filters/register callbacks, then
 * start(). HAL IRQ handlers remain the board's responsibility. One Can owns
 * the callbacks of one handle; never attach the old driver to that same handle.
 * The handle, Can and borrowed callback objects must outlive peripheral use.
 * Destruction only detaches the C++ instance; it does not stop the peripheral.
 * Do not mutate the HAL handle or its Init resource layout after start().
 *
 * Reception is broadcast synchronously in ISR context, in registration order.
 * Callbacks must not block, throw, destroy this bus or retain a FrameView.
 * Registration/filter changes after start() are rejected; there is no unregister.
 * send() is nonblocking and callable from a task or ISR on a single Cortex-M
 * core. Only hardware submission order is preserved, not CAN arbitration order.
 * Receive frames larger than their configured hardware element are discarded,
 * never delivered as apparently valid truncated payloads.
 *
 * Filter changes affect only the specified entry/policy. FDCAN reset defaults
 * may accept unmatched frames: apply GlobalFilter{} explicitly to reject them.
 * Configure all bxCAN shared banks and their split before either bus starts.
 *
 * CAN_DRIVER_TX_QUEUE_SIZE selects the usable software TX capacity (including
 * zero). CAN_DRIVER_MAX_CALLBACKS and CAN_DRIVER_MAX_INSTANCES are fixed
 * resource limits. Define them consistently for all users of this package.
 * TX capacity defaults to zero on FDCAN and eight on bxCAN.
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

#if CAN_DRIVER_FDCAN
using NativeHandle = FDCAN_HandleTypeDef;
#else
using NativeHandle = CAN_HandleTypeDef;
#endif

class Can final
{
public:
    static constexpr bool        SupportsFD       = CAN_DRIVER_FDCAN != 0;
    static constexpr std::size_t MaxDataLength    = SupportsFD ? 64 : 8;
    static constexpr std::size_t TxQueueCapacity  = CAN_DRIVER_TX_QUEUE_SIZE;
    static constexpr std::size_t CallbackCapacity = CAN_DRIVER_MAX_CALLBACKS;

    using ReceiveFunction = void (*)(const FrameView&);

    explicit Can(NativeHandle& handle) noexcept : handle_(handle) {}
    ~Can();
    Can(const Can&)            = delete;
    Can& operator=(const Can&) = delete;
    Can(Can&&)                 = delete;
    Can& operator=(Can&&)      = delete;

    /**
     * Install this handle's callbacks, start hardware and enable required IRQ
     * notifications. Does not initialize GPIO, timing, message RAM or TDC.
     * HAL startup runs with interrupts available so HAL timeouts can advance.
     * Call from thread mode with interrupts unmasked, never from an ISR.
     */
    [[nodiscard]] Status start() noexcept;

    /**
     * Validate and submit a frame, or copy it into the software queue. A full
     * software queue replaces its oldest entry and returns ReplacedOldest.
     * Submitted/Queued/ReplacedOldest do not promise successful transmission.
     * FD data lengths must be exactly representable; no padding or fragmentation.
     */
    [[nodiscard]] SendResult send(const FrameView& frame) noexcept;

    /// Only before start(). Resource indices are explicit and bounds checked.
    [[nodiscard]] Status configure_filter(const FilterConfig& config) noexcept;

    [[nodiscard]] Status register_callback(ReceiveFunction function) noexcept;

    /// Borrow an lvalue callable; temporary capturing lambdas are not accepted.
    template <class Callable>
    requires(std::is_object_v<Callable>&& std::is_invocable_r_v<void, Callable&, const FrameView&>)
            [[nodiscard]] Status register_callback(Callable& callable) noexcept
    {
        Callback callback{};
        callback.target.object = std::addressof(callable);
        callback.invoke        = [](const Callback& entry, const FrameView& frame) noexcept
        {
            // Callable retains its original const qualification. Mutable
            // callables were supplied as non-const lvalues at registration.
            auto* object = static_cast<Callable*>(const_cast<void*>(entry.target.object));
            std::invoke(*object, frame);
        };
        return add_callback(callback);
    }

    /// Example: bus.register_callback<&Motor::on_can>(motor).
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
    struct Callback
    {
        union Target
        {
            const void*     object{};
            ReceiveFunction function;
        } target{};
        void (*invoke)(const Callback&, const FrameView&) noexcept {};
    };

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

    NativeHandle&                          handle_;
    std::array<Callback, CallbackCapacity> callbacks_{};
    std::size_t                            callback_count_{};
    bool                                   starting_{};
    bool                                   started_{};
    bool hardware_started_{}; ///< Backend rollback must never stop another owner's handle.
#if CAN_DRIVER_TX_QUEUE_SIZE > 0
    libs::RingBuffer<StoredFrame, CAN_DRIVER_TX_QUEUE_SIZE + 1, true> tx_queue_;
#endif

    static std::array<Can*, CAN_DRIVER_MAX_INSTANCES> instances_;
    [[nodiscard]] static Can*                         find(NativeHandle* handle) noexcept;
    [[nodiscard]] static bool       shared_filter_bus_started(const NativeHandle& handle) noexcept;
    [[nodiscard]] Status            claim_instance() noexcept;   ///< Caller holds ISRGuard.
    void                            release_instance() noexcept; ///< Caller holds ISRGuard.
    [[nodiscard]] Status            add_callback(const Callback& callback) noexcept;
    [[nodiscard]] static Status     validate_frame(const FrameView& frame) noexcept;
    [[nodiscard]] static SendResult send_result(Status status) noexcept;
    [[nodiscard]] Status            flush_tx() noexcept; ///< Caller holds ISRGuard.
    void                            on_tx_available() noexcept;
    void                            on_receive(std::uint32_t location) noexcept;

    // Implemented by exactly one backend. start_hardware() may wait on HAL;
    // enable_notifications() is called with interrupts masked and must not wait.
    [[nodiscard]] Status start_hardware() noexcept;
    [[nodiscard]] Status enable_notifications() noexcept;
    void                 rollback_start() noexcept;
    [[nodiscard]] Status validate_hardware_frame(const FrameView& frame) const noexcept;
    [[nodiscard]] Status write(const FrameView& frame, bool padded_storage) noexcept;
    [[nodiscard]] Status read(std::uint32_t location, StoredFrame& frame) noexcept;
    [[nodiscard]] Status configure_filter_hardware(const FilterConfig& config) noexcept;

#if CAN_DRIVER_FDCAN
    static void rx_fifo0_irq(NativeHandle* handle, std::uint32_t events);
    static void rx_fifo1_irq(NativeHandle* handle, std::uint32_t events);
    static void rx_buffers_irq(NativeHandle* handle);
    static void tx_buffers_irq(NativeHandle* handle, std::uint32_t buffers);
#else
    static void rx_fifo0_irq(NativeHandle* handle);
    static void rx_fifo1_irq(NativeHandle* handle);
#endif
    static void tx_irq(NativeHandle* handle);
};

} // namespace bsp::can

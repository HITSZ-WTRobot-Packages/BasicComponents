/** @file can_driver.cpp @brief Shared instance, callback and TX queue handling. */
#include "can_driver.hpp"
#include "isr_lock.h"

#include <cstring>

namespace bsp::can
{

constinit std::array<Can*, CAN_DRIVER_MAX_INSTANCES> Can::instances_{};

Can::~Can()
{
    ISRGuard guard;
    release_instance();
}

Can* Can::find(NativeHandle* const handle) noexcept
{
    for (auto* const instance : instances_)
    {
        if (instance != nullptr && &instance->handle_ == handle)
            return instance;
    }
    return nullptr;
}

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
    // The peripheral, not just the address of the HAL wrapper, is exclusive.
    for (auto* const instance : instances_)
    {
        if (instance == this)
            return Status::Ok;
        if (instance != nullptr &&
            (&instance->handle_ == &handle_ || instance->handle_.Instance == handle_.Instance))
            return Status::HandleInUse;
    }
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

    // HAL_Start/Stop may use tick-based timeouts: never mask interrupts across
    // those calls. starting_ freezes configuration while startup is in flight.
    auto result = start_hardware();
    if (result == Status::Ok)
    {
        ISRGuard guard;
        started_ = true;
        result = enable_notifications();
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

Status Can::configure_filter(const FilterConfig& config) noexcept
{
    ISRGuard guard;
    if (starting_ || started_)
        return Status::InvalidState;
    if (handle_.Instance == nullptr)
        return Status::InvalidConfiguration;
    // Reject configuring a handle/peripheral already claimed by another Can,
    // including another wrapper using a different HAL handle for the same CAN.
    for (auto* const instance : instances_)
    {
        if (instance != nullptr && instance != this &&
            instance->handle_.Instance == handle_.Instance)
            return Status::HandleInUse;
    }
    return configure_filter_hardware(config);
}

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

Status Can::register_callback(const ReceiveFunction function) noexcept
{
    if (function == nullptr)
        return Status::InvalidArgument;
    Callback callback{};
    callback.target.function = function;
    callback.invoke = [](const Callback& entry, const FrameView& frame) noexcept
    {
        entry.target.function(frame);
    };
    return add_callback(callback);
}

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
        if constexpr (!supports_fd)
            return Status::Unsupported;
    }
    return Status::Ok;
}

SendResult Can::send_result(const Status status) noexcept
{
    switch (status)
    {
    case Status::Ok: return SendResult::Submitted;
    case Status::Busy: return SendResult::Busy;
    case Status::Unsupported: return SendResult::Unsupported;
    case Status::InvalidArgument:
    case Status::InvalidFrame:
    case Status::OutOfRange: return SendResult::InvalidFrame;
    case Status::BusOff: return SendResult::BusOff;
    case Status::InvalidState: return SendResult::NotStarted;
    default: return SendResult::HardwareError;
    }
}

void Can::StoredFrame::assign(const FrameView& frame) noexcept
{
    header = frame.header;
    length = static_cast<std::uint8_t>(frame.data.size());
    if (length != 0)
        std::memcpy(data.data(), frame.data.data(), length);

    // bxCAN HAL reads all eight bytes. FDCAN HAL reads complete words, even
    // for a 1/2/3/5/6/7-byte payload. Initialize only the needed readable tail.
    const std::size_t readable = !supports_fd || header.type == FrameType::Remote
                                     ? 8U
                                     : (static_cast<std::size_t>(length) + 3U) & ~std::size_t{3};
    if (readable > length)
        std::memset(data.data() + length, 0, readable - length);
}

Status Can::flush_tx() noexcept
{
#if CAN_DRIVER_TX_QUEUE_SIZE > 0
    while (const auto* const pending = tx_queue_.front())
    {
        const auto frame = pending->view();
        const auto result = write(frame, true);
        if (result != Status::Ok)
            return result;
        (void)tx_queue_.pop();
    }
#endif
    return Status::Ok;
}

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

void Can::on_tx_available() noexcept
{
    ISRGuard guard;
    if (started_)
        (void)flush_tx();
}

void Can::on_receive(const std::uint32_t location) noexcept
{
    if (!started_)
        return;
    StoredFrame storage;
    for (;;)
    {
        const auto result = read(location, storage);
        if (result == Status::InvalidFrame)
            continue; // Backend already acknowledged the unusable frame.
        if (result != Status::Ok)
            return;
        const auto frame = storage.view();
        // Immutable once started; user code runs without an additional global
        // interrupt mask. FIFO0/FIFO1 IRQ priorities remain board configuration.
        for (std::size_t index = 0; index < callback_count_; ++index)
            callbacks_[index].invoke(callbacks_[index], frame);
    }
}

} // namespace bsp::can

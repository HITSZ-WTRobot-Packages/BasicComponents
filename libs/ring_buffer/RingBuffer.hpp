/**
 * @file    RingBuffer.hpp
 * @author  syhanjin
 * @date    2026-02-24
 * @brief   a ring buffer
 */
#pragma once
namespace libs
{

#pragma once
#include <cstddef>
#include <cstdint>

template <typename T, std::size_t Capacity> class RingBuffer
{
    static_assert(Capacity >= 2, "capacity must be >= 2");

public:
    RingBuffer() = default;

    // push element into buffer
    // return false if buffer full
    bool push(const T& value) noexcept;

    // pop element from buffer
    // return false if buffer empty
    bool pop(T& out) noexcept;

    // utilities
    bool empty() const noexcept;
    bool full() const noexcept;

    std::size_t           size() const noexcept;
    constexpr std::size_t capacity() const noexcept
    {
        return Capacity - 1;
    }

private:
    alignas(T) T buffer_[Capacity];

    volatile std::size_t head_{ 0 };
    volatile std::size_t tail_{ 0 };

private:
    static constexpr std::size_t next(std::size_t idx) noexcept
    {
        return (idx + 1) % Capacity;
    }
};
template <typename T, std::size_t Capacity>
bool RingBuffer<T, Capacity>::push(const T& value) noexcept
{
    const std::size_t next_head = next(head_);

    // buffer full
    if (next_head == tail_)
        return false;

    buffer_[head_] = value;

    // publish after write
    head_ = next_head;
    return true;
}

template <typename T, std::size_t Capacity> bool RingBuffer<T, Capacity>::pop(T& out) noexcept
{
    // buffer empty
    if (head_ == tail_)
        return false;

    out = buffer_[tail_];

    tail_ = next(tail_);
    return true;
}

template <typename T, std::size_t Capacity> bool RingBuffer<T, Capacity>::empty() const noexcept
{
    return head_ == tail_;
}

template <typename T, std::size_t Capacity> bool RingBuffer<T, Capacity>::full() const noexcept
{
    return next(head_) == tail_;
}

template <typename T, std::size_t Capacity>
std::size_t RingBuffer<T, Capacity>::size() const noexcept
{
    if (head_ >= tail_)
        return head_ - tail_;
    return Capacity - (tail_ - head_);
}

} // namespace libs

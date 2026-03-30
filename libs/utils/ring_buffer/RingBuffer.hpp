/**
 * @file    RingBuffer.hpp
 * @author  syhanjin
 * @date    2026-02-24
 * @brief   a ring buffer
 */
#pragma once

#include <cstddef>
#include <type_traits>
namespace libs
{

template <typename T, std::size_t Capacity, bool Overwrite = false> class RingBuffer
{
    static_assert(Capacity >= 2, "capacity must be >= 2");

public:
    RingBuffer() = default;

    // push element into buffer
    bool push(const T& value) noexcept
    {
        const std::size_t next_head = next(head_);

        if (next_head == tail_)
        {
            if constexpr (Overwrite)
            {
                // overwrite oldest
                tail_ = next(tail_);
            }
            else
            {
                return false;
            }
        }

        buffer_[head_] = value;

        // publish after write
        head_ = next_head;
        return true;
    }

    // push element with builder
    template <typename Builder,
              typename = std::enable_if_t<std::is_same_v<void, std::invoke_result_t<Builder, T&>>>>
    bool push(Builder&& builder) noexcept
    {
        const std::size_t next_head = next(head_);
        if (next_head == tail_)
        {
            if constexpr (Overwrite)
                tail_ = next(tail_);
            else
                return false;
        }
        builder(buffer_[head_]);
        head_ = next_head;
        return true;
    }

    T* emplace() noexcept
    {
        const std::size_t next_head = next(head_);
        if (next_head == tail_)
        {
            if constexpr (Overwrite)
                tail_ = next(tail_);
            else
                return nullptr;
        }

        T* out = &buffer_[head_];

        head_ = next_head;
        return out;
    }

    // pop element from buffer
    // return false if buffer empty
    bool pop(T& out) noexcept
    {
        // buffer empty
        if (head_ == tail_)
            return false;

        out = buffer_[tail_];

        tail_ = next(tail_);
        return true;
    }

    // pop element ptr from buffer
    // return nullptr if buffer empty
    T* pop() noexcept
    {
        if (head_ == tail_)
            return nullptr;
        T* out = &buffer_[tail_];
        tail_  = next(tail_);
        return out;
    }

    // utilities
    [[nodiscard]] bool empty() const noexcept { return head_ == tail_; }
    [[nodiscard]] bool full() const noexcept { return next(head_) == tail_; }

    [[nodiscard]] std::size_t size() const noexcept
    {
        if (head_ >= tail_)
            return head_ - tail_;
        return Capacity - (tail_ - head_);
    }

    static constexpr std::size_t capacity() noexcept { return Capacity - 1; }

private:
    alignas(T) T buffer_[Capacity];

    volatile std::size_t head_{ 0 };
    volatile std::size_t tail_{ 0 };

private:
    static constexpr std::size_t next(const std::size_t idx) noexcept
    {
        return (idx + 1) % Capacity;
    }
};

} // namespace libs

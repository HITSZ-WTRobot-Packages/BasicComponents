/**
 * @file    RingBuffer.hpp
 * @author  syhanjin
 * @date    2026-02-24
 * @brief   固定容量环形缓冲区。
 *
 * 这个容器适合生产者和消费者之间做小规模缓存，尤其是在中断和任务之间传递短消息时。
 * 容量设计上会保留一个空槽位来区分“满”和“空”；如果 Overwrite=true，缓冲区满时会自动丢弃最旧元素。
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

    /**
     * @brief 直接把一个元素拷贝入队。
     *
     * @return true 表示入队成功；如果缓冲区已满且不允许覆盖，则返回 false。
     */
    bool push(const T& value) noexcept
    {
        const std::size_t next_head = next(head_);

        if (next_head == tail_)
        {
            if constexpr (Overwrite)
            {
                // 队列满时丢弃最旧数据，让新数据进入缓存。
                tail_ = next(tail_);
            }
            else
            {
                return false;
            }
        }

        buffer_[head_] = value;

        // 先写数据，再移动 head，对外可见时元素已经就绪。
        head_ = next_head;
        return true;
    }

    /**
     * @brief 使用 builder 原地构造元素，避免额外拷贝。
     *
     * builder 会直接拿到队头位置的引用，适合构造较大的对象或者需要分步骤填充对象的场景。
     */
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
        // 和 push(builder) 类似，但只把空槽位指针返回给调用者。
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

    /**
     * @brief 弹出一个元素到 out。
     *
     * 空队列时返回 false。这个接口适合调用方需要保留一份拷贝的情况。
     */
    bool pop(T& out) noexcept
    {
        if (head_ == tail_)
            return false;

        out = buffer_[tail_];

        tail_ = next(tail_);
        return true;
    }

    /**
     * @brief 返回队头元素指针并前移读指针。
     *
     * 调用方拿到的是缓冲区内部元素的直接地址，因此应该尽快消费，不要长期持有。
     */
    T* pop() noexcept
    {
        if (head_ == tail_)
            return nullptr;
        T* out = &buffer_[tail_];
        tail_  = next(tail_);
        return out;
    }

    /**
     * @brief 清空缓冲区中的所有元素。
     *
     * 这个操作只会复位读写指针，不会改写底层存储。
     */
    void clear() noexcept
    {
        head_ = 0;
        tail_ = 0;
    }

    /**
     * @brief 查询队列状态。
     */
    [[nodiscard]] bool empty() const noexcept { return head_ == tail_; }
    [[nodiscard]] bool full() const noexcept { return next(head_) == tail_; }

    [[nodiscard]] std::size_t size() const noexcept
    {
        if (head_ >= tail_)
            return head_ - tail_;
        return Capacity - (tail_ - head_);
    }

    /**
     * @brief 返回实际可存储的元素个数。
     *
     * 由于需要留出一个空槽位，所以有效容量比模板参数少 1。
     */
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

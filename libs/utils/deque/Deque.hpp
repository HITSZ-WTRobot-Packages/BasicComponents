/**
 * @file    Deque.hpp
 * @author  syhanjin
 * @date    2026-03-13
 * @brief   固定容量双端队列。
 *
 * 这个容器不做动态扩容，适合缓存固定长度数据、滑动窗口或者最近 N 次采样值。
 * 它的实现目标是简单、可预测，而不是像标准库 deque 那样覆盖所有复杂用例。
 */
#pragma once

#include <cstddef>

template <typename T, std::size_t N> class Deque
{
    static_assert(N > 0, "deque capacity must be > 0");

public:
    constexpr Deque() = default;

    /* capacity */
    static constexpr std::size_t        capacity() noexcept { return N; }
    [[nodiscard]] constexpr std::size_t size() const noexcept { return size_; }
    [[nodiscard]] constexpr bool        empty() const noexcept { return size_ == 0; }
    [[nodiscard]] constexpr bool        full() const noexcept { return size_ == N; }

    /* element access */
    constexpr T&       front() noexcept { return data_[head_]; }
    constexpr const T& front() const noexcept { return data_[head_]; }

    constexpr T& back() noexcept { return data_[index(size_ - 1)]; }

    constexpr const T& back() const noexcept { return data_[index(size_ - 1)]; }

    constexpr T& operator[](std::size_t i) noexcept { return data_[index(i)]; }

    constexpr const T& operator[](std::size_t i) const noexcept { return data_[index(i)]; }

    constexpr T& operator[](int i) noexcept { return data_[logical_index(i)]; }

    constexpr const T& operator[](int i) const noexcept { return data_[logical_index(i)]; }

    constexpr T& at(int i) noexcept { return data_[logical_index(i)]; }

    constexpr const T& at(int i) const noexcept { return data_[logical_index(i)]; }

    /* modifiers */
    constexpr void clear() noexcept
    {
        head_ = 0;
        size_ = 0;
    }

    constexpr void push_back(const T& v) noexcept
    {
        if (full())
        {
            // 满时覆盖最旧元素，保持队列长度不变。
            data_[index(size_)] = v;
            head_               = (head_ + 1) % N; // overwrite oldest
        }
        else
        {
            data_[index(size_)] = v;
            ++size_;
        }
    }

    constexpr void push_front(const T& v) noexcept
    {
        // 先把头指针往前挪，再写新值。
        head_ = (head_ + N - 1) % N;

        if (full())
        {
            data_[head_] = v; // overwrite newest
        }
        else
        {
            data_[head_] = v;
            ++size_;
        }
    }

    constexpr void pop_back() noexcept
    {
        if (size_ > 0)
            --size_;
    }

    constexpr void pop_front() noexcept
    {
        if (size_ > 0)
        {
            head_ = (head_ + 1) % N;
            --size_;
        }
    }

private:
    [[nodiscard]] constexpr std::size_t index(std::size_t i) const noexcept
    {
        // 物理索引 = 头部偏移 + 逻辑偏移。
        return (head_ + i) % N;
    }
    [[nodiscard]] constexpr std::size_t logical_index(int i) const noexcept
    {
        // assume: -static_cast<int>(size_) <= i < static_cast<int>(size_)
        if (i >= 0)
        {
            return (head_ + static_cast<std::size_t>(i)) % N;
        }
        else
        {
            // i = -1 -> last
            return (head_ + size_ + static_cast<std::size_t>(i)) % N;
        }
    }

private:
    T           data_[N]{};
    std::size_t head_{ 0 };
    std::size_t size_{ 0 };

private:
    template <typename Deque> class DequeIterator
    {
    public:
        constexpr DequeIterator(Deque* dq, int idx) noexcept : dq_(dq), idx_(idx) {}

        constexpr T& operator*() const noexcept { return (*dq_)[idx_]; }

        constexpr DequeIterator& operator++() noexcept
        {
            ++idx_;
            return *this;
        }

        constexpr bool operator!=(const DequeIterator& other) const noexcept
        {
            return idx_ != other.idx_;
        }

    private:
        Deque* dq_;
        int    idx_;
    };

    template <typename Deque> class DequeRange
    {
    public:
        using iterator = DequeIterator<Deque>;

        constexpr DequeRange(Deque* dq, int begin, int end) noexcept :
            dq_(dq), begin_(begin), end_(end)
        {
        }

        constexpr iterator begin() const noexcept { return iterator(dq_, begin_); }

        constexpr iterator end() const noexcept { return iterator(dq_, end_); }

    private:
        Deque* dq_;
        int    begin_;
        int    end_;
    };

public:
    /**
     * @brief 生成半开区间 [a, b) 的迭代范围。
     *
     * 这只是一个轻量遍历辅助，不会复制数据。
     */
    constexpr auto range(int a, int b) noexcept { return DequeRange<Deque>(this, a, b); }

    constexpr auto range(int a, int b) const noexcept
    {
        return DequeRange<const Deque>(this, a, b);
    }
};

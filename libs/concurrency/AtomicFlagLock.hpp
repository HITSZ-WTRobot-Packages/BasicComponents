/**
 * @file    AtomicFlagLock.hpp
 * @author  syhanjin
 * @date    2026-03-25
 * @brief   基于原子变量的轻量状态锁。
 *
 * 这里不是传统互斥锁，而是给“是否已占用”这类状态提供一个无额外依赖的原子标记。
 * 适合表示单一资源是否处于占用状态，或者某个模块是否已经进入工作区间。
 */
#pragma once
#include <atomic>

class AtomicFlagLock
{
public:
    /**
     * @brief 尝试标记为 locked。
     *
     * 这个类并不提供阻塞等待，只负责以非阻塞方式尝试把状态改成“占用中”。
     * 只有从 unlocked 成功切换到 locked 时才返回 true；已经 locked 时返回 false，
     * 且调用方不得在失败后调用 unlock()。
     */
    [[nodiscard]] bool lock() noexcept
    {
        bool expected = false;
        return flag_.compare_exchange_strong(expected,
                                             true,
                                             std::memory_order_acquire,
                                             std::memory_order_relaxed);
    }

    /**
     * @brief 标记为 unlocked。
     */
    void unlock() noexcept { flag_.store(false, std::memory_order_release); }

    /**
     * @brief 查询当前是否处于 locked 状态。
     */
    [[nodiscard]] bool is_locked() const noexcept { return flag_.load(std::memory_order_acquire); }

private:
    std::atomic_bool flag_{ false };
};

class AtomicFlagGuard
{
public:
    /**
     * @brief RAII 封装：构造时尝试加锁，析构时仅释放自己成功获取的锁。
     */
    explicit AtomicFlagGuard(AtomicFlagLock& lock) noexcept : lock_(lock), locked_(lock_.lock()) {}

    ~AtomicFlagGuard()
    {
        if (locked_)
            lock_.unlock();
    }

    /**
     * @brief 判断当前 Guard 是否成功获取了锁。
     */
    [[nodiscard]] explicit operator bool() const noexcept { return locked_; }

    AtomicFlagGuard(const AtomicFlagGuard&)            = delete;
    AtomicFlagGuard& operator=(const AtomicFlagGuard&) = delete;

private:
    AtomicFlagLock& lock_;

    bool locked_;
};

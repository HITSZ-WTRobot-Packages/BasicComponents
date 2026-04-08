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
     * @brief 标记为 locked。
     *
     * 这个类并不提供阻塞等待，只负责把状态改成“占用中”。
     */
    void lock() noexcept { flag_.store(true, std::memory_order_release); }

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
     * @brief RAII 封装：构造时加锁，析构时释放。
     */
    explicit AtomicFlagGuard(AtomicFlagLock& lock) noexcept : lock_(lock) { lock_.lock(); }

    ~AtomicFlagGuard() { lock_.unlock(); }

    AtomicFlagGuard(const AtomicFlagGuard&)            = delete;
    AtomicFlagGuard& operator=(const AtomicFlagGuard&) = delete;

private:
    AtomicFlagLock& lock_;
};

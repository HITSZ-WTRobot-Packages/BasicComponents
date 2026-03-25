/**
 * @file    AtomicFlagLock.hpp
 * @author  syhanjin
 * @date    2026-03-25
 */
#pragma once
#include <atomic>

class AtomicFlagLock
{
public:
    void lock() noexcept { flag_.store(true, std::memory_order_release); }

    void unlock() noexcept { flag_.store(false, std::memory_order_release); }

    [[nodiscard]] bool is_locked() const noexcept { return flag_.load(std::memory_order_acquire); }

private:
    std::atomic_bool flag_{ false };
};

class AtomicFlagGuard
{
public:
    explicit AtomicFlagGuard(AtomicFlagLock& lock) noexcept : lock_(lock) { lock_.lock(); }

    ~AtomicFlagGuard() { lock_.unlock(); }

    AtomicFlagGuard(const AtomicFlagGuard&)            = delete;
    AtomicFlagGuard& operator=(const AtomicFlagGuard&) = delete;

private:
    AtomicFlagLock& lock_;
};
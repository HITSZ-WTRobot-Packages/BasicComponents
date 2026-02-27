/**
 * @file    watchdog.cpp
 * @author  syhanjin
 * @date    2026-01-30
 * @brief   Brief description of the file
 *
 * Detailed description (optional).
 *
 */
#include "watchdog.hpp"

namespace service
{

static Watchdog*          watchdogs[MAX_WATCHDOG_NUM];
static std::atomic_size_t watchdog_count = 0;

Watchdog::Watchdog()
{
    const size_t count_now = watchdog_count.fetch_add(1, std::memory_order_relaxed);
    if (count_now < MAX_WATCHDOG_NUM)
    {
        watchdogs[count_now] = this;
        id_                  = count_now;
    }
    else
    {
        watchdog_count.fetch_sub(1, std::memory_order_relaxed);
    }
}

Watchdog::~Watchdog()
{
    // watchdog should not be destructed
}

void Watchdog::feed()
{
    snacks_.store(WATCHDOG_SNACKS, std::memory_order_relaxed);
}

void Watchdog::eat()
{
    snacks_.fetch_sub(1, std::memory_order_relaxed);
}

bool Watchdog::isFed() const
{
    return snacks_.load(std::memory_order_relaxed) > 0;
}

void Watchdog::EatAll()
{
    for (size_t i = 0; i < watchdog_count.load(std::memory_order_relaxed); ++i)
        watchdogs[i]->eat();
}

bool Watchdog::isFull()
{
    return watchdog_count.load(std::memory_order_relaxed) == MAX_WATCHDOG_NUM;
}

} // namespace service
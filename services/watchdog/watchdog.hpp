/**
 * @file    watchdog.hpp
 * @author  syhanjin
 * @date    2026-01-30
 * @brief   watchdog service
 */
#ifndef WATCHDOG_HPP
#define WATCHDOG_HPP
#include <atomic>
#include <cstdint>
#include <cstddef>

namespace service
{

#ifndef MAX_WATCHDOG_NUM
#    define MAX_WATCHDOG_NUM (64)
#endif

#define WATCHDOG_SNACKS (10)

class Watchdog
{
public:
    Watchdog();
    ~Watchdog();
    void feed();
    void eat();

    [[nodiscard]] bool isFed() const;

    Watchdog(Watchdog&)  = delete;
    Watchdog(Watchdog&&) = delete;

    static void EatAll();
    static bool isFull();

private:
    std::atomic_int32_t snacks_{ 0 };
    uint32_t            id_;
};

} // namespace service

#endif // WATCHDOG_HPP

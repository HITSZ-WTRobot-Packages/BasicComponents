/**
 * @file    watchdog.cpp
 * @author  syhanjin
 * @date    2026-01-30
 * @brief   看门狗服务实现。
 *
 * 这里维护一张全局注册表，方便统一递减所有 Watchdog 实例的“剩余生命值”。
 */
#include "watchdog.hpp"

namespace service {

// 全局注册表：所有 Watchdog 实例都放在这里，便于统一递减。
static Watchdog *watchdogs[MAX_WATCHDOG_NUM];
static std::atomic_size_t watchdog_count = 0;

Watchdog::Watchdog() {
  // 原子地申请一个全局槽位。
  const size_t count_now =
      watchdog_count.fetch_add(1, std::memory_order_relaxed);
  if (count_now < MAX_WATCHDOG_NUM) {
    // 记录 this 指针，后续 EatAll() 会遍历它。
    watchdogs[count_now] = this;
    id_ = count_now;
  } else {
    // 超出容量则回滚计数。
    watchdog_count.fetch_sub(1, std::memory_order_relaxed);
  }
}

Watchdog::~Watchdog() {
  // watchdog should not be destructed
}

void Watchdog::feed() {
  // 默认喂食量，保证一段时间内不会被 EatAll() 消耗完。
  snacks_.store(WATCHDOG_SNACKS, std::memory_order_relaxed);
}

void Watchdog::feed(uint32_t snacks) {
  snacks_.store(static_cast<int32_t>(snacks), std::memory_order_relaxed);
}

void Watchdog::eat() { snacks_.fetch_sub(1, std::memory_order_relaxed); }

bool Watchdog::isFed() const {
  return snacks_.load(std::memory_order_relaxed) > 0;
}

void Watchdog::EatAll() {
  // 周期性扫描所有已注册对象并递减“剩余生存时间”。
  for (size_t i = 0; i < watchdog_count.load(std::memory_order_relaxed); ++i)
    watchdogs[i]->eat();
}

bool Watchdog::isFull() {
  return watchdog_count.load(std::memory_order_relaxed) == MAX_WATCHDOG_NUM;
}

} // namespace service

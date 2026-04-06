/**
 * @file    watchdog.hpp
 * @author  syhanjin
 * @date    2026-01-30
 * @brief   轻量看门狗服务。
 *
 * 每个 Watchdog 代表一个需要周期性“喂食”的对象；外部周期性调用 EatAll() 递减，
 * 对象自身调用 feed() 重新补满 snacks。
 *
 * 这个实现只做最基础的生命周期看护，不追求复杂策略，便于在任务调度或传感器链路里快速接入。
 */
#ifndef WATCHDOG_HPP
#define WATCHDOG_HPP
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace service {

#ifndef MAX_WATCHDOG_NUM
#define MAX_WATCHDOG_NUM (64)
#endif

#define WATCHDOG_SNACKS (10)

class Watchdog {
public:
  /**
   * @brief 新建一个受监控对象，并自动登记到全局表中。
   *
   * 如果全局表已满，构造仍然可以完成，但该实例不会被纳入统一递减。
   */
  Watchdog();
  ~Watchdog();
  /**
   * @brief 使用默认喂食量补满。
   */
  void feed();
  /**
   * @brief 以自定义喂食量补满。
   */
  void feed(uint32_t snacks);
  /**
   * @brief 消耗一个 tick。
   */
  void eat();

  [[nodiscard]] bool isFed() const;

  Watchdog(Watchdog &) = delete;
  Watchdog(Watchdog &&) = delete;

  /**
   * @brief 对所有登记对象统一执行一次 eat。
   *
   * 通常由系统定时任务调用。
   */
  static void EatAll();
  /**
   * @brief 全局 watchdog 表是否已满。
   */
  static bool isFull();

private:
  // 剩余“可存活”次数，<= 0 表示超时。
  std::atomic_int32_t snacks_{0};
  // 在全局注册表中的索引。
  uint32_t id_;
};

} // namespace service

#endif // WATCHDOG_HPP

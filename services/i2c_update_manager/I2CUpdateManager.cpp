/**
 * @file    I2CUpdateManager.cpp
 * @brief   I2C 周期更新管理器实现
 */
#include "I2CUpdateManager.hpp"

#include "main.h"

namespace
{
// 处理 HAL_GetTick() 回卷后的“是否到期”判断。
bool tickReached(const uint32_t now_ms, const uint32_t due_ms)
{
    return static_cast<int32_t>(now_ms - due_ms) >= 0;
}
} // namespace

I2CUpdateManager::I2CUpdateManager(I2CBusDMA& bus) : bus_(bus) {}

bool I2CUpdateManager::registerDevice(I2CDevice&       device,
                                      const uint32_t   period_ms,
                                      const uint32_t   phase_ms,
                                      const uint32_t   timeout_ms)
{
    // 运行期不支持动态注册；否则会和后台任务并发读写调度表。
    if (run_flag_ || task_handle_ != nullptr)
        return false;

    if (entry_count_ >= MaxDevices || period_ms == 0U)
        return false;

    // manager 只保存调度信息，设备对象本身不持有周期配置。
    Entry& entry        = entries_[entry_count_++];
    entry.device        = &device;
    entry.period_ms     = period_ms;
    entry.phase_ms      = phase_ms;
    entry.timeout_ms    = timeout_ms;
    entry.next_due_ms   = HAL_GetTick() + phase_ms;
    entry.enabled       = true;
    entry.initialized   = false;
    return true;
}

bool I2CUpdateManager::start()
{
    return start(Config{});
}

bool I2CUpdateManager::start(const Config& config)
{
    if (run_flag_)
        return true;

    // 先落盘配置，再启动后台任务，避免任务先跑起来但配置还没写完。
    config_   = config;
    run_flag_ = true;

    const BaseType_t result =
        xTaskCreate(taskEntry, config_.task_name, config_.stack_words, this, config_.priority, &task_handle_);

    if (result != pdPASS)
    {
        run_flag_    = false;
        task_handle_ = nullptr;
        return false;
    }

    return true;
}

void I2CUpdateManager::stop()
{
    run_flag_ = false;
}

I2CUpdateManager::Entry* I2CUpdateManager::selectReadyEntry(const uint32_t now_ms)
{
    Entry* best = nullptr;
    for (std::size_t i = 0; i < entry_count_; ++i)
    {
        Entry& entry = entries_[i];
        if (!entry.enabled || entry.device == nullptr)
            continue;

        if (!tickReached(now_ms, entry.next_due_ms))
            continue;

        // 这里选择“最早到期”的那一个，避免周期短的设备被周期长的设备持续挤压。
        if (best == nullptr || tickReached(best->next_due_ms, entry.next_due_ms))
            best = &entry;
    }
    return best;
}

uint32_t I2CUpdateManager::computeSleepMs(const uint32_t now_ms) const
{
    uint32_t min_wait_ms = config_.max_sleep_ms;

    for (std::size_t i = 0; i < entry_count_; ++i)
    {
        const Entry& entry = entries_[i];
        if (!entry.enabled || entry.device == nullptr)
            continue;

        if (tickReached(now_ms, entry.next_due_ms))
            // 只要已有设备到期，manager 就尽快醒来处理，不再继续长睡眠。
            return 1U;

        const uint32_t wait_ms = entry.next_due_ms - now_ms;
        if (wait_ms < min_wait_ms)
            min_wait_ms = wait_ms;
    }

    return min_wait_ms == 0U ? 1U : min_wait_ms;
}

void I2CUpdateManager::serviceEntry(Entry& entry, const uint32_t now_ms)
{
    if (!entry.initialized)
    {
        const bool ok     = entry.device->init(bus_, entry.timeout_ms);
        entry.initialized = ok;
        if (ok) {
            entry.device->markInitialized(now_ms);
        }
        else {
            entry.device->markFailure(now_ms);
        }
        entry.next_due_ms += entry.period_ms;
        return;
    }

    // 新周期开始：记录本轮的名义起点（即注册时或上一轮结束时算出的 next_due_ms）。
    // Pending 阶段会覆写 next_due_ms，所以必须在第一次调用时先把它存起来。
    if (!entry.pending_)
        entry.cycle_start_ms = entry.next_due_ms;

    const UpdateStatus status = entry.device->update(bus_, now_ms, entry.timeout_ms);

    if (status == UpdateStatus::Pending)
    {
        entry.pending_    = true;
        // 把 next_due_ms 推到转换完成时刻，让 selectReadyEntry 和 computeSleepMs
        // 知道这段时间内可以去调度其他设备或休眠，不再空转。
        entry.next_due_ms = entry.device->conversionDeadlineMs();
        return;
    }

    entry.pending_ = false;
    if (status == UpdateStatus::Complete) entry.device->markSuccess(now_ms);
    else                                  entry.device->markFailure(now_ms);
    // 如果这一轮已经明显落后，就直接跳到未来最近的周期点，而不是补跑历史周期。
    // 这样能保留周期相位，又避免任务恢复后短时间内把旧周期全部重放一遍。
    const uint32_t elapsed_ms    = now_ms - entry.cycle_start_ms;
    const uint32_t missed_cycles = elapsed_ms / entry.period_ms;
    entry.next_due_ms            = entry.cycle_start_ms + (missed_cycles + 1U) * entry.period_ms;
}

void I2CUpdateManager::run()
{
    while (run_flag_)
    {
        const uint32_t now_ms = HAL_GetTick();
        if (Entry* entry = selectReadyEntry(now_ms); entry != nullptr)
        {
            // 一次循环只推进一个设备，确保这条总线始终是串行访问。
            serviceEntry(*entry, now_ms);
            taskYIELD();
            continue;
        }

        // 当前没有到期设备时，按最近到期时间进入短暂休眠。
        vTaskDelay(pdMS_TO_TICKS(computeSleepMs(now_ms)));
    }

    task_handle_ = nullptr;
    vTaskDelete(nullptr);
}

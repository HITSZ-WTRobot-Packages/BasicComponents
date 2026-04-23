/**
 * @file    I2CUpdateManager.hpp
 * @brief   单条 I2C 总线的周期更新管理器
 */
#pragma once

#include "FreeRTOS.h"
#include "I2CBusDMA.hpp"
#include "I2CDevice.hpp"
#include "task.h"
#include <cstddef>
#include <cstdint>

// 单条 I2C 总线的周期调度器。不保证精准，可能有1~2ms的时间误差，适用于不那么精确周期的数据获取
//
// 该类拥有一个 FreeRTOS 任务，并串行调度注册到同一条总线上的所有设备。
// 每个设备由一个 Entry 描述其周期、相位和超时。manager 每次只推进一个设备，
// 从而保证总线上始终只有一个活跃事务。
class I2CUpdateManager final
{
public:
    static constexpr std::size_t MaxDevices = 8;

    // 描述 manager 自身任务的运行配置。
    struct Config
    {
        const char*    task_name{ "I2CUpdate" };
        uint16_t       stack_words{ 512 };
        UBaseType_t    priority{ tskIDLE_PRIORITY + 2U };
        uint32_t       max_sleep_ms{ 500U };
    };

    // 描述一个已注册设备的调度参数。
    struct Entry
    {
        I2CDevice* device{ nullptr };
        uint32_t   period_ms{ 0 };
        uint32_t   phase_ms{ 0 };
        uint32_t   timeout_ms{ 20 };
        uint32_t   next_due_ms{ 0 };   // 下次应被调度的时刻
        uint32_t   cycle_start_ms{ 0 }; // 本轮周期的名义起点，用于计算下一个 next_due_ms
        bool       enabled{ false };
        bool       initialized{ false };
        bool       pending_{ false };   // 当前是否正处于 Trigger→Read 的等待阶段
    };

    // 使用一条总线构造 manager。bus 的生命周期必须长于 manager。
    explicit I2CUpdateManager(I2CBusDMA& bus);

    // 注册一个周期设备。
    //
    // period_ms 是更新周期，phase_ms 用于错峰启动，timeout_ms 是单次事务超时。
    bool registerDevice(I2CDevice& device, uint32_t period_ms, uint32_t phase_ms = 0U, uint32_t timeout_ms = 20U);

    // 使用默认配置创建并启动后台调度任务。
    bool start();

    // 使用给定配置创建并启动后台调度任务。
    bool start(const Config& config);

    // 请求后台任务退出。任务会在下一轮循环结束时自删。
    void stop();

    [[nodiscard]] bool        isRunning() const { return run_flag_; }
    [[nodiscard]] std::size_t deviceCount() const { return entry_count_; }

private:
    // 返回当前已经到期、且应该优先被调度的设备项。
    Entry* selectReadyEntry(uint32_t now_ms);

    // 计算下一次轮询前最多可以休眠多久。
    uint32_t computeSleepMs(uint32_t now_ms) const;

    // 推进一个设备项的初始化或更新流程。
    void     serviceEntry(Entry& entry, uint32_t now_ms);

    // 静态中转函数：FreeRTOS 不能直接启动成员函数，需要通过 static 桥接。
    static void taskEntry(void* pvParameters) {
        auto* manager = static_cast<I2CUpdateManager*>(pvParameters);
        manager->run();
    }

    // 后台任务主循环。
    void run();

    I2CBusDMA&   bus_;
    Entry        entries_[MaxDevices]{};
    std::size_t  entry_count_{ 0 };
    TaskHandle_t task_handle_{ nullptr };
    Config       config_{};
    bool         run_flag_{ false };
};

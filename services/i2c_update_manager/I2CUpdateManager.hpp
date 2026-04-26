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

    /**
     * @brief 描述 manager 自身任务的运行配置
     */
    struct Config
    {
        const char*    task_name{ "I2CUpdate" };          ///< 调度任务名称
        uint16_t       stack_words{ 512 };                ///< 调度任务栈大小，单位 word
        UBaseType_t    priority{ tskIDLE_PRIORITY + 2U }; ///< 调度任务优先级
        uint32_t       max_sleep_ms{ 500U };              ///< 空闲时单次最长休眠时间，单位毫秒
    };

    /**
     * @brief 描述一个已注册设备的调度参数
     */
    struct Entry
    {
        I2CDevice* device{ nullptr };      ///< 设备对象指针
        uint32_t   period_ms{ 0 };         ///< 周期调度间隔，单位毫秒
        uint32_t   phase_ms{ 0 };          ///< 初始错峰相位，单位毫秒
        uint32_t   timeout_ms{ 20 };       ///< 单次设备事务超时时间，单位毫秒
        uint32_t   next_due_ms{ 0 };       ///< 下次应被调度的时刻
        uint32_t   cycle_start_ms{ 0 };    ///< 当前周期的名义起点
        bool       enabled{ false };       ///< 当前条目是否启用
        bool       initialized{ false };   ///< 设备是否已经完成初始化
        bool       pending_{ false };      ///< 当前是否正处于 Trigger 到 Read 的等待阶段
    };

    /**
     * @brief 使用一条总线构造调度器
     * @param bus 要管理的 I2C 总线
     */
    explicit I2CUpdateManager(I2CBusDMA& bus);

    /**
     * @brief 注册一个周期设备
     * @param device 要注册的设备对象
     * @param period_ms 更新周期，单位毫秒
     * @param phase_ms 初始错峰相位，单位毫秒
     * @param timeout_ms 单次事务超时时间，单位毫秒
     * @return 设备是否注册成功
     */
    bool registerDevice(I2CDevice& device, uint32_t period_ms, uint32_t phase_ms = 0U, uint32_t timeout_ms = 20U);

    /**
     * @brief 使用默认配置创建并启动后台调度任务
     * @return 调度任务是否成功启动
     */
    bool start();

    /**
     * @brief 使用给定配置创建并启动后台调度任务
     * @param config 调度任务配置
     * @return 调度任务是否成功启动
     */
    bool start(const Config& config);

    /**
     * @brief 请求后台任务退出
     */
    void stop();

    /**
     * @brief 查询调度任务当前是否在运行
     * @return 调度器是否处于运行状态
     */
    [[nodiscard]] bool        isRunning() const { return run_flag_; }
    /**
     * @brief 获取当前已注册设备数量
     * @return 当前设备条目数量
     */
    [[nodiscard]] std::size_t deviceCount() const { return entry_count_; }

private:
    /**
     * @brief 选择当前已经到期且应优先调度的设备条目
     * @param now_ms 当前时间戳，单位毫秒
     * @return 选中的设备条目，若无可调度设备则返回空指针
     */
    Entry* selectReadyEntry(uint32_t now_ms);

    /**
     * @brief 计算下一次轮询前最多可以休眠多久
     * @param now_ms 当前时间戳，单位毫秒
     * @return 推荐休眠时长，单位毫秒
     */
    uint32_t computeSleepMs(uint32_t now_ms) const;

    /**
     * @brief 推进一个设备条目的初始化或更新流程
     * @param entry 要推进的设备条目
     * @param now_ms 当前时间戳，单位毫秒
     */
    void     serviceEntry(Entry& entry, uint32_t now_ms);

    /**
     * @brief FreeRTOS 任务入口的静态桥接函数
     * @param pvParameters 传入的调度器对象指针
     */
    static void taskEntry(void* pvParameters) {
        auto* manager = static_cast<I2CUpdateManager*>(pvParameters);
        manager->run();
    }

    /**
     * @brief 后台调度任务主循环
     */
    void run();

    I2CBusDMA&   bus_;                        ///< 当前调度器独占管理的 I2C 总线
    Entry        entries_[MaxDevices]{};      ///< 已注册设备的调度表
    std::size_t  entry_count_{ 0 };           ///< 当前已注册设备数量
    TaskHandle_t task_handle_{ nullptr };     ///< 后台调度任务句柄
    Config       config_{};                   ///< 当前采用的任务配置
    bool         run_flag_{ false };          ///< 后台调度任务是否应继续运行
};

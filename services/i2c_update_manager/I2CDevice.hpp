/**
 * @file    I2CDevice.hpp
 * @brief   面向 manager 周期调度的 I2C 设备抽象基类
 * @author ChillyHigh
 */
#pragma once

#include <cstdint>

class I2CBusDMA;
class I2CUpdateManager;

// 描述一次 update() 推进后的结果。
enum class UpdateStatus : uint8_t { Complete, Pending, Failed };

// I2C 周期设备的抽象基类。
//
// 这个基类内置了 Trigger -> Wait -> Read 三段状态机，子类只需要实现
// 具体协议钩子，不需要自己维护等待时间或阶段切换。manager 在每个周期
// 调用 update() 推进一步；如果设备还在等待内部转换，则返回 Pending，
// 调度器可以先去服务同一总线上的其它设备。
class I2CDevice
{
public:
    virtual ~I2CDevice() = default;

    // 返回设备名，仅用于日志或调试显示。
    virtual const char* name() const = 0;

    // 返回 7 位 I2C 设备地址。
    virtual uint8_t     address7bit() const = 0;

    // 执行一次初始化或探活流程。
    //
    // 通常在这里做上电配置、WHO_AM_I 检查或最小读操作。返回 true 表示
    // 初始化成功，manager 后续会开始调度 update()。
    virtual bool        init(I2CBusDMA& bus, uint32_t timeout_ms) = 0;

    // 推进一次设备状态机。
    //
    // 该函数不是虚函数，目的是把阶段推进逻辑统一收敛到基类，避免每个
    // 设备子类重复实现 Trigger/Wait/Read 状态切换。
    UpdateStatus update(I2CBusDMA& bus, uint32_t now_ms, uint32_t timeout_ms);

    // 返回本轮转换的预期完成时刻（trigger_ms_ + conversionMs()）。
    // manager 在收到 Pending 后用这个值更新 next_due_ms，以便休眠并为其他设备让路。
    [[nodiscard]] uint32_t conversionDeadlineMs() const { return trigger_ms_ + conversionMs(); }

    [[nodiscard]] bool     isInitialized() const { return initialized_; }
    [[nodiscard]] bool     isOnline() const { return online_; }
    [[nodiscard]] uint32_t lastAttemptMs() const { return last_attempt_ms_; }
    [[nodiscard]] uint32_t lastSuccessMs() const { return last_success_ms_; }
    [[nodiscard]] uint32_t lastFailureMs() const { return last_failure_ms_; }
    [[nodiscard]] uint8_t  consecutiveFailures() const { return consecutive_failures_; }

protected:
    // 发送一次“开始转换”或“开始采样”命令。
    //
    // 默认实现直接返回 true，适用于无需显式触发、可直接读取数据的设备。
    virtual bool onTrigger(I2CBusDMA& /*bus*/, uint32_t /*timeout_ms*/) { return true; }

    // 返回触发后需要等待的最短转换时间，单位毫秒。
    //
    // 返回 0 表示 onTrigger() 后可以直接进入 onRead()。
    virtual uint32_t conversionMs() const { return 0; }

    // 读取一次设备数据并更新内部缓存。
    //
    // 返回 true 表示本轮更新完成并生成了有效数据；返回 false 表示本轮更新失败。
    virtual bool onRead(I2CBusDMA& bus, uint32_t now_ms, uint32_t timeout_ms) = 0;

    // 记录一次成功更新。
    void markSuccess(uint32_t now_ms)
    {
        initialized_          = true;
        online_               = true;
        last_attempt_ms_      = now_ms;
        last_success_ms_      = now_ms;
        consecutive_failures_ = 0;
    }

    // 记录一次失败更新。
    void markFailure(uint32_t now_ms)
    {
        last_attempt_ms_ = now_ms;
        last_failure_ms_ = now_ms;
        if (consecutive_failures_ < 255U) ++consecutive_failures_;
        online_ = false;
    }

private:
    // 描述基类状态机当前处于哪个阶段。
    enum class Phase : uint8_t { Trigger, Wait, Read };

    // phase_ 表示当前阶段；trigger_ms_ 记录最近一次触发发生的时刻。
    Phase    phase_{ Phase::Trigger };
    uint32_t trigger_ms_{ 0 };

    // 以下字段由 manager 维护，用于健康状态和时间戳统计。
    bool     initialized_{ false };
    bool     online_{ false };
    uint32_t last_attempt_ms_{ 0 };
    uint32_t last_success_ms_{ 0 };
    uint32_t last_failure_ms_{ 0 };
    uint8_t  consecutive_failures_{ 0 };

    friend class I2CUpdateManager;
};

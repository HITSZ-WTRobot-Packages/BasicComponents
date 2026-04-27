/**
 * @file    I2CDevice.hpp
 * @brief   面向 manager 周期调度的 I2C 设备抽象基类
 * @author ChillyHigh
 */
#pragma once

#include <cstdint>

class I2CBusDMA;
class I2CUpdateManager;

/**
 * @brief 描述一次 update() 推进后的结果
 */
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
    /**
     * @brief 虚析构函数
     */
    virtual ~I2CDevice() = default;

    /**
     * @brief 获取设备名称
     * @return 设备名字符串，仅用于日志或调试显示
     */
    virtual const char* name() const = 0;

    /**
     * @brief 获取设备的 7 位 I2C 地址
     * @return 设备地址
     */
    virtual uint8_t     address7bit() const = 0;

    /**
     * @brief 执行一次初始化或探活流程
     * @param bus 当前设备所在的 I2C 总线
     * @param timeout_ms 单次事务超时时间，单位毫秒
     * @return 初始化是否成功
     */
    virtual bool        init(I2CBusDMA& bus, uint32_t timeout_ms) = 0;

    /**
     * @brief 推进一次设备状态机
     * @param bus 当前设备所在的 I2C 总线
     * @param now_ms 当前时间戳，单位毫秒
     * @param timeout_ms 单次事务超时时间，单位毫秒
     * @return 本次推进后的状态结果
     */
    UpdateStatus update(I2CBusDMA& bus, uint32_t now_ms, uint32_t timeout_ms);

    /**
     * @brief 获取本轮转换的预期完成时刻
     * @return `trigger_ms_ + conversionMs()`
     */
    [[nodiscard]] uint32_t conversionDeadlineMs() const { return trigger_ms_ + conversionMs(); }

    /**
     * @brief 查询设备是否已经初始化成功
     * @return 当前是否已经成功初始化
     */
    [[nodiscard]] bool     isInitialized() const { return initialized_; }
    /**
     * @brief 查询设备当前是否在线
     * @return 最近一次访问后设备是否在线
     */
    [[nodiscard]] bool     isOnline() const { return online_; }
    /**
     * @brief 查询设备当前是否持有有效数据
     * @return 最近一次成功更新后数据是否仍有效
     */
    [[nodiscard]] bool     hasValidData() const { return data_valid_; }
    /**
     * @brief 查询设备数据是否仍然新鲜
     * @param now_ms 当前时间戳，单位毫秒
     * @param stale_ms 允许的数据陈旧阈值，单位毫秒
     * @return 数据是否仍在可接受的新鲜时间窗口内
     */
    [[nodiscard]] bool     isDataFresh(const uint32_t now_ms, const uint32_t stale_ms) const;
    /**
     * @brief 获取最近一次尝试访问设备的时间
     * @return 最近一次访问尝试的时间戳
     */
    [[nodiscard]] uint32_t lastAttemptMs() const { return last_attempt_ms_; }
    /**
     * @brief 获取最近一次成功更新设备的时间
     * @return 最近一次成功更新时间戳
     */
    [[nodiscard]] uint32_t lastSuccessMs() const { return last_success_ms_; }
    /**
     * @brief 获取最近一次更新失败的时间
     * @return 最近一次失败时间戳
     */
    [[nodiscard]] uint32_t lastFailureMs() const { return last_failure_ms_; }
    /**
     * @brief 获取连续失败次数
     * @return 当前累计的连续失败次数
     */
    [[nodiscard]] uint8_t  consecutiveFailures() const { return consecutive_failures_; }

protected:
    /**
     * @brief 发送一次“开始转换”或“开始采样”命令
     * @param bus 当前设备所在的 I2C 总线
     * @param timeout_ms 单次事务超时时间，单位毫秒
     * @return 触发是否成功
     */
    virtual bool onTrigger(I2CBusDMA& /*bus*/, uint32_t /*timeout_ms*/) { return true; }

    /**
     * @brief 获取触发后需要等待的最短转换时间
     * @return 转换等待时间，单位毫秒；返回 0 表示可直接进入读取阶段
     */
    virtual uint32_t conversionMs() const { return 0; }

    /**
     * @brief 读取一次设备数据并更新内部缓存
     * @param bus 当前设备所在的 I2C 总线
     * @param now_ms 当前时间戳，单位毫秒
     * @param timeout_ms 单次事务超时时间，单位毫秒
     * @return 本轮读取是否成功
     */
    virtual bool onRead(I2CBusDMA& bus, uint32_t now_ms, uint32_t timeout_ms) = 0;

    /**
     * @brief 当父类判定数据失效时，同步清理子类缓存标记
     */
    virtual void onDataInvalidated() {}

    /**
     * @brief 记录一次成功更新
     * @param now_ms 当前时间戳，单位毫秒
     */
    void markSuccess(uint32_t now_ms)
    {
        initialized_          = true;
        online_               = true;
        data_valid_           = true;
        last_attempt_ms_      = now_ms;
        last_success_ms_      = now_ms;
        consecutive_failures_ = 0;
    }


    void markInitialized(uint32_t now_ms)
    {
        initialized_          = true;
        online_               = true;
        data_valid_           = false;
        last_attempt_ms_      = now_ms;
        last_success_ms_      = 0;
        last_failure_ms_      = 0;
        consecutive_failures_ = 0;
    }

    /**
     * @brief 记录一次失败更新
     * @param now_ms 当前时间戳，单位毫秒
     */
    void markFailure(uint32_t now_ms)
    {
        last_attempt_ms_ = now_ms;
        last_failure_ms_ = now_ms;
        if (consecutive_failures_ < 255U) ++consecutive_failures_;
        online_     = false;
        data_valid_ = false;
        onDataInvalidated();
    }

private:
    /**
     * @brief 描述基类状态机当前处于哪个阶段
     */
    enum class Phase : uint8_t { Trigger, Wait, Read };

    Phase    phase_{ Phase::Trigger };      ///< 当前状态机阶段
    uint32_t trigger_ms_{ 0 };              ///< 最近一次触发采样的时间戳

    bool     initialized_{ false };         ///< 设备是否已经成功初始化
    bool     online_{ false };              ///< 设备最近一次访问后是否在线
    bool     data_valid_{ false };          ///< 当前缓存的数据是否有效
    uint32_t last_attempt_ms_{ 0 };         ///< 最近一次尝试访问设备的时间戳
    uint32_t last_success_ms_{ 0 };         ///< 最近一次成功更新时间戳
    uint32_t last_failure_ms_{ 0 };         ///< 最近一次失败时间戳
    uint8_t  consecutive_failures_{ 0 };    ///< 当前连续失败次数

    friend class I2CUpdateManager;          ///< 允许调度器访问内部状态字段
};

/**
 * @file    I2CDevice.cpp
 * @brief   I2CDevice 基类状态机实现
 */
#include "I2CDevice.hpp"

bool I2CDevice::isDataFresh(const uint32_t now_ms, const uint32_t stale_ms) const
{
    if (!data_valid_)
        return false;

    return stale_ms == 0U || (now_ms - last_success_ms_ <= stale_ms);
}

UpdateStatus I2CDevice::update(I2CBusDMA& bus, const uint32_t now_ms, const uint32_t timeout_ms)
{
    if (phase_ == Phase::Trigger)
    {
        // Trigger 阶段只负责发起一次采样，不在这里做任何忙等。
        if (!onTrigger(bus, timeout_ms))
            return UpdateStatus::Failed;
        trigger_ms_ = now_ms;
        phase_      = Phase::Wait;
    }

    if (phase_ == Phase::Wait)
    {
        // 未到最短转换时间时立即返回 Pending，让 manager 去调度其它设备。
        if (now_ms - trigger_ms_ < conversionMs())
            return UpdateStatus::Pending;
        phase_ = Phase::Read;
    }

    // Read 阶段负责真正取回数据。无论成功失败，下一轮都从 Trigger 重新开始。
    const bool ok = onRead(bus, now_ms, timeout_ms);
    phase_ = Phase::Trigger;
    return ok ? UpdateStatus::Complete : UpdateStatus::Failed;
}

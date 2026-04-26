/**
 * @file    I2CBusDMA.hpp
 * @brief   基于 STM32 HAL DMA 和 FreeRTOS 任务通知的单 owner I2C 总线封装
 */
#pragma once

#include "FreeRTOS.h"
#include "i2c.h"
#include "task.h"
#include <cstddef>
#include <cstdint>

// 一条 I2C 总线的 DMA 封装。
//
// 该类假设总线只有一个 owner task。调用者通过 memRead()/memWrite() 等接口
// 发起 DMA 事务，然后阻塞等待 HAL 回调通过任务通知唤醒自己。这样对上层
// 看起来仍是同步接口，但底层传输期间不会忙等占用 CPU。
class I2CBusDMA final
{
public:
    // 描述一次总线事务的最终错误状态。
    enum class Error : uint8_t
    {
        None,
        InvalidHandle,
        Busy,
        StartFailed,
        Timeout,
        HalError,
        RecoveryFailed,
    };

    // 使用一个 HAL I2C 句柄构造总线对象。
    explicit I2CBusDMA(I2C_HandleTypeDef* hi2c);

    [[nodiscard]] I2C_HandleTypeDef* handle() const { return hi2c_; }

    // 返回总线当前是否仍有事务在飞，或 HAL 状态是否未回到 READY。
    [[nodiscard]] bool               isBusy() const;

    // 返回最近一次事务的抽象错误码和底层 HAL 错误码。
    [[nodiscard]] Error              lastError() const { return last_error_; }
    [[nodiscard]] uint32_t           lastHalError() const { return last_hal_error_; }

    // 发起一次带寄存器地址的 DMA 读事务。
    bool memRead(uint8_t device_addr_7bit, uint8_t reg, uint8_t* data, uint16_t len, uint32_t timeout_ms);

    // 发起一次带寄存器地址的 DMA 写事务。
    bool memWrite(uint8_t device_addr_7bit,
                  uint8_t device_reg,
                  const uint8_t* data,
                  uint16_t len,
                  uint32_t timeout_ms);

    // 发起一次原始 DMA 读写事务。
    bool read(uint8_t device_addr_7bit, uint8_t* data, uint16_t len, uint32_t timeout_ms);
    bool write(uint8_t device_addr_7bit, const uint8_t* data, uint16_t len, uint32_t timeout_ms);

    // 尝试通过重新初始化 I2C 外设恢复总线。
    bool recover();

    // 根据 HAL 句柄反查 I2CBusDMA 对象，供全局 HAL 回调桥接使用。
    static I2CBusDMA* fromHandle(I2C_HandleTypeDef* hi2c);

    // 由 HAL 回调在 ISR 中调用，通知当前等待任务传输已经结束。
    void onTxCompleteFromISR();
    void onRxCompleteFromISR();
    void onErrorFromISR();

private:
    static constexpr std::size_t MaxInstances = 4;

    // 在启动 DMA 前检查总线状态，并记录当前等待完成的任务。
    bool prepareTransfer();

    // 阻塞等待 DMA 完成通知，超时后尝试恢复总线。
    bool waitForTransfer(uint32_t timeout_ms);

    // 在 ISR 中收敛完成状态，并唤醒等待任务。
    void completeFromISR(bool success, uint32_t hal_error);

    // 清空当前事务的软件状态，不改变对外暴露的错误语义。
    void clearTransferState();

    // 记录失败原因并执行恢复流程。
    bool failAndRecover(Error error, uint32_t hal_error);

    // 把实例注册到静态表，供全局 HAL 回调反查。
    static bool registerInstance(I2CBusDMA* instance);

    I2C_HandleTypeDef* hi2c_{ nullptr };
    TaskHandle_t       waiting_task_{ nullptr };
    volatile bool      transmitting_{ false };
    volatile bool      completed_{ false };
    volatile uint32_t  next_transfer_id_{ 1U };
    volatile uint32_t  current_transfer_id_{ 0U };
    volatile uint32_t  completed_transfer_id_{ 0U };
    volatile Error     last_error_{ Error::InvalidHandle };
    volatile uint32_t  last_hal_error_{ HAL_I2C_ERROR_NONE };

    // 所有 bus 实例共享的反查表。
    static I2CBusDMA* instances_[MaxInstances];
};

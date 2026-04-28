/**
 * @file    I2CBusDMA.hpp
 * @brief   基于 STM32 HAL DMA 和 CMSIS-RTOS v2 线程标志的单 owner I2C 总线封装
 */
#pragma once

#include "cmsis_os2.h"
#include "i2c.h"
#include <cstddef>
#include <cstdint>

// 一条 I2C 总线的 DMA 封装。
//
// 该类假设总线只有一个 owner thread。调用者通过 memRead()/memWrite() 等接口
// 发起 DMA 事务，然后阻塞等待 HAL 回调通过线程标志唤醒自己。这样对上层
// 看起来仍是同步接口，但底层传输期间不会忙等占用 CPU。
class I2CBusDMA final
{
public:
    /**
     * @brief 描述一次总线事务的最终错误状态
     */
    enum class Error : uint8_t
    {
        None,           ///< 最近一次事务成功完成
        InvalidHandle,  ///< HAL I2C 句柄无效
        InvalidContext, ///< 调用上下文不是可阻塞等待的 CMSIS 线程
        Busy,           ///< 总线或驱动当前忙碌，无法启动新事务
        StartFailed,    ///< DMA 事务启动失败
        Timeout,        ///< 等待完成超时
        HalError,       ///< HAL 在事务过程中报告错误
        RecoveryFailed, ///< 失败后的恢复流程也未成功
    };

    /**
     * @brief 使用 HAL I2C 句柄构造总线对象
     * @param hi2c 要绑定的 HAL I2C 句柄
     */
    explicit I2CBusDMA(I2C_HandleTypeDef* hi2c);

    /**
     * @brief 获取底层 HAL I2C 句柄
     * @return 当前绑定的 HAL I2C 句柄
     */
    [[nodiscard]] I2C_HandleTypeDef* handle() const { return hi2c_; }

    /**
     * @brief 查询总线当前是否忙碌
     * @return 当前是否仍有事务未完成，或 HAL 状态尚未回到 READY
     */
    [[nodiscard]] bool               isBusy() const;

    /**
     * @brief 获取最近一次事务的抽象错误码
     * @return 最近一次事务记录的错误状态
     */
    [[nodiscard]] Error              lastError() const { return last_error_; }
    /**
     * @brief 获取最近一次事务的 HAL 原始错误码
     * @return 最近一次事务记录的 HAL 错误码
     */
    [[nodiscard]] uint32_t           lastHalError() const { return last_hal_error_; }

    /**
     * @brief 发起一次带寄存器地址的 DMA 读事务
     * @param device_addr_7bit 7 位设备地址
     * @param reg 目标寄存器地址
     * @param data 读数据缓冲区
     * @param len 读取字节数
     * @param timeout_ms 等待完成超时时间，单位毫秒
     * @return 事务是否成功完成
     */
    bool memRead(uint8_t device_addr_7bit, uint8_t reg, uint8_t* data, uint16_t len, uint32_t timeout_ms);

    /**
     * @brief 发起一次带寄存器地址的 DMA 写事务
     * @param device_addr_7bit 7 位设备地址
     * @param device_reg 目标寄存器地址
     * @param data 写数据缓冲区
     * @param len 写入字节数
     * @param timeout_ms 等待完成超时时间，单位毫秒
     * @return 事务是否成功完成
     */
    bool memWrite(uint8_t device_addr_7bit,
                  uint8_t device_reg,
                  const uint8_t* data,
                  uint16_t len,
                  uint32_t timeout_ms);

    /**
     * @brief 发起一次原始 DMA 读事务
     * @param device_addr_7bit 7 位设备地址
     * @param data 读数据缓冲区
     * @param len 读取字节数
     * @param timeout_ms 等待完成超时时间，单位毫秒
     * @return 事务是否成功完成
     */
    bool read(uint8_t device_addr_7bit, uint8_t* data, uint16_t len, uint32_t timeout_ms);
    /**
     * @brief 发起一次原始 DMA 写事务
     * @param device_addr_7bit 7 位设备地址
     * @param data 写数据缓冲区
     * @param len 写入字节数
     * @param timeout_ms 等待完成超时时间，单位毫秒
     * @return 事务是否成功完成
     */
    bool write(uint8_t device_addr_7bit, const uint8_t* data, uint16_t len, uint32_t timeout_ms);

    /**
     * @brief 尝试恢复当前 I2C 总线
     * @return 恢复后总线是否重新回到可用状态
     */
    bool recover();

    /**
     * @brief 根据 HAL 句柄反查 I2CBusDMA 实例
     * @param hi2c HAL I2C 句柄
     * @return 对应的 I2CBusDMA 实例，不存在则返回空指针
     */
    static I2CBusDMA* fromHandle(I2C_HandleTypeDef* hi2c);

    /**
     * @brief 在 ISR 中上报发送完成
     */
    void onTxCompleteFromISR();
    /**
     * @brief 在 ISR 中上报接收完成
     */
    void onRxCompleteFromISR();
    /**
     * @brief 在 ISR 中上报事务错误
     */
    void onErrorFromISR();

private:
    static constexpr std::size_t MaxInstances = 4;

    /**
     * @brief 在启动 DMA 前准备一次事务
     * @return 当前是否允许启动新事务
     */
    bool prepareTransfer();

    /**
     * @brief 阻塞等待当前事务完成
     * @param timeout_ms 等待完成超时时间，单位毫秒
     * @return 当前事务是否成功完成
     */
    bool waitForTransfer(uint32_t timeout_ms);

    /**
     * @brief 在 ISR 中记录完成状态并唤醒等待线程
     * @param success 本次完成是否成功
     * @param hal_error HAL 层返回的错误码
     */
    void completeFromISR(bool success, uint32_t hal_error);

    /**
     * @brief 清空当前事务的软件状态
     */
    void clearTransferState();

    /**
     * @brief 记录失败原因并执行恢复流程
     * @param error 本次失败的抽象错误码
     * @param hal_error 本次失败对应的 HAL 错误码
     * @return 始终返回 false，便于直接作为失败出口
     */
    bool failAndRecover(Error error, uint32_t hal_error);

    /**
     * @brief 把实例注册到静态反查表
     * @param instance 要注册的总线对象
     * @return 注册是否成功
     */
    static bool registerInstance(I2CBusDMA* instance);

    I2C_HandleTypeDef* hi2c_{ nullptr };                     ///< 绑定的 HAL I2C 句柄
    osThreadId_t       waiting_thread_{ nullptr };           ///< 当前阻塞等待事务完成的线程句柄
    volatile bool      transmitting_{ false };               ///< 当前是否已有事务启动且尚未完成收敛
    volatile bool      completed_{ false };                  ///< 当前事务是否已收到完成记录
    volatile uint32_t  next_transfer_id_{ 1U };              ///< 下一次启动事务时要分配的事务编号
    volatile uint32_t  current_transfer_id_{ 0U };           ///< 当前正在等待的事务编号
    volatile uint32_t  completed_transfer_id_{ 0U };         ///< 最近一次完成记录对应的事务编号
    volatile Error     last_error_{ Error::InvalidHandle };  ///< 最近一次事务记录的抽象错误状态
    volatile uint32_t  last_hal_error_{ HAL_I2C_ERROR_NONE }; ///< 最近一次事务记录的 HAL 错误码

    static I2CBusDMA* instances_[MaxInstances]; ///< 所有 bus 实例共享的 HAL 句柄反查表
};

/**
 * @file    I2CBusDMA.cpp
 * @brief   I2C DMA 总线封装实现
 */
#include "I2CBusDMA.hpp"

namespace
{
constexpr uint32_t TransferCompleteFlag = 1U << 0;
constexpr uint32_t BusPulseDelayCycles  = 256U;

uint32_t kernelTicksFromMs(uint32_t timeout_ms)
{
    if (timeout_ms == 0U)
        timeout_ms = 1U;

    const uint32_t tick_freq = osKernelGetTickFreq();
    if (tick_freq == 0U)
        return timeout_ms;

    const uint64_t ticks = (static_cast<uint64_t>(timeout_ms) * tick_freq + 999ULL) / 1000ULL;
    return static_cast<uint32_t>(ticks == 0U ? 1ULL : ticks);
}

bool isThreadFlagsError(const uint32_t flags)
{
    return (flags & osFlagsError) != 0U;
}

void shortDelay()
{
    for (volatile uint32_t i = 0; i < BusPulseDelayCycles; ++i)
    {
        __NOP();
    }
}
} // namespace

I2CBusDMA* I2CBusDMA::instances_[I2CBusDMA::MaxInstances] = { nullptr };

I2CBusDMA::I2CBusDMA(I2C_HandleTypeDef* hi2c, const BusPins pins) : hi2c_(hi2c), pins_(pins)
{
    // 构造时把当前 bus 实例登记到静态表中，后续 HAL 全局回调才能反查到对象。
    if (hi2c_ != nullptr && registerInstance(this))
    {
        last_error_ = Error::None;
    }
}

bool I2CBusDMA::isBusy() const
{
    if (hi2c_ == nullptr)
        return false;

    return transmitting_ || (HAL_I2C_GetState(hi2c_) != HAL_I2C_STATE_READY);
}

bool I2CBusDMA::memRead(const uint8_t  device_addr_7bit,
                        const uint8_t  reg,
                        uint8_t*       data,
                        const uint16_t len,
                        const uint32_t timeout_ms)
{
    // 发起 DMA 之前先确认总线空闲，并记录当前等待完成的线程。
    if (!prepareTransfer())
        return false;

    const HAL_StatusTypeDef status =
        HAL_I2C_Mem_Read_DMA(hi2c_, static_cast<uint16_t>(device_addr_7bit << 1U), reg, I2C_MEMADD_SIZE_8BIT, data, len);

    if (status != HAL_OK)
    {
        return failAndRecover(Error::StartFailed, HAL_I2C_GetError(hi2c_));
    }

    return waitForTransfer(timeout_ms);
}

bool I2CBusDMA::memWrite(const uint8_t        device_addr_7bit,
                         const uint8_t        device_reg,
                         const uint8_t* const data,
                         const uint16_t       len,
                         const uint32_t       timeout_ms)
{
    // mem write 和 mem read 使用同一套等待机制，区别只在 HAL 启动接口。
    if (!prepareTransfer())
        return false;

    const HAL_StatusTypeDef status = HAL_I2C_Mem_Write_DMA(hi2c_,
                                                           static_cast<uint16_t>(device_addr_7bit << 1U),
                                                           device_reg,
                                                           I2C_MEMADD_SIZE_8BIT,
                                                           const_cast<uint8_t*>(data),
                                                           len);

    if (status != HAL_OK)
    {
        return failAndRecover(Error::StartFailed, HAL_I2C_GetError(hi2c_));
    }

    return waitForTransfer(timeout_ms);
}

bool I2CBusDMA::read(const uint8_t  device_addr_7bit,
                     uint8_t*       data,
                     const uint16_t len,
                     const uint32_t timeout_ms)
{
    if (!prepareTransfer())
        return false;

    const HAL_StatusTypeDef status =
        HAL_I2C_Master_Receive_DMA(hi2c_, static_cast<uint16_t>(device_addr_7bit << 1U), data, len);

    if (status != HAL_OK)
    {
        return failAndRecover(Error::StartFailed, HAL_I2C_GetError(hi2c_));
    }

    return waitForTransfer(timeout_ms);
}

bool I2CBusDMA::write(const uint8_t        device_addr_7bit,
                      const uint8_t* const data,
                      const uint16_t       len,
                      const uint32_t       timeout_ms)
{
    if (!prepareTransfer())
        return false;

    const HAL_StatusTypeDef status =
        HAL_I2C_Master_Transmit_DMA(hi2c_, static_cast<uint16_t>(device_addr_7bit << 1U), const_cast<uint8_t*>(data), len);

    if (status != HAL_OK)
    {
        return failAndRecover(Error::StartFailed, HAL_I2C_GetError(hi2c_));
    }

    return waitForTransfer(timeout_ms);
}

bool I2CBusDMA::recover()
{
    if (hi2c_ == nullptr || pins_.scl_port == nullptr || pins_.sda_port == nullptr)
    {
        last_error_ = Error::InvalidHandle;
        return false;
    }

    // 恢复前先把软件态清空，避免 manager 误以为还有旧事务未完成。
    clearTransferState();

    // 对超时和错误路径，先尽量终止底层 DMA，减少旧完成中断漂到后续事务。
    if (hi2c_->hdmarx != nullptr)
        (void) HAL_DMA_Abort(hi2c_->hdmarx);
    if (hi2c_->hdmatx != nullptr)
        (void) HAL_DMA_Abort(hi2c_->hdmatx);

    if (HAL_I2C_DeInit(hi2c_) != HAL_OK)
    {
        last_error_     = Error::RecoveryFailed;
        last_hal_error_ = HAL_I2C_GetError(hi2c_);
        return false;
    }

    if (!recoverBusLines())
    {
        last_error_     = Error::RecoveryFailed;
        last_hal_error_ = HAL_I2C_ERROR_TIMEOUT;
        return false;
    }

    if (HAL_I2C_Init(hi2c_) != HAL_OK)
    {
        last_error_     = Error::RecoveryFailed;
        last_hal_error_ = HAL_I2C_GetError(hi2c_);
        return false;
    }

    last_hal_error_ = HAL_I2C_ERROR_NONE;
    return true;
}

bool I2CBusDMA::recoverBusLines()
{
    GPIO_InitTypeDef gpio_init{};
    gpio_init.Pin   = pins_.scl_pin | pins_.sda_pin;
    gpio_init.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio_init.Pull  = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(pins_.scl_port, &gpio_init);

    HAL_GPIO_WritePin(pins_.scl_port, pins_.scl_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(pins_.sda_port, pins_.sda_pin, GPIO_PIN_SET);
    shortDelay();

    for (uint32_t pulse = 0; pulse < 9U && HAL_GPIO_ReadPin(pins_.sda_port, pins_.sda_pin) == GPIO_PIN_RESET; ++pulse)
    {
        HAL_GPIO_WritePin(pins_.scl_port, pins_.scl_pin, GPIO_PIN_RESET);
        shortDelay();
        HAL_GPIO_WritePin(pins_.scl_port, pins_.scl_pin, GPIO_PIN_SET);
        shortDelay();
    }

    HAL_GPIO_WritePin(pins_.sda_port, pins_.sda_pin, GPIO_PIN_RESET);
    shortDelay();
    HAL_GPIO_WritePin(pins_.scl_port, pins_.scl_pin, GPIO_PIN_SET);
    shortDelay();
    HAL_GPIO_WritePin(pins_.sda_port, pins_.sda_pin, GPIO_PIN_SET);
    shortDelay();

    const bool released = HAL_GPIO_ReadPin(pins_.scl_port, pins_.scl_pin) == GPIO_PIN_SET &&
                          HAL_GPIO_ReadPin(pins_.sda_port, pins_.sda_pin) == GPIO_PIN_SET;

    gpio_init.Mode      = GPIO_MODE_AF_OD;
    gpio_init.Alternate = pins_.alternate_function;
    HAL_GPIO_Init(pins_.scl_port, &gpio_init);

    return released;
}

I2CBusDMA* I2CBusDMA::fromHandle(I2C_HandleTypeDef* hi2c)
{
    for (auto* instance : instances_)
    {
        if (instance != nullptr && instance->hi2c_ == hi2c)
            return instance;
    }
    return nullptr;
}

void I2CBusDMA::onTxCompleteFromISR()
{
    completeFromISR(true, HAL_I2C_ERROR_NONE);
}

void I2CBusDMA::onRxCompleteFromISR()
{
    completeFromISR(true, HAL_I2C_ERROR_NONE);
}

void I2CBusDMA::onErrorFromISR()
{
    completeFromISR(false, HAL_I2C_GetError(hi2c_));
}

bool I2CBusDMA::prepareTransfer()
{
    if (hi2c_ == nullptr)
    {
        last_error_ = Error::InvalidHandle;
        return false;
    }

    if (osKernelGetState() != osKernelRunning)
    {
        last_error_ = Error::InvalidContext;
        return false;
    }

    if (transmitting_ || (HAL_I2C_GetState(hi2c_) != HAL_I2C_STATE_READY))
    {
        // 这套封装默认一条总线同一时刻只允许一个事务在飞。
        last_error_ = Error::Busy;
        return false;
    }

    // 记录当前调用线程，DMA 完成后通过线程标志把它唤醒。
    waiting_thread_ = osThreadGetId();
    if (waiting_thread_ == nullptr)
    {
        last_error_ = Error::InvalidContext;
        return false;
    }

    transmitting_          = true;
    completed_             = false;
    current_transfer_id_   = next_transfer_id_++;
    completed_transfer_id_ = 0U;
    last_error_            = Error::None;
    last_hal_error_        = HAL_I2C_ERROR_NONE;

    // 清掉可能残留的线程标志，避免把旧完成事件误当成本次 DMA 完成。
    (void) osThreadFlagsClear(TransferCompleteFlag);
    return true;
}

bool I2CBusDMA::waitForTransfer(const uint32_t timeout_ms)
{
    const uint32_t timeout_ticks = kernelTicksFromMs(timeout_ms);
    const uint32_t start_ticks   = osKernelGetTickCount();
    const uint32_t transfer_id   = current_transfer_id_;

    // 这里必须循环等待，而不能假设一次线程标志就对应当前事务。
    // 超时恢复后的旧 DMA/IRQ 可能晚到；只有事务编号匹配时，这条完成记录
    // 才能被当成当前事务的真实结束事件。
    while (true)
    {
        const uint32_t elapsed_ticks = osKernelGetTickCount() - start_ticks;
        if (elapsed_ticks >= timeout_ticks)
        {
            // 超时通常意味着 DMA 回调没回来，或者总线卡死，交给 recoverFromFailure 做恢复。
            return failAndRecover(Error::Timeout, HAL_I2C_GetError(hi2c_));
        }

        const uint32_t remain_ticks = timeout_ticks - elapsed_ticks;
        const uint32_t wait_result  = osThreadFlagsWait(TransferCompleteFlag, osFlagsWaitAny, remain_ticks);
        if (wait_result == osFlagsErrorTimeout)
        {
            return failAndRecover(Error::Timeout, HAL_I2C_GetError(hi2c_));
        }
        if (isThreadFlagsError(wait_result))
        {
            return failAndRecover(Error::InvalidContext, HAL_I2C_ERROR_NONE);
        }

        if (!completed_ || completed_transfer_id_ != transfer_id)
        {
            // 只接受当前事务的完成记录；旧事务晚到的线程标志直接丢弃。
            continue;
        }

        waiting_thread_ = nullptr;
        transmitting_   = false;
        completed_      = false;
        if (last_error_ != Error::None)
        {
            return failAndRecover(last_error_, last_hal_error_);
        }

        return true;
    }
}

void I2CBusDMA::completeFromISR(const bool success, const uint32_t hal_error)
{
    // 这里运行在 HAL 的中断回调上下文，只做状态落盘和唤醒等待线程。
    // 是否属于“当前事务”由等待侧根据 transfer id 再次确认，ISR 里不做复杂判断。
    completed_transfer_id_ = current_transfer_id_;
    completed_             = true;
    last_hal_error_        = hal_error;
    last_error_            = success ? Error::None : Error::HalError;

    if (waiting_thread_ != nullptr)
    {
        (void) osThreadFlagsSet(waiting_thread_, TransferCompleteFlag);
    }
}

void I2CBusDMA::clearTransferState()
{
    waiting_thread_        = nullptr;
    transmitting_          = false;
    completed_             = false;
    current_transfer_id_   = 0U;
    completed_transfer_id_ = 0U;
}

bool I2CBusDMA::failAndRecover(const Error error, const uint32_t hal_error)
{
    // 先保留这次失败的上层错误语义，再尽量把总线拉回可用状态。
    // 即使恢复成功，对外也应当仍然看到“这次事务失败了”，不能把失败语义吞掉。
    last_error_     = error;
    last_hal_error_ = hal_error;
    clearTransferState();

    if (!recover())
    {
        last_error_ = Error::RecoveryFailed;
        return false;
    }

    last_error_ = error;
    return false;
}

bool I2CBusDMA::registerInstance(I2CBusDMA* const instance)
{
    for (auto& slot : instances_)
    {
        if (slot == nullptr)
        {
            slot = instance;
            return true;
        }
    }
    return false;
}

extern "C" void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    // HAL 只给 C 风格全局回调，需要先反查 bus 对象再转发。
    if (auto* bus = I2CBusDMA::fromHandle(hi2c); bus != nullptr)
        bus->onTxCompleteFromISR();
}

extern "C" void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    if (auto* bus = I2CBusDMA::fromHandle(hi2c); bus != nullptr)
        bus->onRxCompleteFromISR();
}

extern "C" void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    if (auto* bus = I2CBusDMA::fromHandle(hi2c); bus != nullptr)
        bus->onTxCompleteFromISR();
}

extern "C" void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    if (auto* bus = I2CBusDMA::fromHandle(hi2c); bus != nullptr)
        bus->onRxCompleteFromISR();
}

extern "C" void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c)
{
    if (auto* bus = I2CBusDMA::fromHandle(hi2c); bus != nullptr)
        bus->onErrorFromISR();
}

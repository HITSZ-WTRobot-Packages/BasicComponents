/**
 * @file    gpio_driver.c
 * @author  syhanjin
 * @date    2026-01-24
 * @brief   GPIO / EXTI 封装实现
 *
 * 这里主要负责把 HAL 的 EXTI 回调变成可注册、可计数、可携带用户数据的形式。
 */

#include "gpio_driver.h"
#ifdef __cplusplus
extern "C"
{
#endif

#ifdef USE_EXTI
#    include "cmsis_compiler.h"
// 以 pin 位序号作为数组索引，避免中断上下文中的查表开销。
static struct
{
    const GPIO_t* gpio;     ///< 被注册的 GPIO
    uint32_t      counter;  ///< 触发次数
    void*         data;     ///< 回调对应的数据
    EXTI_Callback callback; ///< 回调
} EXTI_CallbackMap[16];

/**
 * @brief 把 GPIO pin 转成索引。
 *
 * pin 必须是单 bit 值，例如 GPIO_PIN_0、GPIO_PIN_1。
 */
size_t gpio_pin_to_index(uint16_t pin)
{
#    if defined(__GNUC__)
    // GCC/Clang 可直接取最低位 1 的位置。
    return __builtin_ctz(pin);
#    else
    // 通用 CMSIS 实现
    return __CLZ(__RBIT(pin));
#    endif
}

/**
 * @brief 清零某个 EXTI 线的触发计数。
 */
void GPIO_EXTI_ResetCounter(const GPIO_t* gpio)
{
    const size_t index              = gpio_pin_to_index(gpio->pin);
    EXTI_CallbackMap[index].counter = 0;
}

/**
 * @brief 注册某个 GPIO 的 EXTI 回调。
 *
 * 这里按 pin 位序号直接索引数组，速度比 map 查找更适合中断上下文。
 */
void GPIO_EXTI_RegisterCallback(const GPIO_t* gpio, const EXTI_Callback callback, void* data)
{
    const size_t index               = gpio_pin_to_index(gpio->pin);
    EXTI_CallbackMap[index].gpio     = gpio;
    EXTI_CallbackMap[index].callback = callback;
    EXTI_CallbackMap[index].counter  = 0;
    EXTI_CallbackMap[index].data     = data;
}

/**
 * @brief 取消注册 EXTI 回调。
 */
void GPIO_EXTI_UnregisterCallback(const GPIO_t* gpio)
{
    const size_t index               = gpio_pin_to_index(gpio->pin);
    EXTI_CallbackMap[index].gpio     = NULL;
    EXTI_CallbackMap[index].callback = NULL;
    EXTI_CallbackMap[index].counter  = 0;
}

/**
 * @brief 统一的 EXTI 中断分发入口。
 *
 * 应在 `HAL_GPIO_EXTI_Callback` 中调用，把 HAL 回调转发到本库。
 */
void GPIO_EXTI_Callback(const uint16_t GPIO_Pin)
{
    const size_t index = gpio_pin_to_index(GPIO_Pin);
    if (EXTI_CallbackMap[index].gpio != NULL && EXTI_CallbackMap[index].callback != NULL)
    {
        EXTI_CallbackMap[index].counter++;
        // 传入触发计数，方便上层做边沿计数或按键去抖。
        EXTI_CallbackMap[index].callback(EXTI_CallbackMap[index].gpio,
                                         EXTI_CallbackMap[index].counter,
                                         EXTI_CallbackMap[index].data);
    }
}
#endif

#ifdef __cplusplus
}
#endif

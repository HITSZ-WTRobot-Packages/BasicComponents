/**
 * @file    gpio_driver.c
 * @author  syhanjin
 * @date    2026-01-24
 */

#include "gpio_driver.h"
#ifdef __cplusplus
extern "C"
{
#endif

#ifdef USE_EXTI
#    include "cmsis_compiler.h"
static struct
{
    const GPIO_t* gpio;     ///< 被注册的 GPIO
    uint32_t      counter;  ///< 触发次数
    void*         data;     ///< 回调对应的数据
    EXTI_Callback callback; ///< 回调
} EXTI_CallbackMap[16];

size_t gpio_pin_to_index(uint16_t pin)
{
#    if defined(__GNUC__)
    return __builtin_ctz(pin);
#    else
    // 通用 CMSIS 实现
    return __CLZ(__RBIT(pin));
#    endif
}

/**
 * 清零 EXTI 触发计数器
 * @param gpio gpio
 */
void GPIO_EXTI_ResetCounter(const GPIO_t* gpio)
{
    const size_t index              = gpio_pin_to_index(gpio->pin);
    EXTI_CallbackMap[index].counter = 0;
}

/**
 * 注册 EXTI 回调
 * @param gpio gpio
 * @param callback 回调函数
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
 * 取消注册 EXTI 回调
 * @param gpio gpio
 */
void GPIO_EXTI_UnregisterCallback(const GPIO_t* gpio)
{
    const size_t index               = gpio_pin_to_index(gpio->pin);
    EXTI_CallbackMap[index].gpio     = NULL;
    EXTI_CallbackMap[index].callback = NULL;
    EXTI_CallbackMap[index].counter  = 0;
}

/**
 * EXTI 回调处理函数
 *
 * @note 在 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin); 中调用本函数
 *       因为 EXTI 的回调无法被注册
 * @param GPIO_Pin
 */
void GPIO_EXTI_Callback(const uint16_t GPIO_Pin)
{
    const size_t index = gpio_pin_to_index(GPIO_Pin);
    if (EXTI_CallbackMap[index].gpio != NULL && EXTI_CallbackMap[index].callback != NULL)
    {
        EXTI_CallbackMap[index].counter++;
        EXTI_CallbackMap[index].callback(EXTI_CallbackMap[index].gpio,
                                         EXTI_CallbackMap[index].counter,
                                         EXTI_CallbackMap[index].data);
    }
}
#endif

#ifdef __cplusplus
}
#endif
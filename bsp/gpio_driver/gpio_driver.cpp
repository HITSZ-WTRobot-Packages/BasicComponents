/**
 * @file    gpio_driver.cpp
 * @author  syhanjin
 * @date    2026-06-30
 * @brief   GPIO 面向对象封装 — EXTI 实现
 *
 * 独立的 EXTI 回调系统实现，使用 bsp::gpio 新类型，
 * 与 gpio_driver.c 中的旧实现互不依赖。
 */
#include "gpio_driver.hpp"

#ifdef USE_EXTI
#    include "cmsis_compiler.h"

namespace bsp::gpio
{

namespace
{
// 以 pin 位序号作为数组索引，避免中断上下文中的查表开销。
struct ExtiSlot
{
    const GpioPin* gpio;     ///< 被注册的 GPIO
    uint32_t       counter;  ///< 触发次数
    ExtiCallback   callback; ///< 回调
};

ExtiSlot g_exti_callback_map[16];

/**
 * @brief 把 GPIO pin 转成索引。
 *
 * pin 必须是单 bit 值，例如 GPIO_PIN_0、GPIO_PIN_1。
 */
size_t pin_to_index(const uint16_t pin)
{
#    if defined(__GNUC__)
    return __builtin_ctz(pin);
#    else
    return __CLZ(__RBIT(pin));
#    endif
}
} // namespace

void RegisterExtiCallback(const GpioPin* gpio, const ExtiCallback callback)
{
    const size_t index                   = pin_to_index(gpio->pin);
    g_exti_callback_map[index].gpio     = gpio;
    g_exti_callback_map[index].callback = callback;
    g_exti_callback_map[index].counter  = 0;
}

void UnregisterExtiCallback(const GpioPin* gpio)
{
    const size_t index                   = pin_to_index(gpio->pin);
    g_exti_callback_map[index].gpio     = nullptr;
    g_exti_callback_map[index].callback = nullptr;
    g_exti_callback_map[index].counter  = 0;
}

void DispatchExtiInterrupt(const uint16_t GPIO_Pin)
{
    const size_t index = pin_to_index(GPIO_Pin);
    if (g_exti_callback_map[index].gpio != nullptr && g_exti_callback_map[index].callback != nullptr)
    {
        g_exti_callback_map[index].counter++;
        g_exti_callback_map[index].callback(g_exti_callback_map[index].gpio,
                                            g_exti_callback_map[index].counter);
    }
}

} // namespace bsp::gpio
#endif // USE_EXTI

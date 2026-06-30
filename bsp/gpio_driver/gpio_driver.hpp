/**
 * @file    gpio_driver.hpp
 * @author  syhanjin
 * @date    2026-06-30
 * @brief   GPIO 面向对象封装（C++）
 *
 * 此头文件提供面向对象的 GPIO 操作 API，推荐用于所有新代码。
 * 旧 C 风格 API 在 gpio_driver.h 中保留但已标记为弃用。
 *
 * --------------------------------------------------------------------------
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * Project repository: https://github.com/HITSZ-WTRobot-Packages/BasicComponents
 */
#pragma once
#include "main.h"

#ifndef __cplusplus
#    error "GpioPin driver does not support standard C"
#endif

#ifndef HAL_GPIO_MODULE_ENABLED
#    error "GPIO driver requires HAL GPIO enabled. Please enable GPIO in CubeMX."
#endif

// 开启外部中断封装；如果不需要 EXTI，可关闭以减少代码体积。
#define USE_EXTI

namespace bsp::gpio
{

/**
 * @brief 面向对象的 GPIO 引脚句柄。
 *
 * 轻量值类型，可复制，public 成员兼容 brace 初始化。
 * 推荐使用此类型替代 GPIO_t。
 */
struct GpioPin
{
    GPIO_TypeDef* port      = nullptr;
    uint16_t      pin       = 0;
    void*         user_data = nullptr;

    [[nodiscard]] GPIO_PinState read() const { return HAL_GPIO_ReadPin(port, pin); }

    void write(const GPIO_PinState state) const { HAL_GPIO_WritePin(port, pin, state); }

    void set() const { HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET); }

    void reset() const { HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET); }

    void toggle() const { HAL_GPIO_TogglePin(port, pin); }
};

#define GpioPinWithName(__NAME__) { __NAME__##_GPIO_Port, __NAME__##_Pin }

#ifdef USE_EXTI

/// EXTI 回调签名。上下文通过 gpio->user_data 获取。
using ExtiCallback = void (*)(const GpioPin* gpio, uint32_t counter);

/// 注册某个 GPIO 的 EXTI 回调。回调上下文从 gpio->user_data 读取。
void RegisterExtiCallback(const GpioPin* gpio, ExtiCallback callback);

/// 取消注册 EXTI 回调。
void UnregisterExtiCallback(const GpioPin* gpio);

/// 统一的 EXTI 中断分发入口，应在 HAL_GPIO_EXTI_Callback 中调用。
void DispatchExtiInterrupt(uint16_t GPIO_Pin);

#endif // USE_EXTI

} // namespace bsp::gpio

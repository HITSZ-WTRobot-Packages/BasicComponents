/**
 * @file    gpio_driver.h
 * @author  syhanjin
 * @date    2025-09-10
 * @brief   GPIO / EXTI / PWM 的基础封装
 *
 * 这个模块把 HAL 的 GPIO、外部中断和 PWM 接口做了一层很薄的封装，核心目标是统一句柄形式，
 * 让上层代码只需要围绕 GPIO_t 和 PWM_t 这类结构体操作，而不必反复处理底层 HAL 句柄细节。
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
#ifndef GPIO_DRIVER_H
#define GPIO_DRIVER_H
#include "main.h"

#ifdef __cplusplus
extern "C"
{
#endif

// 开启外部中断封装；如果不需要 EXTI，可关闭以减少代码体积。
#define USE_EXTI

typedef struct
{
    /**
     * @brief GPIO 句柄。
     *
     * 这里把端口和引脚打包成一个轻量结构体，便于在中断回调和普通业务函数之间传递。
     */
    GPIO_TypeDef* port;
    uint16_t      pin;
} GPIO_t;

/**
 * @brief 读取 GPIO 输入状态。
 *
 * 直接转发到底层 HAL 接口。
 */
static inline GPIO_PinState GPIO_ReadPin(GPIO_t* hgpio)
{
    return HAL_GPIO_ReadPin(hgpio->port, hgpio->pin);
}

/**
 * @brief 写入 GPIO 输出状态。
 */
static inline void GPIO_WritePin(GPIO_t* hgpio, const GPIO_PinState PinState)
{
    HAL_GPIO_WritePin(hgpio->port, hgpio->pin, PinState);
}

/**
 * @brief 输出高电平。
 */
static inline void GPIO_SetPin(GPIO_t* hgpio)
{
    GPIO_WritePin(hgpio, GPIO_PIN_SET);
}

/**
 * @brief 输出低电平。
 */
static inline void GPIO_ResetPin(GPIO_t* hgpio)
{
    GPIO_WritePin(hgpio, GPIO_PIN_RESET);
}

/**
 * @brief 翻转 GPIO 输出状态。
 */
static inline void GPIO_TogglePin(GPIO_t* hgpio)
{
    HAL_GPIO_TogglePin(hgpio->port, hgpio->pin);
}

#ifdef USE_EXTI
/**
 * @brief EXTI 回调函数签名。
 *
 * 回调里带上触发计数和用户数据，通常用于去抖、边沿累计或者事件分发。
 */
typedef void (*EXTI_Callback)(const GPIO_t* gpio, uint32_t counter, void* data);
void GPIO_EXTI_RegisterCallback(const GPIO_t* gpio, EXTI_Callback callback, void* data);
void GPIO_EXTI_UnregisterCallback(const GPIO_t* gpio);
void GPIO_EXTI_Callback(const uint16_t GPIO_Pin);
#endif

#ifdef __cplusplus
}
#endif

#endif // GPIO_DRIVER_H

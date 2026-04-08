/**
 * @file    pwm.h
 * @author  syhanjin
 * @date    2025-09-10
 * @brief   TIM PWM 的基础封装
 *
 * 这个头文件只做最常见的 PWM 启停和占空比换算，目的是让上层代码不用反复接触 __HAL_TIM_* 宏。
 * 这样做的好处是调用方式统一，也更容易在代码里看出“这是一个 PWM 句柄，而不是裸 TIM 句柄”。
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
#ifndef PWM_H
#define PWM_H

#include "main.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    /**
     * @brief PWM 句柄。
     *
     * 一个 PWM_t 对应一个定时器通道，上层只需要传这个结构体即可操作输出。
     */
    TIM_HandleTypeDef* htim;
    uint32_t           channel;
} PWM_t;

/**
 * @brief 启动 PWM 输出。
 */
static inline void PWM_Start(PWM_t* hpwm)
{
    HAL_TIM_PWM_Start(hpwm->htim, hpwm->channel);
}

/**
 * @brief 停止 PWM 输出。
 */
static inline void PWM_Stop(PWM_t* hpwm)
{
    HAL_TIM_PWM_Stop(hpwm->htim, hpwm->channel);
}

/**
 * @brief 直接设置比较值。
 *
 * compare 超过 ARR 时不生效，避免写入非法值。
 */
static inline void PWM_SetCompare(PWM_t* hpwm, const uint32_t compare)
{
    // 只允许写入不超过 ARR 的比较值，避免生成非法占空比。
    if (compare <= __HAL_TIM_GET_AUTORELOAD(hpwm->htim))
        __HAL_TIM_SET_COMPARE(hpwm->htim, hpwm->channel, compare);
}

/**
 * @brief 按占空比设置 PWM 输出。
 *
 * duty_circle 的有效范围是 [0, 1]；越界时会钳位到边界值。
 */
static inline void PWM_SetDutyCircle(PWM_t* hpwm, const float duty_circle)
{
    // duty_circle 视为 [0, 1] 区间；越界时直接钳位。
    if (duty_circle < 0.0f)
        PWM_SetCompare(hpwm, 0);
    else if (duty_circle > 1.0f)
        PWM_SetCompare(hpwm, __HAL_TIM_GET_AUTORELOAD(hpwm->htim));
    else
        PWM_SetCompare(hpwm, __HAL_TIM_GET_AUTORELOAD(hpwm->htim) * duty_circle + 0.5);
}

#ifdef __cplusplus
}
#endif

#endif // PWM_H

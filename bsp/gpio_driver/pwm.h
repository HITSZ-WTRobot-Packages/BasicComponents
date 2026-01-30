/**
 * @file    pwm.h
 * @author  syhanjin
 * @date    2025-09-10
 * @brief   tim driver
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
 * Project repository: https://github.com/HITSZ-WTR2026/bsp_drivers
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
    TIM_HandleTypeDef* htim;
    uint32_t           channel;
} PWM_t;

static inline void PWM_Start(PWM_t* hpwm)
{
    HAL_TIM_PWM_Start(hpwm->htim, hpwm->channel);
}

static inline void PWM_Stop(PWM_t* hpwm)
{
    HAL_TIM_PWM_Stop(hpwm->htim, hpwm->channel);
}

static inline void PWM_SetCompare(PWM_t* hpwm, const uint32_t compare)
{
    if (compare <= __HAL_TIM_GET_AUTORELOAD(hpwm->htim))
        __HAL_TIM_SET_COMPARE(hpwm->htim, hpwm->channel, compare);
}

static inline void PWM_SetDutyCircle(PWM_t* hpwm, const float duty_circle)
{
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

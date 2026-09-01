/**
 * @file    can_driver.hpp
 * @author  syhanjin
 * @date    2025-09-04
 * @brief   CAN wrapper based on HAL library
 *
 * 本驱动是对 HAL 库的一层简要封装
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

#if !defined(HAL_CAN_MODULE_ENABLED) && !defined(HAL_FDCAN_MODULE_ENABLED)
#    error "REQUIRE_HAL_CAN is set but neither HAL_CAN_MODULE_ENABLED nor HAL_FDCAN_MODULE_ENABLED is enabled"
#endif
#if defined(HAL_CAN_MODULE_ENABLED) && defined(HAL_FDCAN_MODULE_ENABLED)
#    error ""
#endif

#ifdef HAL_FDCAN_MODULE_ENABLED
// 当启用 FDCAN 时，定义 bxCAN 的结构以保证用 bxCAN 实现的上层能正常运行
using CAN_HandleTypeDef = FDCAN_HandleTypeDef;
#    include "can_hal_def.h"
#    if !(USE_HAL_FDCAN_REGISTER_CALLBACKS)
#        error "CAN driver requires HAL FDCAN RegisterCallback enabled. Please enable it in CubeMX: Project Manager -> Advanced Settings -> Register Callbacks -> FDCAN"
#    endif

#    define CAN_DRIVER_FDCAN_ENABLED 1
#endif

#ifdef HAL_CAN_MODULE_ENABLED
#    if !(USE_HAL_CAN_REGISTER_CALLBACKS)
#        error "CAN driver requires HAL CAN RegisterCallback enabled. Please enable it in CubeMX: Project Manager -> Advanced Settings -> Register Callbacks -> CAN"
#    endif
#endif

#define CAN_SEND_FAILED (0xFFFF)

// 一条 CAN 最多注册的回调数量
#ifndef CAN_MAX_CALLBACK_NUM
#    define CAN_MAX_CALLBACK_NUM (14)
#endif

// CAN 数量
#ifndef CAN_NUM
#    if defined(CAN3) || defined(FDCAN3)
#        define CAN_NUM (3)
#    elif defined(CAN2) || defined(FDCAN2)
#        define CAN_NUM (2)
#    elif defined(CAN1) || defined(FDCAN1)
#        define CAN_NUM (1)
#    else
#        define CAN_NUM (0)
#    endif
#endif

#if CAN_DRIVER_FDCAN_ENABLED
// FDCAN 发送 软件缓冲区大小
#    ifndef FDCAN_TX_QUEUE_SIZE
#        define FDCAN_TX_QUEUE_SIZE (0)
#    endif
#    if FDCAN_TX_QUEUE_SIZE > 0
#        define FDCAN_ENABLE_SOFT_TX_QUEUE 1
#    endif
#else
// CAN 发送 软件缓冲区大小
#    ifndef CAN_TX_QUEUE_SIZE
#        define CAN_TX_QUEUE_SIZE (8)
#    endif
#endif

typedef void (*CAN_FifoReceiveCallback_t)(const CAN_HandleTypeDef*   hcan,
                                          const CAN_RxHeaderTypeDef* header,
                                          const uint8_t*             data);

// TODO: 增加更完善的错误返回逻辑

uint32_t CAN_SendMessage(CAN_HandleTypeDef*         hcan,
                         const CAN_TxHeaderTypeDef* header,
                         const uint8_t              data[]);

void CAN_InitMainCallback(CAN_HandleTypeDef* hcan);

void CAN_Start(CAN_HandleTypeDef* hcan, uint32_t ActiveITs);

void CAN_RegisterCallback(CAN_HandleTypeDef* hcan, CAN_FifoReceiveCallback_t callback);

// void CAN_UnregisterCallback(CAN_HandleTypeDef* hcan, uint32_t filter_match_index);
// void CAN_Fifo0ReceiveCallback(CAN_HandleTypeDef* hcan);
// void CAN_Fifo1ReceiveCallback(CAN_HandleTypeDef* hcan);

#if CAN_DRIVER_FDCAN_ENABLED
typedef void (*FDCAN_FifoReceiveCallback_t)(const FDCAN_HandleTypeDef*   hcan,
                                            const FDCAN_RxHeaderTypeDef* header,
                                            const uint8_t*               data);

uint32_t FDCAN_SendMessage(FDCAN_HandleTypeDef*         hcan,
                           const FDCAN_TxHeaderTypeDef* header,
                           const uint8_t                data[]);

void FDCAN_InitMainCallback(FDCAN_HandleTypeDef* hcan);

void FDCAN_Start(FDCAN_HandleTypeDef* hcan, uint32_t ActiveITs);

void FDCAN_RegisterCallback(FDCAN_HandleTypeDef* hcan, FDCAN_FifoReceiveCallback_t callback);
#endif
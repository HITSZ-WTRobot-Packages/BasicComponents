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

#if defined(HAL_FDCAN_MODULE_ENABLED)
#    error "this can_driver do not support FDCAN"
#endif

#if !(USE_HAL_CAN_REGISTER_CALLBACKS)
#    error "CAN driver requires HAL CAN RegisterCallback enabled. Please enable it in CubeMX: Project Manager -> Advanced Settings -> Register Callbacks -> CAN"
#endif

#define CAN_SEND_FAILED (0xFFFF)

// 一条 CAN 最多注册的回调数量
#ifndef CAN_MAX_CALLBACK_NUM
#    define CAN_MAX_CALLBACK_NUM (14)
#endif

// CAN 数量
#ifndef CAN_NUM
#    define CAN_NUM (2)
#endif

// CAN 发送 软件缓冲区大小
#ifndef CAN_TX_QUEUE_SIZE
#    define CAN_TX_QUEUE_SIZE (8)
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

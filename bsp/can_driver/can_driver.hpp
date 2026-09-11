/**
 * @file    can_driver.hpp
 * @author  syhanjin
 * @date    2025-09-04
 * @brief   CAN wrapper based on HAL library
 *
 * 本驱动是对 HAL 库的一层简要封装。根据 HAL 启用的外设自动选择后端：
 * - 定义 HAL_FDCAN_MODULE_ENABLED 时使用 FDCAN 外设，对外提供 FDCAN_* 原生接口，
 *   同时保留仅支持 classic 帧的 CAN_* 兼容接口；
 * - 定义 HAL_CAN_MODULE_ENABLED 时使用传统 bxCAN 外设，只提供 CAN_* 接口。
 *
 * 由于 HAL 对同一事件只允许注册一个回调函数，接收方向统一由主回调进入驱动内部的
 * 回调表，再分发给业务模块注册的多个回调函数（见 CAN_RegisterCallback）。
 *
 * TODO: 统一化 FDCAN 与 bxCAN 的接口，避免 bxCAN 类 API 与 FDCAN 类 API 互转造成的性能损失
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

/**
 * 发送失败时 CAN_SendMessage() / FDCAN_SendMessage() 的返回值
 */
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
// FDCAN 发送软件缓冲区大小（0 表示不启用软件队列，硬件 Tx FIFO 满时直接发送失败）
#    ifndef FDCAN_TX_QUEUE_SIZE
#        define FDCAN_TX_QUEUE_SIZE (0)
#    endif
#    if FDCAN_TX_QUEUE_SIZE > 0
#        define FDCAN_ENABLE_SOFT_TX_QUEUE 1
#    endif
#else
// CAN 发送软件缓冲区大小（队列满时覆盖最早的帧）
#    ifndef CAN_TX_QUEUE_SIZE
#        define CAN_TX_QUEUE_SIZE (8)
#    endif
#endif

/**
 * bxCAN 风格接收回调函数类型
 * @param hcan 触发接收的 CAN 句柄
 * @param header 接收帧头，包含标准/扩展 ID、数据/远程帧标志与 DLC
 * @param data 接收数据，有效长度由 header->DLC 决定（不超过 8 字节）
 */
typedef void (*CAN_FifoReceiveCallback_t)(const CAN_HandleTypeDef*   hcan,
                                          const CAN_RxHeaderTypeDef* header,
                                          const uint8_t*             data);

// TODO: 增加更完善的错误返回逻辑

/**
 * 发送一条 CAN 消息
 * @param hcan can handle
 * @param header 发送帧头，包含 ID 类型、数据/远程帧标志与 DLC
 * @param data 待发送数据，长度由 header->DLC 决定（不超过 8 字节）
 * @note 线程安全：内部会短暂关闭中断
 * @note DLC 大于 8 时不会发出空帧，直接返回 CAN_SEND_FAILED
 * @return 成功时返回发送使用的 mailbox 编号（FDCAN 兼容接口返回 0），
 *         失败时返回 CAN_SEND_FAILED
 */
uint32_t CAN_SendMessage(CAN_HandleTypeDef*         hcan,
                         const CAN_TxHeaderTypeDef* header,
                         const uint8_t              data[]);

/**
 * 注册 CAN 主回调函数
 *
 * 需在 HAL_CAN_Init() 之后、CAN_Start() 之前调用，注册后驱动才能收到接收与发送完成中断。
 * @param hcan can handle
 */
void CAN_InitMainCallback(CAN_HandleTypeDef* hcan);

/**
 * 启动 CAN 并开启中断
 * @param hcan can handle
 * @param ActiveITs 需要额外开启的中断，如
 *        CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING。
 *        bxCAN 后端使用 HAL 原生宏；FDCAN 后端使用 can_hal_def.h 提供的兼容
 *        别名（取值等于对应的 FDCAN_IT_*），直接传 FDCAN_IT_* 亦可。
 * @note 驱动会额外开启发送完成中断，以驱动软件发送队列
 */
void CAN_Start(CAN_HandleTypeDef* hcan, uint32_t ActiveITs);

/**
 * 注册 CAN 接收回调
 *
 * 同一条总线上可以注册多个回调，收到帧后会依次调用。
 * @param hcan can handle
 * @param callback 接收回调函数，不可为空
 * @note 非线程安全，建议在总线启动前完成注册
 */
void CAN_RegisterCallback(CAN_HandleTypeDef* hcan, CAN_FifoReceiveCallback_t callback);

// void CAN_UnregisterCallback(CAN_HandleTypeDef* hcan, uint32_t filter_match_index);
// void CAN_Fifo0ReceiveCallback(CAN_HandleTypeDef* hcan);
// void CAN_Fifo1ReceiveCallback(CAN_HandleTypeDef* hcan);

#if CAN_DRIVER_FDCAN_ENABLED
/**
 * FDCAN 风格接收回调函数类型
 * @param hcan 触发接收的 FDCAN 句柄
 * @param header 接收帧头，包含 ID 类型、帧格式与 DataLength 编码
 * @param data 接收数据，有效长度由 header->DataLength 决定（最长 64 字节）
 */
typedef void (*FDCAN_FifoReceiveCallback_t)(const FDCAN_HandleTypeDef*   hcan,
                                            const FDCAN_RxHeaderTypeDef* header,
                                            const uint8_t*               data);

/**
 * 发送一条 FDCAN 消息
 * @param hcan can handle
 * @param header 发送帧头，包含 ID 类型、帧格式与 DataLength 编码
 * @param data 待发送数据，长度由 header->DataLength 决定
 * @note 线程安全：内部会短暂关闭中断
 * @note DataLength 必须是合法编码（FDCAN_DLC_BYTES_0 ~ FDCAN_DLC_BYTES_64），
 *       非法编码不会发出空帧，直接返回 CAN_SEND_FAILED
 * @note 启用软件发送队列（FDCAN_TX_QUEUE_SIZE > 0）时按调用顺序发送：
 *       队列未清空之前的新帧一律入队，不会插到更早提交的帧之前
 * @return 成功返回 0，失败返回 CAN_SEND_FAILED
 */
uint32_t FDCAN_SendMessage(FDCAN_HandleTypeDef*         hcan,
                           const FDCAN_TxHeaderTypeDef* header,
                           const uint8_t                data[]);

/**
 * 注册 FDCAN 主回调函数
 *
 * 注册 Rx FIFO0/FIFO1 与软件发送队列使用的 Tx FIFO 空闲回调，
 * 需在 HAL_FDCAN_Init() 之后、FDCAN_Start() 之前调用。
 * @param hcan can handle
 */
void FDCAN_InitMainCallback(FDCAN_HandleTypeDef* hcan);

/**
 * 启动 FDCAN 并开启中断
 * @param hcan can handle
 * @param ActiveITs 需要额外开启的中断，如 FDCAN_IT_RX_FIFO0_NEW_MESSAGE
 * @note 驱动会额外开启传输完成中断，以驱动软件发送队列
 */
void FDCAN_Start(FDCAN_HandleTypeDef* hcan, uint32_t ActiveITs);

/**
 * 注册 FDCAN 接收回调
 *
 * 同一条总线上可以注册多个回调，收到帧后会依次调用。
 * @param hcan can handle
 * @param callback 接收回调函数，不可为空
 * @note 非线程安全，建议在总线启动前完成注册
 */
void FDCAN_RegisterCallback(FDCAN_HandleTypeDef* hcan, FDCAN_FifoReceiveCallback_t callback);

/**
 * bxCAN 过滤器配置兼容接口
 *
 * 将 bxCAN 风格的 CAN_FilterTypeDef 转换为 FDCAN 滤波器并写入。
 * 支持 32 位 ID 掩码 / ID 列表（bxCAN 的 FR1/FR2 字布局，含只填单个半字的用法）。
 * bxCAN 的 16 位 scale 在一个 bank 内放两组 16 位值对，单个 FDCAN 滤波器元素
 * 表达不了，一律返回 HAL_ERROR（避免静默丢弃部分 ID）。
 *
 * @note 需要先在 CubeMX 中为该 ID 类型分配滤波器元素（FDCAN 的
 *       Std Filters Nbr / Ext Filters Nbr）。分配数量为 0 时 HAL 仍会写入
 *       message RAM 但硬件不评估滤波器，本接口会直接返回 HAL_ERROR。
 * @param hcan can handle
 * @param filterConfig bxCAN 风格过滤器配置
 * @return 配置结果，成功返回 HAL_OK
 */
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef*       hcan,
                                       const CAN_FilterTypeDef* filterConfig);
#endif
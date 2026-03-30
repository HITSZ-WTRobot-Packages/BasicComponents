/**
 * @file    can_driver.cpp
 * @author  syhanjin
 * @date    2025-09-04
 * @brief
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
#include "can_driver.hpp"

#include "RingBuffer.hpp"
#include "isr_lock.h"

#include <cassert>
#include <cstring>

namespace
{

struct CAN_MessageDef
{
    CAN_TxHeaderTypeDef header;
    uint8_t             data[8];
};

struct CAN_CallbackMap
{
    CAN_HandleTypeDef*        hcan{ nullptr };
    CAN_FifoReceiveCallback_t callbacks[CAN_MAX_CALLBACK_NUM]{};
    uint32_t                  callback_count{ 0 };

    libs::RingBuffer<CAN_MessageDef, CAN_TX_QUEUE_SIZE, true> buffer;
};

CAN_CallbackMap maps[CAN_NUM];
size_t          map_size = 0;

CAN_CallbackMap* get_map(const CAN_HandleTypeDef* hcan)
{
    for (size_t i = 0; i < map_size; i++)
        if (maps[i].hcan == hcan)
            return &maps[i];

    return nullptr;
}
} // namespace

/**
 * 发送一条 CAN 消息
 * @param hcan can handle
 * @param header CAN_TxHeaderTypeDef
 * @param data 数据
 * @note 本身想做成内联展开，但是必须写到 .h 文件，调研发现性能损失不大，所以直接放到此处
 * @attention 本函数大部分情况是线程安全的，少数情况（中断被中断打断）会出现不安全的情况。
 * @return mailbox, 0xFFFF 表示发送失败
 */
uint32_t CAN_SendMessage(CAN_HandleTypeDef*         hcan,
                         const CAN_TxHeaderTypeDef* header,
                         const uint8_t              data[])
{
    uint32_t mailbox = CAN_SEND_FAILED;

    // 直接锁定中断，这里锁定了中断就无法进行任务调度. 裸机与 RTOS 都适用
    ISRGuard guard;
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
    {
        // 直接执行发送
        if (HAL_CAN_AddTxMessage(hcan, header, data, &mailbox) != HAL_OK)
        {
            Error_Handler();
        }
    }
    else
    {
        // 已满，加入队列
        get_map(hcan)->buffer.push(
                [&](CAN_MessageDef& msg)
                {
                    assert(header->DLC <= 8);

                    msg.header = *header;
                    memcpy(msg.data, data, header->DLC);
                    memset(msg.data + header->DLC, 0, 8 - header->DLC);
                });
    }

    return mailbox;
}

/**
 * CAN 初始化
 * @param hcan can handle
 * @param ActiveITs CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING
 */
void CAN_Start(CAN_HandleTypeDef* hcan, const uint32_t ActiveITs)
{
    if (HAL_CAN_Start(hcan) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(hcan, ActiveITs | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * 注册 CAN Fifo 处理回调
 *
 * @attention 本函数非线程安全，调用时请注意
 * @param hcan hcan
 * @param callback 回调函数指针
 */
void CAN_RegisterCallback(CAN_HandleTypeDef* hcan, const CAN_FifoReceiveCallback_t callback)
{
    CAN_CallbackMap* map = get_map(hcan);

    if (map == nullptr)
    {
        if (map_size >= CAN_NUM)
        {
            Error_Handler();
            return;
        }
        maps[map_size] = (CAN_CallbackMap){ .hcan = hcan, .callbacks = {}, .callback_count = 0 };
        map            = &maps[map_size];
        map_size++;
    }
    if (map->callback_count < CAN_MAX_CALLBACK_NUM)
        map->callbacks[map->callback_count++] = callback;
    else
        Error_Handler();
}
/**
 * 取消注册 CAN Fifo 处理回调
 *
 * @attention 本函数非线程安全，调用时请注意
 * @param hcan can handle
 * @param filter_match_index 需要取消注册对应的过滤器对应的 id
 */
// void CAN_UnregisterCallback(CAN_HandleTypeDef* hcan, const uint32_t filter_match_index)
// {
//     CAN_FifoReceiveCallback_t* callbacks = get_callbacks(hcan);
//     if (callbacks != NULL)
//         callbacks[filter_match_index] = NULL;
// }

/**
 * CAN Fifo0 接收处理函数
 *
 * 本函数将会根据 hcan 和 rx_header 内部的 filter_id 来调用对应的回调函数
 * @param hcan can handle
 */
void CAN_Fifo0ReceiveCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t             data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
    {
        Error_Handler();
        return;
    }
    const CAN_CallbackMap* map = get_map(hcan);
    if (map != nullptr)
        for (size_t i = 0; i < map->callback_count; i++)
            map->callbacks[i](hcan, &header, data);
}
/**
 * CAN Fifo1 接收处理函数
 *
 * 本函数将会根据 hcan 和 rx_header 内部的 filter_id 来调用对应的回调函数
 * @param hcan can handle
 */
void CAN_Fifo1ReceiveCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t             data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &header, data) != HAL_OK)
    {
        Error_Handler();
        return;
    }
    const CAN_CallbackMap* map = get_map(hcan);
    if (map != nullptr)
        for (size_t i = 0; i < map->callback_count; i++)
            map->callbacks[i](hcan, &header, data);
}

void CAN_TxMailboxCpltCallback(CAN_HandleTypeDef* hcan)
{
    auto map = get_map(hcan);
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0 && !map->buffer.empty())
    {
        uint32_t mailbox = CAN_SEND_FAILED;

        const auto msg = map->buffer.pop();
        if (HAL_CAN_AddTxMessage(hcan, &msg->header, msg->data, &mailbox) != HAL_OK)
        {
            Error_Handler();
        }
    }
}

void CAN_InitMainCallback(CAN_HandleTypeDef* hcan)
{
    assert(hcan != nullptr);

    HAL_CAN_RegisterCallback(hcan, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, CAN_Fifo0ReceiveCallback);
    HAL_CAN_RegisterCallback(hcan, HAL_CAN_RX_FIFO1_MSG_PENDING_CB_ID, CAN_Fifo1ReceiveCallback);
    HAL_CAN_RegisterCallback(hcan, HAL_CAN_TX_MAILBOX0_COMPLETE_CB_ID, CAN_TxMailboxCpltCallback);
    HAL_CAN_RegisterCallback(hcan, HAL_CAN_TX_MAILBOX1_COMPLETE_CB_ID, CAN_TxMailboxCpltCallback);
    HAL_CAN_RegisterCallback(hcan, HAL_CAN_TX_MAILBOX2_COMPLETE_CB_ID, CAN_TxMailboxCpltCallback);
}
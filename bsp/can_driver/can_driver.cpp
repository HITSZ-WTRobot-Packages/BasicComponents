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

/**
 * 储存于软件缓冲区的 CAN 消息类型
 *
 * 包括 TxHeader 和 至多 8 bytes 的数据
 */
struct CAN_MessageDef
{
    CAN_TxHeaderTypeDef header;
    uint8_t             data[8];
};

/**
 * CAN 回调函数表
 *
 * @note 由于 HAL 只允许将一个函数作为回调函数，如果想在一条总线上处理不同种类的
 *       信息（有多个不同的回调函数），就必须要通过一个主回调函数进行分发
 *
 * @note STM32 的 CAN mailbox 数量往往有限，但是在很短的时间内可能连续发送多条消息
 *       自带的 mailbox 无法满足要求，故需要做一个软件缓冲区来临时储存溢出的消息
 */
struct CAN_CallbackMap
{
    CAN_HandleTypeDef*        hcan{ nullptr };
    CAN_FifoReceiveCallback_t callbacks[CAN_MAX_CALLBACK_NUM]{};
    uint32_t                  callback_count{ 0 };
    // 使用环形缓冲区实现发送队列，队列长度 CAN_TX_QUEUE_SIZE，Overwrite=true
    // 当队列满时会丢弃最早的帧
    libs::RingBuffer<CAN_MessageDef, CAN_TX_QUEUE_SIZE, true> buffer;
};

// 根据 CAN 实例的数量定义回调表
// CAN 实例的数量取决于芯片型号，且无法在编译器预知，故在 .hpp 内通过宏定义
CAN_CallbackMap maps[CAN_NUM];
size_t          map_size = 0;

// 根据 can handle 的指针查找 can map
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
 * @note 本函数是线程安全的
 * @return mailbox, 0xFFFF 表示发送失败
 */
uint32_t CAN_SendMessage(CAN_HandleTypeDef*         hcan,
                         const CAN_TxHeaderTypeDef* header,
                         const uint8_t              data[])
{
    // 储存发送该条消息使用的 CAN mailbox
    uint32_t mailbox = CAN_SEND_FAILED;

    // 直接锁定中断，这里锁定了中断就无法进行任务调度. 裸机与 RTOS 都适用
    ISRGuard guard;
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
    {
        // 直接执行发送
        if (HAL_CAN_AddTxMessage(hcan, header, data, &mailbox) != HAL_OK)
        {
            // TODO: 这里理应有更好的办法，而不是直接进入死循环
            Error_Handler();
        }
    }
    else
    {
        // TODO: fix bug: 当总线未注册回调函数，但连续发送进入该分支时出现 UB
        // 已满，加入队列
        get_map(hcan)->buffer.push(
                // 这里通过构造工厂函数的方式来避免额外值拷贝
                [&](CAN_MessageDef& msg)
                {
                    assert(header->DLC <= 8);

                    msg.header = *header;
                    // 分两次 memcpy 保证 data 的数据都有效
                    memcpy(msg.data, data, header->DLC);
                    memset(msg.data + header->DLC, 0, 8 - header->DLC);
                });
    }
    // 返回邮箱
    // TODO: 修复在邮箱已满后加入队列发送的 bug
    return mailbox;
}

/**
 * CAN 初始化
 * @param hcan can handle
 * @param ActiveITs CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING
 */
void CAN_Start(CAN_HandleTypeDef* hcan, const uint32_t ActiveITs)
{
    // 启动 CAN
    if (HAL_CAN_Start(hcan) != HAL_OK)
    {
        Error_Handler();
    }

    // 开启 CAN 中断
    // 使用 FIFO0 / FIFO1 由用户决定；发送队列实现依赖 TX 中断，所以必须开启
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
    // 查找回调函数表
    CAN_CallbackMap* map = get_map(hcan);

    if (map == nullptr)
    {
        if (map_size >= CAN_NUM)
        {
            // 仅当 CAN_NUM 配置错误时可能触发，此时进入死循环
            Error_Handler();
            return;
        }
        // 如果表未创建则新建一个
        maps[map_size] = (CAN_CallbackMap){ .hcan = hcan, .callbacks = {}, .callback_count = 0 };
        map            = &maps[map_size];
        map_size++;
    }
    // 如果回调函数表未满，则将回调函数注册到末尾
    if (map->callback_count < CAN_MAX_CALLBACK_NUM)
        map->callbacks[map->callback_count++] = callback;
    else
        Error_Handler();
}

// 由于一般不会取消注册，不提供取消注册功能
// 后人可以实现
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
    // 采用 while 循环来确保清空队列
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
    {
        CAN_RxHeaderTypeDef header;
        uint8_t             data[8];
        // 从 FIFO 中获取一帧
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            Error_Handler();
            return;
        }
        // 查找回调函数表
        const CAN_CallbackMap* map = get_map(hcan);

        // 如果该 CAN 被注册
        if (map != nullptr)
            // 依次调用所有的回调函数
            for (size_t i = 0; i < map->callback_count; i++)
                map->callbacks[i](hcan, &header, data);
    }
}
/**
 * CAN Fifo1 接收处理函数
 *
 * 本函数将会根据 hcan 和 rx_header 内部的 filter_id 来调用对应的回调函数
 * @param hcan can handle
 */
void CAN_Fifo1ReceiveCallback(CAN_HandleTypeDef* hcan)
{
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
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
}

/**
 * HAL CAN TX 中断回调
 * @param hcan can handle
 */
void CAN_TxMailboxCpltCallback(CAN_HandleTypeDef* hcan)
{
    // 当上一帧发送完成
    // 获取当前函数的函数表

    // TODO: fixbug 当表未注册使可能产生 UB
    auto map = get_map(hcan);
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0 && !map->buffer.empty())
    {
        uint32_t mailbox = CAN_SEND_FAILED;
        // 从 buffer 内提取一帧
        const auto msg = map->buffer.pop();
        if (HAL_CAN_AddTxMessage(hcan, &msg->header, msg->data, &mailbox) != HAL_OK)
        {
            Error_Handler();
        }
    }
}

/**
 * 注册 CAN 主回调函数
 * @param hcan can handle
 */
void CAN_InitMainCallback(CAN_HandleTypeDef* hcan)
{
    assert(hcan != nullptr);

    HAL_CAN_RegisterCallback(hcan, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, CAN_Fifo0ReceiveCallback);
    HAL_CAN_RegisterCallback(hcan, HAL_CAN_RX_FIFO1_MSG_PENDING_CB_ID, CAN_Fifo1ReceiveCallback);
    HAL_CAN_RegisterCallback(hcan, HAL_CAN_TX_MAILBOX0_COMPLETE_CB_ID, CAN_TxMailboxCpltCallback);
    HAL_CAN_RegisterCallback(hcan, HAL_CAN_TX_MAILBOX1_COMPLETE_CB_ID, CAN_TxMailboxCpltCallback);
    HAL_CAN_RegisterCallback(hcan, HAL_CAN_TX_MAILBOX2_COMPLETE_CB_ID, CAN_TxMailboxCpltCallback);
}
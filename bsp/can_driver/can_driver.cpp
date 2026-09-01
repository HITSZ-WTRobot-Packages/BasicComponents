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
#include <cstddef>
#include <array>
#include <variant>

/**
 * FDCAN 版本
 */
#if CAN_DRIVER_FDCAN_ENABLED
struct CAN_MessageDef
{
    FDCAN_TxHeaderTypeDef header;
    uint8_t               data[64];
};

using FifoReceiveCallback = std::variant<FDCAN_FifoReceiveCallback_t, CAN_FifoReceiveCallback_t>;

struct CAN_CallbackMap
{
    FDCAN_HandleTypeDef* hcan{ nullptr };

    FifoReceiveCallback callbacks[CAN_MAX_CALLBACK_NUM]{};

    uint32_t callback_count{ 0 };

#    if FDCAN_ENABLE_SOFT_TX_QUEUE
    libs::RingBuffer<CAN_MessageDef, FDCAN_TX_QUEUE_SIZE + 1, true> buffer;
#    endif
};

CAN_CallbackMap maps[CAN_NUM];
size_t          map_size = 0;

CAN_CallbackMap* get_map(const FDCAN_HandleTypeDef* hcan)
{
    for (size_t i = 0; i < map_size; i++)
        if (maps[i].hcan == hcan)
            return &maps[i];
    return nullptr;
}

constexpr uint32_t FDCAN_DLC_Bytes(const uint32_t dlc)
{
    if (dlc >= 16)
        return -1;
    constexpr std::array<uint32_t, 16> fdcan_dlc_map{ 0, 1,  2,  3,  4,  5,  6,  7,
                                                      8, 12, 16, 20, 24, 32, 48, 64 };
    return fdcan_dlc_map[dlc];
}

void FDCAN_RxDispatch(FDCAN_HandleTypeDef* hcan, const uint32_t fifo)
{
    while (HAL_FDCAN_GetRxFifoFillLevel(hcan, fifo) > 0)
    {
        FDCAN_RxHeaderTypeDef header;
        uint8_t               data[64]{};
        if (HAL_FDCAN_GetRxMessage(hcan, fifo, &header, data) != HAL_OK)
        {
            Error_Handler();
            return;
        }
        const auto* map = get_map(hcan);
        if (map != nullptr)
            for (uint32_t i = 0; i < map->callback_count; i++)
            {
                std::visit(
                        [&](auto callback)
                        {
                            if (callback == nullptr)
                                return;
                            using T = decltype(callback);

                            if constexpr (std::is_same_v<T, FDCAN_FifoReceiveCallback_t>)
                            {
                                // 如果为 FDCAN 风格的回调函数，直接调用
                                callback(hcan, &header, data);
                            }
                            else if constexpr (std::is_same_v<T, CAN_FifoReceiveCallback_t>)
                            {
                                // 如果为 bxCAN 风格的回调函数，转换帧头后再调用
                                if (header.FDFormat != FDCAN_FRAME_CLASSIC)
                                    return;
                                const bool isExtId = header.IdType == FDCAN_EXTENDED_ID;
                                const CAN_RxHeaderTypeDef can_header{
                                    .StdId = header.Identifier,
                                    .ExtId = header.Identifier,
                                    .IDE   = isExtId ? CAN_ID_EXT : CAN_ID_STD,
                                    .RTR   = header.RxFrameType == FDCAN_DATA_FRAME ? CAN_RTR_DATA
                                                                                    : CAN_RTR_REMOTE,
                                    .DLC   = FDCAN_DLC_Bytes(header.DataLength)
                                };
                                callback(hcan, &can_header, data);
                            }
                        },
                        map->callbacks[i]);
            }
    }
}

void FDCAN_Fifo0ReceiveCallback(FDCAN_HandleTypeDef* hcan, uint32_t)
{
    FDCAN_RxDispatch(hcan, FDCAN_RX_FIFO0);
}

void FDCAN_Fifo1ReceiveCallback(FDCAN_HandleTypeDef* hcan, uint32_t)
{
    FDCAN_RxDispatch(hcan, FDCAN_RX_FIFO1);
}

#    if FDCAN_ENABLE_SOFT_TX_QUEUE
void FDCAN_TxSendMsgFromSoftQueue(FDCAN_HandleTypeDef* hcan)
{
    auto* map = get_map(hcan);
    if (map == nullptr)
    {
        // TODO: fixbug 当表未注册使可能产生 UB
        return;
    }
    while (HAL_FDCAN_GetTxFifoFreeLevel(hcan) > 0 && !map->buffer.empty())
    {
        const auto msg = map->buffer.pop();
        if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &msg->header, msg->data) != HAL_OK)
        {
            // TODO: preserve the queued frame and report a recoverable HAL failure.
            Error_Handler();
            return;
        }
    }
}
#    endif

void FDCAN_InitMainCallback(FDCAN_HandleTypeDef* hcan)
{
    assert(hcan != nullptr);
    if (HAL_FDCAN_RegisterRxFifo0Callback(hcan, FDCAN_Fifo0ReceiveCallback) != HAL_OK ||
        HAL_FDCAN_RegisterRxFifo1Callback(hcan, FDCAN_Fifo1ReceiveCallback) != HAL_OK)
        Error_Handler();
#    if FDCAN_ENABLE_SOFT_TX_QUEUE
    if (HAL_FDCAN_RegisterCallback(hcan,
                                   HAL_FDCAN_TX_FIFO_EMPTY_CB_ID,
                                   FDCAN_TxSendMsgFromSoftQueue) != HAL_OK)
        Error_Handler();
#    endif
}

/**
 * 注册 CAN 主回调函数，兼容 bxCAN
 * @param hcan can handle
 */
void CAN_InitMainCallback(CAN_HandleTypeDef* hcan)
{
    FDCAN_InitMainCallback(hcan);
}

uint32_t FDCAN_SendMessage(FDCAN_HandleTypeDef*         hcan,
                           const FDCAN_TxHeaderTypeDef* header,
                           const uint8_t                data[])
{
    if (hcan == nullptr || header == nullptr || data == nullptr)
        return CAN_SEND_FAILED;

    ISRGuard guard;
    if (HAL_FDCAN_GetTxFifoFreeLevel(hcan) > 0)
    {
        if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, header, data) != HAL_OK)
        {
            // TODO: return the HAL failure without entering the global error handler.
            Error_Handler();
            return CAN_SEND_FAILED;
        }
#    if FDCAN_ENABLE_SOFT_TX_QUEUE
        FDCAN_TxSendMsgFromSoftQueue(hcan);
#    endif
        return 0;
    }
#    if FDCAN_ENABLE_SOFT_TX_QUEUE
    auto* map = get_map(hcan);
    if (map == nullptr)
    {
        // TODO: register the handle before queueing, and support a richer failure result.
        return CAN_SEND_FAILED;
    }
    const uint32_t bytes = FDCAN_DLC_Bytes(header->DataLength);
    map->buffer.push(
            [&](CAN_MessageDef& msg)
            {
                msg.header = *header;
                memcpy(msg.data, data, bytes);
                memset(msg.data + bytes, 0, 64 - bytes);
            });
#    endif
    return CAN_SEND_FAILED;
}

uint32_t CAN_SendMessage(CAN_HandleTypeDef*         hcan,
                         const CAN_TxHeaderTypeDef* header,
                         const uint8_t              data[])
{
    if (hcan == nullptr || header == nullptr || data == nullptr)
        return CAN_SEND_FAILED;

    if (hcan->Init.FrameFormat != FDCAN_FRAME_CLASSIC)
        return CAN_SEND_FAILED;

    const bool isExtId = header->IDE == CAN_ID_EXT;

    const FDCAN_TxHeaderTypeDef fdcan_header{
        .Identifier    = isExtId ? header->ExtId : header->StdId,
        .IdType        = isExtId ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID,
        .TxFrameType   = (header->RTR == CAN_RTR_DATA) ? FDCAN_DATA_FRAME : FDCAN_REMOTE_FRAME,
        .DataLength    = std::min<uint32_t>(header->DLC, 8u),
        .BitRateSwitch = FDCAN_BRS_OFF,
        .FDFormat      = FDCAN_CLASSIC_CAN
    };

    return FDCAN_SendMessage(hcan, &fdcan_header, data);
}

void FDCAN_Start(FDCAN_HandleTypeDef* hcan, uint32_t ActiveITs)
{
    if (HAL_FDCAN_Start(hcan) != HAL_OK ||
        HAL_FDCAN_ActivateNotification(hcan, ActiveITs | FDCAN_IT_TX_FIFO_EMPTY, 0) != HAL_OK)
        Error_Handler();
}

void CAN_Start(CAN_HandleTypeDef* hcan, uint32_t ActiveITs)
{
    // 使用 CAN_Start 必须保证 FDCAN 配置为 classic 模式
    assert(hcan != nullptr);
    assert(hcan->Init.FrameFormat == FDCAN_FRAME_CLASSIC);
    FDCAN_Start(hcan, ActiveITs);
}

void RegisterCallback(FDCAN_HandleTypeDef* hcan, FifoReceiveCallback callback)
{
    auto* map = get_map(hcan);
    if (map == nullptr)
    {
        if (map_size >= CAN_NUM)
        {
            Error_Handler();
            return;
        }
        maps[map_size].hcan = hcan;
        map                 = &maps[map_size++];
    }
    if (map->callback_count >= CAN_MAX_CALLBACK_NUM)
    {
        Error_Handler();
        return;
    }
    map->callbacks[map->callback_count++] = callback;
}

void FDCAN_RegisterCallback(FDCAN_HandleTypeDef* hcan, FDCAN_FifoReceiveCallback_t callback)
{
    assert(hcan != nullptr && callback != nullptr);
    RegisterCallback(hcan, callback);
}

void CAN_RegisterCallback(CAN_HandleTypeDef* hcan, CAN_FifoReceiveCallback_t callback)
{
    assert(hcan != nullptr && callback != nullptr);
    assert(hcan->Init.FrameFormat == FDCAN_FRAME_CLASSIC);
    RegisterCallback(hcan, callback);
}

#else
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
    libs::RingBuffer<CAN_MessageDef, CAN_TX_QUEUE_SIZE + 1, true> buffer;
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

void CAN_RxDispatch(CAN_HandleTypeDef* hcan, uint32_t fifo)
{
    // 采用 while 循环来确保清空队列
    while (HAL_CAN_GetRxFifoFillLevel(hcan, fifo) > 0)
    {
        CAN_RxHeaderTypeDef header;
        uint8_t             data[8];
        // 从 FIFO 中获取一帧
        if (HAL_CAN_GetRxMessage(hcan, fifo, &header, data) != HAL_OK)
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
 * CAN Fifo0 接收处理函数
 *
 * 本函数将会根据 hcan 和 rx_header 内部的 filter_id 来调用对应的回调函数
 * @param hcan can handle
 */
void CAN_Fifo0ReceiveCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxDispatch(hcan, CAN_RX_FIFO0);
}
/**
 * CAN Fifo1 接收处理函数
 *
 * 本函数将会根据 hcan 和 rx_header 内部的 filter_id 来调用对应的回调函数
 * @param hcan can handle
 */
void CAN_Fifo1ReceiveCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxDispatch(hcan, CAN_RX_FIFO1);
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
    if (HAL_CAN_RegisterCallback(hcan,
                                 HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID,
                                 CAN_Fifo0ReceiveCallback) != HAL_OK ||
        HAL_CAN_RegisterCallback(hcan,
                                 HAL_CAN_RX_FIFO1_MSG_PENDING_CB_ID,
                                 CAN_Fifo1ReceiveCallback) != HAL_OK ||
        HAL_CAN_RegisterCallback(hcan,
                                 HAL_CAN_TX_MAILBOX0_COMPLETE_CB_ID,
                                 CAN_TxMailboxCpltCallback) != HAL_OK ||
        HAL_CAN_RegisterCallback(hcan,
                                 HAL_CAN_TX_MAILBOX1_COMPLETE_CB_ID,
                                 CAN_TxMailboxCpltCallback) != HAL_OK ||
        HAL_CAN_RegisterCallback(hcan,
                                 HAL_CAN_TX_MAILBOX2_COMPLETE_CB_ID,
                                 CAN_TxMailboxCpltCallback) != HAL_OK)
        Error_Handler();
}
#endif
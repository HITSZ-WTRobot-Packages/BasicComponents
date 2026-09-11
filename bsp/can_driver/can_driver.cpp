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
namespace
{
/**
 * 储存于软件发送队列的 FDCAN 消息类型
 *
 * 包括 TxHeader 以及至多 64 bytes（CAN-FD 单帧最大负载）的数据
 */
struct CAN_MessageDef
{
    FDCAN_TxHeaderTypeDef header;
    uint8_t               data[64];
};

/**
 * 回调表中保存的接收回调
 *
 * 同时兼容 FDCAN 原生回调与 bxCAN 兼容回调：分发时按实际类型决定是直接
 * 调用，还是先把 FDCAN 帧头转换为 CAN_RxHeaderTypeDef 后再调用。
 */
using FifoReceiveCallback = std::variant<FDCAN_FifoReceiveCallback_t, CAN_FifoReceiveCallback_t>;

/**
 * CAN 回调函数表（FDCAN 后端）
 *
 * @note 由于 HAL 只允许将一个函数作为回调函数，如果想在一条总线上处理不同种类
 *       的信息（有多个不同的回调函数），就必须要通过一个主回调函数进行分发
 *
 * @note FDCAN 硬件 Tx FIFO 深度有限，但是在很短的时间内可能连续发送多条消息，
 *       故可选用软件缓冲区来临时储存溢出的消息（见 FDCAN_TX_QUEUE_SIZE）
 */
struct CAN_CallbackMap
{
    FDCAN_HandleTypeDef* hcan{ nullptr }; ///< 该表对应的 CAN 句柄

    FifoReceiveCallback callbacks[CAN_MAX_CALLBACK_NUM]{}; ///< 已注册的接收回调

    uint32_t callback_count{ 0 }; ///< 已注册的接收回调数量

#    if FDCAN_ENABLE_SOFT_TX_QUEUE
    // 使用环形缓冲区实现发送队列，队列长度 FDCAN_TX_QUEUE_SIZE，Overwrite=true
    // 当队列满时会丢弃最早的帧
    libs::RingBuffer<CAN_MessageDef, FDCAN_TX_QUEUE_SIZE + 1, true> buffer;
#    endif
};

// 根据 CAN 实例的数量定义回调表
// CAN 实例的数量取决于芯片型号，且无法在编译期预知，故在 .hpp 内通过宏定义
CAN_CallbackMap maps[CAN_NUM];
size_t          map_size = 0; ///< 当前已登记的回调表数量

/**
 * 根据 can handle 指针查找对应的回调表
 * @param hcan can handle
 * @return 对应的回调表，未登记时返回 nullptr
 */
CAN_CallbackMap* get_map(const FDCAN_HandleTypeDef* hcan)
{
    for (size_t i = 0; i < map_size; i++)
        if (maps[i].hcan == hcan)
            return &maps[i];
    return nullptr;
}

/**
 * 将 FDCAN 的 DataLength 编码转换为实际数据长度（字节）
 *
 * CAN-FD 的 DLC 编码 9 ~ 15 依次对应 12/16/20/24/32/48/64 字节，并非线性。
 * @param dlc FDCAN 帧的 DataLength 编码
 * @return 实际数据长度；编码非法（> FDCAN_DLC_BYTES_64）时返回 0，
 *         调用方需自行校验编码合法性
 */
constexpr uint32_t FDCAN_DLC_Bytes(const uint32_t dlc)
{
    switch (dlc)
    {
    case FDCAN_DLC_BYTES_0:
        return 0;
    case FDCAN_DLC_BYTES_1:
        return 1;
    case FDCAN_DLC_BYTES_2:
        return 2;
    case FDCAN_DLC_BYTES_3:
        return 3;
    case FDCAN_DLC_BYTES_4:
        return 4;
    case FDCAN_DLC_BYTES_5:
        return 5;
    case FDCAN_DLC_BYTES_6:
        return 6;
    case FDCAN_DLC_BYTES_7:
        return 7;
    case FDCAN_DLC_BYTES_8:
        return 8;
    case FDCAN_DLC_BYTES_12:
        return 12;
    case FDCAN_DLC_BYTES_16:
        return 16;
    case FDCAN_DLC_BYTES_20:
        return 20;
    case FDCAN_DLC_BYTES_24:
        return 24;
    case FDCAN_DLC_BYTES_32:
        return 32;
    case FDCAN_DLC_BYTES_48:
        return 48;
    case FDCAN_DLC_BYTES_64:
        return 64;
    default:
        return 0;
    }
}

constexpr uint32_t can_dlc_to_fdcan_dlc(const uint32_t dlc)
{
    constexpr std::array<uint32_t, 9> can_dlcs{ FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1,
                                                FDCAN_DLC_BYTES_2, FDCAN_DLC_BYTES_3,
                                                FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
                                                FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7,
                                                FDCAN_DLC_BYTES_8 };
    if (dlc > 8)
        return FDCAN_DLC_BYTES_0;
    return can_dlcs[dlc];
}
} // namespace

/**
 * FDCAN 接收分发
 *
 * 循环取出指定 Rx FIFO 中的所有帧，并依次调用回调表中注册的每一个回调函数。
 * bxCAN 风格的回调只会收到 classic 帧：CAN-FD 帧无法用 CAN_RxHeaderTypeDef 表达。
 * @note 转换为 bxCAN 帧头时，FilterMatchIndex 填入的是 FDCAN 滤波器元素索引，
 *       与 bxCAN 的 FilterBank 编号不是同一套编号，按 bank 索引的调用方需注意。
 * @param hcan can handle
 * @param fifo FDCAN_RX_FIFO0 或 FDCAN_RX_FIFO1
 */
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
                                    // 与 bxCAN 一致，只填与 ID 类型对应的字段：
                                    // 29 bit 扩展 ID 放入 StdId 会超出其 0~0x7FF 的合法范围
                                    .StdId = isExtId ? 0U : header.Identifier,
                                    .ExtId = isExtId ? header.Identifier : 0U,
                                    .IDE   = isExtId ? CAN_ID_EXT : CAN_ID_STD,
                                    .RTR   = header.RxFrameType == FDCAN_DATA_FRAME ? CAN_RTR_DATA
                                                                                    : CAN_RTR_REMOTE,
                                    .DLC   = FDCAN_DLC_Bytes(header.DataLength),
                                    .Timestamp        = header.RxTimestamp,
                                    .FilterMatchIndex = header.FilterIndex,
                                };
                                callback(hcan, &can_header, data);
                            }
                        },
                        map->callbacks[i]);
            }
    }
}

/**
 * FDCAN Rx FIFO0 中断处理函数
 * @param hcan can handle
 */
void FDCAN_Fifo0ReceiveCallback(FDCAN_HandleTypeDef* hcan, uint32_t)
{
    FDCAN_RxDispatch(hcan, FDCAN_RX_FIFO0);
}

/**
 * FDCAN Rx FIFO1 中断处理函数
 * @param hcan can handle
 */
void FDCAN_Fifo1ReceiveCallback(FDCAN_HandleTypeDef* hcan, uint32_t)
{
    FDCAN_RxDispatch(hcan, FDCAN_RX_FIFO1);
}

#    if FDCAN_ENABLE_SOFT_TX_QUEUE
/**
 * 将软件发送队列中的消息搬入 FDCAN 硬件 Tx FIFO
 *
 * 在 Tx FIFO 空闲中断或直接发送之后调用，直到硬件 FIFO 占满或软件队列清空为止。
 * @param hcan can handle
 */
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

/**
 * 注册 FDCAN 主回调函数
 *
 * 注册 Rx FIFO0/FIFO1 回调，并在启用软件发送队列时注册 Tx FIFO 空闲回调。
 * @param hcan can handle
 */
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
 * 注册 CAN 主回调函数，兼容 bxCAN 接口
 * @param hcan can handle
 */
void CAN_InitMainCallback(CAN_HandleTypeDef* hcan)
{
    FDCAN_InitMainCallback(hcan);
}

/**
 * 发送一条 FDCAN 消息
 * @param hcan can handle
 * @param header 发送帧头，包含 ID 类型、帧格式与 DataLength 编码
 * @param data 待发送数据，长度由 header->DataLength 决定
 * @note 线程安全：内部会短暂关闭中断
 * @note DataLength 必须是合法编码（FDCAN_DLC_BYTES_0 ~ FDCAN_DLC_BYTES_64），
 *       非法编码不会发出空帧，直接返回 CAN_SEND_FAILED
 * @note 硬件 Tx FIFO 已满且未启用软件发送队列时发送失败
 * @return 成功返回 0，失败返回 CAN_SEND_FAILED
 */
uint32_t FDCAN_SendMessage(FDCAN_HandleTypeDef*         hcan,
                           const FDCAN_TxHeaderTypeDef* header,
                           const uint8_t                data[])
{
    if (hcan == nullptr || header == nullptr || data == nullptr)
        return CAN_SEND_FAILED;

    // DataLength 为 4 bit 的 DLC 编码，合法取值恰为 FDCAN_DLC_BYTES_0 ~ FDCAN_DLC_BYTES_64。
    // 非法编码会被 FDCAN_DLC_Bytes() 映射为 0，从而静默发出空帧，故在此直接拒绝。
    if (header->DataLength > FDCAN_DLC_BYTES_64)
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
        // TODO(fix): 软件队列非空时仍走直发路径，会让本帧先于队列中更早提交的帧发出。
        //            正式修复应先判断队列是否为空、非空则改为入队，由 Tx FIFO 空闲
        //            中断统一搬运；在此之前先用断言暴露该顺序反转。
        assert((get_map(hcan) == nullptr || get_map(hcan)->buffer.empty()) &&
               "FDCAN soft Tx queue not empty: direct send reorders frames");
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
    if (map->buffer.push(
                [&](CAN_MessageDef& msg)
                {
                    msg.header = *header;
                    memcpy(msg.data, data, bytes);
                    memset(msg.data + bytes, 0, 64 - bytes);
                }))
        return 0;
#    endif
    return CAN_SEND_FAILED;
}

/**
 * 以 bxCAN 接口发送一条 classic CAN 帧（FDCAN 后端兼容实现）
 * @param hcan can handle
 * @param header 发送帧头，仅支持 classic 帧
 * @param data 待发送数据，长度由 header->DLC 决定（不超过 8 字节）
 * @return 同 FDCAN_SendMessage()
 */
uint32_t CAN_SendMessage(CAN_HandleTypeDef*         hcan,
                         const CAN_TxHeaderTypeDef* header,
                         const uint8_t              data[])
{
    if (hcan == nullptr || header == nullptr || data == nullptr)
        return CAN_SEND_FAILED;

    if (hcan->Init.FrameFormat != FDCAN_FRAME_CLASSIC)
        return CAN_SEND_FAILED;

    // classic 帧只有 0~8 字节，超出范围的 DLC 会被 can_dlc_to_fdcan_dlc() 映射成 0，
    // 不能静默当作空帧发出。
    if (header->DLC > 8)
        return CAN_SEND_FAILED;

    const bool isExtId = header->IDE == CAN_ID_EXT;

    const FDCAN_TxHeaderTypeDef fdcan_header{
        .Identifier    = isExtId ? header->ExtId : header->StdId,
        .IdType        = isExtId ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID,
        .TxFrameType   = (header->RTR == CAN_RTR_DATA) ? FDCAN_DATA_FRAME : FDCAN_REMOTE_FRAME,
        .DataLength    = can_dlc_to_fdcan_dlc(header->DLC),
        .BitRateSwitch = FDCAN_BRS_OFF,
        .FDFormat      = FDCAN_CLASSIC_CAN
    };

    return FDCAN_SendMessage(hcan, &fdcan_header, data);
}

/**
 * 将 bxCAN 风格的过滤器配置转换为 FDCAN 配置。
 *
 * 只支持 32 位 scale（ID 掩码 / ID 列表）。32 位 bxCAN 过滤器由 FR1/FR2 两个字组成，
 * 每个字均由 FilterIdHigh/FilterMaskIdHigh 作为高 16 位、FilterIdLow/FilterMaskIdLow
 * 作为低 16 位拼接而成，因此 ID 与掩码无论落在高半字还是低半字都按 bxCAN 字布局解码，
 * 不会丢弃低半字。
 *
 * 16 位 scale 无法用单个 FDCAN 滤波器元素无损表达（见实现内注释），
 * 统一返回 HAL_ERROR。
 *
 * @note 需要先在 CubeMX 中为该 ID 类型分配滤波器元素（FDCAN 的 Std Filters
 *       Nbr / Ext Filters Nbr）；分配数量为 0 时返回 HAL_ERROR，
 *       详见下方 filter_capacity 处的说明。
 * @return 配置结果：成功返回 HAL_OK，参数或配置不受支持时返回 HAL_ERROR
 */
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef*       hcan,
                                       const CAN_FilterTypeDef* filterConfig)
{
    if (hcan == nullptr || filterConfig == nullptr)
        return HAL_ERROR;

    FDCAN_FilterTypeDef fdcan_filter{};
    // 直接复用 bxCAN 的 FilterBank 作为 FDCAN 滤波器元素索引
    // TODO(fix): FilterBank 是 bxCAN 的 bank 编号（单 CAN 0~13，双 CAN 0~27，且从站
    //   从 SlaveStartFilterBank 起算），与 FDCAN 每个实例各自独立的滤波器元素索引
    //   不是同一套编号。当前既未做上界校验，也未做 bank 重映射：索引越界时
    //   HAL_FDCAN_ConfigFilter 只会写入 ExtendedFilterSA/StandardFilterSA + index*size，
    //   踩到 Rx FIFO0 等 message RAM（本工程 USE_FULL_ASSERT 关闭，assert_param 为空，
    //   没有任何运行时保护）。修复时应校验索引小于对应类型的滤波器元素数量，并按
    //   实例重映射 bank。
    fdcan_filter.FilterIndex = filterConfig->FilterBank;

    if (filterConfig->FilterActivation == CAN_FILTER_DISABLE)
    {
        fdcan_filter.IdType       = FDCAN_STANDARD_ID;
        fdcan_filter.FilterType   = FDCAN_FILTER_MASK;
        fdcan_filter.FilterConfig = FDCAN_FILTER_DISABLE;
    }
    else if (filterConfig->FilterScale == CAN_FILTERSCALE_16BIT)
    {
        // bxCAN 的 16 位 scale 在一个 bank 内放两组 16 位值对，HAL_CAN_ConfigFilter 的
        // 写入方式为
        //   FR1 = (FilterMaskIdLow  << 16) | FilterIdLow
        //   FR2 = (FilterMaskIdHigh << 16) | FilterIdHigh
        // 即 ID 列表模式下这四个字段都是待匹配 ID，掩码模式下是两组 ID+掩码。
        // 而 FDCAN 单个滤波器元素最多只放两个值（DUAL 两个 ID，MASK 一个 ID 加一个掩码），
        // 无法无损表达：只取其中两个会静默丢弃另外两个。这里直接拒绝，
        // 避免上层以为过滤器已按预期生效。
        return HAL_ERROR;
    }
    else if (filterConfig->FilterScale == CAN_FILTERSCALE_32BIT)
    {
        const uint32_t id_word     = (filterConfig->FilterIdHigh << 16) | filterConfig->FilterIdLow;
        const uint32_t second_word = (filterConfig->FilterMaskIdHigh << 16) |
                                     filterConfig->FilterMaskIdLow;
        // FR1/FR2 布局为 STID[10:0] | EXID[17:0] | IDE | RTR：IDE 位置位表示
        // 扩展帧（ID 位于 bit 20:3，右移 3），否则为标准帧（ID 位于 bit 31:21）。
        // 掩码字与 ID 字布局相同，故共用同一个右移位数。
        const bool     id_is_extended = (id_word & CAN_ID_EXT) != 0U;
        const uint32_t id_shift       = id_is_extended ? 3U : 21U;

        if (filterConfig->FilterMode == CAN_FILTERMODE_IDLIST)
        {
            // ID 列表模式下第二个字是第二个待匹配 ID；FDCAN 的 DUAL 过滤器
            // 要求两个 ID 类型一致，无法表达标准/扩展混合的列表。
            if (((second_word & CAN_ID_EXT) != 0U) != id_is_extended)
                return HAL_ERROR;
            fdcan_filter.FilterType = FDCAN_FILTER_DUAL;
        }
        else
        {
            fdcan_filter.FilterType = FDCAN_FILTER_MASK;
        }

        fdcan_filter.IdType       = id_is_extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
        fdcan_filter.FilterID1    = id_word >> id_shift;
        fdcan_filter.FilterID2    = second_word >> id_shift;
        fdcan_filter.FilterConfig = filterConfig->FilterFIFOAssignment == CAN_FILTER_FIFO1
                                            ? FDCAN_FILTER_TO_RXFIFO1
                                            : FDCAN_FILTER_TO_RXFIFO0;
    }
    else
    {
        return HAL_ERROR;
    }

    // CubeMX 未给该 ID 类型分配滤波器元素时 RXGFC.LSS/LSE 为 0，硬件根本不评估任何
    // 滤波器，而 HAL_FDCAN_ConfigFilter 仍会把配置写进 message RAM 并返回 HAL_OK，
    // 表现为滤波器静默失效。此处直接拒绝，避免上层误以为过滤已生效。
    const uint32_t filter_capacity = fdcan_filter.IdType == FDCAN_EXTENDED_ID
                                             ? hcan->Init.ExtFiltersNbr
                                             : hcan->Init.StdFiltersNbr;
    if (filter_capacity == 0)
        return HAL_ERROR;

    return HAL_FDCAN_ConfigFilter(hcan, &fdcan_filter);
}

/**
 * 启动 FDCAN 并开启中断
 * @param hcan can handle
 * @param ActiveITs 需要额外开启的中断，如 FDCAN_IT_RX_FIFO0_NEW_MESSAGE
 * @note 驱动会强制叠加 FDCAN_IT_TX_FIFO_EMPTY，软件发送队列依赖该中断
 */
void FDCAN_Start(FDCAN_HandleTypeDef* hcan, uint32_t ActiveITs)
{
    if (HAL_FDCAN_Start(hcan) != HAL_OK ||
        HAL_FDCAN_ActivateNotification(hcan, ActiveITs | FDCAN_IT_TX_FIFO_EMPTY, 0) != HAL_OK)
        Error_Handler();
}

/**
 * 启动 CAN（FDCAN 后端兼容实现，仅限 classic 模式）
 * @param hcan can handle
 * @param ActiveITs 需要额外开启的中断
 * @note 要求句柄已配置为 FDCAN_FRAME_CLASSIC，否则触发断言
 */
void CAN_Start(CAN_HandleTypeDef* hcan, uint32_t ActiveITs)
{
    // 使用 CAN_Start 必须保证 FDCAN 配置为 classic 模式
    assert(hcan != nullptr);
    assert(hcan->Init.FrameFormat == FDCAN_FRAME_CLASSIC);
    FDCAN_Start(hcan, ActiveITs);
}

namespace
{
/**
 * 将接收回调登记到对应句柄的回调表，表不存在时按需新建
 * @param hcan can handle
 * @param callback 接收回调
 * @note 回调表数量或表内回调数量达到上限时进入 Error_Handler()
 */
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
} // namespace
/**
 * 注册 FDCAN 接收回调
 * @param hcan can handle
 * @param callback 接收回调函数，不可为空
 * @note 非线程安全，建议在总线启动前完成注册
 */
void FDCAN_RegisterCallback(FDCAN_HandleTypeDef* hcan, FDCAN_FifoReceiveCallback_t callback)
{
    assert(hcan != nullptr && callback != nullptr);
    RegisterCallback(hcan, callback);
}

/**
 * 注册 CAN 接收回调（FDCAN 后端兼容实现）
 * @param hcan can handle
 * @param callback 接收回调函数，不可为空
 * @note 要求句柄已配置为 FDCAN_FRAME_CLASSIC，否则触发断言
 * @note 非线程安全，建议在总线启动前完成注册
 */
void CAN_RegisterCallback(CAN_HandleTypeDef* hcan, CAN_FifoReceiveCallback_t callback)
{
    assert(hcan != nullptr && callback != nullptr);
    assert(hcan->Init.FrameFormat == FDCAN_FRAME_CLASSIC);
    RegisterCallback(hcan, callback);
}

#else
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
    libs::RingBuffer<CAN_MessageDef, CAN_TX_QUEUE_SIZE + 1, true> buffer;
};

// 根据 CAN 实例的数量定义回调表
// CAN 实例的数量取决于芯片型号，且无法在编译期预知，故在 .hpp 内通过宏定义
CAN_CallbackMap maps[CAN_NUM];
size_t          map_size = 0; ///< 当前已登记的回调表数量

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
 * @param header 发送帧头，包含 ID 类型、数据/远程帧标志与 DLC
 * @param data 待发送数据，长度由 header->DLC 决定（不超过 8 字节）
 * @note 本函数是线程安全的
 * @return 发送使用的 mailbox 编号，CAN_SEND_FAILED(0xFFFF) 表示发送失败
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
 * 启动 CAN 并开启中断
 * @param hcan can handle
 * @param ActiveITs 需要额外开启的中断，如
 *        CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING
 * @note 驱动会额外开启 CAN_IT_TX_MAILBOX_EMPTY，软件发送队列依赖该中断
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
 * 注册 CAN 接收回调
 *
 * 同一条总线上可以注册多个回调，收到帧后会依次调用。
 * @attention 本函数非线程安全，调用时请注意
 * @param hcan can handle
 * @param callback 回调函数指针，不可为空
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
 * CAN 接收分发
 *
 * 循环取出指定 FIFO 中的所有帧，并依次调用该总线上注册的各个回调函数。
 * @param hcan can handle
 * @param fifo CAN_RX_FIFO0 或 CAN_RX_FIFO1
 */
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
 * CAN Rx FIFO0 接收处理函数
 *
 * 取出 FIFO0 中的所有待处理帧，并分发到该总线上注册的各个回调函数。
 * @param hcan can handle
 */
void CAN_Fifo0ReceiveCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxDispatch(hcan, CAN_RX_FIFO0);
}
/**
 * CAN Rx FIFO1 接收处理函数
 *
 * 取出 FIFO1 中的所有待处理帧，并分发到该总线上注册的各个回调函数。
 * @param hcan can handle
 */
void CAN_Fifo1ReceiveCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxDispatch(hcan, CAN_RX_FIFO1);
}

/**
 * HAL CAN Tx 完成中断回调
 *
 * 硬件 mailbox 空出后，从软件发送队列中取出待发送帧继续发送。
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
 *
 * 注册接收 FIFO 与三个发送 mailbox 完成回调，需在 HAL_CAN_Init() 之后、
 * CAN_Start() 之前调用。
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
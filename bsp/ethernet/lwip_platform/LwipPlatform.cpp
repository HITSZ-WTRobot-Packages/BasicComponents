/**
 * @file    LwipPlatform.cpp
 * @brief   PHY 无关的 STM32 ETH MAC 与 LwIP netif 平台状态机实现。
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * 锁序固定为 LwIP core lock → 本实例 RX 互斥体，且 PHY 的 start()/readLink() 始终在锁外：
 *   - core lock 保护 netif link 变更与 MAC 转换（Get/Set/Start/Stop）。
 *   - RX 互斥体额外串行化 MAC 转换与 input() 的底层 HAL_ETH_ReadData。
 *   - netif_set_link_down 在取得 RX 互斥体之前执行，netif_set_link_up 在释放之后执行，
 *     避免 link 回调内发送数据造成重入。
 * 发送路径 output() 由 LwIP 在 core lock 内调用，非阻塞、不重试，TX 描述符回收交由
 * 本函数的开头与 poll() 在 core lock 内完成。
 */
#include "LwipPlatform.hpp"

#include "lwip/netif.h"
#include "lwip/opt.h"
#include "lwip/pbuf.h"
#include "lwip/tcpip.h"

#include <cstdint>

#if !LWIP_TCPIP_CORE_LOCKING
#    error "LwipPlatform requires LWIP_TCPIP_CORE_LOCKING=1 to serialize MAC access."
#endif

namespace bsp::ethernet_phy
{

namespace
{

/** @brief 按符号映射四种 Up 状态的 MAC 速度/双工；非 Up 返回 false。 */
bool mac_mode_for(PhyLinkState state, std::uint32_t& speed, std::uint32_t& duplex) noexcept
{
    switch (state)
    {
    case PhyLinkState::Up10Half:
        speed  = ETH_SPEED_10M;
        duplex = ETH_HALFDUPLEX_MODE;
        return true;
    case PhyLinkState::Up10Full:
        speed  = ETH_SPEED_10M;
        duplex = ETH_FULLDUPLEX_MODE;
        return true;
    case PhyLinkState::Up100Half:
        speed  = ETH_SPEED_100M;
        duplex = ETH_HALFDUPLEX_MODE;
        return true;
    case PhyLinkState::Up100Full:
        speed  = ETH_SPEED_100M;
        duplex = ETH_FULLDUPLEX_MODE;
        return true;
    default:
        return false;
    }
}

} // namespace

LwipPlatform::LwipPlatform(ETH_HandleTypeDef& eth, IPhy& phy) noexcept
    : eth_{eth}, phy_{phy}
{
}

bool LwipPlatform::prepare() noexcept
{
    if (prepared_)
        return true;

    if (rx_mutex_ == nullptr)
    {
        const osMutexAttr_t attr = {
            "ethRx", osMutexPrioInherit, &rx_mutex_storage_, sizeof(rx_mutex_storage_)};
        rx_mutex_ = osMutexNew(&attr);
        if (rx_mutex_ == nullptr)
            return false;
    }

    prepared_ = true;
    return true;
}

bool LwipPlatform::init(struct netif& netif, const PhyLinkConfig& config) noexcept
{
    if (!prepared_ || init_attempted_)
        return false;

    init_attempted_ = true;
    bound_netif_    = &netif;

    /* 先在 core lock 内明确 link down，再在锁外按调用方配置启动 PHY。 */
    LOCK_TCPIP_CORE();
    netif_set_link_down(bound_netif_);
    UNLOCK_TCPIP_CORE();

    if (phy_.start(config) != PhyResult::Ok)
        return false;

    applied_state_ = PhyLinkState::Down;
    initialized_   = true;
    return true;
}

bool LwipPlatform::poll() noexcept
{
    if (!initialized_)
        return false;

    PhyLinkState state{};
    if (phy_.readLink(state) != PhyResult::Ok)
    {
        /* 读取失败：不沿用未更新状态，强制 link down 并尽力停止已启动的 MAC。 */
        LOCK_TCPIP_CORE();
        setLinkDownLocked();
        (void)HAL_ETH_ReleaseTxPacket(&eth_);
        (void)stopMacLocked();
        UNLOCK_TCPIP_CORE();
        return false;
    }

    if (state == PhyLinkState::Down || state == PhyLinkState::Negotiating)
    {
        LOCK_TCPIP_CORE();
        setLinkDownLocked();
        (void)HAL_ETH_ReleaseTxPacket(&eth_);
        const bool stop_ok = stopMacLocked();
        UNLOCK_TCPIP_CORE();
        return stop_ok;
    }

    std::uint32_t speed  = 0U;
    std::uint32_t duplex = 0U;
    if (!mac_mode_for(state, speed, duplex))
        return false; /* 未知枚举值：不猜测也不强转。 */

    LOCK_TCPIP_CORE();

    if (mac_started_ && state == applied_state_)
    {
        /* 无模式变更时仅回收已完成 TX，保证最后一次发送的引用最终释放。 */
        (void)HAL_ETH_ReleaseTxPacket(&eth_);
        UNLOCK_TCPIP_CORE();
        return true; /* 模式未变，不重复配置/启动。 */
    }

    setLinkDownLocked();

    if (mac_started_ && !stopMacLocked())
    {
        /* 旧模式停止失败：保持 link down，不继续 Set/Start。 */
        UNLOCK_TCPIP_CORE();
        return false;
    }

    if (!startMacLocked(speed, duplex))
    {
        UNLOCK_TCPIP_CORE();
        return false;
    }

    mac_started_   = true;
    applied_state_ = state;
    /* netif link up 在释放 RX 互斥体后、仍持 core lock 时报告。 */
    if (bound_netif_ != nullptr && netif_is_link_up(bound_netif_) == 0U)
        netif_set_link_up(bound_netif_);

    UNLOCK_TCPIP_CORE();
    return true;
}

void LwipPlatform::input(struct netif& netif, osSemaphoreId_t rx_semaphore)
{
    if (!prepared_ || rx_mutex_ == nullptr || rx_semaphore == nullptr ||
        netif.input == nullptr)
        return;

    for (;;)
    {
        if (osSemaphoreAcquire(rx_semaphore, osWaitForever) != osOK)
            continue;

        /* 本轮持续读取，直到 HAL 报告无包/MAC 未启动或取锁失败。 */
        for (;;)
        {
            if (osMutexAcquire(rx_mutex_, osWaitForever) != osOK)
                break;

            void* received = nullptr;
            const HAL_StatusTypeDef status = HAL_ETH_ReadData(&eth_, &received);
            (void)osMutexRelease(rx_mutex_);

            if (status != HAL_OK || received == nullptr)
                break;

            /* 交包和释放 pbuf 均在 RX 锁外，避免重入接收路径。 */
            struct pbuf* packet = static_cast<struct pbuf*>(received);
            if (netif.input(packet, &netif) != ERR_OK)
                pbuf_free(packet);
        }
    }
}

err_t LwipPlatform::output(struct netif& netif, struct pbuf* p)
{
    if (!prepared_ || !initialized_ || bound_netif_ != &netif || p == nullptr)
        return ERR_IF;

    if (netif_is_link_up(&netif) == 0U || !mac_started_)
        return ERR_IF;

    /* MAC 停止或 HAL 出错时不再提交描述符。 */
    if (eth_.gState != HAL_ETH_STATE_STARTED)
        return ERR_IF;

    ETH_BufferTypeDef         tx_buffers[ETH_TX_DESC_CNT] = {};
    ETH_TxPacketConfigTypeDef tx_config                   = {};

    /* 保持原链长上限；超限时尚未提交 HAL，也未增加 pbuf 引用。 */
    std::uint32_t i = 0U;
    for (struct pbuf* q = p; q != nullptr; q = q->next)
    {
        if (i >= ETH_TX_DESC_CNT)
            return ERR_IF;

        tx_buffers[i].buffer = static_cast<std::uint8_t*>(q->payload);
        tx_buffers[i].len    = q->len;
        if (i > 0U)
            tx_buffers[i - 1U].next = &tx_buffers[i];
        ++i;
    }

    tx_config.Attributes   = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
    tx_config.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
    tx_config.CRCPadCtrl   = ETH_CRC_PAD_INSERT;
    tx_config.Length       = p->tot_len;
    tx_config.TxBuffer     = tx_buffers;
    tx_config.pData        = p;

    /* 先回收已完成 TX（调用其 TxFree 回调），再为本包增加一次引用。 */
    (void)HAL_ETH_ReleaseTxPacket(&eth_);

    pbuf_ref(p);
    if (HAL_ETH_Transmit_IT(&eth_, &tx_config) == HAL_OK)
        return ERR_OK;

    /* 撤销本次引用；用 gState 区分提交期状态错误与描述符 BUSY（不读被 IRQ 更新的 ErrorCode）。 */
    pbuf_free(p);
    return (eth_.gState == HAL_ETH_STATE_STARTED) ? ERR_BUF : ERR_IF;
}

void LwipPlatform::setLinkDownLocked() noexcept
{
    if (bound_netif_ != nullptr && netif_is_link_up(bound_netif_) != 0U)
        netif_set_link_down(bound_netif_);
}

bool LwipPlatform::stopMacLocked() noexcept
{
    if (!mac_started_)
        return true;

    if (osMutexAcquire(rx_mutex_, osWaitForever) != osOK)
        return false;

    const bool stopped = (HAL_ETH_Stop_IT(&eth_) == HAL_OK);
    (void)osMutexRelease(rx_mutex_);

    if (stopped)
        mac_started_ = false;
    return stopped;
}

bool LwipPlatform::startMacLocked(std::uint32_t speed, std::uint32_t duplex) noexcept
{
    if (osMutexAcquire(rx_mutex_, osWaitForever) != osOK)
        return false;

    ETH_MACConfigTypeDef mac_config{};
    bool                 ok = (HAL_ETH_GetMACConfig(&eth_, &mac_config) == HAL_OK);
    if (ok)
    {
        /* 只改速度与双工，其余 MAC 配置保持 HAL 读回的现值。 */
        mac_config.Speed      = speed;
        mac_config.DuplexMode = duplex;
        ok = (HAL_ETH_SetMACConfig(&eth_, &mac_config) == HAL_OK);
    }
    if (ok)
        ok = (HAL_ETH_Start_IT(&eth_) == HAL_OK);

    (void)osMutexRelease(rx_mutex_);
    return ok;
}

} // namespace bsp::ethernet_phy

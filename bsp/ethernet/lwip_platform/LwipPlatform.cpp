/**
 * @file    LwipPlatform.cpp
 * @brief   PHY 无关的 STM32 ETH MAC 与 LwIP netif 平台状态机实现。
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * 锁序固定为 LwIP core lock → 本实例 RX 互斥体，且 PHY 的 start()/readLink() 始终在锁外：
 *   - core lock 保护 netif link 变更与 MAC 转换（Get/Set/Start/Stop）。
 *   - RX 互斥体额外串行化 MAC 转换与底层 low_level_input 的 HAL_ETH_ReadData。
 *   - netif_set_link_down 在取得 RX 互斥体之前执行，netif_set_link_up 在释放之后执行，
 *     避免 link 回调内发送数据造成重入。
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
        (void)stopMacLocked();
        UNLOCK_TCPIP_CORE();
        return false;
    }

    if (state == PhyLinkState::Down || state == PhyLinkState::Negotiating)
    {
        LOCK_TCPIP_CORE();
        setLinkDownLocked();
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

struct pbuf* LwipPlatform::receive(struct netif& netif, ReceiveFunction receive_fn)
{
    if (receive_fn == nullptr || !prepared_ || rx_mutex_ == nullptr)
        return nullptr;

    if (osMutexAcquire(rx_mutex_, osWaitForever) != osOK)
        return nullptr;

    struct pbuf* packet = receive_fn(&netif);
    (void)osMutexRelease(rx_mutex_);
    return packet;
}

void LwipPlatform::input(struct netif& netif, osSemaphoreId_t rx_semaphore,
                         ReceiveFunction receive_fn)
{
    if (!prepared_ || rx_semaphore == nullptr || receive_fn == nullptr ||
        netif.input == nullptr)
        return;

    for (;;)
    {
        if (osSemaphoreAcquire(rx_semaphore, osWaitForever) != osOK)
            continue;

        struct pbuf* packet;
        while ((packet = receive(netif, receive_fn)) != nullptr)
        {
            /* RX 互斥体必须在进入 LwIP 或释放 pbuf 之前释放。 */
            if (netif.input(packet, &netif) != ERR_OK)
                pbuf_free(packet);
        }
    }
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

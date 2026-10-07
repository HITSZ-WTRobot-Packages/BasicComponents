/**
 * @file    LwipPlatform.cpp
 * @brief   PHY 无关的 STM32 ETH MAC 与 LwIP netif 平台状态机实现。
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * 锁序：netif link 变更与 MAC 转换（Get/Set/Start/Stop）统一在 LwIP core lock 内完成，
 * 且 PHY 的 start()/readLink() 始终在锁外调用；本类不持有任何自身互斥体。
 * TX 描述符回收在 core lock 内完成：poll() 每轮回收已完成 TX，releaseTxPackets() 另外
 * 提供只回收 TX、不查询 PHY 的入口，供 TX 完成事件线程复用，避免最后一次发送的 pbuf
 * 引用滞留。
 */
#include "LwipPlatform.hpp"

#include "lwip/netif.h"
#include "lwip/opt.h"
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

bool LwipPlatform::init(struct netif& netif, const PhyLinkConfig& config) noexcept
{
    if (init_attempted_)
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

PollResult LwipPlatform::poll() noexcept
{
    if (!initialized_)
        return PollResult::NotInitialized;

    PhyLinkState    state{};
    const PhyResult read_result = phy_.readLink(state);
    if (read_result != PhyResult::Ok)
    {
        /* 读取失败：不沿用未更新状态，强制 link down 并尽力停止已启动的 MAC。 */
        LOCK_TCPIP_CORE();
        setLinkDownLocked();
        (void)HAL_ETH_ReleaseTxPacket(&eth_);
        const bool stop_ok = stopMacLocked();
        UNLOCK_TCPIP_CORE();

        /* 停止失败优先上报，绝不吞掉；其余按 readLink 的实际失败原因区分。 */
        if (!stop_ok)
            return PollResult::MacStopError;
        if (read_result == PhyResult::ReadError)
            return PollResult::PhyReadError;
        if (read_result == PhyResult::NotInitialized)
            return PollResult::NotInitialized;
        return PollResult::InvalidPhyState;
    }

    if (state == PhyLinkState::Down || state == PhyLinkState::Negotiating)
    {
        LOCK_TCPIP_CORE();
        setLinkDownLocked();
        (void)HAL_ETH_ReleaseTxPacket(&eth_);
        const bool stop_ok = stopMacLocked();
        UNLOCK_TCPIP_CORE();
        return stop_ok ? PollResult::Ok : PollResult::MacStopError;
    }

    std::uint32_t speed  = 0U;
    std::uint32_t duplex = 0U;
    if (!mac_mode_for(state, speed, duplex))
        return PollResult::InvalidPhyState; /* 未知枚举值：不猜测也不强转，不继续配置。 */

    LOCK_TCPIP_CORE();

    if (mac_started_ && state == applied_state_)
    {
        /* 无模式变更时仅回收已完成 TX，保证最后一次发送的引用最终释放。 */
        (void)HAL_ETH_ReleaseTxPacket(&eth_);
        UNLOCK_TCPIP_CORE();
        return PollResult::Ok; /* 模式未变，不重复配置/启动。 */
    }

    setLinkDownLocked();

    if (mac_started_ && !stopMacLocked())
    {
        /* 旧模式停止失败：保持 link down，不继续 Set/Start。 */
        UNLOCK_TCPIP_CORE();
        return PollResult::MacStopError;
    }

    if (!startMacLocked(speed, duplex))
    {
        UNLOCK_TCPIP_CORE();
        return PollResult::MacStartError;
    }

    mac_started_   = true;
    applied_state_ = state;
    /* netif link up 在仍持 core lock 时报告。 */
    if (bound_netif_ != nullptr && netif_is_link_up(bound_netif_) == 0U)
        netif_set_link_up(bound_netif_);

    UNLOCK_TCPIP_CORE();
    return PollResult::Ok;
}

void LwipPlatform::releaseTxPackets() noexcept
{
    if (!initialized_)
        return; /* 未 init：不得触碰 HAL 描述符或调用 TxFree 回调。 */

    LOCK_TCPIP_CORE();
    (void)HAL_ETH_ReleaseTxPacket(&eth_);
    UNLOCK_TCPIP_CORE();
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

    const bool stopped = (HAL_ETH_Stop_IT(&eth_) == HAL_OK);

    if (stopped)
        mac_started_ = false;
    return stopped;
}

bool LwipPlatform::startMacLocked(std::uint32_t speed, std::uint32_t duplex) noexcept
{
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

    return ok;
}

} // namespace bsp::ethernet_phy

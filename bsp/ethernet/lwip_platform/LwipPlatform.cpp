/**
 * @file    LwipPlatform.cpp
 * @brief   PHY 无关的 STM32 ETH MAC 与 LwIP netif 平台实现（拥有链路线程）。
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * 锁序：netif link 变更与 MAC 转换（Get/Set/Start/Stop）统一在 LwIP core lock 内完成，
 * 且 PHY 的 start()/readLink()/acknowledgeInterrupt() 始终在锁外调用；本类不持有任何自身
 * 互斥体。链路线程内所有 PHY/MAC/netif 操作串行执行；TX 描述符回收在 core lock 内完成。
 *
 * 线程与事件：start() 静态分配创建唯一 EthLink 线程，线程内先发布自身 ID（release），再
 * 启动 PHY 并接入 PHY 事件回调。TX 完成与 PHY 事件在 ISR 中只置线程标志（acquire 读取已
 * 发布的 ID），真正的 MDIO/HAL 操作全部回到线程完成，ISR 内不访问 MDIO、不调用 IPhy 虚
 * 函数。无中断模式由轮询回收 TX，notifyTxComplete() 直接返回；中断模式不调用
 * osThreadFlagsClear，避免旧 CMSIS 实现的读改写丢失并发 TX 位。
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

bool LwipPlatform::start(struct netif& netif, const PhyLinkConfig& config, ErrorHandler on_error) noexcept
{
    if (start_attempted_ || on_error == nullptr)
        return false; /* 重复启动或空回调：不访问 PHY、不创建线程。 */

    start_attempted_ = true;

    /* 创建线程前把所有运行参数存妥；线程可能在 osThreadNew 返回前就已运行。 */
    bound_netif_    = &netif;
    link_config_    = config;
    on_error_       = on_error;
    interrupt_mode_ = phy_.usesInterrupt();

    osThreadAttr_t attr{};
    attr.name       = "EthLink";
    attr.stack_mem  = thread_stack_;
    attr.stack_size = sizeof(thread_stack_);
    attr.cb_mem     = &thread_control_block_;
    attr.cb_size    = sizeof(thread_control_block_);
    attr.priority   = osPriorityBelowNormal;

    const osThreadId_t created = osThreadNew(&LwipPlatform::link_thread, this, &attr);
    /* 只检查局部返回值；线程 ID 由工作线程自身发布，创建者不写回成员。 */
    return created != nullptr;
}

void LwipPlatform::notifyTxComplete() noexcept
{
    /* 先 acquire 读取已发布的 ID，与工作线程的 release 发布同步后再读 interrupt_mode_。 */
    const osThreadId_t id = thread_id_.load(std::memory_order_acquire);
    if (id == nullptr)
        return;
    if (!interrupt_mode_)
        return; /* 无中断模式由轮询回收 TX，本入口不置标志。 */
    (void)osThreadFlagsSet(id, kTxFlag);
}

void LwipPlatform::onPhyEvent(void* context) noexcept
{
    auto* self = static_cast<LwipPlatform*>(context);
    const osThreadId_t id = self->thread_id_.load(std::memory_order_acquire);
    if (id == nullptr)
        return;
    (void)osThreadFlagsSet(id, kLinkFlag);
}

void LwipPlatform::link_thread(void* argument) noexcept
{
    static_cast<LwipPlatform*>(argument)->run();
}

void LwipPlatform::run() noexcept
{
    /* 以 release 发布自身 ID，供 ISR 与其他上下文定位通知目标。 */
    thread_id_.store(osThreadGetId(), std::memory_order_release);

    if (!initialize())
    {
        fail(PollResult::PhyStartError);
        return;
    }

    /* PHY 启动成功后接入事件回调；首轮快照与 active-low 检查补偿安装通知前的事件。 */
    phy_.setLinkEventCallback(&LwipPlatform::onPhyEvent, this);

    if (!interrupt_mode_)
    {
        for (;;)
        {
            const PollResult result = poll();
            if (result != PollResult::Ok && result != PollResult::PhyReadError)
            {
                fail(result);
                return;
            }
            osDelay(kPollDelayTicks);
        }
    }

    bool first = true;
    for (;;)
    {
        if (first)
        {
            first = false;
            const PollResult result = handleLinkEvent(true);
            if (result != PollResult::Ok)
            {
                fail(result);
                return;
            }
            continue;
        }

        /* INT 仍为低：直接确认，无需再等边沿。 */
        if (phy_.interruptAsserted())
        {
            const PollResult result = handleLinkEvent(false);
            if (result != PollResult::Ok)
            {
                fail(result);
                return;
            }
            continue;
        }

        const std::uint32_t flags = osThreadFlagsWait(kLinkFlag | kTxFlag, osFlagsWaitAny, osWaitForever);
        if ((flags & osFlagsError) != 0U)
        {
            fail(PollResult::ThreadWaitError);
            return;
        }

        if ((flags & kTxFlag) != 0U)
            releaseTxPackets();

        if ((flags & kLinkFlag) != 0U)
        {
            const PollResult result = handleLinkEvent(false);
            if (result != PollResult::Ok)
            {
                fail(result);
                return;
            }
        }
        /* 不 clear 标志；残留 Link 位最多导致一次空 ack，处理完总回到循环复查 INT。 */
    }
}

bool LwipPlatform::initialize() noexcept
{
    /* 先在 core lock 内明确 link down，再在锁外启动 PHY。 */
    LOCK_TCPIP_CORE();
    netif_set_link_down(bound_netif_);
    UNLOCK_TCPIP_CORE();

    if (phy_.start(link_config_) != PhyResult::Ok)
        return false;

    applied_state_ = PhyLinkState::Down;
    initialized_   = true;
    return true;
}

PollResult LwipPlatform::handleLinkEvent(bool force_snapshot) noexcept
{
    bool             link_changed = false;
    const PhyResult  ack          = phy_.acknowledgeInterrupt(link_changed);
    if (ack != PhyResult::Ok)
        return PollResult::PhyInterruptError;

    if (link_changed || force_snapshot)
        return poll();

    return PollResult::Ok;
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
        return; /* 未启动：不得触碰 HAL 描述符或调用 TxFree 回调。 */

    LOCK_TCPIP_CORE();
    (void)HAL_ETH_ReleaseTxPacket(&eth_);
    UNLOCK_TCPIP_CORE();
}

void LwipPlatform::fail(PollResult error) noexcept
{
    /* 断开 PHY 事件回调并清空通知目标，之后不再可能进入本线程的唤醒路径。 */
    phy_.setLinkEventCallback(nullptr, nullptr);
    thread_id_.store(nullptr, std::memory_order_release);

    /* 确保 netif link down；不冒充 MAC 已停止，不重试失败的 HAL 操作。 */
    LOCK_TCPIP_CORE();
    setLinkDownLocked();
    UNLOCK_TCPIP_CORE();

    on_error_(error); /* start 已保证非空；回调可返回或进入工程停机路径。 */

    /* 回调若返回，终止线程且不允许再次 start（start_attempted_ 已置位）。 */
    osThreadExit();
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

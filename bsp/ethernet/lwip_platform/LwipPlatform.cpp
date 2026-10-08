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
 * 函数。两种事件模式共用同一等待循环：TX 完成与 PHY 事件都由 osThreadFlagsWait 唤醒——
 * 该调用默认（未传 osFlagsNoClear）在返回前清除所选标志位，因此无需显式 FlagsClear；显式
 * osThreadFlagsClear 因旧 CMSIS-FreeRTOS 适配层的读改写会丢失并发 TX 置位，那是与“wait
 * 已经清除所选位”无关的另一件事，故不采用。无 INT 模式按绝对 deadline 每 kPollDelayTicks
 * 周期 poll；有 INT 模式在 INT 为高时无限期等待事件，在 INT 卡低且确认后仍无事件时退化为
 * 同一周期 poll，因此每轮都有界阻塞，不存在断言低电平的忙等。
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
    /* 先 acquire 读取工作线程以 release 发布的 ID；未发布（未启动/已终止）则不动作。
       两种事件模式都置 TX 标志：无 INT 模式同样由等待循环唤醒立即回收，不等下一轮 poll。 */
    const osThreadId_t id = thread_id_.load(std::memory_order_acquire);
    if (id == nullptr)
        return;
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

    /* PHY 启动成功后接入事件回调；首轮快照补偿安装通知之前可能已丢失的边沿。 */
    phy_.setLinkEventCallback(&LwipPlatform::onPhyEvent, this);

    if (interrupt_mode_)
    {
        const PollResult result = handleLinkEvent(true);
        if (result != PollResult::Ok)
        {
            fail(result);
            return;
        }
    }

    /* 周期 PHY poll 的绝对 deadline（tick）。无 INT 轮询模式始终启用；有 INT 模式仅在 INT
       卡低、事件等待失去意义时启用。deadline 只在 poll 之后（或首次进入退化时）锚定，因此
       持续到达的 TX 通知只重算剩余量，不会把 100 tick 节拍往后顺延；差值用无符号减法，
       tick 回绕后仍正确。

       loop invariant：每轮至少完成一件实事——处理一个已置位的线程标志、执行一轮 poll，或
       阻塞在 osThreadFlagsWait 中；INT 保持低时该阻塞上限为 remaining <= kPollDelayTicks，
       故不存在无阻塞的忙等。 */
    bool          periodic  = !interrupt_mode_;
    std::uint32_t next_poll = osKernelGetTickCount(); /* 轮询模式首轮立即 poll。 */

    for (;;)
    {
        if (interrupt_mode_)
        {
            if (phy_.interruptAsserted())
            {
                /* INT 已拉低：事件已在线上，立即确认；即使回调未接线漏置标志也不会盲等。 */
                const PollResult result = handleLinkEvent(false);
                if (result != PollResult::Ok)
                {
                    fail(result);
                    return;
                }
            }

            if (phy_.interruptAsserted())
            {
                /* 确认后仍为低：不依赖新的中断边沿，退化为有界等待 + 周期 poll；deadline
                   已锚定则不重置，避免持续断言把节拍无限顺延。 */
                if (!periodic)
                {
                    periodic  = true;
                    next_poll = osKernelGetTickCount() + kPollDelayTicks;
                }
            }
            else
            {
                periodic = false; /* 中断回到高电平：恢复纯事件等待。 */
            }
        }

        std::uint32_t timeout = osWaitForever;
        if (periodic)
        {
            const std::uint32_t remaining = next_poll - osKernelGetTickCount();
            if (remaining == 0U || remaining > kPollDelayTicks)
            {
                /* deadline 已到（含越过 deadline 后差值回绕变大）：先判到期再等待，保证 TX
                   通知不能推迟节拍；也不使用 timeout == 0 的等待，因为无标志时它返回
                   osFlagsErrorResource，会与超时语义混淆。 */
                if (!pollPeriodic())
                    return;
                next_poll = osKernelGetTickCount() + kPollDelayTicks;
                continue;
            }
            timeout = remaining; /* 未到期：remaining 落在 (0, kPollDelayTicks]。 */
        }

        const std::uint32_t flags =
            osThreadFlagsWait(kLinkFlag | kTxFlag, osFlagsWaitAny, timeout);

        if (flags == osFlagsErrorTimeout)
        {
            /* 只有 periodic 分支可能超时：到点执行周期 poll 并锚定下一个 deadline。 */
            if (!periodic)
            {
                fail(PollResult::ThreadWaitError);
                return;
            }
            if (!pollPeriodic())
                return;
            next_poll = osKernelGetTickCount() + kPollDelayTicks;
            continue;
        }
        if ((flags & osFlagsError) != 0U)
        {
            fail(PollResult::ThreadWaitError);
            return;
        }

        /* wait 默认已清除本次返回的所选位，无需显式 FlagsClear；只处理本次返回的位，TX 与
           Link 通知互不吞并（同一次等待同时返回两位时两者都处理）。 */
        if ((flags & kTxFlag) != 0U)
            releaseTxPackets();

        if ((flags & kLinkFlag) != 0U)
        {
            /* 无 INT 模式下 PHY 不触发回调，Link 位不会出现；仅中断模式需要确认。 */
            if (interrupt_mode_)
            {
                const PollResult result = handleLinkEvent(false);
                if (result != PollResult::Ok)
                {
                    fail(result);
                    return;
                }
            }
        }
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
        /* 无模式变更时仅回收已完成 TX，并恢复 netif link up（可能被外部或清理路径置 down）；
           不重复配置/启动 MAC。 */
        (void)HAL_ETH_ReleaseTxPacket(&eth_);
        setLinkUpLocked();
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
    setLinkUpLocked();

    UNLOCK_TCPIP_CORE();
    return PollResult::Ok;
}

bool LwipPlatform::pollPeriodic() noexcept
{
    const PollResult result = poll();

    if (result == PollResult::Ok)
        return true;
    /* 无 INT 轮询模式容忍单轮读取失败（poll 已安全停链），下一周期重试；有 INT 模式
       （含 INT 卡低退化出的周期 poll）与其余结果一律按终止语义上报，不新增错误策略。 */
    if (!interrupt_mode_ && result == PollResult::PhyReadError)
        return true;

    fail(result);
    return false; /* fail 内已 osThreadExit；此处仅为满足返回类型。 */
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

void LwipPlatform::setLinkUpLocked() noexcept
{
    if (bound_netif_ != nullptr && netif_is_link_up(bound_netif_) == 0U)
        netif_set_link_up(bound_netif_);
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

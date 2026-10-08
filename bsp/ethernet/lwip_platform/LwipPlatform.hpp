/**
 * @file    LwipPlatform.hpp
 * @brief   PHY 无关的 STM32 ETH MAC 与 LwIP netif 平台，并拥有链路线程。
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * 职责边界：
 *   - 只通过借用的 IPhy& 启动 PHY、查询链路并切换 MAC 速度/双工；不依赖具体 PHY
 *     类型、地址、MDIO 总线或板级 MSP，器件复位与寄存器配置由具体 PHY 驱动负责。
 *   - 拥有唯一的链路线程（`start()` 内静态分配创建），线程体内完成 PHY 启动、链路
 *     查询与 MAC/netif 切换；调用方不再自行创建 EthLink 线程或轮询。
 *   - 不实现自动重试、遥测或工程错误处理策略；运行期致命错误通过 on_error 回调异步
 *     上报一次，是否停机由调用方决定（本类不硬编码 `Error_Handler`）。
 *   - 不拥有 ETH 句柄、PHY 或 netif；这些由调用方创建并保证生命周期覆盖本对象及线程
 *     的全部运行期。
 *
 * 生命周期与所有权：
 *   - 构造只保存引用并初始化成员，不访问 HAL、PHY 或 RTOS，不产生总线流量，不失败；
 *     因此可安全地声明为静态存储期对象。
 *   - `start(netif, config, on_error)` 只允许从普通线程上下文调用一次；重复调用或
 *     on_error 为空返回 false。其返回 true 仅表示链路线程创建成功，不表示 PHY 已启动
 *     或链路已建立；PHY 初始化异步在线程内进行，失败经 on_error 上报为 PhyStartError。
 *   - 借用 eth、phy 与 netif 必须在本对象整个使用期内有效；对象不可拷贝/移动，也不
 *     提供 stop/restart；线程一经成功创建即运行到终止错误路径，运行中的对象不得销毁。
 *   - 析构不销毁 PHY、不停止 MAC、不操作 RTOS。
 *
 * 锁序与并发：
 *   - MAC 转换（Get/Set/Start/Stop）与 netif link 变更统一在 LwIP core lock 内完成；
 *     本类不持有任何自身互斥体。
 *   - PHY 的 start()/readLink()/acknowledgeInterrupt() 始终在 core lock 外调用。
 *   - 线程内所有 PHY/MAC/netif 操作由唯一 link_thread 串行执行；`notifyTxComplete()` 与
 *     PHY 事件回调可从 ISR 调用，但只通过线程标志通知，不访问 MDIO、HAL 或 RTOS 之外的
 *     状态，也不在 ISR 内调用 IPhy 虚函数。
 *
 * 资源：
 *   - 线程控制块与栈为对象内静态存储：`StaticTask_t` + `StackType_t[1024/sizeof]`，属性
 *     名称 `"EthLink"`、栈 1024 字节、优先级 `osPriorityBelowNormal`（沿用原启动代码）。
 *     依赖 `configSUPPORT_STATIC_ALLOCATION=1`，不新增 RTOS 堆分配。
 *   - 优先级约束（MUST）：`osPriorityBelowNormal` 必须严格低于 CubeMX 生成的 `EthIf` 线程
 *     （`osPriorityRealtime`）。`EthIf` 在其线程上下文调用 `HAL_ETH_ReadData()` 时不持有
 *     LwIP core lock，而本线程的 `HAL_ETH_Start_IT()` 与描述符重建同样不受 core lock 对 RX
 *     的保护；若把 EthLink 提升到 `>= EthIf`，Start/描述符重建可能在 RX 处理中途抢占并破坏
 *     RX 描述符状态。不得提高本线程优先级。
 *
 * 编译前提：
 *   - 调用方须先调用 HAL_ETH_Init 完成 MAC 外设初始化；本类不初始化 MAC 外设。
 *   - 依赖 LWIP_TCPIP_CORE_LOCKING=1（在 .cpp 中以 #error 强制）。
 */
#pragma once

#include "IPhy.hpp"

#include "FreeRTOS.h"
#include "cmsis_os2.h"

#include "main.h"

#include <atomic>
#include <cstdint>

/* LwIP 类型前置声明：必须位于全局命名空间，避免在 bsp::ethernet_phy 内引入同名新类型。 */
struct netif;

namespace bsp::ethernet_phy
{

/**
 * @brief 链路线程的单轮/终止结果；每轮或终止路径恰产生一个值。
 *
 * 覆盖线程处理结果，而不只是单次 PHY 轮询。Ok 表示本轮可继续；PhyReadError 在轮询模式
 * 下可容忍并在下一轮重试（已完成安全停链），在中断模式下视为终止；其余取值均终止线程。
 */
enum class PollResult : std::uint8_t
{
    Ok,                ///< 本轮链路状态已成功应用，含正常断链、协商及 MAC 启动成功。
    PhyReadError,      ///< readLink 寄存器读取失败；已按断链清理，轮询模式下一轮可重试。
    MacStopError,      ///< 需要停止 MAC 但 HAL_ETH_Stop_IT 失败；不谎称已停。
    MacStartError,     ///< 获取/设置 MAC 配置或启动 MAC 失败；链路保持 down。
    NotInitialized,    ///< readLink 报 PHY 未就绪；已按断链清理。
    InvalidPhyState,   ///< readLink 返回契约外结果，或返回未知链路状态；不继续配置。
    PhyStartError,     ///< 线程内 PHY 初始化（start）失败；线程内不再重试。
    PhyInterruptError, ///< 确认 PHY 中断（acknowledgeInterrupt）失败；中断模式终止。
    ThreadWaitError,   ///< 等待线程标志返回非超时错误；线程终止（超时按周期 poll 处理）。
};

/**
 * @brief 借用 ETH 句柄与 IPhy、并拥有链路线程的 LwIP 平台。
 */
class LwipPlatform final
{
public:
    /** @brief 致命错误处理器；由线程调用一次，返回后平台自行结束线程。 */
    using ErrorHandler = void (*)(PollResult error) noexcept;

    /**
     * @brief 只保存借用引用并初始化成员；不访问 HAL、PHY 或 RTOS，不失败。
     * @param[in] eth STM32 ETH 句柄，借用，生命周期须覆盖本对象全部使用期。
     * @param[in] phy 具体 PHY 的 IPhy 接口，借用，生命周期须覆盖本对象全部使用期。
     */
    LwipPlatform(ETH_HandleTypeDef& eth, IPhy& phy) noexcept;

    /** @brief 静态存储期对象；析构不销毁 PHY、不停止 MAC、不操作 RTOS。 */
    ~LwipPlatform() = default;

    LwipPlatform(const LwipPlatform&)            = delete;
    LwipPlatform& operator=(const LwipPlatform&) = delete;
    LwipPlatform(LwipPlatform&&)                 = delete;
    LwipPlatform& operator=(LwipPlatform&&)      = delete;

    /**
     * @brief 一次性启动：保存运行参数并创建唯一链路线程。
     *
     * 前置：调用方已调用 HAL_ETH_Init 完成 MAC 外设初始化，netif 已建立，且从普通线程
     * 上下文调用；本类不初始化 MAC 外设。创建前把所有运行参数（netif 指针、config 副本、
     * 错误回调、是否使用中断）存妥，再由线程自行发布其 ID，创建者不在创建返回后写回 ID。
     *
     * config 被原样传递给 phy_.start()；本类不覆盖调用方配置，也不静默取能力交集。
     * 线程创建失败返回 false，且不再自动重试；返回 true 仅表示线程已创建，PHY 启动在线程
     * 内异步进行，其失败经 on_error 上报 PhyStartError，不代表本函数失败。
     *
     * @param[in] netif 已建立的 LwIP 接口，仅借用，不拥有。
     * @param[in] config 期望的链路配置，原样传递。
     * @param[in] on_error 致命错误回调，必须非空；在线程内被调用一次。
     * @return true 线程创建成功；false 表示重复调用、空回调或线程创建失败，不启动 PHY。
     */
    [[nodiscard]] bool start(struct netif& netif, const PhyLinkConfig& config, ErrorHandler on_error) noexcept;

    /**
     * @brief ETH TX 完成 ISR 入口：仅通知链路线程回收 TX，不访问 PHY/MAC/HAL。
     *
     * 可在 ISR 上下文调用。线程 ID 已发布时无条件置 TX 标志：两种事件模式都经
     * `osThreadFlagsWait` 唤醒处理 TX——无 INT 轮询模式同样立即回收，不等到下一轮 PHY poll。
     * 线程未发布 ID（未启动/已终止）时无操作。
     */
    void notifyTxComplete() noexcept;

private:
    /** @brief 线程入口桥接：转调 run()。 */
    static void link_thread(void* argument) noexcept;

    /** @brief PHY 事件回调（可能 ISR）：仅置 Link 标志并唤醒线程。 */
    static void onPhyEvent(void* context) noexcept;

    /**
     * @brief 链路线程主循环：启动 PHY 后统一等待 TX/Link 线程标志与周期 PHY poll。
     *
     * 无 INT 模式按绝对 deadline 每 kPollDelayTicks 执行一轮 poll；有 INT 模式在 INT 为高时
     * 无限期等待事件，在 INT 卡低且确认后仍无事件时退化为同一周期 poll（每轮有界阻塞，
     * 不存在断言低电平的忙等）。持续到达的 TX 通知不会推迟该节拍。
     */
    void run() noexcept;

    /**
     * @brief 线程内 PHY 启动：core lock 内置 link down，锁外按 config 启动 PHY。
     * @return true 仅当 PHY start 成功；成功置 initialized_。
     */
    [[nodiscard]] bool initialize() noexcept;

    /**
     * @brief 线程内单轮：读取 PHY 链路并按需切换 MAC 与 netif link。
     *
     * readLink() 每轮只调用一次，返回值用于区分失败原因；不额外访问 PHY 总线。
     * @return 本轮结果（语义见 PollResult）。
     */
    [[nodiscard]] PollResult poll() noexcept;

    /**
     * @brief 确认 PHY 中断，并在有链路事件或强制快照时执行一轮 poll。
     * @param[in] force_snapshot 为 true 时无论是否有链路事件都执行一轮 poll（首轮快照）。
     * @return ack 成功时返回 poll 结果或 Ok；ack 失败返回 PhyInterruptError。
     */
    [[nodiscard]] PollResult handleLinkEvent(bool force_snapshot) noexcept;

    /**
     * @brief 执行一轮周期 PHY poll，并按事件模式判定能否继续。
     *
     * 无 INT 轮询模式容忍单轮 PhyReadError（poll 已安全停链），返回 true 以便下一周期重试；
     * 有 INT 模式（含 INT 卡低退化出的周期 poll）与其余非 Ok 结果都走终止路径，不新增策略。
     * @return true 可继续循环；false 表示已调用 fail（线程即将终止）。
     */
    [[nodiscard]] bool pollPeriodic() noexcept;

    /**
     * @brief 只回收已完成 TX：在 core lock 内调用 HAL_ETH_ReleaseTxPacket，不读取 PHY。
     *
     * 供 TX 完成通知路径复用：不产生 MDIO 流量、不改变链路状态。需 core lock，不得在 ISR
     * 调用；未启动时无任何硬件动作直接返回。
     */
    void releaseTxPackets() noexcept;

    /**
     * @brief 终止路径：断开 PHY 事件回调、清空线程 ID、置 link down，再调用 on_error 一次，
     *        然后结束线程（osThreadExit）。不冒充 MAC 已停止，不重试失败的 HAL 操作。
     */
    void fail(PollResult error) noexcept;

    /** @brief 在 core lock 内置链路为 down（幂等）。 */
    void setLinkDownLocked() noexcept;

    /** @brief 在 core lock 内置链路为 up（幂等；仅在 netif 尚未 up 时调用）。 */
    void setLinkUpLocked() noexcept;

    /**
     * @brief 停止 MAC；前置：已持 core lock。
     * @return true 已停止或本就未启动；false HAL_ETH_Stop_IT 失败，此时保留
     *         mac_started_（停止失败不得谎称已停）。
     */
    bool stopMacLocked() noexcept;

    /**
     * @brief 配置并启动 MAC；前置：已持 core lock 且 MAC 已停止。
     * @return true 仅当 Get/Set/Start 全部成功。
     */
    bool startMacLocked(std::uint32_t speed, std::uint32_t duplex) noexcept;

    /** @brief Link 事件线程标志（ISR 置位）；由 osThreadFlagsWait 唤醒时默认清除。 */
    static constexpr std::uint32_t kLinkFlag{ 1UL << 0U };
    /** @brief TX 完成线程标志（ISR 置位）；由 osThreadFlagsWait 唤醒时默认清除。 */
    static constexpr std::uint32_t kTxFlag{ 1UL << 1U };
    /** @brief 链路线程栈字节数，沿用原 EthLink 线程参数。 */
    static constexpr std::uint32_t kStackBytes{ 1024U };
    /** @brief 周期 PHY poll 节拍（tick）；无 INT 轮询模式与 INT 卡低退化均按绝对 deadline 使用。 */
    static constexpr std::uint32_t kPollDelayTicks{ 100U };

    ETH_HandleTypeDef& eth_; ///< 借用，不拥有。
    IPhy&              phy_; ///< 借用，不拥有。

    /* 线程运行参数：必须在创建线程前存妥，创建后只读。 */
    struct netif* bound_netif_{ nullptr };           ///< 借用 netif，创建前绑定。
    PhyLinkConfig link_config_{};                    ///< PHY 启动配置副本。
    ErrorHandler  on_error_{ nullptr };              ///< 致命错误回调，start 校验非空。
    bool          interrupt_mode_{ false };          ///< start 前缓存的 phy_.usesInterrupt()。
    std::atomic<osThreadId_t> thread_id_{ nullptr }; ///< 由工作线程自身发布，供 ISR 通知定位。

    bool          initialized_{ false };   ///< 仅线程内 PHY start 成功后置 true。
    bool          start_attempted_{ false }; ///< start 一次性尝试标志，仅 start 检查。
    bool          mac_started_{ false };
    /* 最近一次成功提交的 MAC 链路模式；仅当 mac_started_ 时有效。 */
    PhyLinkState applied_state_{ PhyLinkState::Down };

    /* 静态线程资源：覆盖线程整个生命周期；FreeRTOSConfig 已启用静态分配。 */
    StaticTask_t thread_control_block_{};
    alignas(8) StackType_t thread_stack_[kStackBytes / sizeof(StackType_t)]{};
};

static_assert(std::atomic<osThreadId_t>::is_always_lock_free,
              "LwipPlatform 需要无锁 atomic<osThreadId_t> 以在线程与 ISR 间安全发布 ID。");

} // namespace bsp::ethernet_phy

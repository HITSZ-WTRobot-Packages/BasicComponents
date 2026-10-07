/**
 * @file    LwipPlatform.hpp
 * @brief   PHY 无关的 STM32 ETH MAC 与 LwIP netif 平台状态机。
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * 职责边界：
 *   - 只通过借用的 IPhy& 启动 PHY、查询链路并切换 MAC 速度/双工；不依赖具体 PHY
 *     类型、地址、MDIO 总线或板级 MSP，器件复位与寄存器配置由具体 PHY 驱动负责。
 *   - 不实现自动重试、遥测或工程错误处理策略；init 返回 false、poll 返回非 Ok 时
 *     由调用方决定处理流程。
 *   - 不拥有 ETH 句柄、PHY 或 netif；这些由调用方创建并保证生命周期覆盖线路线程的
 *     全部运行期。
 *
 * 生命周期与所有权：
 *   - 构造只保存引用并初始化成员，不访问 HAL、PHY 或 RTOS，不产生总线流量，不失败；
 *     因此可安全地声明为静态存储期对象。
 *   - 借用 eth 与 phy 必须在其整个使用期内有效；对象不被拷贝/移动，也不提供 shutdown
 *     或运行期替换 PHY。
 *   - 析构不销毁 PHY、不停止 MAC、不操作 RTOS。
 *
 * 锁序与并发：
 *   - MAC 转换（Get/Set/Start/Stop）与 netif link 变更统一在 LwIP core lock 内完成；
 *     本类不持有任何自身互斥体。
 *   - PHY 的 start()/readLink() 始终在 core lock 外调用。
 *   - init()/poll()/releaseTxPackets() 只由同一 EthLink 线程串行调用，不得并发。
 *
 * 编译前提：
 *   - 调用方须先调用 HAL_ETH_Init 完成 MAC 外设初始化；本类不初始化 MAC 外设。
 *   - 依赖 LWIP_TCPIP_CORE_LOCKING=1（在 .cpp 中以 #error 强制）。
 */
#pragma once

#include "IPhy.hpp"

#include "main.h"

#include <cstdint>

/* LwIP 类型前置声明：必须位于全局命名空间，避免在 bsp::ethernet_phy 内引入同名新类型。 */
struct netif;

namespace bsp::ethernet_phy
{

/**
 * @brief poll() 的单轮结果；每轮恰返回一个值，调用方据此决定重试、容忍或退出。
 *
 * 仅 Ok 与 PhyReadError 表示本轮可继续运行（后者可由调用方容忍并在下一轮重试）；
 * 其余取值均要求调用方按失败处理。
 */
enum class PollResult : std::uint8_t
{
    Ok,              ///< 本轮链路状态已成功应用，含正常断链、协商及 MAC 启动成功。
    PhyReadError,    ///< readLink 寄存器读取失败；已按断链清理，下一轮可重试。
    MacStopError,    ///< 需要停止 MAC 但 HAL_ETH_Stop_IT 失败；不谎称已停。
    MacStartError,   ///< 获取/设置 MAC 配置或启动 MAC 失败；链路保持 down。
    NotInitialized,  ///< 本对象未 init，或 readLink 报 PHY 未就绪；后者已按断链清理。
    InvalidPhyState, ///< readLink 返回契约外结果，或返回未知链路状态；不继续配置。
};

/**
 * @brief 借用 ETH 句柄与 IPhy 的 LwIP 平台状态机。
 */
class LwipPlatform final
{
public:
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
     * @brief 一次性初始化：绑定 netif、置 link down，并把 config 原样交给 PHY 启动。
     *
     * 前置：调用方已调用 HAL_ETH_Init 完成 MAC 外设初始化；本类不初始化 MAC 外设。
     * config 被原样传递给 phy_.start()；本类不覆盖调用方配置，也不静默取能力交集，
     * 不支持的模式由具体 IPhy::start 报错并令本函数返回 false。
     *
     * @param[in] netif 已建立的 LwIP 接口，仅借用，不拥有。
     * @param[in] config 期望的链路配置，原样传递。
     * @return true 平台可用于 poll；false（重复 init 或 PHY 启动失败）。
     */
    bool init(struct netif& netif, const PhyLinkConfig& config) noexcept;

    /**
     * @brief 读取 PHY 链路并按需切换 MAC 与 netif link。
     *
     * readLink() 每轮只调用一次，返回值用于区分失败原因；不额外访问 PHY 总线。
     *
     * @return 本轮结果：
     *         - Ok：链路状态已成功应用，含正常断链、协商及 MAC 启动成功；稳定 Up
     *           且模式未变时只回收 TX。
     *         - PhyReadError：readLink 报寄存器读取失败；已置 link down、回收已完成
     *           TX，并确认 MAC 已停止或本就未启动，可由下一周期重新检查。
     *         - NotInitialized：本对象未 init 时直接返回；若 readLink 报 PHY 未就绪，
     *           则完成断链清理后返回。
     *         - InvalidPhyState：readLink 返回契约外的非 Ok 结果（已按断链清理）；或
     *           readLink 成功但链路状态既非 Down/Negotiating 也非四种已知 Up 模式
     *           （不继续配置，链路保持原样）。
     *         - MacStopError：断链或模式切换需要停 MAC，但 HAL_ETH_Stop_IT 失败；
     *           不继续后续配置，停止失败优先上报而不被其它错误掩盖。
     *         - MacStartError：获取/设置 MAC 配置或启动 MAC 失败；链路保持 down。
     *         本函数不重试，错误处理留给调用方。
     */
    [[nodiscard]] PollResult poll() noexcept;

    /**
     * @brief 只回收已完成 TX：在 core lock 内调用 HAL_ETH_ReleaseTxPacket，不读取 PHY。
     *
     * 供 TX 完成通知路径复用：事件线程被 TX 完成中断唤醒后调用本函数，即可释放
     * 最后一次发送占用的描述符与 pbuf 引用，而不产生任何 MDIO 流量、不改变链路
     * 状态。与 poll() 不同，本函数不查询 PHY、不切换 MAC，可在纯 TX 事件下单独调用。
     *
     * 前置：由 EthLink 线程调用（需要 core lock，不可在 ISR 中调用），且不得与 init()/
     * poll() 并发。未 init 时无任何硬件动作直接返回。
     */
    void releaseTxPackets() noexcept;

private:
    /** @brief 在 core lock 内置链路为 down（幂等）。 */
    void setLinkDownLocked() noexcept;

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

    ETH_HandleTypeDef& eth_; ///< 借用，不拥有。
    IPhy&              phy_; ///< 借用，不拥有。

    bool          initialized_{ false };
    bool          init_attempted_{ false };
    bool          mac_started_{ false };
    struct netif* bound_netif_{ nullptr };
    /* 最近一次成功提交的 MAC 链路模式；仅当 mac_started_ 时有效。 */
    PhyLinkState applied_state_{ PhyLinkState::Down };
};

} // namespace bsp::ethernet_phy

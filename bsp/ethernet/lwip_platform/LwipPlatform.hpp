/**
 * @file    LwipPlatform.hpp
 * @brief   PHY 无关的 STM32 ETH MAC 与 LwIP netif 平台状态机。
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * 职责边界：
 *   - 只通过借用的 IPhy& 启动 PHY、查询链路并切换 MAC 速度/双工；不依赖具体 PHY
 *     类型、地址、MDIO 总线或板级 MSP，器件复位与寄存器配置由具体 PHY 驱动负责。
 *   - 不实现自动重试、遥测或工程错误处理策略；prepare/init/poll 返回 false 时由
 *     调用方决定终止流程。
 *   - 不拥有 ETH 句柄、PHY、netif 或 RX 信号量；这些由调用方创建并保证生命周期覆盖
 *     接收线程与链路线程的全部运行期。
 *
 * 生命周期与所有权：
 *   - 构造只保存引用并初始化成员，不访问 HAL、PHY 或 RTOS，不产生总线流量，不失败；
 *     因此可安全地声明为静态存储期对象。
 *   - 借用 eth 与 phy 必须在其整个使用期内有效；对象不被拷贝/移动（静态 mutex 控制块
 *     地址属于本实例），也不提供 shutdown 或运行期替换 PHY。
 *   - 析构不销毁 PHY、不停止 MAC、不操作 RTOS。
 *
 * 锁序与并发：
 *   - 固定锁序为 LwIP core lock → 本实例 RX 互斥体。
 *   - PHY 的 start()/readLink() 始终在 core lock 外调用。
 *   - MAC 转换（Get/Set/Start/Stop）同时受 core lock 与 RX 互斥体保护；原始接收回调
 *     在 RX 互斥体内执行，netif link 变更与 netif->input/pbuf_free 在 RX 互斥体外。
 *   - init()/poll() 只由同一 EthLink 线程串行调用，不得并发。
 *
 * 编译前提：
 *   - 依赖 LWIP_TCPIP_CORE_LOCKING=1（在 .cpp 中以 #error 强制）。
 */
#pragma once

#include "FreeRTOS.h"
#include "IPhy.hpp"

#include "cmsis_os2.h"
#include "main.h"

#include <cstdint>

/* LwIP 类型前置声明：必须位于全局命名空间，避免在 bsp::ethernet_phy 内引入同名新类型。 */
struct netif;
struct pbuf;

namespace bsp::ethernet_phy
{

/**
 * @brief 借用 ETH 句柄与 IPhy 的 LwIP 平台状态机。
 */
class LwipPlatform final
{
public:
    /** @brief 原始接收回调类型；负责 DMA/RX pool 读取，返回收到的 pbuf 或 nullptr。 */
    using ReceiveFunction = struct pbuf* (*)(struct netif*);

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
     * @brief 创建本实例的静态 RX 互斥体；不注册 MSP、不启动 PHY。
     * @return true 已就绪（可重复调用，幂等）；false 互斥体创建失败，调用方须终止初始化。
     */
    bool prepare() noexcept;

    /**
     * @brief 一次性初始化：绑定 netif、置 link down，并把 config 原样交给 PHY 启动。
     *
     * config 被原样传递给 phy_.start()；本类不覆盖调用方配置，也不静默取能力交集，
     * 不支持的模式由具体 IPhy::start 报错并令本函数返回 false。
     *
     * @param[in] netif 已建立的 LwIP 接口，仅借用，不拥有。
     * @param[in] config 期望的链路配置，原样传递。
     * @return true 平台可用于 poll；false（未 prepare、重复 init 或 PHY 启动失败）。
     */
    bool init(struct netif& netif, const PhyLinkConfig& config) noexcept;

    /**
     * @brief 读取 PHY 链路并按需切换 MAC 与 netif link。
     * @return true 本轮状态已处理（含正常断链/协商）；false 表示 PHY 读取或 MAC
     *         配置/启停失败，链路保持在 down。本函数不重试，错误处理留给调用方。
     */
    bool poll() noexcept;

    /**
     * @brief 在 RX 互斥体内执行原始接收回调，返回前释放互斥体。
     * @return 收到的 pbuf，或 nullptr（未 prepare、空回调或取锁失败）。不得从 ISR 调用。
     */
    struct pbuf* receive(struct netif& netif, ReceiveFunction receive_fn);

    /**
     * @brief RX 线程入口：等待给定信号量，逐包提交给 LwIP，提交失败则释放。
     *
     * 未 prepare、信号量为空、回调为空或 netif.input 为空时直接返回，不访问 HAL；
     * 本函数不创建/删除任务或信号量。
     */
    void input(struct netif& netif, osSemaphoreId_t rx_semaphore, ReceiveFunction receive_fn);

private:
    /** @brief 在 core lock 内置链路为 down（幂等）。 */
    void setLinkDownLocked() noexcept;

    /**
     * @brief 停止 MAC；前置：已持 core lock，内部获取/释放 RX 互斥体。
     * @return true 已停止或本就未启动；false 取锁失败或 HAL_ETH_Stop_IT 失败，
     *         此时保留 mac_started_（停止失败不得谎称已停）。
     */
    bool stopMacLocked() noexcept;

    /**
     * @brief 配置并启动 MAC；前置：已持 core lock 且 MAC 已停止。
     * @return true 仅当 Get/Set/Start 全部成功。
     */
    bool startMacLocked(std::uint32_t speed, std::uint32_t duplex) noexcept;

    ETH_HandleTypeDef& eth_; ///< 借用，不拥有。
    IPhy&              phy_; ///< 借用，不拥有。

    StaticSemaphore_t rx_mutex_storage_{}; ///< 本实例静态互斥体控制块。
    osMutexId_t       rx_mutex_{ nullptr };
    bool              prepared_{ false };
    bool              initialized_{ false };
    bool              init_attempted_{ false };
    bool              mac_started_{ false };
    struct netif*     bound_netif_{ nullptr };
    /* 最近一次成功提交的 MAC 链路模式；仅当 mac_started_ 时有效。 */
    PhyLinkState applied_state_{ PhyLinkState::Down };
};

} // namespace bsp::ethernet_phy

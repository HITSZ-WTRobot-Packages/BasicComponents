/**
 * @file    DP83822Phy.hpp
 * @brief   DP83822 系列以太网 PHY（RMII，10BASE-Te / 100BASE-TX 铜缆）的 IPhy 实现
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * 本实现直接通过 STM32 HAL 的 MDIO 接口访问 DP83822 寄存器，不依赖任何中间驱动
 * 后端、状态码映射或 TCP/IP 协议栈。完整许可与第三方声明见 DP83822Phy.cpp。
 */
#pragma once

#include "IPhy.hpp"

#include "main.h"

#ifndef HAL_ETH_MODULE_ENABLED
#    error "DP83822Phy requires HAL ETH enabled. Please enable ETH in CubeMX."
#endif

#include <cstdint>

namespace bsp::ethernet_phy
{

// DP83822（RMII，10BASE-Te / 100BASE-TX 铜缆）的 IPhy 实现。
//
// 生命周期与所有权：
//   - 调用者必须先完成 HAL_ETH_Init；本类不初始化 ETH/MAC，也不控制 MAC。
//   - eth 句柄为借用引用，其生命周期必须覆盖本对象的全部使用期；本类不拥有它。
//   - 本类自身不可拷贝、不可移动；析构不访问硬件，PHY 与 ETH 均保持原状。
//   - 构造函数只保存配置，不访问硬件、不产生总线流量、不会失败；因此可安全声明为
//     静态存储期对象，构造顺序不受 HAL 初始化影响。
//   - 会失败的启动集中在 start(config)：必须在 HAL_ETH_Init 完成、MDIO 时钟与 HAL_GetTick()
//     可用之后调用；成功后对象可用（status() == Ok）。start() 可重复调用，每次重新完整
//     启动并按 config 配置；运行期写入失败后再次调用 start(config) 即可恢复，无需销毁重建。
//
// 并发与初始化前提：
//   - 同一 MDIO 资源上必须串行使用，调用者负责串行化；本类不提供锁或重试。
//   - 只有 start() 与操作方法才发起 MDIO 访问；构造与析构都不访问硬件。
//   - start() 要求 ETH 外设及 MDIO 时钟已经就绪，且 HAL_GetTick() 可用。
//
// 错误与输出契约：
//   - start() 总是执行完整复位初始化，可能中断既有链路，并按传入 config 应用协商通告掩码
//     或强制单一模式；不会先写入默认的全部模式协商配置。
//   - status() 在 start() 后固定：Ok 表示就绪；未成功 start() 前为 NotInitialized；start()
//     失败时为该次失败原因；运行期写入失败后为 WriteError。该结果不因其它调用改变。
//   - getCapabilities()/readLink() 仅在成功时更新输出，失败时输出保持不变；能力缓存来自
//     start()，不产生额外总线读取。
//   - configureLink() 比较真实寄存器，不重复写入相同有效配置；写失败使对象失效。
//
// 前置条件：
//   - start() 依赖 HAL_ETH_Init 完成与可用的 HAL tick（复位自清零轮询）；除复位等待
//     （上限 500ms）外不做链路等待，整体为阻塞调用；该上限不是任何上下文下的总耗时上限。
//   - MDIO 为共享总线，同一总线上的全部 PHY 访问必须由调用者串行化。
class DP83822Phy final : public IPhy
{
public:
    /** @brief 自动探测 PHY 地址的哨兵值：扫描 0–31 并按 PHY ID 匹配。 */
    static constexpr std::uint32_t AutoAddress = 0xFFFFFFFFU;

    /**
     * @brief 只保存配置的构造函数：不访问硬件、不产生总线流量、不会失败。
     * @param[in] eth     已存在的 ETH 句柄，借用并保存引用；其生命周期须覆盖本对象。
     * @param[in] address 原理图确定的 PHY 地址（0–31），或 AutoAddress 自动探测。
     *
     * @note 构造后对象尚未就绪（status() == NotInitialized）；必须在 HAL_ETH_Init 完成、
     *       MDIO 时钟与 HAL_GetTick() 可用之后调用 start(config)。因此构造函数可安全用于
     *       静态存储期对象。
     */
    DP83822Phy(ETH_HandleTypeDef& eth, std::uint32_t address) noexcept;

    /**
     * @brief 启动驱动并按 config 配置链路；每次调用都完整重建。
     *
     * 流程：地址校验与 config 形状校验（InvalidArgument，不访问总线）→ 设置 MDIO 时钟范围
     * → 绑定或探测 PHY 地址 → 通过 PHYRCR bit 15 执行等效硬件复位并等待自清零 → 使能
     * Auto-MDIX 与 Robust Auto-MDIX → 从 BMSR 检测并缓存能力（只要有一种支持模式即成功，
     * 自动协商能力单独记录）→ 按缓存能力校验 config（不受支持返回 Unsupported，此时可能
     * 已复位并写入寄存器）→ 按 config 下发通告掩码或强制模式（相同有效配置不重复写入）。
     *
     * config 被原样应用，不会被默认全部模式协商或模式交集替换。可重复调用：中断既有链路并
     * 丢弃此前配置。阻塞调用，复位自清零等待上限 500ms，该上限不是任何上下文下的总耗时上限。
     *
     * @note 完整复位会重新锁存 strap，芯片可能自行按 strap 启动协商；本接口只保证软件不先
     *       下发额外的默认全部模式协商配置。恢复时由调用方再次传入期望配置，不隐式保存或回退。
     *
     * @par 配置示例
     * start({true, PhyLinkMode::Base10Full | PhyLinkMode::Base100Full}) 仅通告全双工模式；
     * start({false, PhyLinkMode::Base100Full}) 强制 100 Mbps 全双工。两者均须检查返回值，
     * 再通过 readLink() 查询链路；本驱动不启动 MAC，也不更新协议栈状态。
     *
     * @param[in] config 期望的链路配置；强制模式必须恰含一种模式。
     * @return Ok 对象就绪（不表示链路已建立）；InvalidArgument 地址或 config 非法
     *         （零总线访问）；NotFound/AmbiguousAddress 选址失败；ReadError MDIO 读取失败；
     *         ResetTimeout 复位未完成；WriteError 写失败；Unsupported 器件无任何支持模式，
     *         或请求的模式/自动协商不受支持（此前可能已复位或写入寄存器）。
     *         任何失败都使对象不可用，失败后再次调用 start(config) 即可重试/恢复。
     */
    [[nodiscard]] PhyResult start(const PhyLinkConfig& config) noexcept override;

    /** @brief 析构不访问硬件；PHY 与 ETH 均保持原状。 */
    ~DP83822Phy() = default;

    DP83822Phy(const DP83822Phy&)            = delete;
    DP83822Phy& operator=(const DP83822Phy&) = delete;
    DP83822Phy(DP83822Phy&&)                 = delete;
    DP83822Phy& operator=(DP83822Phy&&)      = delete;

    /**
     * @brief 查询最近一次 start() 的粘性状态。
     * @return Ok 启动成功且对象可用；NotInitialized 尚未成功 start()；其它值表示最近一次
     *         start() 失败原因，或运行期写入失败导致对象失效后的 WriteError。该结果不因
     *         其它调用改变。
     */
    [[nodiscard]] PhyResult status() const noexcept override;

    /**
     * @brief 读取当前链路状态。
     * @param[out] link_state 仅在成功时写入解码后的链路状态。
     * @return Ok 成功（含正常断链 Down）；NotInitialized 表示 status() != Ok（不访问
     *         硬件）；ReadError 表示 MDIO 读取失败。
     * @note 失败时 link_state 保持不变。
     * @note 先读取 BMSR；链路位为低时再读一次，清除历史断链锁存后再解码 PHYSTS。
     */
    [[nodiscard]] PhyResult readLink(PhyLinkState& link_state) noexcept override;

    /**
     * @brief 读取 start() 时缓存的器件能力，不访问硬件。
     * @param[out] capabilities 仅在返回 Ok 时写入缓存值。
     * @return NotInitialized 表示 status() != Ok，输出保持不变。
     */
    [[nodiscard]] PhyResult getCapabilities(PhyCapabilities& capabilities) noexcept override;

    /**
     * @brief 配置自动协商通告掩码或强制单一模式。
     * @param[in] config 非法返回 InvalidArgument，器件不支持返回 Unsupported，均不写寄存器。
     * @return NotInitialized 表示 status() != Ok（不访问硬件）；ReadError 读取现状失败
     *         （对象仍就绪）；WriteError 写入失败使对象失效；相同有效配置不重复写入并返回 Ok。
     *
     * 自动协商配置要求器件支持自动协商（start() 时缓存的能力），否则返回 Unsupported；
     * 强制模式不要求该能力。
     */
    [[nodiscard]] PhyResult configureLink(const PhyLinkConfig& config) noexcept override;

    /**
     * @brief 显式重启自动协商，返回时协商尚未完成。
     * @return NotInitialized 表示 status() != Ok（不访问硬件）；Unsupported 当前为强制模式
     *         （无写入）；ReadError 读取 BMCR 失败（对象仍就绪）；WriteError 写入失败使对象失效。
     */
    [[nodiscard]] PhyResult restartAutoNegotiation() noexcept override;

private:
    /** @brief 纯 config 形状校验：空掩码、未知位或强制多模式 → InvalidArgument；不访问硬件。 */
    [[nodiscard]] static PhyResult validateConfig(const PhyLinkConfig& config) noexcept;
    /** @brief 按缓存能力校验 config 是否受支持；不受支持返回 Unsupported，不访问硬件。 */
    [[nodiscard]] PhyResult validateConfigSupport(const PhyLinkConfig& config) const noexcept;
    /** @brief 读取真实现状并按 config 配置 ANAR/BMCR；相同有效配置不重复写入。 */
    [[nodiscard]] PhyResult applyConfig(const PhyLinkConfig& config) noexcept;
    /** @brief 读取一个 PHY 寄存器低 16 位；失败时不改动 value。 */
    [[nodiscard]] PhyResult readRegister(std::uint16_t reg, std::uint16_t& value) noexcept;
    /** @brief 写入一个 PHY 寄存器；失败返回 WriteError。 */
    [[nodiscard]] PhyResult writeRegister(std::uint16_t reg, std::uint16_t value) noexcept;
    /** @brief 读改写寄存器：只置位 bits 指定的位，其余位保持原值。 */
    [[nodiscard]] PhyResult setRegisterBits(std::uint16_t reg, std::uint16_t bits) noexcept;
    /** @brief 在指定地址读取 PHYIDR1/2 并严格比对型号族 ID。 */
    [[nodiscard]] PhyResult probeAddress(std::uint32_t address, bool& matched) noexcept;
    /** @brief 按 requested_address_ 绑定地址，或自动扫描 0–31 得到唯一地址。 */
    [[nodiscard]] PhyResult selectAddress() noexcept;
    /** @brief 完整复位 PHY 并重新应用默认 Auto-MDIX 配置。 */
    [[nodiscard]] PhyResult resetAndConfigure() noexcept;
    /** @brief 从 BMSR 检测器件能力并缓存；无任何支持模式时返回 Unsupported（不支持自动协商不算失败）。 */
    [[nodiscard]] PhyResult readAndCacheCapabilities() noexcept;
    /** @brief 写失败后使对象失效（status_ = WriteError、address_ = 0）；可被下一次 start() 清除。 */
    PhyResult failAfterWrite() noexcept;

    ETH_HandleTypeDef&  eth_;               /**< 借用的 ETH 句柄，生命周期由调用者保证。 */
    const std::uint32_t requested_address_; /**< 构造时固定的 PHY 地址或 AutoAddress。 */
    std::uint32_t       address_     = 0U;  /**< 当前绑定地址（0–31）；未就绪时为 0。 */
    PhyResult           status_      = PhyResult::NotInitialized; /**< start() 与写失败后的粘性状态。 */
    PhyLinkMode         supported_modes_ = PhyLinkMode::None; /**< start() 时缓存的器件模式。 */
    bool                autoneg_supported_ = false; /**< start() 时缓存的自动协商能力。 */
};

} // namespace bsp::ethernet_phy

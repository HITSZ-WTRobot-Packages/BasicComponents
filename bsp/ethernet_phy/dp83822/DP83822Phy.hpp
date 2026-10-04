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

// DP83822（RMII，10BASE-Te / 100BASE-TX 铜缆）的 IPhy 实现。
//
// 生命周期与所有权：
//   - 调用者必须先完成 HAL_ETH_Init；本类不初始化 ETH/MAC，也不控制 MAC。
//   - eth 句柄为借用引用，其生命周期必须覆盖本对象的全部使用期；本类不拥有它。
//   - 本类自身不可拷贝、不可移动；析构不访问硬件，PHY 与 ETH 均保持原状。
//
// 并发与初始化前提：
//   - 同一 MDIO 资源上必须串行使用，调用者负责串行化；本类不提供锁或重试。
//   - 构造与析构都不访问硬件，只有 init()/readLink() 才会发起 MDIO 访问。
//   - init() 要求 ETH 外设及 MDIO 时钟已经就绪。
//
// 错误与输出契约：
//   - init() 每次都会执行完整复位初始化；任何失败都撤销就绪状态并清空地址，
//     允许显式重试。
//   - readLink() 仅在成功时更新输出，失败时输出保持不变。
class DP83822Phy final : public IPhy
{
public:
    /** @brief 自动探测 PHY 地址的哨兵值：扫描 0–31 并按 PHY ID 匹配。 */
    static constexpr std::uint32_t AutoAddress = 0xFFFFFFFFU;

    /**
     * @brief 构造，仅保存配置，不访问硬件。
     * @param[in] eth     已存在的 ETH 句柄，借用并保存引用；调用 init() 前须完成 HAL_ETH_Init。
     * @param[in] address 原理图确定的 PHY 地址（0–31），或 AutoAddress 自动探测。
     */
    DP83822Phy(ETH_HandleTypeDef& eth, std::uint32_t address) noexcept;

    /** @brief 析构不访问硬件；PHY 与 ETH 均保持原状。 */
    ~DP83822Phy() = default;

    DP83822Phy(const DP83822Phy&)            = delete;
    DP83822Phy& operator=(const DP83822Phy&) = delete;
    DP83822Phy(DP83822Phy&&)                 = delete;
    DP83822Phy& operator=(DP83822Phy&&)      = delete;

    /**
     * @brief 完整复位并初始化 PHY。
     *
     * 流程：校验地址 → 设置 MDIO 时钟范围 → 绑定或探测 PHY 地址 → 通过 PHYRCR
     * bit 15 执行等效硬件复位并等待自清零 → 使能 Auto-MDIX 与 Robust Auto-MDIX。
     *
     * @return Ok 成功；AddressError 地址超出 0–31 或未探测到器件；AmbiguousAddress
     *         自动探测命中多个器件；ReadError/WriteError MDIO 访问失败；
     *         ResetTimeout 复位未在时限内自清零。
     * @note 每次调用都会重新执行完整初始化；失败时对象保持未就绪，可显式重试。
     */
    [[nodiscard]] PhyResult init() noexcept override;

    /**
     * @brief 读取当前链路状态。
     * @param[out] link_state 仅在成功时写入解码后的链路状态。
     * @return Ok 成功（含正常断链 Down）；NotInitialized 表示尚未成功初始化（不访问
     *         硬件）；ReadError 表示 MDIO 读取失败。
     * @note 失败时 link_state 保持不变。
     */
    [[nodiscard]] PhyResult readLink(PhyLinkState& link_state) noexcept override;

private:
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

    ETH_HandleTypeDef&  eth_;               /**< 借用的 ETH 句柄，生命周期由调用者保证。 */
    const std::uint32_t requested_address_; /**< 构造时固定的 PHY 地址或 AutoAddress。 */
    std::uint32_t       address_     = 0U;  /**< 当前绑定地址（0–31）；未就绪时为 0。 */
    bool                initialized_ = false; /**< 仅当最近一次 init() 完整成功时为 true。 */
};

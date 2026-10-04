/**
 * @file    DP83822Phy.cpp
 * @brief   DP83822 系列以太网 PHY 的 IPhy 实现（原生 C++ 寄存器驱动）
 *
 * 寄存器访问全部通过 STM32 HAL 的 MDIO 接口（HAL_ETH_ReadPHYRegister /
 * HAL_ETH_WritePHYRegister）完成，没有任何动态分配、共享全局状态或 RTOS 依赖。
 * 所有错误都直接以 PhyResult 返回，不经过中间状态码映射。
 *
 * --------------------------------------------------------------------------
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * Project repository: https://github.com/HITSZ-WTRobot-Packages/BasicComponents
 *
 * --------------------------------------------------------------------------
 * DP83822 寄存器定义与 Auto-MDIX 行为参考了 TI 官方 ti-ethernet-software 仓库中的
 * rtos_drivers/include/dp83822.h、rtos_drivers/src/dp83822.c 与
 * rtos_drivers/src/dp83822_priv.h（BSD-3-Clause），并已为 STM32 HAL 重新实现，
 * 未引入其 phy_common / port 框架。以下 TI 版权与许可声明必须保留：
 *
 *  Copyright (c) Texas Instruments Incorporated 2020
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "DP83822Phy.hpp"

namespace
{
/* MDIO 数据域宽度，读取时统一裁剪到低 16 位。 */
constexpr std::uint32_t kDataMask = 0xFFFFU;

/* 地址与器件标识。 */
constexpr std::uint32_t kAddressMin = 0U;          /* MDIO 5 位地址域下限 */
constexpr std::uint32_t kAddressMax = 31U;         /* MDIO 5 位地址域上限 */
constexpr std::uint32_t kPhyId      = 0x2000A240U; /* 型号族 PHY ID 基准 */
constexpr std::uint32_t kPhyIdMask  = 0xFFFFFFF0U; /* 低 4 位为版本号，不比较 */

/* 寄存器地址。 */
constexpr std::uint16_t kRegBmcr    = 0x0000U; /* 基本模式控制 */
constexpr std::uint16_t kRegPhyidr1 = 0x0002U; /* PHY 标识符 #1 */
constexpr std::uint16_t kRegPhyidr2 = 0x0003U; /* PHY 标识符 #2 */
constexpr std::uint16_t kRegCr1     = 0x0009U; /* PHY 控制 #1 */
constexpr std::uint16_t kRegPhysts  = 0x0010U; /* PHY 状态 */
constexpr std::uint16_t kRegPhycr   = 0x0019U; /* PHY 控制 */
constexpr std::uint16_t kRegPhyrcr  = 0x001FU; /* PHY 复位控制 */

/* BMCR 位。 */
constexpr std::uint16_t kBmcrAutonegEnable = 0x1000U; /* bit 12：自动协商使能 */

/* PHYSTS 位。 */
constexpr std::uint16_t kPhystsLinkStatus     = 0x0001U; /* bit 0：链路已建立 */
constexpr std::uint16_t kPhystsSpeed          = 0x0002U; /* bit 1：1 = 10 Mbps */
constexpr std::uint16_t kPhystsDuplex         = 0x0004U; /* bit 2：1 = 全双工 */
constexpr std::uint16_t kPhystsAutonegComplete = 0x0010U; /* bit 4：自动协商完成 */

/* CR1 位。 */
constexpr std::uint16_t kCr1RobustAutoMdix = 0x0020U; /* bit 5：Robust Auto-MDIX */

/* PHYCR 位。 */
constexpr std::uint16_t kPhycrAutoMdixEnable = 0x8000U; /* bit 15：Auto-MDIX */

/* PHYRCR 位。 */
constexpr std::uint16_t kPhyrcrSoftReset = 0x8000U; /* bit 15：等效硬件复位（W1S） */

/* PHYRCR 软件复位自清零等待上限，单位 ms。 */
constexpr std::uint32_t kResetTimeoutMs = 500U;
} // namespace

DP83822Phy::DP83822Phy(ETH_HandleTypeDef& eth, const std::uint32_t address) noexcept
    : eth_(eth),
      requested_address_(address)
{
    /* 构造只保存配置，不访问硬件。 */
}

PhyResult DP83822Phy::init() noexcept
{
    /* 先撤销旧的就绪状态与地址：任何失败路径都不会残留上一次的成功状态。 */
    initialized_ = false;
    address_     = 0U;

    /* 非法地址无需访问硬件即可判定。 */
    if ((requested_address_ != AutoAddress) && (requested_address_ > kAddressMax))
        return PhyResult::AddressError;

    HAL_ETH_SetMDIOClockRange(&eth_);

    PhyResult status = selectAddress();
    if (status != PhyResult::Ok)
    {
        address_ = 0U;
        return status;
    }

    status = resetAndConfigure();
    if (status != PhyResult::Ok)
    {
        address_ = 0U;
        return status;
    }

    initialized_ = true;
    return PhyResult::Ok;
}

PhyResult DP83822Phy::readLink(PhyLinkState& link_state) noexcept
{
    /* 尚未成功初始化时不触碰硬件，也不改动输出。 */
    if (!initialized_)
        return PhyResult::NotInitialized;

    std::uint16_t physts = 0U;
    PhyResult status = readRegister(kRegPhysts, physts);
    if (status != PhyResult::Ok)
        return status;

    /* 断链时无需读取 BMCR，直接报告 Down。 */
    if ((physts & kPhystsLinkStatus) == 0U)
    {
        link_state = PhyLinkState::Down;
        return PhyResult::Ok;
    }

    std::uint16_t bmcr = 0U;
    status = readRegister(kRegBmcr, bmcr);
    if (status != PhyResult::Ok)
        return status;

    /*
     * PHYSTS bit 4（自动协商完成）仅在自动协商使能时有效：自动协商模式下该位为 0 时
     * 速度/双工尚未确定，报告 Negotiating；强制模式下直接按 PHYSTS 位报告。
     */
    if (((bmcr & kBmcrAutonegEnable) != 0U) && ((physts & kPhystsAutonegComplete) == 0U))
    {
        link_state = PhyLinkState::Negotiating;
        return PhyResult::Ok;
    }

    const bool speed_10m   = (physts & kPhystsSpeed) != 0U;
    const bool full_duplex = (physts & kPhystsDuplex) != 0U;

    PhyLinkState decoded;
    if (speed_10m)
        decoded = full_duplex ? PhyLinkState::Up10Full : PhyLinkState::Up10Half;
    else
        decoded = full_duplex ? PhyLinkState::Up100Full : PhyLinkState::Up100Half;

    link_state = decoded;
    return PhyResult::Ok;
}

PhyResult DP83822Phy::readRegister(const std::uint16_t reg, std::uint16_t& value) noexcept
{
    std::uint32_t data = 0U;

    if (HAL_ETH_ReadPHYRegister(&eth_, address_, static_cast<std::uint32_t>(reg), &data) != HAL_OK)
        return PhyResult::ReadError;

    value = static_cast<std::uint16_t>(data & kDataMask);
    return PhyResult::Ok;
}

PhyResult DP83822Phy::writeRegister(const std::uint16_t reg, const std::uint16_t value) noexcept
{
    if (HAL_ETH_WritePHYRegister(&eth_,
                                 address_,
                                 static_cast<std::uint32_t>(reg),
                                 static_cast<std::uint32_t>(value)) != HAL_OK)
        return PhyResult::WriteError;

    return PhyResult::Ok;
}

PhyResult DP83822Phy::setRegisterBits(const std::uint16_t reg, const std::uint16_t bits) noexcept
{
    std::uint16_t value = 0U;

    const PhyResult status = readRegister(reg, value);
    if (status != PhyResult::Ok)
        return status;

    return writeRegister(reg, static_cast<std::uint16_t>(value | bits));
}

PhyResult DP83822Phy::probeAddress(const std::uint32_t address, bool& matched) noexcept
{
    matched   = false;
    address_  = address;

    std::uint16_t id1 = 0U;
    PhyResult status = readRegister(kRegPhyidr1, id1);
    if (status != PhyResult::Ok)
        return status;

    std::uint16_t id2 = 0U;
    status = readRegister(kRegPhyidr2, id2);
    if (status != PhyResult::Ok)
        return status;

    const std::uint32_t id =
        (static_cast<std::uint32_t>(id1) << 16) | static_cast<std::uint32_t>(id2);
    matched = (id & kPhyIdMask) == kPhyId;
    return PhyResult::Ok;
}

PhyResult DP83822Phy::selectAddress() noexcept
{
    if (requested_address_ == AutoAddress)
    {
        std::uint32_t candidate = 0U;
        std::uint32_t matches   = 0U;

        for (std::uint32_t addr = kAddressMin; addr <= kAddressMax; ++addr)
        {
            bool matched = false;

            /* 无器件时 MDIO 读可能失败，这类地址按无匹配处理并继续扫描。 */
            if (probeAddress(addr, matched) != PhyResult::Ok)
                continue;

            if (!matched)
                continue;

            candidate = addr;
            ++matches;
            if (matches > 1U)
                return PhyResult::AmbiguousAddress;
        }

        if (matches == 0U)
            return PhyResult::AddressError;

        address_ = candidate;
        return PhyResult::Ok;
    }

    bool matched = false;
    const PhyResult status = probeAddress(requested_address_, matched);
    if (status != PhyResult::Ok)
        return status;

    if (!matched)
        return PhyResult::AddressError;

    address_ = requested_address_;
    return PhyResult::Ok;
}

PhyResult DP83822Phy::resetAndConfigure() noexcept
{
    std::uint16_t phyrcr = 0U;

    /*
     * PHYRCR bit 15 与 RESET_N 引脚等效，会重新锁存 strap 并复位整个 PHY；
     * BMCR bit 15 只复位 PCS 且不重新锁存 strap，不能替代完整复位。
     */
    PhyResult status = readRegister(kRegPhyrcr, phyrcr);
    if (status != PhyResult::Ok)
        return status;

    status = writeRegister(kRegPhyrcr, static_cast<std::uint16_t>(phyrcr | kPhyrcrSoftReset));
    if (status != PhyResult::Ok)
        return status;

    const std::uint32_t start_tick = HAL_GetTick();
    for (;;)
    {
        status = readRegister(kRegPhyrcr, phyrcr);
        if (status != PhyResult::Ok)
            return status;

        if ((phyrcr & kPhyrcrSoftReset) == 0U)
            break;

        if ((HAL_GetTick() - start_tick) >= kResetTimeoutMs)
            return PhyResult::ResetTimeout;
    }

    /* 复位清空了寄存器配置，重新应用默认的 Auto-MDIX 配置。 */
    status = setRegisterBits(kRegPhycr, kPhycrAutoMdixEnable);
    if (status != PhyResult::Ok)
        return status;

    return setRegisterBits(kRegCr1, kCr1RobustAutoMdix);
}

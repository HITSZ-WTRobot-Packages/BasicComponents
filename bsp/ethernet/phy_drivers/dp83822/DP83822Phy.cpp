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
constexpr std::uint16_t kRegBmsr    = 0x0001U; /* 基本模式状态 */
constexpr std::uint16_t kRegPhyidr1 = 0x0002U; /* PHY 标识符 #1 */
constexpr std::uint16_t kRegPhyidr2 = 0x0003U; /* PHY 标识符 #2 */
constexpr std::uint16_t kRegAnar    = 0x0004U; /* 自动协商通告 */
constexpr std::uint16_t kRegCr1     = 0x0009U; /* PHY 控制 #1 */
constexpr std::uint16_t kRegPhysts  = 0x0010U; /* PHY 状态 */
constexpr std::uint16_t kRegPhyscr  = 0x0011U; /* PHY 特定控制（中断输出配置） */
constexpr std::uint16_t kRegMisr1   = 0x0012U; /* MII 中断状态 #1：低字节使能、高字节状态（读清） */
constexpr std::uint16_t kRegMisr2   = 0x0013U; /* MII 中断状态 #2：低字节使能、高字节状态（读清） */
constexpr std::uint16_t kRegPhycr   = 0x0019U; /* PHY 控制 */
constexpr std::uint16_t kRegPhyrcr  = 0x001FU; /* PHY 复位控制 */

/* BMCR 位。 */
constexpr std::uint16_t kBmcrAutonegEnable  = 0x1000U; /* bit 12：自动协商使能 */
constexpr std::uint16_t kBmcrRestartAutoneg = 0x0200U; /* bit 9：重启自动协商（自清零） */
constexpr std::uint16_t kBmcrSpeed100       = 0x2000U; /* bit 13：1 = 100 Mbps */
constexpr std::uint16_t kBmcrDuplexFull     = 0x0100U; /* bit 8：1 = 全双工 */

/* BMSR 位。 */
constexpr std::uint16_t kBmsrLinkStatus     = 0x0004U; /* bit 2：链路状态，低电平锁存 */
constexpr std::uint16_t kBmsrAutonegCapable = 0x0008U; /* bit 3：支持自动协商 */
constexpr std::uint16_t kBmsrModeShift      = 11U;     /* bits 11..14：10H/10F/100H/100F */
constexpr std::uint16_t kBmsrModeMask       = 0x7800U;

/* ANAR 位：bits5..8 与 PhyLinkMode 低 4 位同序，bits4..0 为 IEEE 802.3 选择子域。 */
constexpr std::uint16_t kAnarModeShift    = 5U;
constexpr std::uint16_t kAnarModeMask     = 0x01E0U;
constexpr std::uint16_t kAnarSelectorMask = 0x001FU;
constexpr std::uint16_t kAnarIeee8023Sel  = 0x0001U; /* 选择子域值 1 */

/* PHYSTS 位。 */
constexpr std::uint16_t kPhystsLinkStatus      = 0x0001U; /* bit 0：链路已建立 */
constexpr std::uint16_t kPhystsSpeed           = 0x0002U; /* bit 1：1 = 10 Mbps */
constexpr std::uint16_t kPhystsDuplex          = 0x0004U; /* bit 2：1 = 全双工 */
constexpr std::uint16_t kPhystsAutonegComplete = 0x0010U; /* bit 4：自动协商完成 */

/* CR1 位。 */
constexpr std::uint16_t kCr1RobustAutoMdix = 0x0020U; /* bit 5：Robust Auto-MDIX */

/* PHYCR 位。 */
constexpr std::uint16_t kPhycrAutoMdixEnable = 0x8000U; /* bit 15：Auto-MDIX */

/* PHYRCR 位。 */
constexpr std::uint16_t kPhyrcrSoftReset = 0x8000U; /* bit 15：等效硬件复位（W1S） */

/*
 * PHYSCR 位（数据手册 SNLS505H 表 8-17）：bit3 中断极性（1 = 中断时输出低），
 * bit2 测试中断（写 1 会强制产生中断，必须始终为 0），bit1 事件中断使能，
 * bit0 把 INT/PWDN_N 配置为中断输出。
 */
constexpr std::uint16_t kPhyscrInterruptActiveLow    = 0x0008U;
constexpr std::uint16_t kPhyscrTestInterrupt         = 0x0004U;
constexpr std::uint16_t kPhyscrInterruptEnable       = 0x0002U;
constexpr std::uint16_t kPhyscrInterruptOutputEnable = 0x0001U;

/*
 * MISR1/MISR2（表 8-18/8-19）：低字节为各事件的中断使能，高字节为对应状态（读取即清除）。
 * 本驱动只使能链路变化、速率变化、双工变化、协商完成四类事件。
 */
constexpr std::uint16_t kMisr1EventEnable = 0x003CU; /* bits5..2：链路/速率/双工/协商完成 */
constexpr std::uint16_t kMisr1StatusMask  = 0x3C00U; /* bits13..10：上述四类事件的状态 */
constexpr std::uint16_t kMisr2NoEvents    = 0x0000U; /* 关闭全部 MISR2 事件 */

/* PHYRCR 软件复位自清零等待上限，单位 ms。 */
constexpr std::uint32_t kResetTimeoutMs = 500U;

/* RESET_N 引脚复位时序：拉低保持时间与释放后的建立时间，单位 ms（数据手册只要求 >= 10us）。 */
constexpr std::uint32_t kResetPinLowMs     = 1U;
constexpr std::uint32_t kResetPinReleaseMs = 2U;

/* 全部受支持模式位，用于判定未知位与统计强制模式位数。 */
constexpr std::uint8_t kKnownModeBits = static_cast<std::uint8_t>(
        bsp::ethernet_phy::PhyLinkMode::All10_100);

/* 掩码中是否含未定义的模式位。 */
constexpr bool hasUnknownModeBits(const bsp::ethernet_phy::PhyLinkMode modes) noexcept
{
    return (static_cast<std::uint8_t>(modes) & static_cast<std::uint8_t>(~kKnownModeBits)) != 0U;
}

/* 置位个数，用于强制模式“恰好一种模式”的校验。 */
constexpr unsigned bitCount(const std::uint8_t value) noexcept
{
    unsigned count = 0U;
    for (std::uint8_t bits = value; bits != 0U; bits >>= 1U)
        count += static_cast<unsigned>(bits & 1U);
    return count;
}

/* 把模式掩码映射为 ANAR bits5..8 的通告位。 */
constexpr std::uint16_t anarAdvertisement(const bsp::ethernet_phy::PhyLinkMode modes) noexcept
{
    return static_cast<std::uint16_t>(
            static_cast<std::uint16_t>(static_cast<std::uint8_t>(modes) & kKnownModeBits)
            << kAnarModeShift);
}

/* GPIO_t 的“未连接”唯一表示：port 与 pin 同时为空。 */
constexpr bool gpioUnconnected(const GPIO_t& gpio) noexcept
{
    return (gpio.port == nullptr) && (gpio.pin == 0U);
}

/* 已提供的 GPIO 必须 port 非空且 pin 恰为单个 bit，否则视为非法配置。 */
constexpr bool gpioSingleBit(const GPIO_t& gpio) noexcept
{
    return (gpio.port != nullptr) && (gpio.pin != 0U) &&
           ((static_cast<std::uint32_t>(gpio.pin) & (static_cast<std::uint32_t>(gpio.pin) - 1U)) == 0U);
}
} // namespace

namespace bsp::ethernet_phy
{

DP83822Phy::DP83822Phy(ETH_HandleTypeDef& eth,
                       const std::uint32_t address,
                       const GPIO_t        reset_n,
                       const GPIO_t        interrupt_n) noexcept :
    eth_(eth),
    requested_address_(address),
    reset_n_(reset_n),
    interrupt_n_(interrupt_n),
    interrupt_input_(interrupt_n.port, interrupt_n.pin, this)
{
    /* 只保存配置：不访问硬件、不产生总线流量、不会失败，可安全用于静态存储期对象。 */
    /* interrupt_input_ 以 this 作为 user_data，供 EXTI 分发器找回本实例；构造时不注册。 */
}

PhyResult DP83822Phy::start(const PhyLinkConfig& config) noexcept
{
    /* 每次调用都从干净状态完整重建：清除此前就绪/失效状态与能力缓存。 */
    status_            = PhyResult::NotInitialized;
    address_           = 0U;
    supported_modes_   = PhyLinkMode::None;
    autoneg_supported_ = false;

    /* 重建前先撤销自身的旧 EXTI 注册（保留 callback/context，供本次成功后重新挂载）。 */
    if (exti_registered_)
    {
        const std::uint32_t primask = __get_PRIMASK();
        __disable_irq();
        bsp::gpio::UnregisterExtiCallback(&interrupt_input_);
        exti_registered_ = false;
        __set_PRIMASK(primask);
    }

    /* 地址、config 形状与 GPIO 引脚非法必须在任何 HAL/MDIO/引脚动作之前判定。 */
    PhyResult status = PhyResult::Ok;
    if ((requested_address_ != AutoAddress) && (requested_address_ > kAddressMax))
        status = PhyResult::InvalidArgument;
    if (status == PhyResult::Ok)
        status = validateConfig(config);
    if (status == PhyResult::Ok)
        status = validatePins(reset_n_, interrupt_n_);

    if (status == PhyResult::Ok)
    {
        HAL_ETH_SetMDIOClockRange(&eth_);

        /*
         * RESET_N 提供时必须在任何 MDIO 探测之前释放引脚并等待建立时间；此时不再使用
         * PHYRCR 复位，避免对同一芯片做两次完整复位。
         */
        if (hasResetPin())
            pulseResetPin();
    }
    if (status == PhyResult::Ok)
        status = selectAddress();
    if (status == PhyResult::Ok)
        status = softReset(); /* 提供 RESET_N 时空操作 */
    if (status == PhyResult::Ok)
        status = configureAutoMdix();
    if (status == PhyResult::Ok)
        status = readAndCacheCapabilities();
    /* 能力来自启动过程中读到的寄存器：此处 Unsupported 不保证此前没有写入。 */
    if (status == PhyResult::Ok)
        status = validateConfigSupport(config);
    if (status == PhyResult::Ok)
        status = applyConfig(config);
    /* 中断最后才打开：避免把启动过程中的配置写入当成链路事件上报。 */
    if ((status == PhyResult::Ok) && hasInterruptPin())
        status = configureInterrupt();

    if (status != PhyResult::Ok)
        address_ = 0U;

    status_ = status;

    /* 成功设置 status_ 后才为有效 INT 引脚注册软件 EXTI；失败不留下注册，无 INT 不注册。 */
    if ((status_ == PhyResult::Ok) && hasInterruptPin())
    {
        const std::uint32_t primask = __get_PRIMASK();
        __disable_irq();
        bsp::gpio::RegisterExtiCallback(&interrupt_input_, &DP83822Phy::onInterrupt);
        exti_registered_ = true;
        __set_PRIMASK(primask);
    }

    return status_;
}

DP83822Phy::~DP83822Phy() noexcept
{
    /* 只撤销软件 EXTI 回调注册：不访问 MDIO、不复位器件、不动引脚。 */
    if (exti_registered_)
    {
        const std::uint32_t primask = __get_PRIMASK();
        __disable_irq();
        bsp::gpio::UnregisterExtiCallback(&interrupt_input_);
        exti_registered_ = false;
        __set_PRIMASK(primask);
    }
}

void DP83822Phy::setLinkEventCallback(const LinkEventCallback callback, void* const context) noexcept
{
    /*
     * 只保存参数：不注册/修改 EXTI、不改 PHY 配置、不访问硬件。用保存/恢复 PRIMASK 的短
     * 临界区原子更新两个字段，避免 ISR 读到半配置（旧回调配新 context 之类）。
     */
    const std::uint32_t primask = __get_PRIMASK();
    __disable_irq();
    link_event_callback_ = callback;
    link_event_context_  = context;
    __set_PRIMASK(primask);
}

void DP83822Phy::onInterrupt(const bsp::gpio::GpioPinInput* const gpio,
                             const std::uint32_t               counter) noexcept
{
    /* counter 仅用于分发器内部累计，此处不需要。 */
    (void)counter;

    if (gpio == nullptr)
        return;

    const auto* self = static_cast<const DP83822Phy*>(gpio->user_data);
    if (self == nullptr)
        return;

    /* 只转发事件通知：不访问 MDIO、不调用 RTOS；callback/context 由短临界区原子维护。 */
    if (self->link_event_callback_ != nullptr)
        self->link_event_callback_(self->link_event_context_);
}

PhyResult DP83822Phy::validateConfig(const PhyLinkConfig& config) noexcept
{
    if ((config.modes == PhyLinkMode::None) || hasUnknownModeBits(config.modes))
        return PhyResult::InvalidArgument;

    /* 强制模式必须恰含一种模式；这是纯参数约束，与器件能力无关。 */
    if (!config.autoNegotiation && (bitCount(static_cast<std::uint8_t>(config.modes)) != 1U))
        return PhyResult::InvalidArgument;

    return PhyResult::Ok;
}

PhyResult DP83822Phy::validateConfigSupport(const PhyLinkConfig& config) const noexcept
{
    const std::uint8_t requested = static_cast<std::uint8_t>(config.modes);
    const std::uint8_t supported = static_cast<std::uint8_t>(supported_modes_);

    if (config.autoNegotiation)
    {
        /* 自动协商既要求器件支持该能力，也要求请求模式全部在支持范围内。 */
        if (!autoneg_supported_ || ((requested & static_cast<std::uint8_t>(~supported)) != 0U))
            return PhyResult::Unsupported;
        return PhyResult::Ok;
    }

    if ((requested & supported) == 0U)
        return PhyResult::Unsupported;

    return PhyResult::Ok;
}

PhyResult DP83822Phy::status() const noexcept
{
    return status_;
}

PhyResult DP83822Phy::readLink(PhyLinkState& link_state) noexcept
{
    /* 未就绪时不触碰硬件，也不改动输出。 */
    if (status_ != PhyResult::Ok)
        return PhyResult::NotInitialized;

    /*
     * BMSR bit 2 会锁存断链事件；PHYSTS bit 0 是其副本，读 PHYSTS 不会清锁存。
     * 先读 BMSR，若为低再读一次取得当前状态，避免物理链路恢复后仍报告旧 Down。
     * 参见 DP83822 数据手册 SNLS505H 表 8-2 与表 8-16。
     */
    std::uint16_t bmsr   = 0U;
    PhyResult     status = readRegister(kRegBmsr, bmsr);
    if (status != PhyResult::Ok)
        return status;

    if ((bmsr & kBmsrLinkStatus) == 0U)
    {
        status = readRegister(kRegBmsr, bmsr);
        if (status != PhyResult::Ok)
            return status;

        if ((bmsr & kBmsrLinkStatus) == 0U)
        {
            link_state = PhyLinkState::Down;
            return PhyResult::Ok;
        }
    }

    std::uint16_t physts = 0U;
    status               = readRegister(kRegPhysts, physts);
    if (status != PhyResult::Ok)
        return status;

    /* BMSR 与 PHYSTS 读取之间仍可能再次断链，不能使用此时的速率/双工。 */
    if ((physts & kPhystsLinkStatus) == 0U)
    {
        link_state = PhyLinkState::Down;
        return PhyResult::Ok;
    }

    std::uint16_t bmcr = 0U;
    status             = readRegister(kRegBmcr, bmcr);
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

PhyResult DP83822Phy::getCapabilities(PhyCapabilities& capabilities) noexcept
{
    /* 能力缓存仅在 start() 成功后有效，不产生额外总线读取。 */
    if (status_ != PhyResult::Ok)
        return PhyResult::NotInitialized;

    capabilities.modes           = supported_modes_;
    capabilities.autoNegotiation = autoneg_supported_;
    return PhyResult::Ok;
}

PhyResult DP83822Phy::configureLink(const PhyLinkConfig& config) noexcept
{
    if (status_ != PhyResult::Ok)
        return PhyResult::NotInitialized;

    /* 非法/不支持配置必须在任何寄存器访问之前返回。 */
    PhyResult status = validateConfig(config);
    if (status != PhyResult::Ok)
        return status;

    status = validateConfigSupport(config);
    if (status != PhyResult::Ok)
        return status;

    return applyConfig(config);
}

PhyResult DP83822Phy::applyConfig(const PhyLinkConfig& config) noexcept
{
    /* 写入之前先读取真实现状；读取失败只报告错误，不使对象失效。 */
    std::uint16_t bmcr   = 0U;
    std::uint16_t anar   = 0U;
    PhyResult     status = readRegister(kRegBmcr, bmcr);
    if (status != PhyResult::Ok)
        return status;
    status = readRegister(kRegAnar, anar);
    if (status != PhyResult::Ok)
        return status;

    if (!config.autoNegotiation)
    {
        const bool speed_100 = (config.modes & PhyLinkMode::Base100Half) != PhyLinkMode::None ||
                               (config.modes & PhyLinkMode::Base100Full) != PhyLinkMode::None;
        const bool full_duplex = (config.modes & PhyLinkMode::Base10Full) != PhyLinkMode::None ||
                                 (config.modes & PhyLinkMode::Base100Full) != PhyLinkMode::None;

        /*
         * 强制模式：先清掉自动协商使能、速率/双工与 bit9，再按所选模式设置速率/双工位
         * （10M/半双工对应位为 0），其余控制位保持。bit9（Restart Auto-Negotiation）只在
         * bit12 置位时才有意义，因此这里既不置位，也一并清掉继承来的 bit9：强制模式下不得
         * 触发或残留一次协商重启。
         */
        std::uint16_t desired = bmcr;
        desired &= static_cast<std::uint16_t>(
                ~static_cast<std::uint16_t>(kBmcrAutonegEnable | kBmcrSpeed100 | kBmcrDuplexFull |
                                            kBmcrRestartAutoneg));
        if (speed_100)
            desired |= kBmcrSpeed100;
        if (full_duplex)
            desired |= kBmcrDuplexFull;

        /*
         * 幂等比较包含 bit9：即使速率/双工/协调使能已相同，继承来的 bit9=1 也必须通过
         * 一次写入清掉，因此这里直接比较完整寄存器值，不做任何位屏蔽。
         */
        if (bmcr == desired)
            return PhyResult::Ok;

        if (writeRegister(kRegBmcr, desired) != PhyResult::Ok)
            return failAfterWrite();
        return PhyResult::Ok;
    }

    /* 自动协商：按通告掩码更新 ANAR，保留暂停等无关位。 */
    const std::uint16_t desired_anar = static_cast<std::uint16_t>(
            (anar & static_cast<std::uint16_t>(~(kAnarModeMask | kAnarSelectorMask))) |
            anarAdvertisement(config.modes) | kAnarIeee8023Sel);

    const bool was_forced            = (bmcr & kBmcrAutonegEnable) == 0U;
    const bool advertisement_changed = desired_anar != anar;

    /* 已是自动协商且通告未变：不写寄存器、不重启协商。 */
    if (!was_forced && !advertisement_changed)
        return PhyResult::Ok;

    if (advertisement_changed)
    {
        if (writeRegister(kRegAnar, desired_anar) != PhyResult::Ok)
            return failAfterWrite();
    }

    const std::uint16_t desired_bmcr = static_cast<std::uint16_t>(bmcr | kBmcrAutonegEnable |
                                                                  kBmcrRestartAutoneg);
    if (writeRegister(kRegBmcr, desired_bmcr) != PhyResult::Ok)
        return failAfterWrite();

    return PhyResult::Ok;
}

PhyResult DP83822Phy::restartAutoNegotiation() noexcept
{
    if (status_ != PhyResult::Ok)
        return PhyResult::NotInitialized;

    std::uint16_t bmcr   = 0U;
    PhyResult     status = readRegister(kRegBmcr, bmcr);
    if (status != PhyResult::Ok)
        return status;

    /* 强制模式下没有可重启的自动协商；不产生任何写入。 */
    if ((bmcr & kBmcrAutonegEnable) == 0U)
        return PhyResult::Unsupported;

    status = writeRegister(kRegBmcr, static_cast<std::uint16_t>(bmcr | kBmcrRestartAutoneg));
    if (status != PhyResult::Ok)
        return failAfterWrite();

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
    matched  = false;
    address_ = address;

    std::uint16_t id1    = 0U;
    PhyResult     status = readRegister(kRegPhyidr1, id1);
    if (status != PhyResult::Ok)
        return status;

    std::uint16_t id2 = 0U;
    status            = readRegister(kRegPhyidr2, id2);
    if (status != PhyResult::Ok)
        return status;

    const std::uint32_t id = (static_cast<std::uint32_t>(id1) << 16) |
                             static_cast<std::uint32_t>(id2);
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

            /* 读取失败立即上报：不能把未读到的地址当作“无器件”，否则唯一性不成立。 */
            const PhyResult status = probeAddress(addr, matched);
            if (status != PhyResult::Ok)
                return status;

            if (!matched)
                continue;

            candidate = addr;
            ++matches;
            if (matches > 1U)
                return PhyResult::AmbiguousAddress;
        }

        if (matches == 0U)
            return PhyResult::NotFound;

        address_ = candidate;
        return PhyResult::Ok;
    }

    bool            matched = false;
    const PhyResult status  = probeAddress(requested_address_, matched);
    if (status != PhyResult::Ok)
        return status;

    if (!matched)
        return PhyResult::NotFound;

    address_ = requested_address_;
    return PhyResult::Ok;
}

PhyResult DP83822Phy::validatePins(const GPIO_t& reset_n, const GPIO_t& interrupt_n) noexcept
{
    /* {} 是唯一的“未连接”表示：只有 port/pin 同时为空才算未连接。 */
    if (!gpioUnconnected(reset_n) && !gpioSingleBit(reset_n))
        return PhyResult::InvalidArgument;
    if (!gpioUnconnected(interrupt_n) && !gpioSingleBit(interrupt_n))
        return PhyResult::InvalidArgument;

    /* 同一条物理引脚不能既做复位又做中断。 */
    if (gpioSingleBit(reset_n) && gpioSingleBit(interrupt_n) && reset_n.port == interrupt_n.port &&
        reset_n.pin == interrupt_n.pin)
        return PhyResult::InvalidArgument;

    return PhyResult::Ok;
}

bool DP83822Phy::hasResetPin() const noexcept
{
    return gpioSingleBit(reset_n_);
}

bool DP83822Phy::hasInterruptPin() const noexcept
{
    return gpioSingleBit(interrupt_n_);
}

void DP83822Phy::pulseResetPin() noexcept
{
    /*
     * RESET_N 低有效复位输入：拉低至少 1ms 后释放，再等待 2ms 让内部 PLL/strap 建立，
     * 之后才允许 MDIO 访问。引脚在复位后保持高电平，本驱动不在其它时机驱动它。
     */
    HAL_GPIO_WritePin(reset_n_.port, reset_n_.pin, GPIO_PIN_RESET);
    HAL_Delay(kResetPinLowMs);
    HAL_GPIO_WritePin(reset_n_.port, reset_n_.pin, GPIO_PIN_SET);
    HAL_Delay(kResetPinReleaseMs);
}

PhyResult DP83822Phy::softReset() noexcept
{
    /* RESET_N 已做过引脚复位时不再叠加 PHYRCR 复位，避免双重复位并延长启动时间。 */
    if (hasResetPin())
        return PhyResult::Ok;

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
        return failAfterWrite();

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

    return PhyResult::Ok;
}

PhyResult DP83822Phy::configureAutoMdix() noexcept
{
    /* 复位清空了寄存器配置，重新应用默认的 Auto-MDIX 配置。 */
    PhyResult status = setRegisterBits(kRegPhycr, kPhycrAutoMdixEnable);
    if (status != PhyResult::Ok)
        return status;

    return setRegisterBits(kRegCr1, kCr1RobustAutoMdix);
}

PhyResult DP83822Phy::configureInterrupt() noexcept
{
    std::uint16_t scratch = 0U;

    /*
     * 先清掉两个 MISR 里可能残留的 pending 状态（读取即清除），否则使能 PHYSCR 中断输出的
     * 瞬间会把启动前的旧事件当成一次中断上报。
     */
    PhyResult status = readRegister(kRegMisr1, scratch);
    if (status != PhyResult::Ok)
        return status;
    status = readRegister(kRegMisr2, scratch);
    if (status != PhyResult::Ok)
        return status;

    /* MISR2 的事件全部关闭：本驱动只关心链路/速率/双工/协商完成。 */
    if (writeRegister(kRegMisr2, kMisr2NoEvents) != PhyResult::Ok)
        return failAfterWrite();

    /* 只使能 MISR1 中链路/速率/双工/协商完成四类事件（低字节使能，高字节为读清状态）。 */
    if (writeRegister(kRegMisr1, kMisr1EventEnable) != PhyResult::Ok)
        return failAfterWrite();

    /*
     * PHYSCR：置 active-low 中断极性、事件中断使能，并把 INT/PWDN_N 配成中断输出；
     * bit2 的测试中断会强制拉中断，必须显式清零且永远不置位。
     */
    std::uint16_t physcr = 0U;
    status               = readRegister(kRegPhyscr, physcr);
    if (status != PhyResult::Ok)
        return status;

    const std::uint16_t desired = static_cast<std::uint16_t>(
            (physcr & static_cast<std::uint16_t>(~kPhyscrTestInterrupt)) |
            kPhyscrInterruptActiveLow | kPhyscrInterruptEnable | kPhyscrInterruptOutputEnable);

    if (writeRegister(kRegPhyscr, desired) != PhyResult::Ok)
        return failAfterWrite();

    return PhyResult::Ok;
}

bool DP83822Phy::usesInterrupt() const noexcept
{
    return hasInterruptPin();
}

bool DP83822Phy::interruptAsserted() const noexcept
{
    if (!hasInterruptPin())
        return false;

    /* INT/PWDN_N 为 active-low 电平输出：低表示仍有未清除的中断事件。 */
    return HAL_GPIO_ReadPin(interrupt_n_.port, interrupt_n_.pin) == GPIO_PIN_RESET;
}

PhyResult DP83822Phy::acknowledgeInterrupt(bool& link_changed) noexcept
{
    if (status_ != PhyResult::Ok)
        return PhyResult::NotInitialized;

    if (!hasInterruptPin())
        return PhyResult::Unsupported;

    /* 读取 MISR1/MISR2 即清除各自的 pending 状态；两次都成功才更新输出。 */
    std::uint16_t misr1  = 0U;
    PhyResult     status = readRegister(kRegMisr1, misr1);
    if (status != PhyResult::Ok)
        return status;

    /* MISR2 的事件在 start() 中已全部禁用，这里只读取以清除其 pending。 */
    std::uint16_t misr2 = 0U;
    status              = readRegister(kRegMisr2, misr2);
    if (status != PhyResult::Ok)
        return status;

    link_changed = (misr1 & kMisr1StatusMask) != 0U;
    return PhyResult::Ok;
}

PhyResult DP83822Phy::readAndCacheCapabilities() noexcept
{
    std::uint16_t   bmsr   = 0U;
    const PhyResult status = readRegister(kRegBmsr, bmsr);
    if (status != PhyResult::Ok)
        return status;

    /* BMSR bits11..14 与 PhyLinkMode 低 4 位同序，直接映射。 */
    const PhyLinkMode modes = static_cast<PhyLinkMode>(
            static_cast<std::uint8_t>((bmsr & kBmsrModeMask) >> kBmsrModeShift));

    /* 无任何支持模式时无法建立配置（强制模式也无从选择）；不做臆造回退。 */
    if (modes == PhyLinkMode::None)
    {
        supported_modes_   = PhyLinkMode::None;
        autoneg_supported_ = false;
        return PhyResult::Unsupported;
    }

    /* 不支持自动协商仍可用强制模式，因此这里仍算成功，单独记录该能力。 */
    supported_modes_   = modes;
    autoneg_supported_ = (bmsr & kBmsrAutonegCapable) != 0U;
    return PhyResult::Ok;
}

PhyResult DP83822Phy::failAfterWrite() noexcept
{
    /* 写失败后硬件配置不确定：对象失效；清除故障后再次调用 start() 即可恢复。 */
    status_  = PhyResult::WriteError;
    address_ = 0U;
    return PhyResult::WriteError;
}

} // namespace bsp::ethernet_phy

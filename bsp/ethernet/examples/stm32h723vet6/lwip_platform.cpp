/**
 * @file    lwip_platform.cpp
 * @brief   工程 PHY/MSP 选择与 CubeMX 的 C 入口绑定。
 *
 * 通用状态机由 EthernetPHY::LwipPlatform 提供；本文件只持有工程实例与配置。
 */
#include "LwipPlatform.hpp"
#include "DP83822Phy.hpp"
#include "lwip_platform.hpp"
#include "msp.hpp"

#include <cstdint>

/* ethernetif.c 定义的全局句柄；平台仅借用，不重建。 */
extern "C" ETH_HandleTypeDef heth;
extern "C" osSemaphoreId_t   RxPktSemaphore;

namespace
{
using bsp::ethernet_phy::DP83822Phy;
using bsp::ethernet_phy::LwipPlatform;
using bsp::ethernet_phy::PhyLinkConfig;
using bsp::ethernet_phy::PhyLinkMode;

// 目标板 strap 须与该地址一致；更换 PHY 时只改工程绑定与配置。
constexpr std::uint32_t kPhyAddress = 1U;
constexpr PhyLinkConfig kLinkConfig{ true, PhyLinkMode::All10_100 };

// 构造仅保存引用；具体 PHY 必须先于借用它的平台实例构造。
DP83822Phy   phy{ heth, kPhyAddress };
LwipPlatform platform{ heth, phy };
bool         msp_registered = false;
} // namespace

extern "C" bool lwip_platform_prepare(void)
{
    if (!platform.prepare())
        return false;

    // 只在首次 HAL_ETH_Init 前注册一次；这里不启动 PHY。
    if (!msp_registered)
    {
        if (!eth_msp_register(&heth))
            return false;
        msp_registered = true;
    }
    return true;
}

extern "C" bool lwip_platform_init(struct netif* netif)
{
    return netif != nullptr && msp_registered && platform.init(*netif, kLinkConfig);
}

extern "C" bool lwip_platform_poll(void)
{
    return platform.poll();
}

extern "C" void lwip_platform_input(void* argument)
{
    if (argument == nullptr)
    {
        Error_Handler();
        return;
    }

    auto* netif = static_cast<struct netif*>(argument);
    // 信号量在创建 RX 线程前由 ethernetif.c 建立，不能在静态构造时捕获。
    platform.input(*netif, RxPktSemaphore);
    Error_Handler(); // 正常 RX 循环不返回；无效前置条件交给工程错误策略。
}

extern "C" err_t lwip_platform_output(struct netif* netif, struct pbuf* packet)
{
    if (netif == nullptr)
        return ERR_IF;

    return platform.output(*netif, packet);
}

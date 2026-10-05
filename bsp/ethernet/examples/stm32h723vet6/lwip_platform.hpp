/**
 * @file    lwip_platform.hpp
 * @brief   CubeMX ethernetif.c 使用的工程 C 绑定入口。
 *
 * 具体 PHY、地址、链路配置和 MSP 由工程 .cpp 选择；通用状态机位于
 * EthernetPHY::LwipPlatform 包，借用 IPhy 与 ETH 句柄。
 *
 * 生命周期：prepare → HAL_ETH_Init → RX pool/信号量/线程建立 → init → 周期 poll。
 * init/poll 由同一 EthLink 线程串行调用；input 供 RX 线程使用，output 在 LwIP core lock 内调用。
 * prepare/init/poll 返回 false 时调用方走 Error_Handler；init 成功不代表 link up。
 *
 * ethernetif.c 仅在 USER CODE 区接入平台收发；保持 KeepUserCode=true。
 * .ld 的 ETH 段保留在 MEMORYMAP 自动生成标记之外，地址与 .ioc 堆配置一致。
 */
#ifndef USERCODE_ETH_LWIP_PLATFORM_HPP
#define USERCODE_ETH_LWIP_PLATFORM_HPP

#include <stdbool.h>
#include "lwip/err.h"

#ifdef __cplusplus
extern "C"
{
#endif

struct netif;
struct pbuf;

/**
 * @brief 在首次 HAL_ETH_Init 之前调用：创建 RX/HAL 互斥体并注册工程选择的 ETH MSP 回调。
 * @return true 已就绪（可重复调用，幂等）；false 互斥体或 MSP 注册失败，调用方须 Error_Handler，
 *         不得继续 HAL 初始化。
 */
bool lwip_platform_prepare(void);

/**
 * @brief 由 EthLink 线程调用一次：绑定 netif、复位关联 PHY 并启动自动协商。
 * @param[in] netif 已建立的 LwIP 接口，非空；仅借用，不拥有。
 * @return true 平台就绪并可用于 poll；false（空 netif、未 prepare、重复 init 或 PHY 启动失败）
 *         表示不可继续，调用方须 Error_Handler。重复调用不产生额外 PHY 复位。
 */
bool lwip_platform_init(struct netif* netif);

/**
 * @brief 由 EthLink 线程周期调用：读取 PHY 链路并按需切换 MAC 与 netif link。
 * @return true 本轮状态已处理（含正常断链/协商）；false 表示 PHY 读取或 MAC 配置/启停失败，
 *         链路保持在 down，调用方须 Error_Handler。本函数不做内部重试。
 */
bool lwip_platform_poll(void);

/** @brief RX 线程入口；等待既有 RX 信号量，逐包提交给 LwIP，提交失败则释放。 */
void lwip_platform_input(void* argument);

/** @brief 非阻塞 linkoutput；调用方持 LwIP core lock，描述符繁忙时返回 ERR_BUF。 */
err_t lwip_platform_output(struct netif* netif, struct pbuf* packet);

#ifdef __cplusplus
}
#endif

#endif /* USERCODE_ETH_LWIP_PLATFORM_HPP */

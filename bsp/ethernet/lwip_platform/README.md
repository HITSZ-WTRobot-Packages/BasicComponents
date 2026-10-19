# LwipPlatform

`EthernetPHY::LwipPlatform` 是 PHY 无关的 STM32 ETH MAC / LwIP netif 链路管理包，拥有唯一的链路线程。它借用 `IPhy&`，不绑定具体 PHY、MDIO 地址、外设实例或板级 MSP。接口及精确结果契约见 [LwipPlatform.hpp](LwipPlatform.hpp) 的 Doxygen；PHY 契约见 [core README](../core/README.md)，具体器件示例见 [DP83822 README](../phy_drivers/dp83822/README.md)。

## 依赖与静态配置边界

- 依赖 STM32 HAL1 ETH、LwIP、FreeRTOS 与 CMSIS-RTOS v2。本包不承诺所有 STM32 系列或所有 HAL ETH 版本兼容。
- 宿主通过 CubeMX 配置 ETH、GPIO、时钟、IRQ、RTOS 与 LwIP 的可配置项目，完成 `HAL_ETH_Init` 并建立 netif；本包不初始化 MAC 外设、MSP 或网络数据面。
- 要求 `LWIP_TCPIP_CORE_LOCKING=1`（实现有编译期检查）及 `configSUPPORT_STATIC_ALLOCATION=1`。RX 线程必须具备其数据面所需的栈资源和下述优先级关系。
- 传入 HAL 句柄、PHY 与 netif 的生命周期必须覆盖本对象及后台线程的全部使用期。不得另行并发运行对相同资源执行链路切换的管理器。
- HAL TX 完成回调应保留宿主数据面原有的完成通知，再调用 `notifyTxComplete()`。若使用 HAL 动态回调注册，须在 HAL 允许的 READY 状态完成注册，并在资源就绪后启动平台。
- netif 的 RX input、linkoutput、描述符及 pbuf 资源由宿主数据面管理，本包不替代收发实现。平台运行期根据协商结果切换 MAC 速度/双工并更新 netif link，不等于重新配置宿主静态硬件资源。

这里列出前提，不代表当前实现已完整校验所有 CubeMX 配置。若 HAL1 能力不足或必要配置无法在库内校验，应先通过 Ask 确认；本包不提供修改 CubeMX 生成文件或 CMake configure 阶段补丁的集成路线。

## 生命周期与错误处理

- 构造仅保存引用，不访问 HAL、PHY 或 RTOS，可用于静态存储期对象。
- `start(netif, config, on_error)` 前必须完成 `HAL_ETH_Init`，且传入的 netif 已建立并加入 LwIP 栈；只能在普通线程上下文调用一次。配置原样交给 PHY，不静默取能力交集。
- 返回 `true` 仅表示链路线程创建成功，PHY 在线程内异步启动，不表示 PHY 已就绪或链路已建立。
- 重复启动、空错误回调或线程创建失败返回 `false`，不启动 PHY、不访问总线。一旦合法启动尝试失败，不自动重试；没有 stop/restart 接口。
- 对象不可拷贝/移动，运行中不得销毁；析构不停止 MAC、不销毁 PHY、不操作 RTOS。
- 致命错误由链路线程调用 `on_error(PollResult)` 一次；回调可返回，随后线程退出。是否停机由调用方决定。
- 终止时断开 PHY 通知、清空发布的线程 ID 并置 netif link down；不会假称失败的 MAC 停止操作成功，也不自动重试失败的 HAL 操作。

## 线程、资源与锁序

- 线程控制块与 1024 字节栈存于对象内，不新增 RTOS 堆分配；线程名为 `EthLink`，固定优先级为 `osPriorityBelowNormal`。
- **平台线程优先级必须严格低于宿主 ETH RX 线程**。对于调用 `HAL_ETH_ReadData()` 且不持有 LwIP core lock 的 RX 路径，仅有 core lock 不足以保护 RX 与 MAC 启停的交错。不得提高平台线程优先级，也不能把这一优先级约束当成任意线程模型的通用同步保证。
- MAC Get/Set/Start/Stop 与 netif link 变更在 LwIP core lock 内执行；PHY 的 start/readLink/acknowledgeInterrupt 始终在锁外执行。平台不持有自身互斥体。
- 所有 PHY/MAC/netif 管理操作在平台线程中串行执行；宿主必须继续满足 PHY 共享总线串行化约束。
- PHY 事件回调与 `notifyTxComplete()` 可由 ISR 调用，只置线程标志，不访问 PHY/MAC/HAL。线程 ID 尚未发布或已清空时，TX 通知为空操作。

## 事件模式

- **无 INT**：首轮立即查询，此后按 100 RTOS tick 节拍轮询。单轮 `PhyReadError` 已安全停链，可在下一周期重试；其他非 `Ok` 结果终止。
- **有 INT**：先获取一次链路快照，此后在 INT 为高时纯事件等待，空闲不访问 MDIO；INT 持续为低时退化为有界等待加 100 tick 周期轮询，不忙等，恢复为高后回到事件等待。此模式（含退化轮询）的任何非 `Ok` 查询结果均终止。
- 两种模式都通过 TX 完成通知及时回收已完成 TX，不必等下一次 PHY 轮询。持续 TX 通知不会推迟周期查询节拍。

## 结果边界

| `PollResult` | 对调用方的含义 |
| --- | --- |
| `Ok` | 当前状态已处理；正常断链或协商中也属于成功 |
| `PhyReadError` | PHY 读取失败，已安全停链；仅无 INT 轮询模式允许下一周期恢复 |
| `MacStopError` | 停止 MAC 失败，优先于同时出现的 PHY 读取错误，不保证 MAC 已停 |
| `MacStartError` | 获取/设置 MAC 配置或启动失败，链路保持 down |
| `NotInitialized` | PHY 报告尚未就绪 |
| `InvalidPhyState` | PHY 返回未知状态或契约外结果 |
| `PhyStartError` | 线程内启动 PHY 失败 |
| `PhyInterruptError` | 确认/清除 PHY 事件失败 |
| `ThreadWaitError` | 等待线程标志失败；正常超时不属于此错误 |

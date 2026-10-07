# LwipPlatform

PHY 无关的 STM32 ETH MAC 与 LwIP netif 平台状态机，并拥有唯一的链路线程。
平台只通过借用的 `IPhy&` 启动 PHY、查询链路并切换 MAC 速度/双工，不依赖具体 PHY 类型、
地址、MDIO 总线或板级 MSP。

## 生命周期

```cpp
bool start(struct netif& netif, const PhyLinkConfig& config, ErrorHandler on_error) noexcept;
void notifyTxComplete() noexcept;   // ETH TX 完成 ISR 入口
```

- `start()` 只允许在普通线程上下文调用一次。返回 `true` **仅表示链路线程创建成功**，
  PHY 初始化在线程内异步执行，不表示 link up。
- 返回 `false`：重复启动、`on_error` 为空或线程创建失败；此时不启动 PHY、不访问总线、不自动重试。
- 一次合法启动尝试之后不再重启；对象不支持 `stop`/`restart`，运行中不得销毁。
- 运行期致命错误由链路线程调用 `on_error(PollResult)` 一次；回调返回后线程自行退出，
  因此回调可以返回（例如设置标志）或直接进入工程停机路径。
- `notifyTxComplete()` 可在 ISR 中调用：只置线程标志，不访问 PHY/MAC/HAL；线程未发布 ID 时为空操作。

## PollResult

`PollResult` 覆盖线程处理结果，而不只是单次 PHY 轮询结果。

| 结果 | 含义 |
| --- | --- |
| `Ok` | 当前链路状态已处理，正常断链和协商中也属于成功。 |
| `PhyReadError` | PHY 读取失败，已安全停链；仅无 INT 轮询模式允许下一周期恢复。 |
| `MacStopError` | 停止 MAC 失败，优先于同时发生的 PHY 读取错误；不谎称 MAC 已停止。 |
| `MacStartError` | 获取/设置 MAC 配置或启动失败；链路保持 down。 |
| `NotInitialized` | PHY 报告未就绪。 |
| `InvalidPhyState` | PHY 返回未知状态或契约外结果。 |
| `PhyStartError` | 线程内 PHY 启动失败。 |
| `PhyInterruptError` | 确认/清除 PHY 中断失败。 |
| `ThreadWaitError` | 等待线程标志失败。 |

所有终止路径都会断开 PHY 通知、清空发布的线程 ID 并置 netif link down；
不冒充 MAC 已停止，也不自动重试失败的 HAL 操作。

## 线程与资源

- 线程控制块与 1024 字节栈为对象内静态存储（`configSUPPORT_STATIC_ALLOCATION=1`），
  属性为名称 `"EthLink"`、优先级 `osPriorityBelowNormal`，不新增 RTOS 堆分配。
- 线程 ID 由工作线程自己以无锁原子方式发布（release），创建者不在 `osThreadNew` 返回后写回，
  因此允许线程在创建函数返回前就开始运行。
- 借用 `eth`/`phy`/`netif` 必须覆盖线程整个生命周期；对象不可拷贝/移动。

## 锁序与并发

- MAC 转换（Get/Set/Start/Stop）与 netif link 变更统一在 LwIP core lock 内完成；
  本类不持有自身互斥体。
- PHY 的 `start()`/`readLink()`/`acknowledgeInterrupt()` 始终在 core lock 外调用。
- 线程内所有 PHY/MAC/netif 操作串行执行；ISR 只置线程标志。

## 两种事件模式

- **无 INT**：立即读取首轮，此后每轮只延时 100 RTOS tick；`PhyReadError` 允许下一周期恢复，
  TX 由每轮检查回收，因此 `notifyTxComplete()` 直接返回、不置标志。
- **有 INT**：首轮确认中断并强制读取一次快照（补偿安装通知之前的事件）；之后检查已拉低的 INT，
  否则 `osThreadFlagsWait(Link | Tx, osFlagsWaitAny, osWaitForever)`。只有真实链路事件才重新读取链路，
  空闲不周期访问 MDIO；TX-only 唤醒仅回收已完成 TX。
  不调用 `osThreadFlagsClear()`：旧 CMSIS-FreeRTOS 适配层的读改写会覆盖并发 TX 位，
  标志由 wait 消费，残留的 Link 标志最多导致一次空确认，不重复读取链路。

## 集成要求（消费工程）

- 调用方须先完成 `HAL_ETH_Init`（MAC 外设就绪）；本类不初始化 MAC 外设与板级 MSP。
- 依赖 `LWIP_TCPIP_CORE_LOCKING=1`（`.cpp` 中以 `#error` 强制）。
- 必须在 HAL 处于 READY 且 netif 已建立之后再注册 TX 完成回调并调用 `start()`；
  HAL 的动态回调只允许在 READY 状态注册。
- TX 完成接线：在 HAL 回调里先保留 Cube 默认的 `TxPktSemaphore` 通知，再调用
  `notifyTxComplete()`。
- PHY 侧要求见 [`../phy_drivers/dp83822/README.md`](../phy_drivers/dp83822/README.md) 与
  [`../core/README.md`](../core/README.md)。

示例（CoreV2_ETH 的 `UserCode/app.cpp`；该工程把全部接线放在应用侧，
`LWIP/App/lwip.c` 与 `LWIP/Target/ethernetif.c/.h` 保持 CubeMX 生成原样）：

```cpp
using namespace bsp::ethernet_phy;
DP83822Phy   phy{heth, 1U};
LwipPlatform platform{heth, phy};
constexpr PhyLinkConfig config{true, PhyLinkMode::All10_100};

// Init 任务优先级高于执行 MX_LWIP_Init 的 defaultTask，必须让出 CPU 等 netif 就绪。
while ((netif_default == nullptr) || (netif_is_up(netif_default) == 0U))
    osDelay(1U);

if (HAL_ETH_RegisterCallback(&heth, HAL_ETH_TX_COMPLETE_CB_ID,
                             [](ETH_HandleTypeDef* handle) {
                                 HAL_ETH_TxCpltCallback(handle);
                                 platform.notifyTxComplete();
                             }) != HAL_OK)
    Error_Handler();
if (!platform.start(*netif_default, config, [](PollResult) noexcept { Error_Handler(); }))
    Error_Handler();
```

CubeMX 再生成注意：生成代码会在 `MX_LWIP_Init` 末尾创建名为 `EthLink` 的空链路线程
（函数体只有 `osDelay(100)`，不访问 PHY/MAC 或 netif link）。它与本平台的链路线程并存，
链路管理完全由平台线程负责；如需去掉该空线程，须同时删除生成文件中的创建与定义。

## 默认 input/output

平台不接管收发，沿用 CubeMX 生成的 `ethernetif.c` 实现：

- RX 由 `ethernetif_input()` 等待 `RxPktSemaphore` 后经 `low_level_input()`/`HAL_ETH_ReadData()`
  提交给 `netif->input`，提交失败释放 pbuf。
- `netif->linkoutput` 使用 `low_level_output()`；描述符 BUSY 时沿用 `TxPktSemaphore` 的等待、
  回收与重试流程。
- 板级 MSP 以全局 `extern "C" HAL_ETH_MspInit/DeInit` 强覆盖 HAL weak 默认函数，
  `HAL_ETH_Init/DeInit` 自动调用，无需预注册。

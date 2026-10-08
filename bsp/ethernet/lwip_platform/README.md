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
- 优先级安全不变式：`EthLink` 优先级必须严格低于生成 `EthIf` 线程的
  `osPriorityRealtime`（本类固定为 `osPriorityBelowNormal`）。RX 路径
  （`ethernetif_input` → `low_level_input` → `HAL_ETH_ReadData`）在 `EthIf` 内执行且
  **不持有** LwIP core lock，而本线程的 MAC 转换（Get/Set/Start/Stop）在 core lock 内进行；
  只有该优先级顺序才能保证本线程不会抢占正在执行 RX HAL 的 `EthIf`。任何提高本线程
  优先级的改动都会破坏该不变式。
- 线程 ID 由工作线程自己以无锁原子方式发布（release），创建者不在 `osThreadNew` 返回后写回，
  因此允许线程在创建函数返回前就开始运行。
- 借用 `eth`/`phy`/`netif` 必须覆盖线程整个生命周期；对象不可拷贝/移动。

## 锁序与并发

- MAC 转换（Get/Set/Start/Stop）与 netif link 变更统一在 LwIP core lock 内完成；
  本类不持有自身互斥体。
- PHY 的 `start()`/`readLink()`/`acknowledgeInterrupt()` 始终在 core lock 外调用。
- 线程内所有 PHY/MAC/netif 操作串行执行；ISR 只置线程标志。

## 两种事件模式

`run()` 是单一等待循环，用绝对 tick deadline 统一驱动周期 poll 与事件等待：

- **无 INT（轮询）**：周期模式恒开，首轮立即 poll；此后按 100 RTOS tick 的绝对 deadline
  推进。`PhyReadError` 容忍一轮（当轮 `poll()` 已安全停链），下一周期重试。
- **有 INT**：先做一次强制快照（`handleLinkEvent(true)`，补偿安装通知前已丢失的事件）。此后
  每轮采样 INT：INT 拉低则立即 `handleLinkEvent(false)` 确认，避免回调漏置标志时盲等。
  无论本轮是否确认中断，都会再次检查 INT 电平：高电平恢复**纯事件等待**
  （`osThreadFlagsWait(..., osWaitForever)`，空闲不访问 MDIO），包括在有界等待期间自行
  恢复为高的情况；低电平则退化为**有界等待 + 周期 poll**（等待上限 `<= 100 tick`）。
  退化只取决于 INT 电平，与本次确认是否报告了链路变化无关。除首轮快照外，纯事件模式
  只在链路事件时重新读取链路；退化模式还在节拍到期时 poll。任一轮 poll（含退化出的
  周期 poll）的非 `Ok` 结果都按终止语义上报。

等待与节拍细节：

- `osThreadFlagsWait(Link | Tx, osFlagsWaitAny, timeout)`：`timeout` 为 `osWaitForever`
  （纯事件等待）或到下一 deadline 的剩余量，取值 `(0, 100]`；不使用 `timeout == 0`，因为
  无标志时它返回 `osFlagsErrorResource`，会与超时语义混淆。
- TX 完成（`notifyTxComplete()`）在两种模式下都无条件置 `Tx` 标志（线程 ID 未发布时为空
  操作），所以 TX-only 唤醒会立即回收已完成 TX，不等到下一轮 poll。
- `next_poll` 是绝对 deadline，仅在两种时机锚定为 `now + 100 tick`：首次进入退化状态时，
  以及每次周期 poll 之后。TX 唤醒、以及在退化等待尚未到期时的反复 INT 断言都**不会**重新
  锚定，所以持续不断的 TX 不会把 100 tick 的 PHY poll 节拍往后顺延。剩余量用无符号减法
  计算，tick 回绕后仍然正确。
- `osFlagsErrorTimeout` 不是错误：它只在周期分支出现，按到点执行一次周期 poll 处理；其余
  `osFlagsError` 仍按 `ThreadWaitError` 终止。
- `osThreadFlagsWait()` 未传 `osFlagsNoClear`，默认就在返回时自动清除本次命中的标志位，
  因此 `Link`/`Tx` 标志由 wait 自身消费，且两位互不吞并（同一次返回两位时都处理）；实现
  不调用 `osThreadFlagsClear()`——旧 CMSIS-FreeRTOS 适配层里该函数是读改写，会覆盖并发的
  TX 位。“wait 默认自动清除”与“不做显式 FlagsClear”是两件事。

## 集成要求（消费工程）

- 调用方须先完成 `HAL_ETH_Init`（MAC 外设就绪）；本类不初始化 MAC 外设与板级 MSP。
- 依赖 `LWIP_TCPIP_CORE_LOCKING=1`（`.cpp` 中以 `#error` 强制）。
- 必须在 HAL 处于 READY 且 netif 已建立之后再注册 TX 完成回调并调用 `start()`；
  HAL 的动态回调只允许在 READY 状态注册。
- TX 完成接线：在 HAL 回调里先保留 Cube 默认的 `TxPktSemaphore` 通知，再调用
  `notifyTxComplete()`。
- PHY 侧要求见 [`../phy_drivers/dp83822/README.md`](../phy_drivers/dp83822/README.md) 与
  [`../core/README.md`](../core/README.md)。

示例（CoreV2_ETH 的 `UserCode/app.cpp`；该工程把全部接线放在应用侧）：

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

## CubeMX 生成文件与 configure 补丁

`LWIP/App/lwip.c`、`LWIP/Target/ethernetif.c/.h` 的接线代码不做手工修改，但**不能称为逐字节
原样**：CubeMX 模板把 EthIf 线程栈硬编码为 `INTERFACE_THREAD_STACK_SIZE ( 350 )`，而
CMSIS-RTOS V2 下 `ethernetif_input()` 需要 1024 字节。工程根 `CMakeLists.txt` 在 configure
阶段把 `LWIP/Target/ethernetif.c` 中的该宏改写为 `( 1024 )`，并把该文件加入
`CMAKE_CONFIGURE_DEPENDS`：CubeMX 重新生成后，下一次 configure 由该依赖触发，补丁再次
把 350 改回 1024（值已正确时不重复写入）。因此生成文件里只有这一处宏由构建系统维护，
重新生成不会让它丢失，其余生成内容保持不变。

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

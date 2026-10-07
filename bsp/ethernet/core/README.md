# Ethernet PHY Core (IPhy)

`IPhy` 是 PHY 的同步硬件契约：不依赖 RTOS、不创建或拥有线程、不做后台重试；
所有总线操作由调用方串行化。

## 接口

| 方法 | 说明 |
| --- | --- |
| `PhyResult start(const PhyLinkConfig&)` | 阻塞启动并按配置完整重建；可重复调用，失败后再次调用即可恢复。成功只表示驱动就绪，不表示链路已建立。 |
| `PhyResult status() const` | 最近一次 `start()` 的粘性结果。 |
| `PhyResult readLink(PhyLinkState&)` | 查询当前链路；正常断链返回 `Ok` 并写 `Down`，失败不改变输出。 |
| `PhyResult getCapabilities(PhyCapabilities&)` | 读取 `start()` 时缓存的器件能力，不做额外总线读取。 |
| `PhyResult configureLink(const PhyLinkConfig&)` | 下发自动协商通告掩码或强制模式；相同有效配置不重复写入。 |
| `PhyResult restartAutoNegotiation()` | 显式重启协商，返回时不等待协商完成。 |

## 事件与中断能力

这些方法用于取代上层的轮询/中断接线，具体实现决定是否真正接到某个引脚：

| 方法 | 说明 |
| --- | --- |
| `setLinkEventCallback(LinkEventCallback, void* context)` | 注册硬件事件通知。回调可能在 ISR 中执行，因此只应发出该上下文允许的通知（例如唤醒线程），**不得访问 MDIO**；实现必须保证 `callback`/`context` 的更新是原子的，ISR 不能观察到半配置。`nullptr` 关闭通知，不改变 PHY 配置。 |
| `bool usesInterrupt() const` | 是否配置了中断事件源（只反映配置，不访问硬件）。 |
| `bool interruptAsserted() const` | 采样中断线电平，判断事件是否仍 pending；只读引脚，任意上下文可调用。 |
| `PhyResult acknowledgeInterrupt(bool& link_changed)` | 读取并清除事件源，报告是否存在链路相关事件。`NotInitialized` 表示尚未成功 `start()`；`Unsupported` 表示未配置 INT；其它返回值为读取失败原因，且不改变输出。会访问 MDIO，**只能在线程上下文调用**。 |

`LinkEventCallback` 的语义是“有事件需要处理”，不承诺已确认链路变化，也不保证事件仍 pending；
读取、确认与状态转换应由调用方在线程上下文中完成。

## 约束

- `status() != Ok` 时所有操作方法返回 `NotInitialized`，不访问总线、不改变输出。
- 线程归属：本接口不创建、拥有或管理线程（没有 `start`/`stop`/`join`）；链路线程由上层
  `LwipPlatform` 拥有，并在该线程内串行调用本接口。

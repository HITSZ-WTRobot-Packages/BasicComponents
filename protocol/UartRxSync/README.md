# UART RX Sync

`protocol::UartRxSync<HeaderLen, FrameLen, DecodeWithHeader>` 为固定长度 UART 帧提供帧头搜索和 DMA 接收，不是通用协议解析器。cpkg 包名为 `protocol::UartRxSync`，依赖 `stm32cubemx`、[services::Watchdog](../../services/watchdog/README.md) 和 [memory::DMABuffer](../../memory/DMABuffer/README.md)；入口为 [UartRxSync.hpp](UartRxSync.hpp)，依赖清单见 [cpkg.toml](cpkg.toml)。随宿主工程集成，不独立构建。

## 帧格式与集成入口

- `HeaderLen > 0`，`FrameLen > HeaderLen`，`FrameLen` 包含帧头。协议必须有固定字节帧头与固定总长度；长度还应符合 HAL UART 接收长度参数范围，不能超过其 16 位长度上限。
- 派生类实现 `header()` 与 `decode()`。`header()` 返回固定长度数组的引用，引用必须在使用期间有效，不能返回临时对象；运行中不要并发修改帧头。
- 默认 `DecodeWithHeader = false`，`decode()` 接收 `FrameLen - HeaderLen` 字节负载；为 true 时接收完整 `FrameLen` 字节，调用前会把配置帧头复制到缓冲区开头。
- 宿主启用 HAL UART、DMA 和 `USE_HAL_UART_REGISTER_CALLBACKS`，初始化 UART RX DMA 为 `DMA_CIRCULAR` 并接好 IRQ/HAL 中断处理。`startReceive()` 只检查 UART、RX DMA 指针和循环模式，再启动单字节中断接收；不负责外设初始化或完整配置校验。
- 将该 UART 的 RX complete 回调转发到实例的 `receiveCallback()`，error 回调转发到 `errorHandler()`。现有 `UartRxSync_RegisterCallback(obj, huart)` 宏可完成这两项注册，但不返回注册结果；需保证回调能访问对应实例，并由宿主检查 HAL 注册结果或维护明确的注册流程。空的 `UartRxSync_DefineCallback` 宏不是额外处理入口。

## 同步与校验职责

开始时用单字节 IT 接收维护滑动窗口搜索帧头；匹配后用 DMA 接收首帧剩余部分。首帧完成后 abort 当前接收，切换到整帧循环 DMA，并立即解码首帧；后续完成回调检查帧头，失败则 abort 并回到单字节搜索。没有帧间超时重同步或变长帧支持，丢失字节后依赖后续帧头匹配恢复，不能保证零丢包。

CRC、长度字段、数值范围、序号和应用有效性均由 `decode()` 校验。只有返回 true 才喂内部软件 watchdog；返回 false 只丢弃本帧，不主动重同步。`isConnected()` 要求当前为整帧 DMA 状态且 watchdog 尚未耗尽，不表示最近一帧内容一定有效。

`timeout()` 默认返回 10，单位是外部 `Watchdog::EatAll()` 的调用次数，不是毫秒。宿主必须周期推进该服务；若不推进，首次成功解码后连接状态不会因时间流逝自行过期。

## 缓冲区、回调与并发

构造函数借用 `UART_HandleTypeDef*` 和 `Buffer&`，其中 `Buffer` 是 `memory::DMABuffer<FrameLen>`。UART 接收与所用 HAL 回调应由该实例独占；不要同时启动其他接收者或覆盖回调。DMA 内存可达性、cache clean/invalidate 和 CPU/DMA 交接由宿主负责，32 字节对齐并不保证一致性。

`decode()` 在 `receiveCallback()` 内同步执行，通常位于 UART/DMA ISR，不是后台任务。它必须短小、不阻塞，并遵守实际调用上下文限制。传入指针直接引用同一接收缓冲区，没有快照或双缓冲；首帧解码前已重启下一帧 DMA，后续循环 DMA 也会继续写入，因此不能保存指针供异步使用，也不能假定解码期间字节不会被覆盖。需要稳定数据时由宿主设计传输时序与安全复制/交接，不能只依赖回调正在执行。

对象、UART、缓冲区以及派生类资源必须覆盖全部 DMA 与回调访问。类没有 stop 接口，析构不 abort 接收、不注销回调；内部 watchdog 的全局登记也不会随析构撤销，宜使用长期存活实例，不能靠离开作用域安全停止。状态与缓冲区没有线程锁，启动、回调和状态查询的跨上下文同步由宿主负责。

## 错误与实现边界

`startReceive()` 的 false 表示基本配置不满足或首个 `HAL_UART_Receive_IT` 未成功，不等同于完整回滚；它不提供可重复启动时的全面状态重置。之后的重启/切换接收未检查 HAL 返回值，不能把内部状态当作硬件已成功接收的证明。

错误入口只处理 PE/FE/NE/ORE 与 RX DMA 错误，清除相应标志或 RX DMA 本地错误码后重启 RX 搜索，不主动恢复 TX；其他错误留给上层。当前错误入口也没有“已停止”状态保护，不应把任意时刻调用它视作无副作用操作。帧头比对复用搜索阶段的环形索引，源码没有在进入整帧 DMA 时统一归零；不能据此宣称所有多字节帧头及初始相位都已验证。具体行为以头文件实现为准，本说明不代表硬件收帧或抗丢包验证。

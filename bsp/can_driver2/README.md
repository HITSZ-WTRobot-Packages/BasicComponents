# CANDriver2

`bsp::CANDriver2` 提供固定存储的 STM32 CAN/FDCAN 驱动，C++ 命名空间为 `bsp::can`。无堆分配、无 RX 队列、无 stop API，需要 C++20、STM32 MCU HAL1 与 HAL 回调注册支持。接口及字段的精确契约见 [can_driver.hpp](can_driver.hpp) 和 [can_driver_types.hpp](can_driver_types.hpp) 的既有 Doxygen。

## 能力与兼容性边界

- 仅适用于具有 CAN/FDCAN 外设的 STM32 MCU；不支持 STM32MP 或 HAL2（含 STM32C5）。
- 要求 `std::atomic<std::uint32_t>::is_always_lock_free == true`，否则静态断言失败。Cortex-M0/M0+ 等不满足此要求的目标暂不支持，不提供关中断兼容分支。
- 一个固件只能启用一个 HAL CAN 后端，同时启用 CAN 与 FDCAN 模块会在编译期报错。后端按 HAL 模块选择，不按系列名称推断外设。
- 单核 Cortex-M 的任务/ISR 调用模型不构成跨核同步保证。双核目标须由宿主选择相应核并保证同一外设仅由一个核独占。

| HAL 模块 | 帧能力 | 必要回调宏 |
| --- | --- | --- |
| `HAL_CAN_MODULE_ENABLED` | bxCAN，Classic CAN，最多 8 字节 | `USE_HAL_CAN_REGISTER_CALLBACKS == 1` |
| `HAL_FDCAN_MODULE_ENABLED` | FDCAN，Classic / FD，最多 64 字节 | `USE_HAL_FDCAN_REGISTER_CALLBACKS == 1` |

### FDCAN 资源差异

- G4 / L5 / H5 / U5 / U3 / H7RS 的固定 message RAM 布局中，Rx FIFO0/1 与 Tx FIFO/Queue 各 3 个 element，每个数据 element 为 18 word（64 字节负载）。过滤容量取已初始化数量与硬件上限 28/8 的较小值；需要过滤器时须在 HAL Init 前启用所需数量，零容量返回 `NoCapacity`。这些目标不支持专用 Rx Buffer 或校准报文功能，返回 `Unsupported`，不静默改用 FIFO。
- 传统 H7（不含 H7RS）与 N6 使用可配置 RAM 布局，容量取 HAL Init 与硬件上限（filter 128/64、Rx element 64）。所有已分配的 Rx FIFO0、Rx FIFO1 和专用 Rx Buffer element 必须能容纳 64 字节，即使仅使用 Classic CAN 也如此；不满足时 `start()` 在注册回调及启动 HAL 前返回 `InvalidConfiguration`。未分配资源的尺寸不参与判断。
- 驱动不改写已初始化的 message RAM 布局。接收兼容 HAL 的移位/原始 DLC 编码；Classic DLC 9..15 按 8 字节交付，RTR 保留请求长度且不带负载。

### bxCAN 资源差异

覆盖 F1/F2/F3/F4/F7/L4 的 bxCAN 路径，只支持 Classic CAN。保留单控制器 14 bank、CAN1/CAN2 共享 28 bank 与独立 CAN3 路径。FD 格式（含 BRS/ESI）不降级发送；启用发送帧校验时返回 `Unsupported`，关闭时调用方须自行保证输入符合后端能力。

上述范围是实现的能力边界，不构成交叉编译或板上验证承诺，也不保证全部历史 HAL 版本与订货型号。宿主仍须验证时钟、时序、IRQ、收发及核归属。

## 宿主配置与生命周期

宿主通过 CubeMX 配置 GPIO、时钟、位时序、message RAM 与 IRQ，并完成 HAL Init 和 IRQ 转发；驱动不初始化这些静态配置。必要配置若无法在库内校验或 HAL1 能力不足，应先通过 Ask 确认，不增加寄存器替代方案。

使用顺序：HAL Init → 按需配置过滤器/TDC、注册回调 → `start()`。过滤器与 TDC 是现有显式运行期配置入口，会向 HAL 提交设置，并非只校验配置。

- 一个 `Can` 独占一个外设；不同句柄指向同一外设仍返回 `HandleInUse`。句柄、对象与被借用的回调对象须覆盖外设使用期。
- `start()` 安装 HAL 回调、启动硬件并使能 IRQ 通知；只能在线程模式且中断未屏蔽（`IPSR`/`PRIMASK`/`BASEPRI` 为 0）时调用，否则返回 `InvalidState`。不得在 ISR 调用。
- 回调安装不是事务：启动失败只回滚本实例启动的硬件并释放实例槽位，不恢复或注销已安装的 HAL 回调；不能假定失败后句柄可立即交给其他 owner。
- 析构只解除实例登记，不停止外设、不注销 HAL 回调；销毁前由调用方确保硬件已停。
- 外设回调由本驱动独占；启动后不得更改 HAL 句柄或 Init 布局，也不能新增接收回调或更改过滤器/TDC。

## 接收与过滤器

接收回调在 ISR 中按注册顺序同步广播，不得阻塞、抛异常、销毁总线或保存 `FrameView`；其 data 借用内部缓冲，回调返回立即失效。仅支持启动前注册，无注销接口，表满返回 `NoCapacity`。一次 RX 中断尽量排空对应位置，已释放的 `InvalidFrame` 跳过，其余读取错误结束此次中断处理。

过滤能力取决于所选后端及宿主已分配的接收资源。过滤器类型、匹配模式、字段含义、参数约束和返回状态以 [can_driver_types.hpp](can_driver_types.hpp) 的既有 Doxygen 为准，不在 README 维护平行的符号级契约。

宿主须明确未匹配帧的接收策略，不能依赖 FDCAN 复位默认行为来保证拒收。

- bxCAN CAN1/CAN2 共享 bank 必须在两路 HAL Init 完成后、任一总线启动前配置；共享域已有总线运行时拒绝修改。共享分界由下表宏统一决定，CAN3 独立 bank 不受影响。

## 发送延迟补偿（TDC）

FDCAN 的可选 `configure_tx_delay_compensation()` 在 HAL Init 后、`start()` 前立即应用；bxCAN 返回 `Unsupported`。不调用时保持宿主既有设置，`start()` 不改写 TDC。

- `offset_mtq` 与 `filter_mtq` 单位为 FDCAN 内核周期，各不超过 127；使能时要求 FD+BRS 模式且 `DataPrescaler` 为 1 或 2。
- 禁用时忽略两个数值，不检查 FD 模式与数据预分频。`offset_mtq == 0` 是合法附加偏移，不是 auto 模式；没有 auto/manual 枚举。
- 句柄须有效且处于 READY；HAL 失败返回 `HardwareError`。重新 HAL Init 会清除使能位，需重新配置。参数必须结合宿主总线时序选择，不能从默认值推导板上可用性。

## 发送与计数

`send()` 非阻塞，可由单核 Cortex-M 的任务或 ISR 调用。负载在返回前已消费或复制入软件队列，返回后调用方可复用缓冲区。

- 仅保持硬件提交顺序，不保持仲裁顺序。`Submitted` / `Queued` / `ReplacedOldest` 表示受理，不保证最终发送成功。
- 未启动返回 `NotStarted`；硬件忙时尝试软件队列，满时丢弃最旧项并返回 `ReplacedOldest`；队列禁用且硬件忙返回 `Busy`。
- FD 长度必须精确可表示，不填充、不分片。Bus-off 恢复前发送返回 `BusOff`，恢复动作由宿主负责。
- `counters()` 提供每实例 RX/TX 一致快照，可由任务或 ISR 查询。RX 统计成功读取并可交付的帧，与回调数量无关；不统计无效帧、硬件过滤或溢出丢失。TX 只统计实际提交到硬件的帧，包含队列重放，不计单纯入队，也不保证最终成功。
- 两个 `std::uint32_t` 计数从构造时的 0 累计，读取不清零，自然回绕，不提供频率或周期任务。

## 编译期资源与发送检查

以下宏须在驱动及所有使用它的翻译单元中保持一致，由同一驱动目标编译定义统一提供。

| 宏 | 默认值 | 约束 |
| --- | --- | --- |
| `CAN_DRIVER_TX_QUEUE_SIZE` | FDCAN `0`，bxCAN `8` | 软件 TX 容量，可为 0 |
| `CAN_DRIVER_MAX_CALLBACKS` | `14` | 接收回调容量 |
| `CAN_DRIVER_MAX_INSTANCES` | 依 `FDCAN3`/`CAN3`（3）、`FDCAN2`/`CAN2`（2），否则 1 | 实例容量 |
| `CAN_DRIVER_VALIDATE_TX_FRAME` | `0` | 非零时检查发送帧结构与后端能力 |
| `CAN_DRIVER_BXCAN_CAN2_START_BANK` | `14` | 合法范围 0..27；0 表示共享 bank 全归 CAN2 |

**发送检查默认关闭**。调用方必须保证 ID、字段组合、负载指针、FD 长度、后端能力与 TX element 容量均合法；非法输入可能越界，不保证返回 `InvalidFrame` 或 `Unsupported`。启用检查也不改变启动状态、Bus-off、发送空位、HAL 错误、接收和过滤器检查的语义。

# CANDriver2

固定存储的 STM32 CAN/FDCAN 驱动，命名空间 `bsp::can`。无堆分配、无 stop API、无 RX
队列，面向带 CAN/FDCAN 外设的 STM32 MCU HAL1，需要 C++20 与 HAL 回调注册支持。不支持
STM32MP 或 HAL2（含 STM32C5），也不包含没有 CAN/FDCAN 外设的型号。

另要求 `std::atomic<std::uint32_t>::is_always_lock_free == true`，不满足时静态断言报错。
Cortex-M0/M0+ 等不支持无锁 32 位原子读改写的目标暂不支持，不提供关中断兼容分支。

## 文件

| 文件 | 内容 |
| --- | --- |
| `can_driver.hpp` | `Can` 类、编译期后端选择与资源配置宏；文件头含系列兼容性与验证记录 |
| `can_driver_types.hpp` | 与 HAL 无关的帧描述（`FrameHeader`/`FrameView`）、收发计数（`TrafficCounters`）、状态码、过滤器类型与发送延迟补偿配置（`TxDelayCompensation`） |
| `can_driver.cpp` | 实例注册表、回调派发、软件 TX 队列的共享实现 |
| `can_driver_bxcan.cpp` | bxCAN 后端实现（仅 `HAL_CAN` 固件参与编译） |
| `can_driver_fdcan.cpp` | FDCAN 后端实现（仅 `HAL_FDCAN` 固件参与编译） |

## 后端选择与 HAL 要求

后端在编译期由启用的 HAL 模块决定，不按 STM32 系列名称限制；一个固件只能启用其中一个
HAL CAN 后端（同时启用会在编译期报错）。

| HAL 模块 | 后端 | 帧能力 | 依赖宏 |
| --- | --- | --- | --- |
| `HAL_CAN_MODULE_ENABLED` | bxCAN | Classic CAN，负载上限 8 字节 | `USE_HAL_CAN_REGISTER_CALLBACKS == 1` |
| `HAL_FDCAN_MODULE_ENABLED` | FDCAN | Classic / FD，负载上限 64 字节 | `USE_HAL_FDCAN_REGISTER_CALLBACKS == 1` |

板级代码负责 HAL 初始化（GPIO、时钟、时序、message RAM）与 IRQ 转发；驱动不初始化这些
内容。TDC 参数由板级选择，可在 HAL Init 之后、`start()` 之前通过
`configure_tx_delay_compensation()` 立即应用，也可由板级直接经 HAL 配置；不调用该 API
就保持板级已写入的设置。`start()` 不改写 TDC。

## 系列兼容范围

### FDCAN，固定 message RAM

G4 / L5 / H5 / U5 / U3 / H7RS 使用固定布局：Rx FIFO0/1 与 Tx FIFO/Queue 各 3 个
element，每个数据 element 18 word（64 字节负载），HAL 的 `Init` 不含 FIFO 数量或 element
尺寸字段。过滤能力取 `Init.StdFiltersNbr` / `Init.ExtFiltersNbr` 与硬件上限 28/8 的较小值：
需要过滤器时须在 HAL Init 前启用所需数量，数量为 0 时配置 ID 过滤器返回 `NoCapacity`。专用
`RxBuffer` 与校准报文功能不支持，返回 `Unsupported`，不会静默改用 FIFO。

### 传统 H7（不含 H7RS）与 N6，可配置 RAM 布局

从 `Init` 读取 FIFO/Buffer 数量与 element 尺寸（硬件上限：filter 128/64、Rx element 64）。
所有已分配的 Rx FIFO0、Rx FIFO1 与专用 Rx Buffer element 必须能容纳 64 字节负载，否则
`start()` 在注册 Callback、调用 HAL Start 之前返回 `InvalidConfiguration`；未分配资源的尺寸
不参与判断。该要求也适用于 Classic 模式，因为 HAL 按原始 DLC 复制负载。驱动不会改写已
初始化的 message RAM 布局，板级须在 HAL Init 前配置足够容量。

接收长度按当前 HAL 的 `FDCAN_DLC_BYTES_*` 宏反查，兼容旧 HAL 的移位编码与新 HAL 的原始
编码。Classic DLC 9..15 仍按 8 字节交付；RTR 保留请求长度且负载为空。发送兼容旧 HAL 的
非 const 数据参数声明，不增加负载复制。

### bxCAN

覆盖 F1/F2/F3/F4/F7/L4，只支持 Classic CAN。保留单控制器 14 bank、CAN1/CAN2 共享 28
bank 与独立 CAN3 路径。FD 格式（含 BRS/ESI）以 `Unsupported` 拒绝，不降级发送。滤波器编程
完全经 `HAL_CAN_ConfigFilter()` 完成，本后端不直接写滤波器寄存器。

## 生命周期与线程约束

使用顺序：先完成 HAL 句柄 Init，再按需配置 TDC、过滤器与注册回调，最后 `start()`。

- 一个 `Can` 实例独占一个 HAL 句柄指向的外设：同一外设即使使用不同句柄也被判为冲突
  （`HandleInUse`）。句柄、`Can` 对象与被借用的回调对象必须比外设的使用期更长。
- `start()` 安装 HAL 回调、启动硬件并使能 IRQ 通知；不初始化 GPIO/时序/message RAM，
  不改写 TDC（可选配置入口见「发送延迟补偿（TDC）」）。
  必须在线程模式且中断未屏蔽（`IPSR`/`PRIMASK`/`BASEPRI` 为 0）时调用，否则返回
  `InvalidState`；绝不可在 ISR 中调用，因为 HAL 启动依赖可运行的中断与 tick 超时。
- HAL Callback 安装不是事务：任何启动失败都不会恢复或注销已成功安装的 Callback。失败时
  仅回滚本实例启动的硬件并释放实例槽位；调用方须自行保证 `start()` 成功，不得假定失败后
  句柄可直接交给其他 owner。
- 析构只解除本实例的登记（释放注册表槽位），不会停止外设、也不注销 HAL 回调。销毁前须由
  调用方确保硬件已停止。
- 一个外设的回调由该 `Can` 独占，不得再把其他驱动挂到同一句柄上。`start()` 之后不得再改动
  HAL 句柄或其 Init 资源布局。

## 接收与回调

- `register_callback()` 提供三种重载：无状态函数指针、借用的左值可调用对象、以及
  `register_callback<&Motor::on_can>(motor)` 形式的成员函数回调。成员函数签名要求
  `void(Object&, const FrameView&)`。
- 回调在 ISR 上下文按注册顺序同步广播。回调不得阻塞、抛异常、销毁本总线或保存
  `FrameView`；`FrameView::data` 借用内部缓冲，回调返回后立即失效。
- 仅可在 `start()` 之前注册，否则返回 `InvalidState`；表满返回 `NoCapacity`。没有注销
  （unregister）接口，`start()` 后的注册/过滤变更会被拒绝。
- 一次 RX 中断会尽量读空该 location；`InvalidFrame`（已确认并释放但不可交付的帧）会被
  跳过并继续排空，其余错误终止本轮。

## 过滤器

`configure_filter(const FilterConfig&)` 仅可在 `start()` 之前调用，`FilterConfig` 是
`IdFilter` / `BankFilter` / `GlobalFilter` / `ExtendedIdMask` 的 variant。资源 index 由配置
显式给出并做边界检查：越界/无容量返回 `OutOfRange`/`NoCapacity`，后端不支持的备选项返回
`Unsupported`，句柄为空返回 `InvalidConfiguration`，外设已被其他实例占用返回 `HandleInUse`。

**IdFilter**（后端相关）

- bxCAN：`index` 为 filter bank 编号。`Mask` 映射为一个 32 位掩码 bank（标准/扩展均不比较
  RTR，数据帧与远程帧都能匹配）；标准 ID 的 `List` 映射为同一 bank 内两个 16 位掩码项。
  一个 bxCAN bank 无法表达两个不同扩展 ID 的精确 `List` 语义，此类请求返回 `Unsupported`。
  `Range`/`RangeWithoutExtendedMask`、`Reject`/`PriorityFifo0`/`PriorityFifo1`/`RxBuffer`、
  `GlobalFilter` 与 `ExtendedIdMask` 在 bxCAN 上返回 `Unsupported`。
- FDCAN：`index` 为所选标准/扩展过滤器数组内的 element 下标。`Mask`/`List`/`Range`/
  `RangeWithoutExtendedMask` 均支持；`Range` 要求 `id2 >= id1`，`RangeWithoutExtendedMask`
  仅扩展 ID 列表可用。`RxBuffer` 要求 `mode == Mask` 且 `id2 == 0`，命中帧写入
  `rx_buffer_index` 指定的缓冲区，仅支持专用 RX Buffer 的 FDCAN 可用（固定 RAM 返回
  `Unsupported`）。
- 配置 bxCAN 的 `IdFilter`（包括 `Disable`）时，`id1`/`id2` 超出所选 ID 位宽返回
  `InvalidArgument`；非 `RxBuffer` 动作使用非零 `rx_buffer_index` 也返回 `InvalidArgument`。
  非法禁用请求不会修改已配置的过滤器。

**高优先级接收**：必须显式选择 `FilterAction::PriorityFifo0` 或 `PriorityFifo1`，匹配帧存入
指定 FIFO 并经接收回调交付。不存在无接收目标的 `FilterAction::Priority`：旧调用方须改用显式
FIFO 动作，不保留别名或隐式默认 FIFO。

**BankFilter**：完整 bxCAN filter bank 布局，四个半字原样写入，硬件能表达的每种 IDE/RTR
组合都可达；FDCAN 返回 `Unsupported`。

**GlobalFilter / ExtendedIdMask**（FDCAN 专属）：FDCAN 复位默认可能接受未匹配帧，需要拒收
时请显式提交 `GlobalFilter{}`；目标 FIFO 未分配 element 时返回 `NoCapacity`。`ExtendedIdMask`
按位作用于所有扩展帧，默认 `0x1FFFFFFF`（比较全部 29 位），超过该值返回 `InvalidArgument`。

**bxCAN 共享 bank**：CAN1/CAN2 共享 bank 必须在两路 HAL Init 完成后、任一条总线启动之前
配置完成。共享域中若已有另一条已注册总线在运行，共享写入会被拒绝（`InvalidState`）。共享
分界由编译期宏 `CAN_DRIVER_BXCAN_CAN2_START_BANK` 统一指定（随每次过滤器配置写入 HAL），
不提供运行时分界配置。CAN3 的独立 14 bank 不受该约束。

## 发送延迟补偿（TDC）

`configure_tx_delay_compensation(const TxDelayCompensation&)` 是 FDCAN 专属的可选入口，
用于在 HAL Init 之后、`start()` 之前把 TDC 设置立即写入已初始化的句柄；bxCAN 后端返回
`Unsupported`。

`TxDelayCompensation` 字段按此顺序：

| 字段 | 类型 | 单位 / 含义 |
| --- | --- | --- |
| `enabled` | `bool` | 是否启用发送延迟补偿，默认 `false` |
| `offset_mtq` | `std::uint32_t` | TDCO 附加偏移，单位 FDCAN 内核周期（mtq） |
| `filter_mtq` | `std::uint32_t` | SSP 滤波器窗口，单位 FDCAN 内核周期（mtq） |

调用规则与 `configure_filter()` 一致：使用与其他 API 相同的 ISRGuard，必须在 `start()`
之前调用（否则 `InvalidState`），句柄为空返回 `InvalidConfiguration`，外设已被其他实例占用
返回 `HandleInUse`。该 API 直接把设置写进 HAL 并立即应用，不保存配置，也不修改 `start()`、
发送、帧或队列路径。FDCAN 的 TDC 硬件始终自动测量收发环回延迟，`offset_mtq`（TDCO）只是在
此基础上的附加偏移；`offset_mtq == 0` 是合法取值，不是「auto 模式」，驱动不提供
auto/manual 枚举。

- `enabled == false`：只执行 Disable，忽略 `offset_mtq` / `filter_mtq`，也不检查 FD 模式与
  数据预分频。
- `enabled == true`：先检查 `offset_mtq` / `filter_mtq` 是否超过 127，超出返回 `OutOfRange`；
  再要求 `Init.FrameFormat == FDCAN_FRAME_FD_BRS` 且 `Init.DataPrescaler` 为 1 或 2，否则
  返回 `InvalidConfiguration`；通过后执行 Config 与 Enable。
- HAL 调用失败返回 `HardwareError`；句柄 `Instance` 非法返回 `InvalidConfiguration`；句柄
  状态不是 `READY` 返回 `InvalidState`。

HAL assert 关闭时不能依赖 HAL 的参数校验，驱动自行完成上述范围与模式检查。重新执行
HAL Init 会清除 TDC 使能位，需要再次配置。若希望 SSP 附加偏移对齐数据段普通采样点，
可取 `offset_mtq = Init.DataPrescaler * (1 + Init.DataTimeSeg1)`、`filter_mtq = 0`；这是
初始参数选择，不能替代板上的收发与时序验证。

## 发送

`send(const FrameView&)` 为非阻塞；单核 Cortex-M 上可由任务或 ISR 调用。

- 负载为借用：函数返回后调用方即可复用缓冲区（帧在返回前已被消费或复制进软件队列）。
- 仅保留硬件提交顺序，不保留 CAN 仲裁顺序。
- `Submitted` / `Queued` / `ReplacedOldest` 只表示帧已被受理，不保证真正发送成功。
- 总线尚未 `start()` 时返回 `NotStarted`。硬件无空位且软件队列可用时该帧入队；软件队列满
  时丢弃最旧一项并返回 `ReplacedOldest`；软件队列被禁用（容量 0）且硬件忙时返回 `Busy`。
- FD 数据长度必须精确可表示（见 `is_fd_length`），不做填充或分片。
- Bus-off 由后端实时判断，恢复前 `write()` 返回 `BusOff`（恢复动作由板级负责）。

## 收发累计计数

每个 `Can` 实例独立累计 RX/TX 帧数，通过 `counters()` 返回 `TrafficCounters` 一致快照。
RX 使用 `memory_order_relaxed` 的无锁原子自增，在广播回调前计数，不使用 `ISRGuard`。
TX 继续复用既有发送临界区；`counters()` 仍在短 `ISRGuard` 内同时读取两项计数，保持快照语义。
单核 Cortex-M 上可由任务或 ISR 查询；驱动不会额外屏蔽用户回调。

| 字段 | 统计口径 |
| --- | --- |
| `rx_frames` | 成功读出并可交付的有效帧数；与回调数量无关，没有注册回调也计数。`InvalidFrame`、空队列及读取失败不计数 |
| `tx_submitted_frames` | 成功写入硬件邮箱/FIFO 的帧数；仅入软件队列不计数，队列重放成功时才计数。`Busy`、校验失败及其他提交失败不计数，不代表最终发送成功 |

```cpp
const bsp::can::TrafficCounters counts = bus.counters();
const std::uint32_t rx = counts.rx_frames;
const std::uint32_t tx = counts.tx_submitted_frames;
```

计数从对象构造时的 0 开始累计，读取不清零；两个字段均为 `std::uint32_t`，达到上限后自然
回绕。RX 不包含被硬件过滤或因溢出丢失的帧。该接口不做频率计算，也不引入时间源或周期任务。

## 编译期资源配置宏

以下宏均需在所有使用本驱动的翻译单元中保持一致（通过驱动目标的编译定义统一覆盖）：

| 宏 | 默认值 | 说明 |
| --- | --- | --- |
| `CAN_DRIVER_TX_QUEUE_SIZE` | FDCAN `0`，bxCAN `8` | 软件 TX 队列容量，允许为 0 |
| `CAN_DRIVER_MAX_CALLBACKS` | `14` | 可注册接收回调数上限 |
| `CAN_DRIVER_MAX_INSTANCES` | 依 `FDCAN3`/`CAN3`（3）、`FDCAN2`/`CAN2`（2）否则 1 | `Can` 实例上限 |
| `CAN_DRIVER_VALIDATE_TX_FRAME` | `0` | 是否在发送路径执行运行时帧校验 |
| `CAN_DRIVER_BXCAN_CAN2_START_BANK` | `14` | bxCAN CAN1/CAN2 共享 bank 分界，合法范围 0..27（0 表示全部归 CAN2） |

## 发送帧检查开关

`CAN_DRIVER_VALIDATE_TX_FRAME` 默认值为 `0`，发送时不执行帧结构及后端帧能力校验；非零时
启用原有校验（ID 位宽、字段组合、FD 长度、FD 支持与 TX element 容量）。

- 通过驱动目标的编译定义（如 `-DCAN_DRIVER_VALIDATE_TX_FRAME=1`）统一覆盖，所有使用该
  驱动的翻译单元须保持宏定义一致。
- 关闭检查时，调用方必须保证 ID、字段组合、负载指针及长度合法，并确保帧格式和负载适配已
  配置的外设，尤其是 FDCAN TX element 容量；非法输入可能导致越界访问，不保证返回
  `InvalidFrame` 或 `Unsupported`。
- 该开关不影响启动状态、Bus-Off、发送空位及 HAL 错误处理，也不影响接收和过滤器检查。

## 使用示例

```cpp
#include "can_driver.hpp"

extern FDCAN_HandleTypeDef hfdcan1; // 已完成 HAL Init 的句柄

static bsp::can::Can bus{ hfdcan1 };
static Motor motor{ /* ... */ };

void init_can()
{
    // 1. 过滤器（start() 之前）
    bsp::can::IdFilter filter{};
    filter.index   = 0;
    filter.id_type = bsp::can::IdType::Standard;
    filter.mode    = bsp::can::FilterMode::Mask;
    filter.action  = bsp::can::FilterAction::Fifo0;
    filter.id1     = 0x123;
    filter.id2     = 0x7FF;
    (void)bus.configure_filter(filter);

    // 2. 发送延迟补偿（start() 之前，FDCAN）
    bsp::can::TxDelayCompensation tdc{};
    tdc.enabled    = true;
    tdc.offset_mtq = hfdcan1.Init.DataPrescaler * (1U + hfdcan1.Init.DataTimeSeg1);
    tdc.filter_mtq = 0;
    (void)bus.configure_tx_delay_compensation(tdc);

    // 3. 接收回调（start() 之前）
    (void)bus.register_callback<&Motor::on_can>(motor);

    // 4. 启动（须在线程模式、中断未屏蔽时调用）
    (void)bus.start();
}

void send_frame()
{
    const std::uint8_t payload[4]{ 1, 2, 3, 4 };
    bsp::can::FrameView frame{};
    frame.header.id     = 0x123;
    frame.header.format = bsp::can::FrameFormat::Classic;
    frame.data          = { payload, sizeof payload };
    (void)bus.send(frame);
}
```

## 状态与结果码

`Status`：`Ok`、`Empty`、`Busy`、`InvalidArgument`、`InvalidFrame`、`InvalidState`、
`InvalidConfiguration`、`Unsupported`、`OutOfRange`、`NoCapacity`、`HandleInUse`、`BusOff`、
`HardwareError`；各 API 只返回与其相关的子集。

`SendResult`：`Submitted`、`Queued`、`ReplacedOldest`、`Busy`、`NotStarted`、`InvalidFrame`、
`Unsupported`、`BusOff`、`HardwareError`。完整的字段级说明见 `can_driver_types.hpp`。

## 兼容性验证记录

以 HAL1 driver 版本（不是 STM32Cube 固件包版本）记录，`C` = 在真实 HAL/CMSIS 下交叉编译
三个驱动翻译单元，`S` = 主机 HAL 边界模拟收发（历史记录，`can_driver_filter_test.py` 运行
器已移除）：

以下记录保留为历史证据，不代表新增无锁 RX 计数要求下的全系列验证；F0/G0/C0 当前不支持。

| 系列 | 代表型号 | HAL1 版本 | 验证层级 |
| --- | --- | --- | --- |
| F0 | F072 | 1.7.9 | C |
| F1 | F105 | 1.1.10 | C |
| F2 | F207 | 1.2.9 | C |
| F3 | F303xC | 1.5.9 | C |
| F4 | F405 | 1.8.5 | C |
| F7 | F767 | 1.3.3 | C |
| L4 | L476 | 1.13.6 | C |
| G4 | G431 | 1.2.7 | C + S |
| G0 | G0B1 | 1.4.7 | C + S |
| C0 | C092 | 1.4.1 | C + S |
| L5 | L552 | 1.0.7 | C + S |
| H5 | H563 | 1.7.0 | C + S |
| U5 | U575 | 1.6.3 | C + S |
| U3 | U375 | 1.4.0 | C + S |
| H7RS | H7S7 | 1.2.1 | C + S |
| 传统 H7 | H723 | 1.11.0 / 1.11.6 | C + S（移位 / 原始 DLC） |
| N6 | N657 | 1.4.0 | C + S |

代表型号覆盖 TX 校验 0/1 × 软件队列容量 0/4；所用 CMSIS 中带 CAN/FDCAN 的 160 个芯片宏另
完成交叉编译（TX 校验 1、软件队列 0）。双核 H7 的 CM4/CM7 均编译验证，板级须选择
`CORE_CM4`/`CORE_CM7`，并确保同一外设仅由一个核独占；驱动不提供跨核同步。

上述证据不等于所有 HAL 历史版本或订货号的板上收发验证。**未进行板上验证**；时钟、GPIO、
时序、IRQ 与外设核归属仍由板级负责。主机回归与交叉编译结果都不能替代板上的总线时序、
IRQ 与收发验收。表中历史记录不涵盖 TDC，也不代表新增的 `configure_tx_delay_compensation()`
入口已做过跨系列或板上验收。

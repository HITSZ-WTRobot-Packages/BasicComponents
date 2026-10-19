# CANDriver

`bsp::CANDriver` 是基于 STM32 HAL1 的 C++ CAN/FDCAN 函数式封装，依赖 `stm32cubemx`、`libs::RingBuffer` 与 `utils`，不依赖 RTOS、不创建任务。入口为 [can_driver.hpp](can_driver.hpp)，实现见 [can_driver.cpp](can_driver.cpp)。[CANDriver2](../can_driver2/README.md) 是另一个独立包，不是本包的改名；两者的接口和资源契约不能混用。

## 后端与宿主配置

- 必须且只能启用 `HAL_CAN_MODULE_ENABLED` 或 `HAL_FDCAN_MODULE_ENABLED` 之一，并开启与所用后端对应的 `USE_HAL_CAN_REGISTER_CALLBACKS`（bxCAN）或 `USE_HAL_FDCAN_REGISTER_CALLBACKS`（FDCAN）。未启用或同时启用两种后端均编译报错。
- bxCAN 后端提供 `CAN_*` 接口，只处理 Classic CAN。FDCAN 后端提供原生 `FDCAN_*` 接口，并通过 [can_hal_def.h](can_hal_def.h) 提供 bxCAN 风格兼容类型、过滤器和中断别名。
- FDCAN 后端的 `CAN_Start`、`CAN_RegisterCallback` 要求句柄配置为 `FDCAN_FRAME_CLASSIC`，以断言检查；兼容发送也检查该模式。需要 FD 帧时使用原生 `FDCAN_*` 接口，不把兼容接口理解为完整 FD API。
- 宿主通过 CubeMX 完成 GPIO、时钟、位时序、外设模式、FDCAN message RAM/元素大小、过滤器容量和 NVIC 配置，执行 HAL Init，并在实际 IRQ 中调用相应 `HAL_CAN_IRQHandler` / `HAL_FDCAN_IRQHandler`。本包不初始化这些静态配置，也没有完整校验这些前提。
- 本包使用目标 HAL 提供的 CAN/FDCAN 类型与接口，不承诺所有 STM32 系列、型号或 HAL 版本兼容，也不提供板级收发器控制。

## 接入顺序与资源生命周期

1. 完成宿主 HAL 初始化和过滤器设置。
2. 在 HAL Init 之后、启动之前调用对应的 `CAN_InitMainCallback` / `FDCAN_InitMainCallback`，安装驱动的 FIFO 接收及发送队列相关 HAL 回调。
3. 启动前通过 `CAN_RegisterCallback` / `FDCAN_RegisterCallback` 注册业务接收函数。**业务注册才会建立句柄对应的软件表，主回调初始化不会建立此表**。bxCAN 必须在任何发送前建立该表，否则邮箱满时或发送完成回调中可能解引用空表；FDCAN 开启软件队列后，未登记表的句柄也不能在硬件满时入队。
4. 调用对应的 `*_Start(hcan, ActiveITs)`，传入实际使用的 FIFO 接收通知位。驱动另外启用 bxCAN 的 TX mailbox empty 或 FDCAN 的 TX FIFO empty 通知，用于续发软件队列。

驱动按**句柄指针**登记全局表，不检查不同句柄是否指向同一外设。一个外设应由本驱动及同一稳定句柄管理，不能在运行中替换它安装的 HAL 回调。表没有注销、复位或资源回收接口；句柄与回调所访问的业务状态须覆盖全部中断使用期。停止、屏蔽中断及销毁前的同步由宿主负责，HAL Stop 不会清空本包的登记表或软件队列，不能据此假定可安全重建实例。

## 接收、过滤与回调上下文

- FIFO 中断中循环读取对应 FIFO，按业务注册顺序同步广播给该句柄的所有回调；不提供接收队列或按 ID 分发。业务自行判断帧 ID。
- 回调在 HAL 中断路径执行，不得阻塞；帧头与数据是分发函数栈上的临时对象，只在当前回调调用期间有效，需长期使用时自行复制。注册过程不加锁，必须在启动前串行完成，不得与 ISR 分发并发修改。
- FDCAN 原生回调能收到 Classic/FD 帧，bxCAN 风格回调跳过 FD 帧；兼容头中的 `FilterMatchIndex` 是 FDCAN 元素索引，不是 bxCAN bank 编号。
- bxCAN 使用 HAL 原生过滤器配置。FDCAN 的 `HAL_CAN_ConfigFilter` 兼容实现支持 32 位掩码或同类型 ID 列表；启用 16 位 scale、混合标准/扩展 ID 列表或对应过滤器容量为零时返回 `HAL_ERROR`。
- FDCAN 兼容过滤器直接将 `FilterBank` 当作实例内元素索引，**未校验索引上界，也不重映射 bxCAN 共享 bank**；调用方必须保证索引小于对应 `StdFiltersNbr` / `ExtFiltersNbr`。该转换不完整保留 bxCAN 的 IDE/RTR 匹配语义，不能当作任意 bxCAN 过滤配置的无损迁移。未匹配帧及远程帧的全局策略由宿主管理。

## 发送、队列与失败边界

发送调用以 `ISRGuard` 保存/恢复 PRIMASK、短暂屏蔽本核中断，适用于本核普通任务与普通可屏蔽 ISR 之间的发送互斥；不是跨核锁，也不保护并发 HAL 初始化、停止或注册。没有 RTOS 互斥体或阻塞等待空位。直接发送由 HAL 在调用内消费负载，入队时复制帧头与数据，因此调用返回后可复用发送缓冲；返回值不表示总线最终发送成功。

- bxCAN 默认软件队列容量 `CAN_TX_QUEUE_SIZE=8`，满时覆盖最旧帧。硬件有空位时直接提交并返回 HAL mailbox 值；邮箱满时入队，**仍返回 `CAN_SEND_FAILED`（`0xFFFF`）**，调用方不能据此判断该帧未被受理并盲目重发。该路径也不保证新帧不会越过尚未清空的软件队列。
- bxCAN 必须由调用方提供有效句柄、帧头、数据和 `DLC <= 8`；当前实现仅在入队分支断言 DLC，没有头文件注释所述的统一非法 DLC 返回保障。零长度帧也应传有效数据指针。
- FDCAN 默认 `FDCAN_TX_QUEUE_SIZE=0`，硬件 FIFO 满立即返回 `CAN_SEND_FAILED`。设置为正值后启用覆盖最旧帧的软件队列，队列非空时新帧先入队，按软件提交顺序搬入硬件；仲裁顺序及最终发送成功仍由硬件和总线决定。
- FDCAN 原生发送检查空句柄/帧头/数据以及 DataLength 超出 `FDCAN_DLC_BYTES_64`，失败返回 `CAN_SEND_FAILED`，成功或入队返回 0。DataLength 必须使用目标 HAL 的合法 `FDCAN_DLC_BYTES_*` 编码，其他字段合法性与 message RAM 容量仍由调用方保证。兼容发送另外拒绝 DLC 大于 8。
- HAL 注册、启动、读取或提交失败通常调用宿主 `Error_Handler()`，并非统一的可恢复错误返回；本包不做 bus-off 恢复、超时重试或完整的失败回滚。软件队列重放先出队再调用 HAL，提交失败不会恢复该帧。

每条总线默认最多注册 `CAN_MAX_CALLBACK_NUM=14` 个回调，实例表容量 `CAN_NUM` 按外设宏推导；超限调用 `Error_Handler()`。这些宏及队列宏必须在驱动和使用方保持一致。bxCAN 队列依赖环形缓冲区的容量静态断言，不能用 `CAN_TX_QUEUE_SIZE=0` 禁用它；FDCAN 才有零容量禁用分支。

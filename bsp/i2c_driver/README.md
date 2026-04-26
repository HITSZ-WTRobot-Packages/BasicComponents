# I2C DMA Driver

`I2CBusDMA` 提供单条 I2C 总线的 DMA 同步封装。

## 设计目标

- 对上层暴露同步 `memRead()` / `memWrite()` / `read()` / `write()` 接口
- 对底层使用 STM32 HAL 的 DMA 事务
- 使用 FreeRTOS 任务通知等待完成，不在任务里忙等
- 明确约束为“单总线单 owner task”

## 内部状态

- `transmitting_`：当前是否已有一笔事务启动但尚未完成收敛
- `completed_`：当前任务是否已经收到一条完成记录
- `current_transfer_id_`：当前事务编号
- `completed_transfer_id_`：最近一次完成记录对应的事务编号

完成中断到来后，驱动不会直接把任何通知都当成当前事务完成，而是要求：

- `completed_ == true`
- `completed_transfer_id_ == current_transfer_id_`

这样可以减少过期完成通知对当前等待流程的污染。

## 失败路径

以下路径统一按“失败并恢复”处理：

- HAL 启动 DMA 事务失败
- 等待完成超时
- 收到当前事务的错误完成

恢复前会先清理软件事务状态，并尽量 abort 底层 DMA，然后再执行：

- `HAL_I2C_DeInit()`
- `HAL_I2C_Init()`

恢复成功只表示总线尽量被拉回可用状态，不表示本次事务成功。

## 使用约束

- 同一条 I2C 总线只能由一个任务串行使用
- `memRead()` / `memWrite()` / `read()` / `write()` 依赖 FreeRTOS 任务通知，只能从普通 task 上下文调用
- 不要在 ISR、调度器启动前或裸机轮询上下文中调用 `I2CBusDMA`
- 不要让多个业务任务绕过调度器直接并发访问同一 `I2CBusDMA`
- HAL 完成中断回调是唤醒等待 task 的必要链路；如果回调桥接未接通，事务会一直等到超时
- 如果现场经常出现 SDA 被从机长时间拉低，当前恢复策略可能不够，需要再补 GPIO 脉冲恢复

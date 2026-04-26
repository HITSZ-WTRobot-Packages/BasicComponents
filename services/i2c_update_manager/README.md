# I2C Update Manager

该目录提供两层抽象：

- `I2CDevice`：面向周期采样设备的通用基类
- `I2CUpdateManager`：单总线周期调度器

## I2CDevice

`I2CDevice` 内置统一的三段状态机：

- `Trigger`
- `Wait`
- `Read`

子类只需要实现：

- `init()`
- `onTrigger()`
- `conversionMs()`
- `onRead()`

## 数据有效性语义

父类统一维护设备侧的数据有效性：

- `markSuccess()` 会把 `data_valid_` 置为 `true`
- `markFailure()` 会把 `data_valid_` 置为 `false`

对外可以通过：

- `hasValidData()`
- `isDataFresh(now_ms, stale_ms)`

来判断最近缓存是否还能使用。

如果子类还维护了自己的缓存有效位，可以重写：

- `onDataInvalidated()`

在父类判定失败时同步清理自己的缓存标记。

## I2CUpdateManager

`I2CUpdateManager` 持有一张设备表，并在后台任务中串行推进每个设备。

当前实现不支持运行期动态注册或反注册设备：

- `registerDevice()` 只能在 `start()` 前调用
- 一旦后台任务启动，调度表就视为只读

这样做是为了避免注册流程与后台任务并发读写 `entries_` / `entry_count_`。

当设备进入 `Pending` 时，manager 会把 `next_due_ms` 暂时推到：

- `conversionDeadlineMs()`

这样等待窗口可以让给同一条总线上的其他设备。

## 周期调度策略

设备完成一轮更新后，manager 不再补跑已经错过的历史周期，而是直接跳到未来最近的周期点。

这样做的目的是：

- 避免任务卡顿后连续 replay backlog
- 保留周期相位的基本一致性
- 避免单个设备恢复后短时间占满整条总线

## 新设备接入建议

1. 在子类里实现 `init()` 做最小探活
2. 如果设备需要显式触发采样，实现 `onTrigger()`
3. 在 `onRead()` 中更新自己的缓存
4. 如果维护了额外的缓存有效标记，实现 `onDataInvalidated()`
5. 在 `start()` 前把设备注册到对应总线的 `I2CUpdateManager`

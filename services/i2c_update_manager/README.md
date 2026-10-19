# I2C Update Manager

`services::I2CUpdateManager` 在单个 CMSIS-RTOS v2 后台线程中串行调度同一 I2C 总线的周期设备，依赖 [I2C DMA Driver](../../bsp/i2c_driver/README.md)。它不是精确实时采样时钟，不保证严格周期。

## 设备模型与数据有效性

`I2CDevice` 提供 Trigger → Wait → Read 的更新模型。设备实现初始化/探活与读取缓存；需要触发转换的设备可覆写触发与转换等待钩子。返回 `Pending` 时，调度器等待该设备的转换期限，同时可服务同一总线上的其他设备。

初始化成功、在线与数据有效是不同概念，初始化成功不表示已有有效采样。使用方通过 `hasValidData()` 和 `isDataFresh(now_ms, stale_ms)` 判断缓存可用性；成功更新使数据有效，失败使数据失效。子类若还有独立缓存标记，应通过 `onDataInvalidated()` 同步清理，不能只检查旧缓存内容。

方法、时间戳和状态转换的精确契约见 [I2CDevice.hpp](I2CDevice.hpp) 的 Doxygen，不在 README 复制内部字段或逐状态实现。

## 注册、资源与线程归属

- 调度器借用总线及设备对象；它们必须覆盖后台访问的整个使用期。
- 最多注册 8 个设备；`registerDevice()` 只能在 `start()` 前调用，不支持运行期动态注册或反注册。启动后调度表视为只读。
- 每条总线由对应调度线程独占，不得让其他线程绕过 manager 并发访问同一 `I2CBusDMA`。
- 周期、初始相位、单次事务超时及线程配置由调用方提供；RTOS、DMA 和 I2C 硬件配置由宿主维护，遵循底层驱动前提。
- `stop()` 是退出请求，不是线程 join；不能把调用返回当作可以立即销毁借用资源的保证。

## 调度语义

设备等待转换时让出总线。完成一轮后，manager 跳过已错过的历史周期，推进到未来最近的周期点，不连续补跑 backlog；这保留基本周期相位并避免恢复后短时占满总线，但不构成精确时序保证。

接入时先实现设备协议钩子与缓存有效性，再于启动前注册设备并检查注册/启动结果。调度接口和资源参数见 [I2CUpdateManager.hpp](I2CUpdateManager.hpp) 的 Doxygen。

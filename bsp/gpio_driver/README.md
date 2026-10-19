# GPIO Driver

`bsp::GPIO_Driver` 提供 GPIO 电平操作、EXTI 回调分发与 PWM 启停/比较值辅助，cpkg 依赖为 `stm32cubemx`，不依赖 RTOS、不创建任务。新 C++ 代码入口是 [gpio_driver.hpp](gpio_driver.hpp) 的 `bsp::gpio`；[gpio_driver.h](gpio_driver.h) 保留已标记弃用的 C 接口；PWM 的 C/C++ 入口为 [pwm.h](pwm.h)。

## CubeMX 与运行期边界

- GPIO 入口要求 `HAL_GPIO_MODULE_ENABLED`，PWM 入口要求 `HAL_TIM_MODULE_ENABLED`。宿主通过 CubeMX 配置 GPIO 模式、上下拉、速度、复用、时钟、EXTI 触发和 NVIC，以及 TIM 时基、PWM 模式、通道、极性和输出引脚，并完成相应 HAL 初始化。
- 本包不调用 GPIO/TIM Init，不选择引脚、不修改 GPIO 模式，也不配置 EXTI 路由、PWM 频率或 ARR。它只在现有配置上读写电平、分发 EXTI、调用 HAL PWM Start/Stop 和更新比较值；不会验证 CubeMX 配置是否适合当前操作。
- 接口依托目标 STM32 HAL1，具体外设及 HAL 能力由宿主确认，不代表所有系列或通道已验证兼容。

## GPIO 句柄与所有权

`GpioPin`、`GpioPinOutput`、`GpioPinInput` 以及旧 `GPIO_t` 都只描述端口和引脚，不拥有、初始化或释放硬件。普通读写句柄可按值复制，但默认 `GpioPin` 的空 port/零 pin 不能直接用于操作；调用方须先提供有效端口、引脚和匹配的静态模式。读写、置位、复位及翻转直接调用 HAL，不返回额外错误码，也不做空指针校验。

`PWM_t` 同样只借用 `TIM_HandleTypeDef*` 并保存通道号；HAL 句柄及底层硬件必须覆盖所有操作期。没有析构自动停止输出或资源独占机制，同一引脚/定时器的不同包装值也不会互相协调。

## EXTI 接入与回调生命周期

宿主 IRQ 先进入目标 HAL 的 EXTI 处理，再从 `HAL_GPIO_EXTI_Callback` 调用 `bsp::gpio::DispatchExtiInterrupt(GPIO_Pin)`；若所用 HAL 使用不同的边沿回调入口，由宿主在对应位置转发。本包不定义这些 HAL 弱回调，也不直接安装 IRQ。

- [C++ 实现](gpio_driver.cpp) 的表按 pin 位号索引，共 16 个槽位，每条 EXTI 线只有一个回调，不区分 GPIO 端口；注册同一位号会覆盖旧注册并将计数清零。传入的 pin 必须是非零单 bit 值，未做校验；零值进入取最低置位位号运算，多 bit 只会选取最低位，不能用于批量分发。
- `RegisterExtiCallback` 保存 `GpioPinInput` 的**地址**，不是副本。该对象、`user_data` 指向的上下文和回调必须保持有效，直到在不与中断竞争的条件下注销完成；回调期间不要修改或销毁它们。
- 回调在分发调用的上下文同步执行，正常 HAL 接入时就是 ISR。每次先将 `uint32_t` 计数加一再调用回调，计数自然回绕，不含去抖或时间测量。回调应短小且不阻塞。
- 注销清除槽位和计数，但不屏蔽硬件 EXTI、不等待在途回调；注册/注销/分发没有锁或原子同步，宿主应在启用中断前完成注册，运行期变更须自己与该 IRQ 串行化。
- 旧 [C 实现](gpio_driver.c) 使用独立的表，保存 `GPIO_t*`、回调及 data；与 C++ 表互不联动。仅注册某一套不会收到另一套分发入口的通知，宿主应明确采用哪套入口，避免意外重复分发。

## PWM 的实际边界

- `PWM_Start` / `PWM_Stop` 直接调用 `HAL_TIM_PWM_Start` / `HAL_TIM_PWM_Stop`，**忽略 HAL 返回状态**，不能通过包装函数得知启停是否成功。
- `PWM_SetCompare` 仅在 `compare <= ARR` 时写入，否则静默保持原比较值；不返回错误、不调整 ARR。
- `PWM_SetDutyCircle` 对小于 0 或大于 1 的值分别写入 0 或 ARR，区间内按 `ARR * duty_circle + 0.5` 转换比较值。它使用 ARR 而非 ARR+1，实际电气占空比仍取决于计数/PWM 模式，不能把参数 1 当作任意定时器配置下的严格恒高保证。调用方应提供有限数值，接口没有 NaN 检查。
- 所有 GPIO/PWM 操作均无包级互斥；特别是翻转和对同一 TIM 通道的配置/启停不能据此获得线程安全保证。任务与 ISR 对共享资源的顺序和目标 HAL 的上下文限制由宿主负责。

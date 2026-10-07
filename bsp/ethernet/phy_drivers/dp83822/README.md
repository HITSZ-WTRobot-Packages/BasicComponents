# DP83822 PHY

DP83822（RMII，10BASE-Te / 100BASE-TX 铜缆）的 `IPhy` 实现，经 STM32 HAL MDIO 访问寄存器，
不依赖中间驱动后端或协议栈。

## 构造与可选引脚

```cpp
const GPIO_t reset_n{ PHY_RESET_N_GPIO_Port, PHY_RESET_N_Pin };
const GPIO_t interrupt_n{ PHY_INT_N_GPIO_Port, PHY_INT_N_Pin };
bsp::ethernet_phy::DP83822Phy phy{ heth, 1U, reset_n, interrupt_n };
```

构造只保存配置（含 `GPIO_t`），不访问硬件、不产生总线流量、不会失败，可安全声明为静态存储期对象。
两根引脚可独立省略，用 `{}` 表示未连接；`PHY_*_GPIO_Port/PHY_*_Pin` 必须成对定义。

- 未提供 `RESET_N`：走 `PHYRCR` bit15 完整复位（自清零等待上限 500 ms）。
- 提供 `RESET_N`：拉低至少 1 ms、释放后等待至少 2 ms 再探测地址，不叠加寄存器复位。
  `RESET_N` 配置为输出且初始为高。
- 提供 `INT_PWDN_N`：配置为带上拉的输入，MCU 不能将其拉低（该复用脚上电默认是低有效掉电输入）；
  `start()` 最后才把它切换为中断输出。
- `start()` 前置：`HAL_ETH_Init` 已完成、HAL tick 可用、GPIO 时钟/模式与 PHY 参考时钟就绪；
  MDIO 为共享总线，同一总线上的全部访问须由调用方串行化。

## 中断与 EXTI 归属

- 提供 INT 时，`start()` 配置低有效中断输出，只使能链路/速率/双工/协商完成事件
  （`MISR2` 全部关闭）；`usesInterrupt()`/`interruptAsserted()`/`acknowledgeInterrupt(bool&)`
  的语义见 [`core/README.md`](../../core/README.md)。
- 事件通知由本驱动自己维护 EXTI 注册：`setLinkEventCallback` 只保存 `callback`/`context`；
  成功 `start()` 后通过 `bsp::gpio::RegisterExtiCallback` 注册（`user_data` 指向本对象）；
  重复 `start()` 先注销旧注册并保留回调；启动失败不留下注册；析构只注销软件回调，
  不访问 MDIO、不复位器件。`callback`/`context` 更新与注册/注销使用保存-恢复 PRIMASK 的短临界区。
- ISR 路径只做转发：回调内不访问 MDIO、不调用 RTOS 之外的状态，由上层自行决定如何唤醒线程。
- 每条 EXTI 线只能有一个 owner，由板级设计保证。板级要求：下降沿 EXTI、输入上拉，
  IRQ 优先级数值不小于 FreeRTOS 可调用 ISR API 的边界（本工程为 5）；对应 IRQ handler 调用
  `HAL_GPIO_EXTI_IRQHandler`，唯一的 `HAL_GPIO_EXTI_Callback` 转发到
  `bsp::gpio::DispatchExtiInterrupt`。

## 数据手册依据

[TI DP83822 SNLS505H](https://www.ti.com/lit/ds/symlink/dp83822i.pdf)，
§6.8 复位时序、§8 PHYSCR/MISR1/MISR2 寄存器。

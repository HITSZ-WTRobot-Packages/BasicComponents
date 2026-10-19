# DP83822 PHY

`EthernetPHY::DP83822` 是 DP83822（RMII，10BASE-Te / 100BASE-TX 铜缆）的 [IPhy](../../core/README.md) 实现，通过 STM32 HAL MDIO 访问 PHY，不依赖协议栈或中间 PHY 后端。构造、结果码和器件操作细节见 [DP83822Phy.hpp](DP83822Phy.hpp) 的 Doxygen。

## 资源与启动前提

- 构造借用 ETH 句柄并保存 PHY 地址及可选 GPIO，不访问硬件，可用于静态存储期对象。句柄及对象须覆盖所有总线访问与事件通知的使用期。
- `RESET_N` 与 `INT_PWDN_N` 可独立省略，空 `GPIO_t` 表示未连接。引脚与地址由宿主提供，不预设外设实例或板级接线。
- 宿主通过 CubeMX 配置 MAC/RMII、GPIO、时钟及中断；`start()` 前 `HAL_ETH_Init` 已完成，HAL tick 与 PHY 参考时钟可用。
- 提供 `RESET_N` 时，宿主将其配置为初始高电平的输出；驱动运行期执行复位时序。未提供时，当前驱动通过 MDIO 完整复位器件，不保证在器件无响应时仍能恢复。
- 提供 `INT_PWDN_N` 时，宿主配置带上拉的输入，MCU 不得拉低此复用掉电引脚；驱动在启动末尾将 PHY 侧切换为低有效事件输出。
- MDIO 是共享总线，同一总线的全部访问由调用方串行化。本驱动不代替宿主重复初始化硬件静态配置，也不代表已完整校验全部配置。

## 中断所有权

- 每条 EXTI 线只允许一个 owner。宿主通过 CubeMX 配置下降沿触发及中断优先级；若通知使用 RTOS ISR API，优先级须符合宿主 RTOS 的可调用边界，不预设固定数值。
- GPIO 中断须经 HAL 分发到 `bsp::gpio::DispatchExtiInterrupt`；不得同时安装冲突的 EXTI owner。
- 成功启动后驱动维护自身 EXTI 软件注册；重复启动清理旧注册并保留用户回调，启动失败不遗留注册。
- 析构只注销软件回调，不访问 MDIO、不复位 PHY。调用方须先保证不再并发使用本对象。
- ISR 回调只用于通知线程，不能读 MDIO。中断确认和链路读取的线程限制见 [core README](../../core/README.md)。

## 运行期链路行为

驱动支持强制速率/双工或自动协商通告配置。相同有效配置不重复写入；改变自动协商通告或从强制模式进入自动协商时会重启协商，强制模式下显式重启协商返回 `Unsupported`。启动成功不保证链路已建立。

复位、PHY 事件使能与链路协商属于当前器件运行期行为，通过 HAL MDIO 访问；MCU 静态配置仍归宿主 CubeMX。本文不提供直接读写 MCU 寄存器或修改生成文件的替代集成方案；若 HAL1 能力不足或所需配置无法校验，应先通过 Ask 确认。

## 数据手册

[TI DP83822 SNLS505H](https://www.ti.com/lit/ds/symlink/dp83822i.pdf)：复位时序、链路配置与事件寄存器定义；仓库另保留 [器件手册](dp83822h.pdf)。

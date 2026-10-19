# BasicComponents

跨 STM32 宿主项目共享的基础组件仓库，作为 Git SubModule 使用，不独立构建。依赖与包名以各包的 `cpkg.toml` 为准；构建文件由 cpkg 生成。

下列索引列出全部 28 个包。`bsp/ethernet/`、`libs/control/`、`libs/math/`、`libs/utils/` 等是分类目录，不是包。链接优先指向已有包 README；尚无 README 的包链接到包目录，符号级用法见其源码 Doxygen。各驱动的 HAL、芯片能力和 RTOS 前提由对应包限定，不应从仓库共享范围推导全系列兼容性。

## 包索引

### bsp：外设与网络组件

| 包目录 | cpkg 包名 | 用途 |
| --- | --- | --- |
| [can_driver](bsp/can_driver/) | `bsp::CANDriver` | bxCAN 驱动 |
| [can_driver2](bsp/can_driver2/README.md) | `bsp::CANDriver2` | 固定存储 CAN/FDCAN 驱动 |
| [gpio_driver](bsp/gpio_driver/) | `bsp::GPIO_Driver` | GPIO 与 PWM 封装 |
| [i2c_driver](bsp/i2c_driver/README.md) | `bsp::I2CDriver` | 单 owner I2C DMA 同步封装 |
| [ethernet/core](bsp/ethernet/core/README.md) | `EthernetPHY::Core` | 同步 PHY 契约与链路事件接口 |
| [ethernet/phy_drivers/dp83822](bsp/ethernet/phy_drivers/dp83822/README.md) | `EthernetPHY::DP83822` | DP83822 PHY 驱动 |
| [ethernet/lwip_platform](bsp/ethernet/lwip_platform/README.md) | `EthernetPHY::LwipPlatform` | ETH MAC / LwIP netif 链路管理 |
| [ethernet/net/udp_socket](bsp/ethernet/net/udp_socket/) | `Ethernet::UdpSocket` | UDP socket 封装 |

### libs：通用库

| 包目录 | cpkg 包名 | 用途 |
| --- | --- | --- |
| [concurrency](libs/concurrency/) | `libs::Concurrency` | 并发控制 |
| [control/pid_pd](libs/control/pid_pd/) | `libs::PID_PD` | PID/PD 控制 |
| [control/pid_motor](libs/control/pid_motor/) | `libs::PIDMotor` | 电机 PID 控制 |
| [control/mit_pd](libs/control/mit_pd/) | `libs::MIT_PD` | MIT PD 控制 |
| [math/LinearAlgebra](libs/math/LinearAlgebra/) | `Math::LinearAlgebra` | 线性代数 |
| [math/Geometry](libs/math/Geometry/) | `Math::Geometry` | 几何运算 |
| [math/EKF](libs/math/EKF/) | `Math::EKF` | 扩展卡尔曼滤波 |
| [traits](libs/traits/) | `traits` | 类型约束辅助 |
| [utils/crc](libs/utils/crc/) | `libs::CRC` | CRC 运算 |
| [utils/deque](libs/utils/deque/) | `libs::Deque` | 双向队列 |
| [utils/fixed_map](libs/utils/fixed_map/) | `libs::FixedMap` | 固定容量映射 |
| [utils/ring_buffer](libs/utils/ring_buffer/) | `libs::RingBuffer` | 环形缓冲区 |
| [utils/printf](libs/utils/printf/) | `libs::printf` | 格式化输出 |

### memory：内存与对象存储

| 包目录 | cpkg 包名 | 用途 |
| --- | --- | --- |
| [DMABuffer](memory/DMABuffer/) | `memory::DMABuffer` | DMA 缓冲区 |
| [StaticObject](memory/StaticObject/) | `memory::StaticObject` | 静态对象存储 |
| [StaticArena](memory/StaticArena/) | `memory::StaticArena` | 线性内存分配器 |

### protocol：协议辅助

| 包目录 | cpkg 包名 | 用途 |
| --- | --- | --- |
| [UartRxSync](protocol/UartRxSync/) | `protocol::UartRxSync` | 带帧头同步的串口接收 |

### services：运行期服务

`services/` 与 `protocol/` 同级，不属于协议目录。

| 包目录 | cpkg 包名 | 用途 |
| --- | --- | --- |
| [watchdog](services/watchdog/) | `services::Watchdog` | 看门狗服务 |
| [i2c_update_manager](services/i2c_update_manager/README.md) | `services::I2CUpdateManager` | 单 I2C 总线周期设备调度 |

### utils：共享辅助

| 包目录 | cpkg 包名 | 用途 |
| --- | --- | --- |
| [utils](utils/) | `utils` | 中断保护锁等共享辅助函数 |

## 其他目录与维护说明

`bsp/ethernet/examples/` 保存历史板级示例，不含 `cpkg.toml`，不是可依赖包，也不代表通用接线或芯片兼容承诺。宿主硬件静态配置应由其 CubeMX 配置维护，不照搬特定板级示例。

协作与文档归属见 [AGENTS.md](AGENTS.md)。包 README 说明生命周期、资源与集成限制；逐符号契约以包内既有 Doxygen 为准。

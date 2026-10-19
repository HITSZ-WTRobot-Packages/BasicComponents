# BasicComponents

跨 STM32 宿主项目共享的基础组件仓库，作为 Git SubModule 使用，不独立构建。依赖与包名以各包的 `cpkg.toml` 为准；构建文件由 cpkg 生成。

下列索引列出全部 28 个包。`bsp/ethernet/`、`libs/control/`、`libs/math/`、`libs/utils/` 等是分类目录，不是包。所有包链接均使用指向包内 `README.md` 的相对文件路径，不使用目录链接或源码外链代替包文档，以兼容文档快照同步与渲染。包 README 说明生命周期、资源与集成限制，符号级用法见其源码 Doxygen。各驱动的 HAL、芯片能力和 RTOS 前提由对应包限定，不应从仓库共享范围推导全系列兼容性。

## 包索引

### bsp：外设与网络组件

| 包目录 | cpkg 包名 | 用途 |
| --- | --- | --- |
| [can_driver](bsp/can_driver/README.md) | `bsp::CANDriver` | HAL1 CAN/FDCAN 兼容驱动 |
| [can_driver2](bsp/can_driver2/README.md) | `bsp::CANDriver2` | 固定存储 CAN/FDCAN 驱动 |
| [gpio_driver](bsp/gpio_driver/README.md) | `bsp::GPIO_Driver` | GPIO 与 PWM 封装 |
| [i2c_driver](bsp/i2c_driver/README.md) | `bsp::I2CDriver` | 单 owner I2C DMA 同步封装 |
| [ethernet/core](bsp/ethernet/core/README.md) | `EthernetPHY::Core` | 同步 PHY 契约与链路事件接口 |
| [ethernet/phy_drivers/dp83822](bsp/ethernet/phy_drivers/dp83822/README.md) | `EthernetPHY::DP83822` | DP83822 PHY 驱动 |
| [ethernet/lwip_platform](bsp/ethernet/lwip_platform/README.md) | `EthernetPHY::LwipPlatform` | ETH MAC / LwIP netif 链路管理 |
| [ethernet/net/udp_socket](bsp/ethernet/net/udp_socket/README.md) | `Ethernet::UdpSocket` | UDP socket 封装 |

### libs：通用库

| 包目录 | cpkg 包名 | 用途 |
| --- | --- | --- |
| [concurrency](libs/concurrency/README.md) | `libs::Concurrency` | 并发控制 |
| [control/pid_pd](libs/control/pid_pd/README.md) | `libs::PID_PD` | PID/PD 控制 |
| [control/pid_motor](libs/control/pid_motor/README.md) | `libs::PIDMotor` | 电机 PID 控制 |
| [control/mit_pd](libs/control/mit_pd/README.md) | `libs::MIT_PD` | MIT PD 控制 |
| [math/LinearAlgebra](libs/math/LinearAlgebra/README.md) | `Math::LinearAlgebra` | 线性代数 |
| [math/Geometry](libs/math/Geometry/README.md) | `Math::Geometry` | 几何运算 |
| [math/EKF](libs/math/EKF/README.md) | `Math::EKF` | 扩展卡尔曼滤波 |
| [traits](libs/traits/README.md) | `traits` | 类型约束辅助 |
| [utils/crc](libs/utils/crc/README.md) | `libs::CRC` | CRC 运算 |
| [utils/deque](libs/utils/deque/README.md) | `libs::Deque` | 双向队列 |
| [utils/fixed_map](libs/utils/fixed_map/README.md) | `libs::FixedMap` | 固定容量映射 |
| [utils/ring_buffer](libs/utils/ring_buffer/README.md) | `libs::RingBuffer` | 环形缓冲区 |
| [utils/printf](libs/utils/printf/README.md) | `libs::printf` | 格式化输出 |

### memory：内存与对象存储

| 包目录 | cpkg 包名 | 用途 |
| --- | --- | --- |
| [DMABuffer](memory/DMABuffer/README.md) | `memory::DMABuffer` | DMA 缓冲区 |
| [StaticObject](memory/StaticObject/README.md) | `memory::StaticObject` | 静态对象存储 |
| [StaticArena](memory/StaticArena/README.md) | `memory::StaticArena` | 线性内存分配器 |

### protocol：协议辅助

| 包目录 | cpkg 包名 | 用途 |
| --- | --- | --- |
| [UartRxSync](protocol/UartRxSync/README.md) | `protocol::UartRxSync` | 带帧头同步的串口接收 |

### services：运行期服务

`services/` 与 `protocol/` 同级，不属于协议目录。

| 包目录 | cpkg 包名 | 用途 |
| --- | --- | --- |
| [watchdog](services/watchdog/README.md) | `services::Watchdog` | 看门狗服务 |
| [i2c_update_manager](services/i2c_update_manager/README.md) | `services::I2CUpdateManager` | 单 I2C 总线周期设备调度 |

### utils：共享辅助

| 包目录 | cpkg 包名 | 用途 |
| --- | --- | --- |
| [utils](utils/README.md) | `utils` | 中断保护锁等共享辅助函数 |

## 其他目录与维护说明

`bsp/ethernet/examples/` 保存历史板级示例，不含 `cpkg.toml`，不是可依赖包，也不代表通用接线或芯片兼容承诺。宿主硬件静态配置应由其 CubeMX 配置维护，不照搬特定板级示例。

协作与文档归属见 [AGENTS.md](AGENTS.md)。包 README 说明生命周期、资源与集成限制；逐符号契约以包内既有 Doxygen 为准。

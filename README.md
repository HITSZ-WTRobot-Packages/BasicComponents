# BasicComponents

项目基本组件

## List

- bsp: 对 HAL 库的基本封装
    - ![Last Update](https://img.shields.io/github/last-commit/HITSZ-WTRobot-Packages/BasicComponents?path=bsp%2Fcan_driver&label=%E6%9C%80%E5%90%8E%E6%9B%B4%E6%96%B0&color=2ea44f&style=flat-square&logo=github) can_driver ：STM32 bxCAN 驱动
    - can_driver2 ：固定存储的 CAN/FDCAN 驱动，发送帧检查默认关闭（见下文）
    - ![Last Update](https://img.shields.io/github/last-commit/HITSZ-WTRobot-Packages/BasicComponents?path=bsp%2Fgpio_driver&label=%E6%9C%80%E5%90%8E%E6%9B%B4%E6%96%B0&color=2ea44f&style=flat-square&logo=github) gpio_driver ：STM32 GPIO 封装（GPIO + PWM）
    - ethernet: 以太网与 PHY 组件
        - ![Last Update](https://img.shields.io/github/last-commit/HITSZ-WTRobot-Packages/BasicComponents?path=bsp%2Fethernet%2Fcore&label=%E6%9C%80%E5%90%8E%E6%9B%B4%E6%96%B0&color=2ea44f&style=flat-square&logo=github) [core](bsp/ethernet/core/README.md) ：`IPhy` 同步 PHY 契约与链路事件接口
        - ![Last Update](https://img.shields.io/github/last-commit/HITSZ-WTRobot-Packages/BasicComponents?path=bsp%2Fethernet%2Fphy_drivers%2Fdp83822&label=%E6%9C%80%E5%90%8E%E6%9B%B4%E6%96%B0&color=2ea44f&style=flat-square&logo=github) [dp83822](bsp/ethernet/phy_drivers/dp83822/README.md) ：DP83822 PHY 驱动（可选 RESET_N / INT_PWDN_N）
        - ![Last Update](https://img.shields.io/github/last-commit/HITSZ-WTRobot-Packages/BasicComponents?path=bsp%2Fethernet%2Flwip_platform&label=%E6%9C%80%E5%90%8E%E6%9B%B4%E6%96%B0&color=2ea44f&style=flat-square&logo=github) [lwip_platform](bsp/ethernet/lwip_platform/README.md) ：持有链路线程的 STM32 ETH MAC / LwIP netif 平台
        - udp_socket ：UDP socket 封装

- libs:
    - ![Last Update](https://img.shields.io/github/last-commit/HITSZ-WTRobot-Packages/BasicComponents?path=libs%2Fconcurrency&label=%E6%9C%80%E5%90%8E%E6%9B%B4%E6%96%B0&color=2ea44f&style=flat-square&logo=github) concurrency ： 并发控制库
    - ![Last Update](https://img.shields.io/github/last-commit/HITSZ-WTRobot-Packages/BasicComponents?path=libs%2Fcontrol&label=%E6%9C%80%E5%90%8E%E6%9B%B4%E6%96%B0&color=2ea44f&style=flat-square&logo=github) control : 控制算法库（PID 等）
    - math: 数学运算库
        - ![Last Update](https://img.shields.io/github/last-commit/HITSZ-WTRobot-Packages/BasicComponents?path=libs%2Fmath%2FLinearAlgebra&label=%E6%9C%80%E5%90%8E%E6%9B%B4%E6%96%B0&color=2ea44f&style=flat-square&logo=github) LinearAlgebra : 线性代数运算库
        - ![Last Update](https://img.shields.io/github/last-commit/HITSZ-WTRobot-Packages/BasicComponents?path=libs%2Fmath%2FGeometry&label=%E6%9C%80%E5%90%8E%E6%9B%B4%E6%96%B0&color=2ea44f&style=flat-square&logo=github) Geometry : 几何运算库（坐标变换，四元数等）
        - ![Last Update](https://img.shields.io/github/last-commit/HITSZ-WTRobot-Packages/BasicComponents?path=libs%2Fmath%2FEKF&label=%E6%9C%80%E5%90%8E%E6%9B%B4%E6%96%B0&color=2ea44f&style=flat-square&logo=github) EKF : 扩展卡尔曼滤波器
    - ![Last Update](https://img.shields.io/github/last-commit/HITSZ-WTRobot-Packages/BasicComponents?path=libs%2Ftraits&label=%E6%9C%80%E5%90%8E%E6%9B%B4%E6%96%B0&color=2ea44f&style=flat-square&logo=github) traits : 类特征库（NoCopy, NoDelete 等）
    - utils 暂时不知道怎么分类的小工具
        - ![Last Update](https://img.shields.io/github/last-commit/HITSZ-WTRobot-Packages/BasicComponents?path=libs%2Futils%2Fcrc&label=%E6%9C%80%E5%90%8E%E6%9B%B4%E6%96%B0&color=2ea44f&style=flat-square&logo=github) crc : 查表法 CRC 运算库（支持不同长度）
        - ![Last Update](https://img.shields.io/github/last-commit/HITSZ-WTRobot-Packages/BasicComponents?path=libs%2Futils%2Fdeque&label=%E6%9C%80%E5%90%8E%E6%9B%B4%E6%96%B0&color=2ea44f&style=flat-square&logo=github) deque : 双向队列
        - ![Last Update](https://img.shields.io/github/last-commit/HITSZ-WTRobot-Packages/BasicComponents?path=libs%2Futils%2Ffixed_map&label=%E6%9C%80%E5%90%8E%E6%9B%B4%E6%96%B0&color=2ea44f&style=flat-square&logo=github) fixed_map : 离散指针表
        - ![Last Update](https://img.shields.io/github/last-commit/HITSZ-WTRobot-Packages/BasicComponents?path=libs%2Futils%2Fring_buffer&label=%E6%9C%80%E5%90%8E%E6%9B%B4%E6%96%B0&color=2ea44f&style=flat-square&logo=github) ring_buffer : 环形缓冲区
        - printf: printf
- protocol: 通信库
    - ![Last Update](https://img.shields.io/github/last-commit/HITSZ-WTRobot-Packages/BasicComponents?path=protocol%2FUartRxSync&label=%E6%9C%80%E5%90%8E%E6%9B%B4%E6%96%B0&color=2ea44f&style=flat-square&logo=github) UartRxSync : 带帧头同步功能的串口接收库（常用于传感器数据接收）
    - services: 常用服务
        - ![Last Update](https://img.shields.io/github/last-commit/HITSZ-WTRobot-Packages/BasicComponents?path=services%2Fwatchdog&label=%E6%9C%80%E5%90%8E%E6%9B%B4%E6%96%B0&color=2ea44f&style=flat-square&logo=github) watchdog : 看门狗服务
- utils ![Last Update](https://img.shields.io/github/last-commit/HITSZ-WTRobot-Packages/BasicComponents?path=utils&label=%E6%9C%80%E5%90%8E%E6%9B%B4%E6%96%B0&color=2ea44f&style=flat-square&logo=github): 懒得分类的小工具
    - static_arena: 线性内存分配器
    - isr_lock.h: 中断保护锁

具体使用方法请查看代码注释

## can_driver2 HAL 兼容范围

后端按启用的 HAL 模块选择，不按 STM32 系列名称限制：`HAL_CAN` 对应 bxCAN（Classic CAN，最多 8 字节），`HAL_FDCAN` 对应 FDCAN（Classic/FD）。要求 C++20、对应 `USE_HAL_CAN_REGISTER_CALLBACKS` 或 `USE_HAL_FDCAN_REGISTER_CALLBACKS` 为 `1`，且一个固件只能启用其中一个 HAL CAN 后端。板级代码负责 HAL 初始化和 IRQ 转发。

- **STM32G4**：适配固定 Message RAM 布局，Rx FIFO0/1 各 3 个 element，Tx FIFO/Queue 3 个 element，每个数据 element 可容纳 64 字节。标准/扩展过滤器容量取 `Init.StdFiltersNbr` / `Init.ExtFiltersNbr` 与硬件上限 28/8 的较小值；须在 HAL Init 前启用所需数量，数量为 0 时配置 ID 过滤器返回 `NoCapacity`。专用 `RxBuffer` 和校准报文功能不支持，返回 `Unsupported`，不静默改为 FIFO。
- **STM32H7**：保留从 `Init` 读取的可配置 FIFO/Buffer 数量和 element 尺寸。接收按 `FDCAN_DLC_BYTES_*` 宏反查长度，兼容旧 HAL 移位编码与新 HAL 原始编码；Classic DLC 9..15 仍按 8 字节交付，RTR 保留请求长度且负载为空。发送兼容旧 HAL 的非 const 数据参数声明，不增加负载复制。
- **STM32F 的 bxCAN 型号**：保留单控制器 14 bank、CAN1/CAN2 共享 28 bank 和独立 CAN3 路径；只支持 Classic CAN。
- 更广的系列兼容性与历史验证记录见 `bsp/can_driver2/can_driver.hpp` 文件头；编译或主机回归结果不等于板上时序、IRQ 和收发验收。

本次修复后的驱动交叉编译覆盖 F103、F105、F405、F413、G431、G473、C092、U575、U375、H7S7、H723、N657；每个目标均编译三个驱动翻译单元，覆盖 TX 校验 0/1 和软件 TX 队列 0/4。F405、G431、H723 的过滤器主机回归也在这四种配置下通过 ASan/UBSan。未进行板上验证，也未重新验证 H7 HAL v1.11.0 或 H7 双核 CM4。

配置 bxCAN 的 `IdFilter`（包括 `Disable`）时，`id1`/`id2` 超出所选 ID 位宽返回 `InvalidArgument`；非 `RxBuffer` 动作使用非零 `rx_buffer_index` 也返回 `InvalidArgument`。bxCAN 不支持的 `RxBuffer` 和校准报文请求仍返回 `Unsupported`；HAL 句柄配置错误仍返回 `InvalidConfiguration`，filter bank 索引越界仍返回 `OutOfRange`。非法禁用请求不会修改已配置的过滤器。

高优先级接收必须显式选择 `FilterAction::PriorityFifo0` 或 `PriorityFifo1`，匹配帧存入指定 FIFO 并经接收回调交付。已删除没有接收目标的 `FilterAction::Priority`：原生 HP-only 动作只产生优先级元数据通知，不存储帧，而本驱动不提供该通知接口。旧调用方须改用显式 FIFO 动作；不保留别名或隐式默认 FIFO。bxCAN 对这两种 FDCAN 专属动作返回 `Unsupported`。

## can_driver2 主机回归

`bsp/can_driver2/can_driver_filter_test.py` 使用真实 ST HAL/CMSIS 头文件、三个实际驱动翻译单元和宿主 C++20 编译器；仅模拟 HAL 硬件边界、帧注入和 CMSIS 中断原语。临时 C++ fixtures 与构建产物自动清理，不会进入固件源文件扫描。在 BasicComponents 根目录运行：

```sh
python3 bsp/can_driver2/can_driver_filter_test.py --family f4 \
    --drivers /path/to/STM32Cube_FW_F4/Drivers --sanitize
python3 bsp/can_driver2/can_driver_filter_test.py --family g4 \
    --drivers /path/to/STM32Cube_FW_G4/Drivers \
    --validate-tx-frame 1 --tx-queue-size 4 --sanitize
python3 bsp/can_driver2/can_driver_filter_test.py --family h7 \
    --drivers /path/to/STM32Cube_FW_H7/Drivers \
    --validate-tx-frame 1 --tx-queue-size 4 --sanitize
```

回归覆盖 bxCAN 标准/扩展 ID 的禁用边界、非法禁用保留原过滤器、有效禁用停止交付，以及 FDCAN `PriorityFifo0/1` 的 FIFO 路由和完整负载交付、未匹配帧拒收、禁用与未分配 FIFO 的错误。`--device` 可覆盖默认的 F405/G431/H723 芯片宏，`--cxx` 指定宿主编译器；TX 校验可选 0/1，软件队列可选 0/4。测试不模拟总线时序、仲裁、错误计数或 TX 完成，也不能替代板上验收。

## can_driver2 发送帧检查

`CAN_DRIVER_VALIDATE_TX_FRAME` 默认值为 `0`，发送时不执行帧结构及后端帧能力校验；非零时启用原有校验。
通过 CANDriver2 驱动目标的编译定义（如 `-DCAN_DRIVER_VALIDATE_TX_FRAME=1`）统一覆盖，所有使用该驱动的翻译单元须保持宏定义一致。
关闭检查时，调用方必须保证 ID、字段组合、负载指针及长度合法，并确保帧格式和负载适配已配置的外设，尤其是 FDCAN TX element 容量；非法输入可能导致越界访问，不保证返回 `InvalidFrame` 或 `Unsupported`。
该开关不影响启动状态、Bus-Off、发送空位及 HAL 错误处理，也不影响接收和过滤器检查。


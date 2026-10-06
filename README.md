# BasicComponents

项目基本组件

## List

- bsp: 对 HAL 库的基本封装
    - ![Last Update](https://img.shields.io/github/last-commit/HITSZ-WTRobot-Packages/BasicComponents?path=bsp%2Fcan_driver&label=%E6%9C%80%E5%90%8E%E6%9B%B4%E6%96%B0&color=2ea44f&style=flat-square&logo=github) can_driver ：STM32 bxCAN 驱动
    - ![Last Update](https://img.shields.io/github/last-commit/HITSZ-WTRobot-Packages/BasicComponents?path=bsp%2Fgpio_driver&label=%E6%9C%80%E5%90%8E%E6%9B%B4%E6%96%B0&color=2ea44f&style=flat-square&logo=github) gpio_driver ：STM32 GPIO 封装（GPIO + PWM）

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

## Ethernet 链路轮询

`bsp/ethernet/lwip_platform` 的 `LwipPlatform::poll()` 返回 `PollResult`，不再返回 `bool`。
链路状态仍由 `PhyLinkState` 表示；正常断链和协商中不是轮询错误。

| 结果 | 含义 |
| --- | --- |
| `Ok` | 本轮链路状态已成功处理，不代表链路一定为 up。 |
| `PhyReadError` | PHY 寄存器读取失败；接口已置为 down，MAC 已停止或本就未启动，可由下一周期重新检查。 |
| `MacStopError` | 停止 MAC 或其必要取锁失败；优先于同时发生的 PHY 读取错误上报。 |
| `MacStartError` | 获取/设置 MAC 配置、启动 MAC 或其必要取锁失败。 |
| `NotInitialized` | 平台尚未初始化，或 PHY 报告未就绪。 |
| `InvalidPhyState` | PHY 返回未知链路状态，或 `readLink()` 返回契约外结果。 |

STM32H723 示例的 C 入口 `lwip_platform_poll()` 保留 `bool`，无需修改 Cube 生成的调用代码：
仅 `Ok`、`PhyReadError` 返回 `true`，其余结果返回 `false`，仍交由现有 `Error_Handler()` 处理。
PHY 读取失败不会在函数内重试；下一周期读取恢复正常后，平台按实际链路状态重新启动 MAC。
此策略不修正 HAL 的 MDIO 超时判定，也不改变线程优先级。
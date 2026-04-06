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
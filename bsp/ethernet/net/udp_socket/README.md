# UDP Socket

`Ethernet::UdpSocket` 是 `net::UdpSocket` 的 cpkg 包，提供基于 LwIP socket API 的最小 IPv4 UDP 同步封装。入口与结果类型见 [udp_socket.hpp](udp_socket.hpp)，实际系统调用见 [udp_socket.cpp](udp_socket.cpp)。依赖为 `EthernetPHY::Core`、`FreeRTOS`、`LwIP` 与 `stm32cubemx`；接口使用 `std::span`，需要 C++20。

## 集成与线程模型

- 宿主先建立并初始化 LwIP 网络栈、netif 与底层收发数据面，提供可用的 IPv4、UDP、socket/netconn 和 `lwip_select` 支持。本包不初始化 ETH/PHY，不配置地址、路由或链路，不绑定具体 STM32 芯片/板卡；链路管理可参考独立的 [LwipPlatform 文档](../../lwip_platform/README.md)。
- `LWIP_COMPAT_SOCKETS` 必须为 0，否则头文件编译报错；同一翻译单元直接操作原生 socket 时使用显式 `lwip_*` 名称。
- 必须由**单一所属任务**在 LwIP 初始化完成后使用对象。没有互斥体、内部任务、队列、异步接收或回调；不得在 ISR 调用，不得跨任务并发操作同一实例，也不得在收发进行时并发 `close()` 或析构。
- 本包使用同步 socket API，不是 LwIP raw API；接收可能阻塞，不能放在需要持续处理 LwIP 核心消息的 tcpip 线程/回调里。本包不管理 LwIP 核心线程或替宿主建立调用上下文。

## Socket 与端点生命周期

默认构造只将文件描述符置为关闭，不访问网络；对象不可复制、不可移动。`open(local)` 创建 `AF_INET/SOCK_DGRAM/IPPROTO_UDP` socket 并绑定：成功返回 0，已打开返回 `EALREADY` 且不改变原 socket，创建/绑定失败返回正的 errno 并保持关闭；绑定失败会关闭临时 fd。

`UdpEndpoint.address` 是网络序 `ip4_addr_t`，`port` 是主机序 `uint16_t`，封装只转换端口字节序，不做 DNS 解析。默认本地端点全零表示 `0.0.0.0` 和由栈分配端口。端点在调用内转换为栈上 `sockaddr_in`，不会保留调用方端点引用；接收结果里的源端点按值返回。没有查询自动分配端口的包装接口。

`close()` 可重复调用，析构自动调用它；实现忽略 `lwip_close` 的返回值并将自身 fd 置为关闭，因此不是可报告关闭错误的接口。`isOpen()` 只检查是否持有 fd，不表示链路可用或远端可达。

## 发送与缓冲区所有权

`sendTo(remote, data)` 直接调用 `lwip_sendto`，不等待 DMA 完成、不承诺复制负载，也没有发送完成通知。允许空 span 发送零长度数据报；非空 span 必须指向有效缓冲。

**不能把 `sendTo` 返回当作发送缓冲可以复用或释放的信号。** 头文件明确约定原生提交语义：在 `LWIP_NETIF_TX_SINGLE_PBUF==0` 且底层异步发送的路径上，驱动可能在返回后继续借用 payload。调用方必须按宿主实际 LwIP/驱动的完成契约保持数据有效且不变，本包不接管缓冲区所有权，也不提供独立副本来解除此约束。

未打开返回 `EBADF`；LwIP 调用失败返回正的 errno，否则返回 0。成功仅表示原生发送调用成功，不保证远端接收；封装没有发送超时参数或重试，调用是否等待栈资源由底层 socket 实现决定，不能视为 ISR 可用的非阻塞发送。

## 接收、阻塞与错误

`receiveFrom(buffer, timeoutMs)` 每次消费一个数据报，保持数据报边界；接收缓冲仅在调用期间借用，数据在返回前写入，源端点由结果保存。

| timeoutMs | 行为 |
| --- | --- |
| `0` | 使用 `MSG_DONTWAIT` 立即尝试，无数据返回 `EWOULDBLOCK` |
| `WaitForever`（`UINT32_MAX`） | 阻塞等待，不设置超时 |
| 其他正值 | 先 `lwip_select` 等待对应毫秒数；无就绪返回 `ETIMEDOUT`，就绪后使用 `MSG_DONTWAIT` 接收 |

有限等待的 select 或后续接收错误直接返回 errno，不重试；即使 select 曾就绪，后续接收仍可能返回错误。未打开返回 `EBADF`。调用方只在 `result.error == 0` 时读取其他字段：

- `size` 是实际写入的字节数；零长度 UDP 报文也是成功，不是关闭或超时。
- 报文长于 buffer 时，`truncated=true`，`size` 不超过 buffer 大小，多余数据被丢弃，不能通过下一次调用读取余下部分。
- 允许空接收 span：实现使用调用内的一个临时字节完成接收并丢弃数据，结果 `size=0`；非空报文标记截断，零长度报文成功且不截断。

对象自身只保存 fd，没有应用堆缓冲或内部任务分配；**不等于网络路径无分配**，socket、netconn、报文及底层驱动资源仍由 LwIP/宿主配置管理。接收不自动扩容，发送不分片重组应用消息，本包也不提供重传、连接状态或协议级确认。

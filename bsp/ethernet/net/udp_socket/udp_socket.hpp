/**
 * @file    udp_socket.hpp
 * @brief   基于 lwIP socket API 的最小 IPv4 UDP 同步封装。
 *
 * 仅暴露创建/绑定、原生提交发送、带超时单次接收与关闭。不提供异步接收、
 * 内部任务/队列/回调，也不做 DNS 解析。地址与端口使用 ip4_addr_t（网络序）
 * 与主机序 port。
 *
 * 生命周期：本类不自带同步原语，必须由单一所属任务在 lwIP 初始化完成后使用；
 * 不得跨任务并发调用同一实例，也不得在读写进行中并发 close()。
 *
 * sendTo 采用 lwIP 原生提交语义：把调用方缓冲直接交给 netconn（当前
 * LWIP_NETIF_TX_SINGLE_PBUF==0 且底层驱动异步），driver 可能在 sendTo 返回后
 * 仍借用该 payload。调用者不得把 sendTo 返回当作缓冲可复用的信号；本类
 * 不提供发送完成通知，也不改变底层的缓冲生命周期。
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "lwip/ip4_addr.h"
#include "lwip/sockets.h"

/* 先包含 sockets.h，再移除与成员 close 冲突的 POSIX 宏，避免包含顺序影响。
   同一翻译单元若直接关闭原生 socket，请显式调用 lwip_close。 */
#if LWIP_COMPAT_SOCKETS != 0
#    error "Please set LWIP_COMPAT_SOCKETS = 0."
#endif

namespace net
{

/** @brief IPv4 端点：address 为网络字节序的 ip4_addr_t；port 为主机字节序。 */
struct UdpEndpoint
{
    ip4_addr_t    address{};
    std::uint16_t port{};
};

/**
 * @brief 单次接收结果。
 *
 * error 为 0 时表示成功取到报文（含零长度报文），size 是实际写入 buffer 的
 * 字节数；truncated 为 true 表示原数据报比 buffer 长，多出部分已被丢弃，
 * 此时 size==buffer.size()。error 非 0 时其余字段无意义。
 */
struct UdpReceiveResult
{
    int         error{};
    std::size_t size{};
    UdpEndpoint source{};
    bool        truncated{};
};

/**
 * @brief 独占式 IPv4 UDP socket 封装（不可复制、不可移动）。
 *
 * 默认构造不访问网络，仅在 open() 时创建 socket。析构自动 close()。
 * 仅持有 lwIP 文件描述符；不分配应用堆。
 *
 * 所有权：必须由单一所属任务在 lwIP 初始化完成后使用，禁止跨任务并发访问
 * 同一实例，也禁止在读写进行中并发 close()；sendTo 的缓冲借用约束见文件头。
 */
class UdpSocket final
{
public:
    /** @brief 无界等待：receiveFrom 的 timeoutMs 取值。 */
    static constexpr std::uint32_t WaitForever = UINT32_MAX;

    UdpSocket() noexcept = default;
    ~UdpSocket() noexcept;

    UdpSocket(const UdpSocket&)            = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;
    UdpSocket(UdpSocket&&)                 = delete;
    UdpSocket& operator=(UdpSocket&&)      = delete;

    /**
     * @brief 创建 AF_INET UDP socket 并绑定到 local。
     * @param[in] local 本地端点；address 为网络序，port 为主机序。默认全零表示
     *                  0.0.0.0 且由协议栈自动分配端口。
     * @return 成功返回 0；已打开返回 EALREADY；其余返回正的 errno。
     *         创建或绑定失败时保持关闭；已打开时不改变原 socket。
     */
    int open(const UdpEndpoint& local = {}) noexcept;

    /**
     * @brief 以 lwIP 原生语义提交一个 UDP 数据报，不等待 DMA、不承诺拷贝。
     * @param[in] remote 目标端点（address 网络序，port 主机序）。
     * @param[in] data   待发送字节；允许空 span（发送零长度数据报）。
     * @return 成功返回 0；未打开返回 EBADF；其余返回正的 errno。
     */
    int sendTo(const UdpEndpoint& remote, std::span<const std::byte> data) noexcept;

    /**
     * @brief 同步接收一个 UDP 数据报（保持数据报边界）。
     * @param[out] buffer    接收缓冲；允许空 span。
     * @param[in]  timeoutMs 0 表示立即返回（无数据为 EWOULDBLOCK）；WaitForever
     *                       表示无限等待；其余为有限毫秒，超时返回 ETIMEDOUT。
     * @return 结果对象；error==0 表示成功（可能是零长度报文），truncated 标记
     *         报文被截断，size 为实际写入字节数。
     */
    UdpReceiveResult receiveFrom(std::span<std::byte> buffer,
                                 std::uint32_t        timeoutMs = WaitForever) noexcept;

    /** @brief 关闭 socket；幂等，可重复调用。 */
    void close() noexcept;

    /** @brief 是否已打开。 */
    [[nodiscard]] bool isOpen() const noexcept;

private:
    int fd_{ -1 };
};

} // namespace net

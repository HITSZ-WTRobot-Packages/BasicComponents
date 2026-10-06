#include "udp_socket.hpp"

#include <cerrno>
#include <cstring>

namespace
{

/** @brief 端点 → 栈上 sockaddr_in（端口主机序转网络序，地址已是网络序）。 */
void endpointToSockaddr(const net::UdpEndpoint& endpoint, sockaddr_in& address) noexcept
{
    std::memset(&address, 0, sizeof(address));
    address.sin_len         = sizeof(address);
    address.sin_family      = AF_INET;
    address.sin_port        = lwip_htons(endpoint.port);
    address.sin_addr.s_addr = endpoint.address.addr;
}

/** @brief 栈上 sockaddr_in → 端点（端口网络序转主机序，地址保持网络序）。 */
void sockaddrToEndpoint(const sockaddr_in& address, net::UdpEndpoint& endpoint) noexcept
{
    endpoint.address.addr = address.sin_addr.s_addr;
    endpoint.port         = lwip_ntohs(address.sin_port);
}

} // namespace

namespace net
{

UdpSocket::~UdpSocket() noexcept
{
    close();
}

int UdpSocket::open(const UdpEndpoint& local) noexcept
{
    if (fd_ >= 0)
    {
        return EALREADY;
    }

    const int fd = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd < 0)
    {
        return errno;
    }

    sockaddr_in local_address;
    endpointToSockaddr(local, local_address);

    if (lwip_bind(fd, reinterpret_cast<const sockaddr*>(&local_address), sizeof(local_address)) < 0)
    {
        const int error = errno;
        (void)lwip_close(fd);
        return error;
    }

    fd_ = fd;
    return 0;
}

int UdpSocket::sendTo(const UdpEndpoint& remote, std::span<const std::byte> data) noexcept
{
    if (fd_ < 0)
    {
        return EBADF;
    }

    sockaddr_in remote_address;
    endpointToSockaddr(remote, remote_address);

    static constexpr std::byte empty_payload{};
    const void* payload = data.empty() ? static_cast<const void*>(&empty_payload)
                                       : static_cast<const void*>(data.data());

    if (lwip_sendto(fd_, payload, data.size(), 0,
                    reinterpret_cast<const sockaddr*>(&remote_address), sizeof(remote_address)) < 0)
    {
        return errno;
    }

    return 0;
}

UdpReceiveResult UdpSocket::receiveFrom(std::span<std::byte> buffer, std::uint32_t timeoutMs) noexcept
{
    UdpReceiveResult result;

    if (fd_ < 0)
    {
        result.error = EBADF;
        return result;
    }

    int flags = 0;
    if (timeoutMs == 0U)
    {
        /* 立即尝试：无数据时由 lwIP 返回 EWOULDBLOCK。 */
        flags = MSG_DONTWAIT;
    }
    else if (timeoutMs != WaitForever)
    {
        fd_set read_set;
        FD_ZERO(&read_set);
        FD_SET(fd_, &read_set);

        timeval timeout;
        timeout.tv_sec  = static_cast<long>(timeoutMs / 1000U);
        timeout.tv_usec = static_cast<long>((timeoutMs % 1000U) * 1000U);

        const int ready = lwip_select(fd_ + 1, &read_set, nullptr, nullptr, &timeout);
        if (ready < 0)
        {
            result.error = errno;
            return result;
        }
        if (ready == 0)
        {
            result.error = ETIMEDOUT;
            return result;
        }
        flags = MSG_DONTWAIT;
    }

    /* lwip_recvmsg 不接受空 iovec，用调用内的字节接收并丢弃空 buffer 的数据。 */
    std::byte discard;
    void* base = buffer.empty() ? static_cast<void*>(&discard)
                               : static_cast<void*>(buffer.data());
    const std::size_t length = buffer.empty() ? 1U : buffer.size();

    iovec vector;
    vector.iov_base = base;
    vector.iov_len  = length;

    sockaddr_in source_address;
    std::memset(&source_address, 0, sizeof(source_address));

    msghdr message;
    std::memset(&message, 0, sizeof(message));
    message.msg_name    = &source_address;
    message.msg_namelen = sizeof(source_address);
    message.msg_iov     = &vector;
    message.msg_iovlen  = 1;

    const ssize_t received = lwip_recvmsg(fd_, &message, flags);
    if (received < 0)
    {
        result.error = errno;
        return result;
    }

    /* lwIP 返回完整数据报长度，可能大于实际写入的字节数。 */
    const std::size_t datagram_size = static_cast<std::size_t>(received);

    sockaddrToEndpoint(source_address, result.source);
    result.size      = (datagram_size < buffer.size()) ? datagram_size : buffer.size();
    result.truncated = (datagram_size > buffer.size()) || ((message.msg_flags & MSG_TRUNC) != 0);
    return result;
}

void UdpSocket::close() noexcept
{
    if (fd_ >= 0)
    {
        (void)lwip_close(fd_);
        fd_ = -1;
    }
}

bool UdpSocket::isOpen() const noexcept
{
    return fd_ >= 0;
}

} // namespace net

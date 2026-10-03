/**
 * @file    dma_buffer.hpp
 * @brief   C++17 固定容量、32 字节对齐的 DMA 字节存储。
 */
#pragma once
#include <cstddef>
#include <cstdint>

namespace memory
{

/**
 * @brief 内嵌 N 字节存储，不分配堆内存，不允许复制或移动。
 * @tparam N 可用字节数，必须大于零。
 *
 * 使用约定：
 * - 通过引用传给驱动；存储必须保持有效，直到 DMA 及相关回调不再访问它。
 * - data() 指向对象内部，size() 返回可用容量，不包含对齐填充。
 * - sizeof(DMABuffer<N>) 可能大于 N；链接器按包含填充的实际大小预留空间。
 * - 内存区域由实例声明和链接脚本决定；32 字节对齐不保证 DMA 可达或不可缓存。
 * - 本类型不配置 MPU/Cache、不校验地址，也不提供 CPU/DMA 同步。
 * - 默认零初始化；若放入自定义 section，项目必须提供对应的启动初始化。
 *   NOLOAD 不会自动清零，不能仅依靠成员初始化器假定上电后存储已经为零。
 */
template <std::size_t N, std::size_t Alignment = 32> class alignas(Alignment) DMABuffer
{
    static_assert(N > 0, "DMABuffer<N> requires a positive byte capacity");

public:
    constexpr DMABuffer() noexcept          = default;
    DMABuffer(const DMABuffer&)             = delete;
    DMABuffer& operator=(const DMABuffer&)  = delete;
    DMABuffer(DMABuffer&&)                  = delete;
    DMABuffer& operator=(DMABuffer&&)       = delete;

    [[nodiscard]] constexpr std::uint8_t* data() noexcept { return storage_; }
    [[nodiscard]] constexpr const std::uint8_t* data() const noexcept { return storage_; }
    [[nodiscard]] static constexpr std::size_t size() noexcept { return N; }

private:
    std::uint8_t storage_[N]{};
};

} // namespace memory

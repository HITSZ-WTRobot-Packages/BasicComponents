/**
 * @file    static_arena.hpp
 * @brief   原子化静态区线性分配器。
 *
 * 适合嵌入式场景下的短生命周期对象分配，不支持释放，只支持线性增长和整体清空。
 * 如果系统里只有少量对象需要动态创建，但又不想引入通用堆分配器，这种线性分配器会更容易控制时序和占用。
 */
#pragma once
#include <atomic>
#include <cstddef>
#include <new>
#include <utility>

template <size_t Size> class StaticArena
{
private:
    alignas(std::max_align_t) std::byte buffer_[Size]{};
    std::atomic<size_t> offset_{ 0 };

public:
    // 禁止拷贝，避免多个实例误共享同一块缓冲区。
    StaticArena(const StaticArena&)            = delete;
    StaticArena& operator=(const StaticArena&) = delete;
    StaticArena()                              = default;

    /**
     * @brief 原子化基础分配接口。
     *
     * 通过 CAS 确保并发任务下只有一个分配者能成功推进 offset_。
     * 这里没有 free 接口，因为分配器的目标就是“只增不减”，从而让实现尽量简单可靠。
     */
    void* allocate(const size_t size, const size_t alignment = alignof(std::max_align_t))
    {
        size_t expected = offset_.load(std::memory_order_relaxed);
        size_t aligned_offset;

        while (true)
        {
            // 先把当前位置按目标对齐向上取整。
            aligned_offset       = (expected + alignment - 1) & ~(alignment - 1);
            const size_t desired = aligned_offset + size;

            if (desired > Size)
            {
                return nullptr; // 容量耗尽
            }

            // 只有当 offset_ 仍等于 expected 时才推进到 desired。
            if (offset_.compare_exchange_weak(
                        expected, desired, std::memory_order_acq_rel, std::memory_order_relaxed))
            {
                break;
            }
        }

        return &buffer_[aligned_offset];
    }

    /**
     * @brief 线程安全的对象构造接口。
     *
     * 注意：这里只保证“拿到一块唯一内存”是原子的，构造函数本身是否线程安全取决于 T 自己。
     * 如果 T 的构造过程里会访问共享资源，调用方仍然需要自己处理并发关系。
     */
    template <typename T, typename... Args> T* create(Args&&... args)
    {
        void* mem = allocate(sizeof(T), alignof(T));
        if (!mem)
            return nullptr;

        // 注意：原子性只覆盖分配阶段，构造函数本身在已分配内存上执行。
        return new (mem) T(std::forward<Args>(args)...);
    }

    // --- 状态查询 (原子加载) ---

    [[nodiscard]] size_t used() const
    {
        return offset_.load(std::memory_order_relaxed);
    }
    [[nodiscard]] size_t capacity() const
    {
        return Size;
    }
    [[nodiscard]] double usage_ratio() const
    {
        return static_cast<double>(used()) / Size;
    }

    /**
     * @brief 重置分配器。
     *
     * 这会让所有已分配对象在逻辑上失效，只有在系统重新初始化时使用。
     * 你可以把它理解为“整块内存池一起回收”，而不是逐个对象析构。
     */
    void clear()
    {
        offset_.store(0, std::memory_order_release);
    }
};

/**
 * @file    static_arena.hpp
 * @brief   原子化嵌入式静态区线性分配器 (C++11/17/20 兼容)
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
    // 禁止拷贝，确保单例安全性
    StaticArena(const StaticArena&)            = delete;
    StaticArena& operator=(const StaticArena&) = delete;
    StaticArena()                              = default;

    /**
     * @brief 原子化基础分配接口
     * 利用 CAS (Compare-And-Swap) 确保多任务下 offset_ 更新的唯一性
     */
    void* allocate(const size_t size, const size_t alignment = alignof(std::max_align_t))
    {
        size_t expected = offset_.load(std::memory_order_relaxed);
        size_t aligned_offset;

        while (true)
        {
            // 计算对齐后的偏移量 (确保 alignment 是 2 的幂)
            aligned_offset       = (expected + alignment - 1) & ~(alignment - 1);
            const size_t desired = aligned_offset + size;

            if (desired > Size)
            {
                return nullptr; // 内存溢出
            }

            /* 尝试更新 offset_：
             * 如果当前 offset_ 仍等于 expected，则更新为 desired 并返回真；
             * 如果被其他任务抢先修改，则将最新的 offset_ 加载到 expected 并返回假，继续循环。
             */
            if (offset_.compare_exchange_weak(
                        expected, desired, std::memory_order_acq_rel, std::memory_order_relaxed))
            {
                break;
            }
        }

        return &buffer_[aligned_offset];
    }

    /**
     * @brief 线程安全的对象构造接口
     */
    template <typename T, typename... Args> T* create(Args&&... args)
    {
        void* mem = allocate(sizeof(T), alignof(T));
        if (!mem)
            return nullptr;

        // 注意：此处仅分配过程是原子的，构造函数本身在分配后的内存上执行
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
     * @brief 重置分配器
     * 警告：该操作会瞬间使所有已分配内存逻辑失效，调用前须确保无任务正在使用
     */
    void clear()
    {
        offset_.store(0, std::memory_order_release);
    }
};
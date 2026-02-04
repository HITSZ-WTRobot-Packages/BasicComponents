/**                                                                                   \
 * @file    static_arena.hpp                                                          \
 * @brief   专为嵌入式静态区设计的线性分配器                          \
 */
#pragma once
#include <cstddef>
#include <cstdint>
#include <new>
#include <utility>

template <size_t Size> class StaticArena
{
private:
    alignas(std::max_align_t) std::byte buffer_[Size]{};
    size_t offset_ = 0;

public:
    // 禁止拷贝，确保单例安全性
    StaticArena(const StaticArena&)            = delete;
    StaticArena& operator=(const StaticArena&) = delete;
    StaticArena()                              = default;

    /**
     * @brief 基础分配接口 (类似 malloc)
     */
    void* allocate(const size_t size, const size_t alignment = alignof(std::max_align_t))
    {
        // 确保对齐
        size_t aligned_offset = (offset_ + alignment - 1) & ~(alignment - 1);

        if (aligned_offset + size > Size)
        {
            return nullptr; // 内存溢出
        }

        void* ptr = &buffer_[aligned_offset];
        offset_   = aligned_offset + size;
        return ptr;
    }

    /**
     * @brief 对象构造接口 (类似 placement new)
     */
    template <typename T, typename... Args> T* create(Args&&... args)
    {
        void* mem = allocate(sizeof(T), alignof(T));
        if (!mem)
            return nullptr;
        // 使用 std::construct_at (C++20) 或 placement new
        return new (mem) T(std::forward<Args>(args)...);
    }

    // 调试辅助
    [[nodiscard]] size_t used() const
    {
        return offset_;
    }
    [[nodiscard]] size_t capacity() const
    {
        return Size;
    }
    [[nodiscard]] double usage_ratio() const
    {
        return static_cast<double>(offset_) / Size;
    }

    // 重置（仅在明确知道对象不再需要且无析构需求时使用）
    void clear()
    {
        offset_ = 0;
    }
};
/**
 * @file    no_heap.hpp
 * @brief   禁止在堆上分配对象。
 *
 * 适合那些必须放在静态存储区、栈上或者外部内存池里的对象。
 * 一旦对象不允许 new / new[]，就能强迫调用方显式管理生命周期来源。
 */
#pragma once

#include <cstddef>

namespace traits
{

// 禁止在堆上分配对象。
struct NoHeap
{
    static void* operator new(std::size_t) = delete;
    static void* operator new[](std::size_t) = delete;
};

} // namespace traits

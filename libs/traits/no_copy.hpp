/**
 * @file    no_copy.hpp
 * @brief   禁止对象拷贝。
 *
 * 适合描述拥有独占资源的对象，比如硬件句柄、锁、缓存区管理器等。
 * 当对象被复制会导致“双重释放”或者状态混乱时，就应该用这种 trait 限制拷贝行为。
 */
#pragma once

namespace traits
{

// 禁止对象拷贝，通常用于管理独占资源的类型。
struct NoCopy
{
    NoCopy() = default;
    NoCopy(const NoCopy&) = delete;
    NoCopy& operator=(const NoCopy&) = delete;
};

} // namespace traits

/**
 * @file    no_copy_move.hpp
 * @brief   禁止对象拷贝和移动。
 *
 * 比 NoCopy 更严格：对象既不能复制，也不能转移所有权。
 * 常见于地址稳定性很重要的资源对象，例如被中断回调持有的实例。
 */
#pragma once

namespace traits
{

// 禁止拷贝和移动，确保对象地址和所有权都保持稳定。
struct NoCopyMove
{
    NoCopyMove() = default;
    NoCopyMove(const NoCopyMove&) = delete;
    NoCopyMove& operator=(const NoCopyMove&) = delete;
    NoCopyMove(NoCopyMove&&) = delete;
    NoCopyMove& operator=(NoCopyMove&&) = delete;
};

} // namespace traits

/**
 * @file    no_move.hpp
 * @brief   禁止对象移动。
 *
 * 当对象被外部引用、地址被缓存，或者移动后会破坏系统内某些指针关联时，就应该禁用移动语义。
 */
#pragma once

namespace traits
{

// 禁止移动语义，保留对象原始地址。
struct NoMove
{
    NoMove() = default;
    NoMove(NoMove&&) = delete;
    NoMove& operator=(NoMove&&) = delete;
};

} // namespace traits

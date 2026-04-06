/**
 * @file    static_only.hpp
 * @brief   禁止实例化，仅用于静态工具类约束。
 *
 * 这个 trait 用来表达“只允许静态成员函数和静态数据”的意图，避免有人误把工具类当对象来创建。
 */
#pragma once

namespace traits
{

/**
 * @brief 仅允许静态使用，不允许实例化。
 *
 * 一般用于纯工具类或静态配置类，避免外部误建对象。
 */
struct StaticOnly
{
    StaticOnly() = delete;
};

} // namespace traits

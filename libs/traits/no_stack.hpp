/**
 * @file    no_stack.hpp
 * @brief   禁止在栈上直接构造对象。
 *
 * 常见于必须通过工厂函数、内存池或者注册表创建的对象。
 * 这样可以强制调用方走统一创建路径，避免局部自动变量生命周期过短。
 */
#pragma once

namespace traits
{

// 禁止在栈上直接构造，通常用于需要强制动态分配或工厂创建的类型。
struct NoStack
{
protected:
    NoStack() = default;
    ~NoStack() = default;
};

} // namespace traits

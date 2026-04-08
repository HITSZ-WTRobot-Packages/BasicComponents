/**
 * @file    interface_only.hpp
 * @brief   仅允许作为接口基类使用，禁止外部直接构造和析构。
 *
 * 这个类型通常用于纯虚接口的基类。构造和析构都被保护起来以后，外部就无法直接实例化它，
 * 只能把它当作父类来继承。
 */
#pragma once

namespace traits
{

// 仅允许作为接口基类使用，禁止直接在外部构造或析构。
struct InterfaceOnly
{
protected:
    InterfaceOnly() = default;
    ~InterfaceOnly() = default;
};

} // namespace traits

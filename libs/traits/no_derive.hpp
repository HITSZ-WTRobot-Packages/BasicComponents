/**
 * @file    no_derive.hpp
 * @brief   禁止从该类型派生。
 *
 * 适合需要固定对象布局或者不希望外部扩展行为的类型。
 * 直接把类型标记成 final，比靠文档提醒“不要继承”更可靠。
 */
#pragma once

namespace traits
{

// final 标记，禁止继承。
struct NoDerive final
{
};

} // namespace traits

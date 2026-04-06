/**
 * @file    no_delete.hpp
 * @brief   禁止通过 delete 释放对象。
 *
 * 这个限制通常和受控生命周期一起使用：对象可以创建，但销毁必须由特定接口或者框架接管。
 */
#pragma once

namespace traits
{

// 禁止外部通过 delete 销毁，通常配合受控生命周期使用。
struct NoDelete
{
protected:
    ~NoDelete() = default;
};

} // namespace traits

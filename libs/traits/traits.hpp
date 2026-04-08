/**
 * @file    traits.hpp
 * @brief   统一导出对象语义约束 trait。
 *
 * 这个头文件本身不实现复杂逻辑，只负责把一组常见的对象语义限制集中导出。
 * 你可以把它理解成“声明式的约束工具箱”：需要禁止拷贝、禁止堆分配、禁止继承时，
 * 直接包含这个头文件即可。
 */
#pragma once

#include "no_copy.hpp"
#include "no_move.hpp"
#include "no_copy_move.hpp"
#include "no_delete.hpp"
#include "no_heap.hpp"
#include "no_stack.hpp"
#include "no_derive.hpp"
#include "interface_only.hpp"
#include "static_only.hpp"

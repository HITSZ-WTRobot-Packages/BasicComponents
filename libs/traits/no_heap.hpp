#pragma once

#include <cstddef>

namespace traits
{

struct NoHeap
{
    static void* operator new(std::size_t) = delete;
    static void* operator new[](std::size_t) = delete;
};

} // namespace traits

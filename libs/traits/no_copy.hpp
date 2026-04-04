#pragma once

namespace traits
{

struct NoCopy
{
    NoCopy() = default;
    NoCopy(const NoCopy&) = delete;
    NoCopy& operator=(const NoCopy&) = delete;
};

} // namespace traits

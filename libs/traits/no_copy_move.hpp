#pragma once

namespace traits
{

struct NoCopyMove
{
    NoCopyMove() = default;
    NoCopyMove(const NoCopyMove&) = delete;
    NoCopyMove& operator=(const NoCopyMove&) = delete;
    NoCopyMove(NoCopyMove&&) = delete;
    NoCopyMove& operator=(NoCopyMove&&) = delete;
};

} // namespace traits

#pragma once

namespace traits
{

struct NoMove
{
    NoMove() = default;
    NoMove(NoMove&&) = delete;
    NoMove& operator=(NoMove&&) = delete;
};

} // namespace traits

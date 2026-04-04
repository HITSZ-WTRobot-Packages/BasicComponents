#pragma once

namespace traits
{

struct NoDelete
{
protected:
    ~NoDelete() = default;
};

} // namespace traits

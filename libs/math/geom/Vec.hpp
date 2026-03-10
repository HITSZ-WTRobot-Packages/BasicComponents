/**
 * @file    Vec.hpp
 * @author  syhanjin
 * @date    2026-03-09
 */
#pragma once
#include <cmath>
#include <array>
#include <cstddef>
#include <type_traits>

namespace math
{

// 通用向量模板
template <typename T, size_t N> struct Vec
{
    T data[N]{};

    // element access
    constexpr T&       operator[](size_t i) { return data[i]; }
    constexpr const T& operator[](size_t i) const { return data[i]; }

    // zero vector
    static constexpr Vec zero()
    {
        Vec v{};
        for (size_t i = 0; i < N; ++i)
            v[i] = T(0);
        return v;
    }

    // addition
    constexpr Vec operator+(const Vec& other) const
    {
        Vec res{};
        for (size_t i = 0; i < N; ++i)
            res[i] = data[i] + other[i];
        return res;
    }

    // subtraction
    constexpr Vec operator-(const Vec& other) const
    {
        Vec res{};
        for (size_t i = 0; i < N; ++i)
            res[i] = data[i] - other[i];
        return res;
    }

    // scalar multiply
    constexpr Vec operator*(T s) const
    {
        Vec res{};
        for (size_t i = 0; i < N; ++i)
            res[i] = data[i] * s;
        return res;
    }

    // scalar divide
    constexpr Vec operator/(T s) const
    {
        Vec res{};
        for (size_t i = 0; i < N; ++i)
            res[i] = data[i] / s;
        return res;
    }

    // dot product
    constexpr T dot(const Vec& other) const
    {
        T res = T(0);
        for (size_t i = 0; i < N; ++i)
            res += data[i] * other[i];
        return res;
    }

    // norm (requires C++20 for constexpr sqrt)
    constexpr T norm() const { return std::sqrt(dot(*this)); }

    constexpr Vec normalize() const
    {
        T n = norm();
        return (*this) / n;
    }
};

// Vec3 specialization for cross product
template <typename T> struct Vec<T, 3>
{
    T x{}, y{}, z{};

    constexpr Vec() : x(0), y(0), z(0) {}
    constexpr Vec(T x_, T y_, T z_) : x(x_), y(y_), z(z_) {}

    constexpr T&       operator[](size_t i) { return i == 0 ? x : (i == 1 ? y : z); }
    constexpr const T& operator[](size_t i) const { return i == 0 ? x : (i == 1 ? y : z); }

    // arithmetic
    constexpr Vec operator+(const Vec& o) const { return Vec(x + o.x, y + o.y, z + o.z); }
    constexpr Vec operator-(const Vec& o) const { return Vec(x - o.x, y - o.y, z - o.z); }
    constexpr Vec operator*(T s) const { return Vec(x * s, y * s, z * s); }
    constexpr Vec operator/(T s) const { return Vec(x / s, y / s, z / s); }

    // dot & cross
    constexpr T   dot(const Vec& o) const { return x * o.x + y * o.y + z * o.z; }
    constexpr Vec cross(const Vec& o) const
    {
        return Vec(y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x);
    }

    constexpr T   norm() const { return std::sqrt(dot(*this)); }
    constexpr Vec normalize() const { return (*this) / norm(); }

    static constexpr Vec zero() { return Vec(0, 0, 0); }
};

using Vec3f = Vec<float, 3>;
using Vec3d = Vec<double, 3>;

} // namespace math
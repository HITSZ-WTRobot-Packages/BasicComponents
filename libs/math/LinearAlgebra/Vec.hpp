/**
 * @file    Vec.hpp
 * @author  syhanjin
 * @date    2026-03-09
 * @brief   轻量向量模板。
 *
 * 这个文件里的向量类型主要面向嵌入式和小规模数值计算场景：维度固定、接口简单、没有动态分配。
 * 你可以把它当作运动学、姿态、控制量等小向量的基础表达类型。
 */
#pragma once
#include <cmath>
#include <array>
#include <cstddef>
#include <type_traits>

namespace math
{

// 通用向量模板：适用于任意维度。
template <typename T, size_t N> struct Vec
{
    T data[N]{};

    // element access
    constexpr T&       operator[](size_t i) { return data[i]; }
    constexpr const T& operator[](size_t i) const { return data[i]; }

    // 零向量。
    static constexpr Vec zero()
    {
        Vec v{};
        for (size_t i = 0; i < N; ++i)
            v[i] = T(0);
        return v;
    }

    // 向量加法 / 原地加法。
    constexpr Vec operator+(const Vec& other) const
    {
        Vec res{};
        for (size_t i = 0; i < N; ++i)
            res[i] = data[i] + other[i];
        return res;
    }
    constexpr Vec& operator+=(const Vec& other)
    {
        for (size_t i = 0; i < N; ++i)
            data[i] += other[i];
        return *this;
    }

    // 向量减法 / 原地减法。
    constexpr Vec operator-(const Vec& other) const
    {
        Vec res{};
        for (size_t i = 0; i < N; ++i)
            res[i] = data[i] - other[i];
        return res;
    }
    constexpr Vec& operator-=(const Vec& other)
    {
        for (size_t i = 0; i < N; ++i)
            data[i] -= other[i];
        return *this;
    }

    // 标量乘法 / 原地乘法。
    constexpr Vec operator*(T s) const
    {
        Vec res{};
        for (size_t i = 0; i < N; ++i)
            res[i] = data[i] * s;
        return res;
    }
    constexpr Vec& operator*=(T s) const
    {
        for (size_t i = 0; i < N; ++i)
            data[i] *= s;
        return *this;
    }

    // 标量除法 / 原地除法。
    constexpr Vec operator/(T s) const
    {
        Vec res{};
        for (size_t i = 0; i < N; ++i)
            res[i] = data[i] / s;
        return res;
    }
    constexpr Vec& operator/=(T s) const
    {
        for (size_t i = 0; i < N; ++i)
            data[i] /= s;
        return *this;
    }

    // 点积、范数和归一化。
    constexpr T dot(const Vec& other) const
    {
        T res = T(0);
        for (size_t i = 0; i < N; ++i)
            res += data[i] * other[i];
        return res;
    }

    constexpr T norm() const { return std::sqrt(dot(*this)); }

    constexpr Vec normalize() const
    {
        T n = norm();
        return (*this) / n;
    }
};

// Vec3 specialization for cross product。
template <typename T> struct Vec<T, 3>
{
    T x{}, y{}, z{};

    constexpr Vec() : x(0), y(0), z(0) {}
    constexpr Vec(T x_, T y_, T z_) : x(x_), y(y_), z(z_) {}

    constexpr T&       operator[](size_t i) { return i == 0 ? x : (i == 1 ? y : z); }
    constexpr const T& operator[](size_t i) const { return i == 0 ? x : (i == 1 ? y : z); }

    // 三维加减乘除。
    constexpr Vec operator+(const Vec& o) const { return Vec(x + o.x, y + o.y, z + o.z); }
    constexpr Vec operator-(const Vec& o) const { return Vec(x - o.x, y - o.y, z - o.z); }
    constexpr Vec operator*(T s) const { return Vec(x * s, y * s, z * s); }
    constexpr Vec operator/(T s) const { return Vec(x / s, y / s, z / s); }

    constexpr Vec& operator+=(const Vec& o)
    {
        x += o.x, y += o.y, z += o.z;
        return *this;
    }
    constexpr Vec& operator-=(const Vec& o)
    {
        x -= o.x, y -= o.y, z -= o.z;
        return *this;
    }
    constexpr Vec& operator*=(T s) const
    {
        x *= s, y *= s, z *= s;
        return *this;
    }
    constexpr Vec& operator/=(T s) const
    {
        x /= s, y /= s, z /= s;
        return *this;
    }

    // 点积 / 叉积。
    constexpr T   dot(const Vec& o) const { return x * o.x + y * o.y + z * o.z; }
    constexpr Vec cross(const Vec& o) const
    {
        return Vec(y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x);
    }

    constexpr T   norm() const { return std::sqrt(dot(*this)); }
    constexpr Vec normalize() const { return (*this) / norm(); }

    static constexpr Vec zero() { return Vec(0, 0, 0); }
};

// Vec2 specialization for planar kinematics。
template <typename T> struct Vec<T, 2>
{
    T x{}, y{};

    constexpr Vec() : x(0), y(0) {}
    constexpr Vec(T x_, T y_) : x(x_), y(y_) {}

    // 元素访问。
    constexpr T&       operator[](size_t i) { return i == 0 ? x : y; }
    constexpr const T& operator[](size_t i) const { return i == 0 ? x : y; }

    // 二维加减乘除。
    constexpr Vec operator+(const Vec& o) const { return Vec(x + o.x, y + o.y); }
    constexpr Vec operator-(const Vec& o) const { return Vec(x - o.x, y - o.y); }
    constexpr Vec operator*(T s) const { return Vec(x * s, y * s); }
    constexpr Vec operator/(T s) const { return Vec(x / s, y / s); }

    constexpr Vec& operator+=(const Vec& o)
    {
        x += o.x, y += o.y;
        return *this;
    }
    constexpr Vec& operator-=(const Vec& o)
    {
        x -= o.x, y -= o.y;
        return *this;
    }
    constexpr Vec& operator*=(T s) const
    {
        x *= s, y *= s;
        return *this;
    }
    constexpr Vec& operator/=(T s) const
    {
        x /= s, y /= s;
        return *this;
    }

    // 点积。
    constexpr T dot(const Vec& o) const { return x * o.x + y * o.y; }

    // 欧式范数。
    constexpr T norm() const { return std::hypot(x, y); }

    constexpr Vec normalize() const { return (*this) / norm(); }

    // 垂直向量（逆时针 90 度），常用于平面内 omega × r。
    constexpr Vec perp() const { return Vec(-y, x); }

    // 按 yaw 旋转，单位为弧度。
    constexpr Vec rotate(T yaw) const
    {
        T c = std::cos(yaw);
        T s = std::sin(yaw);
        return Vec(c * x - s * y, s * x + c * y);
    }

    static constexpr Vec zero() { return Vec(0, 0); }
};

using Vec3f = Vec<float, 3>;
using Vec3d = Vec<double, 3>;

using Vec2f = Vec<float, 2>;
using Vec2d = Vec<double, 2>;

template <size_t N> using Vecf = Vec<float, N>;
template <size_t N> using Vecd = Vec<double, N>;

} // namespace math

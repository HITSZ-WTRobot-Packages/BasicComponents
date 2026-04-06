/**
 * @file    Quaternion.hpp
 * @author  syhanjin
 * @date    2026-03-09
 * @brief   四元数姿态表达。
 *
 * 用于三维旋转的紧凑表示，避免欧拉角的万向节锁问题。
 */
#pragma once
#include "../LinearAlgebra/Vec.hpp"
#include <cmath>
#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif

namespace math
{
template <typename T, typename AngleUnit> struct EulerZYX;

template <typename T> struct Quaternion
{
    T w, x, y, z;

    // 默认单位四元数。
    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(T w_, T x_, T y_, T z_) : w(w_), x(x_), y(y_), z(z_) { normalize(); }

    // 保持单位长度，避免累计数值误差。
    inline void normalize()
    {
        T n = std::sqrt(w * w + x * x + y * y + z * z);
        w /= n;
        x /= n;
        y /= n;
        z /= n;
    }

    // 共轭等价于旋转逆方向。
    inline Quaternion conjugate() const { return Quaternion(w, -x, -y, -z); }

    inline Quaternion inverse() const
    {
        T n2 = w * w + x * x + y * y + z * z;
        return conjugate() * (1 / n2);
    }

    // 四元数乘法，对应旋转复合。
    inline Quaternion operator*(const Quaternion& q) const
    {
        return Quaternion(w * q.w - x * q.x - y * q.y - z * q.z,
                          w * q.x + x * q.w + y * q.z - z * q.y,
                          w * q.y - x * q.z + y * q.w + z * q.x,
                          w * q.z + x * q.y - y * q.x + z * q.w);
    }

    inline Quaternion operator*(T s) const { return Quaternion(w * s, x * s, y * s, z * s); }

    // 用四元数旋转三维向量。
    inline Vec<T, 3> rotateVector(const Vec<T, 3>& v) const
    {
        Vec<T, 3> qv(x, y, z);
        Vec<T, 3> t = qv.cross(v) * T(2);
        return v + t * w + qv.cross(t);
    }

    template <typename AngleUnit> explicit Quaternion(const EulerZYX<T, AngleUnit>& _e)
    {
        // 输入可以是 deg 或 rad，先统一到弧度。
        EulerZYX<T> e;
        if constexpr (std::is_same_v<AngleUnit, unit::Deg>)
            e = _e.toRad();
        else
            e = _e;

        // ZYX 顺序：yaw -> pitch -> roll。
        const T cr = cos(e.roll * T(0.5));
        const T sr = sin(e.roll * T(0.5));
        const T cp = cos(e.pitch * T(0.5));
        const T sp = sin(e.pitch * T(0.5));
        const T cy = cos(e.yaw * T(0.5));
        const T sy = sin(e.yaw * T(0.5));

        w = cr * cp * cy + sr * sp * sy;
        x = sr * cp * cy - cr * sp * sy;
        y = cr * sp * cy + sr * cp * sy;
        z = cr * cp * sy - sr * sp * cy;
    }
};

using Quatf = Quaternion<float>;
using Quatd = Quaternion<double>;

} // namespace math

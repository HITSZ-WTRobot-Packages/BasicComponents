/**
 * @file    Quaternion.hpp
 * @author  syhanjin
 * @date    2026-03-09
 */
#pragma once
#include "Vec.hpp"
#include <cmath>
#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif

namespace math
{
template <typename T> struct EulerZYX;

template <typename T> struct Quaternion
{
    T w, x, y, z;

    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(T w_, T x_, T y_, T z_) : w(w_), x(x_), y(y_), z(z_) { normalize(); }

    inline void normalize()
    {
        T n = std::sqrt(w * w + x * x + y * y + z * z);
        w /= n;
        x /= n;
        y /= n;
        z /= n;
    }

    inline Quaternion conjugate() const { return Quaternion(w, -x, -y, -z); }

    inline Quaternion inverse() const
    {
        T n2 = w * w + x * x + y * y + z * z;
        return conjugate() * (1 / n2);
    }

    // quaternion multiplication
    inline Quaternion operator*(const Quaternion& q) const
    {
        return Quaternion(w * q.w - x * q.x - y * q.y - z * q.z,
                          w * q.x + x * q.w + y * q.z - z * q.y,
                          w * q.y - x * q.z + y * q.w + z * q.x,
                          w * q.z + x * q.y - y * q.x + z * q.w);
    }

    inline Quaternion operator*(T s) const { return Quaternion(w * s, x * s, y * s, z * s); }

    // rotate a Vec3
    inline Vec<T, 3> rotateVector(const Vec<T, 3>& v) const
    {
        Vec<T, 3> qv(x, y, z);
        Vec<T, 3> t = qv.cross(v) * T(2);
        return v + t * w + qv.cross(t);
    }

    inline EulerZYX<T> toEulerZYX()
    {
        EulerZYX<T> e;
        // pitch (Y)
        const T sinp = T(2) * (w * y - z * x);
        if (std::abs(sinp) >= T(1))
        {
            e.pitch = std::copysign(T(M_PI) / T(2), sinp); // gimbal lock
        }
        else
        {
            e.pitch = std::asin(sinp);
        }

        // roll (X)
        e.roll = std::atan2(T(2) * (w * x + y * z), T(1) - T(2) * (x * x + y * y));

        // yaw (Z)
        e.yaw = std::atan2(T(2) * (w * z + x * y), T(1) - T(2) * (y * y + z * z));

        return e;
    }

    explicit Quaternion(const EulerZYX<T>& e)
    {
        // roll-pitch-yaw (ZYX)
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
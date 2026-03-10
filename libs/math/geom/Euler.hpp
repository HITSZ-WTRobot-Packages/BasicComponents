/**
 * @file    Euler.hpp
 * @author  syhanjin
 * @date    2026-03-09
 */
#pragma once
#include <cmath>
#include <type_traits>

#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif

// forward declaration only (avoid circular dependency)
namespace math
{
template <typename T> struct Quaternion;
}

namespace math
{

namespace unit
{

struct Rad;
struct Deg;

template <typename T> constexpr T deg2rad(T deg)
{
    return deg * T(M_PI) / T(180);
}

template <typename T> constexpr T rad2deg(T rad)
{
    return rad * T(180) / T(M_PI);
}

} // namespace unit

/**
 * @brief Euler angles in ZYX order (yaw-pitch-roll)
 *
 * convention:
 * - right-handed coordinate system
 * - rotation order: Z (yaw) -> Y (pitch) -> X (roll)
 * - internal computation unit: rad
 *
 * note:
 * - this type is for configuration / IO / debug
 * - do NOT use it as internal state for computation
 */
template <typename T, typename AngleUnit = unit::Rad> struct EulerZYX
{
    T roll;  // rotation around X
    T pitch; // rotation around Y
    T yaw;   // rotation around Z

    // default constructor (zero rotation)
    constexpr EulerZYX() : roll(T(0)), pitch(T(0)), yaw(T(0)) {}

    // direct value constructor (assumes unit == AngleUnit)
    constexpr EulerZYX(T r, T p, T y) : roll(r), pitch(p), yaw(y) {}

    /**
     * @brief explicit conversion from quaternion
     *
     * warning:
     * - this conversion may suffer from gimbal lock
     * - provided mainly for debug / visualization
     * - prefer quaternion as internal state
     */
    explicit EulerZYX(const Quaternion<T>& q)
    {
        // --- compute in rad ---
        const T sinp = T(2) * (q.w * q.y - q.z * q.x);

        T r_roll;
        T r_pitch;
        T r_yaw;

        // pitch (Y)
        if (std::abs(sinp) >= T(1))
        {
            r_pitch = std::copysign(T(M_PI) / T(2), sinp); // gimbal lock
        }
        else
        {
            r_pitch = std::asin(sinp);
        }

        // roll (X)
        r_roll = std::atan2(T(2) * (q.w * q.x + q.y * q.z), T(1) - T(2) * (q.x * q.x + q.y * q.y));

        // yaw (Z)
        r_yaw = std::atan2(T(2) * (q.w * q.z + q.x * q.y), T(1) - T(2) * (q.y * q.y + q.z * q.z));

        // --- unit conversion at boundary ---
        if constexpr (std::is_same_v<AngleUnit, unit::Deg>)
        {
            roll  = unit::rad2deg(r_roll);
            pitch = unit::rad2deg(r_pitch);
            yaw   = unit::rad2deg(r_yaw);
        }
        else
        {
            roll  = r_roll;
            pitch = r_pitch;
            yaw   = r_yaw;
        }
    }
    template <typename U = AngleUnit, typename = std::enable_if_t<std::is_same_v<U, unit::Deg>>>
    [[nodiscard]] constexpr EulerZYX<T, unit::Rad> toRad() const
    {
        return { unit::deg2rad(roll), unit::deg2rad(pitch), unit::deg2rad(yaw) };
    }

    template <typename U = AngleUnit, typename = std::enable_if_t<std::is_same_v<U, unit::Rad>>>
    [[nodiscard]] constexpr EulerZYX<T, unit::Deg> toDeg() const
    {
        return { unit::rad2deg(roll), unit::rad2deg(pitch), unit::rad2deg(yaw) };
    }
};

// ---- recommended aliases ----

// rad (default / math interface)
using EulerRadf = EulerZYX<float, unit::Rad>;
using EulerRadd = EulerZYX<double, unit::Rad>;

// deg (debug / UI / log)
using EulerDegf = EulerZYX<float, unit::Deg>;
using EulerDegd = EulerZYX<double, unit::Deg>;

// backward compatibility
using Eulerf = EulerRadf;
using Eulerd = EulerRadd;

} // namespace math
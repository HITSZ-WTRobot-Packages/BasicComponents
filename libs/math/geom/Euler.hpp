/**
 * @file    Euler.hpp
 * @author  syhanjin
 * @date    2026-03-09
 */
#pragma once
#include <cmath>
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

/**
 * @brief Euler angles in ZYX order (yaw-pitch-roll)
 *
 * convention:
 * - right-handed coordinate system
 * - rotation order: Z (yaw) -> Y (pitch) -> X (roll)
 * - unit: rad
 *
 * note:
 * - this type is for configuration / IO / debug
 * - do NOT use it as internal state for computation
 */
template <typename T> struct EulerZYX
{
    T roll;  // rotation around X
    T pitch; // rotation around Y
    T yaw;   // rotation around Z

    // default constructor (zero rotation)
    constexpr EulerZYX() : roll(T(0)), pitch(T(0)), yaw(T(0)) {}

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
        // ZYX convention
        const T sinp = T(2) * (q.w * q.y - q.z * q.x);

        // pitch (Y)
        if (std::abs(sinp) >= T(1))
        {
            pitch = std::copysign(T(M_PI) / T(2), sinp); // gimbal lock
        }
        else
        {
            pitch = std::asin(sinp);
        }

        // roll (X)
        roll = std::atan2(T(2) * (q.w * q.x + q.y * q.z), T(1) - T(2) * (q.x * q.x + q.y * q.y));

        // yaw (Z)
        yaw = std::atan2(T(2) * (q.w * q.z + q.x * q.y), T(1) - T(2) * (q.y * q.y + q.z * q.z));
    }
};

using Eulerf = EulerZYX<float>;
using Eulerd = EulerZYX<double>;

} // namespace math
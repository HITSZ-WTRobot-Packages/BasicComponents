/**
 * @file    Pose.hpp
 * @author  syhanjin
 * @date    2026-03-10
 * @brief   三维刚体位姿表达。
 *
 * position 表示平移，rotation 使用四元数，便于组合和求逆。
 */
#pragma once

#include "Euler.hpp"
#include "Vec.hpp"
#include "Quaternion.hpp"

namespace math
{

/**
 * @brief Rigid body pose (SE3)
 *
 * position : translation
 * rotation : quaternion
 */
template <typename T> struct Pose
{
    using Vec3 = Vec<T, 3>;

    Vec3          pos; // translation
    Quaternion<T> rot; // rotation

    // 默认位姿为原点 + 单位旋转。
    constexpr Pose() : pos(T(0), T(0), T(0)), rot() {}

    constexpr Pose(const Vec3& p, const Quaternion<T>& q) : pos(p), rot(q) {}

    constexpr Pose(const Vec3& p, const EulerZYX<T, unit::Rad>& e) : pos(p), rot(e) {}
    constexpr Pose(const Vec3& p, const EulerZYX<T, unit::Deg>& e) : pos(p), rot(e) {}

    /* identity */
    static inline Pose Identity() { return Pose(Vec3(T(0), T(0), T(0)), Quaternion<T>()); }

    /* transform a point (with translation) */
    inline Vec3 transformPoint(const Vec3& p) const { return rot.rotate(p) + pos; }

    /* transform a vector (no translation) */
    inline Vec3 transformVector(const Vec3& v) const { return rot.rotate(v); }

    inline Pose inverse() const
    {
        // 逆位姿：先取逆旋转，再把平移反变换回来。
        Quaternion<T> r_inv = rot.conjugate();
        Vec3          p_inv = r_inv.rotate(-pos);
        return Pose(p_inv, r_inv);
    }

    /**
     * @brief pose composition
     *
     * this * other
     *
     * T_ac = T_ab * T_bc
     */
    inline Pose operator*(const Pose& other) const
    {
        // 先旋转再平移，符合刚体变换复合规则。
        Pose r;
        r.rot = rot * other.rot;
        r.pos = rot.rotate(other.pos) + pos;
        return r;
    }
};

using Posef = Pose<float>;
using Posed = Pose<double>;

} // namespace math

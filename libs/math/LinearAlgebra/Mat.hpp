/**
 * @file    Mat.hpp
 * @author  syhanjin
 * @date    2026-03-12
 */
#pragma once
#include <cstddef>
#include <cmath>
#include <type_traits>
#include <cassert>

#include "Vec.hpp"

namespace math
{

// TODO: 分析运算库优化

template <typename T, size_t Rows, size_t Cols> struct Mat
{
    T data[Rows][Cols]{};

    /* ---------------- element access ---------------- */

    constexpr T*       operator[](size_t r) { return data[r]; }
    constexpr const T* operator[](size_t r) const { return data[r]; }

    /* ---------------- factories ---------------- */

    static constexpr Mat zero()
    {
        Mat m{};
        for (size_t i = 0; i < Rows; ++i)
            for (size_t j = 0; j < Cols; ++j)
                m[i][j] = T(0);
        return m;
    }

    static constexpr Mat identity()
    {
        static_assert(Rows == Cols, "identity requires square matrix");
        Mat m = zero();
        for (size_t i = 0; i < Rows; ++i)
            m[i][i] = T(1);
        return m;
    }

    /* ---------------- arithmetic ---------------- */

    constexpr Mat operator+(const Mat& o) const
    {
        Mat r{};
        for (size_t i = 0; i < Rows; ++i)
            for (size_t j = 0; j < Cols; ++j)
                r[i][j] = data[i][j] + o.data[i][j];
        return r;
    }
    constexpr Mat operator+(const T o) const
    {
        Mat r{};
        for (size_t i = 0; i < Rows; ++i)
            for (size_t j = 0; j < Cols; ++j)
                r[i][j] = data[i][j] + o;
        return r;
    }

    constexpr Mat operator-(const Mat& o) const
    {
        Mat r{};
        for (size_t i = 0; i < Rows; ++i)
            for (size_t j = 0; j < Cols; ++j)
                r[i][j] = data[i][j] - o.data[i][j];
        return r;
    }
    constexpr Mat operator-(const T o) const
    {
        Mat r{};
        for (size_t i = 0; i < Rows; ++i)
            for (size_t j = 0; j < Cols; ++j)
                r[i][j] = data[i][j] - o;
        return r;
    }

    constexpr Mat& operator+=(const Mat& o)
    {
        for (size_t i = 0; i < Rows; ++i)
            for (size_t j = 0; j < Cols; ++j)
                data[i][j] += o.data[i][j];
        return *this;
    }
    constexpr Mat& operator+=(const T& o)
    {
        for (size_t i = 0; i < Rows; ++i)
            for (size_t j = 0; j < Cols; ++j)
                data[i][j] += o;
        return *this;
    }

    constexpr Mat& operator-=(const Mat& o)
    {
        for (size_t i = 0; i < Rows; ++i)
            for (size_t j = 0; j < Cols; ++j)
                data[i][j] -= o.data[i][j];
        return *this;
    }
    constexpr Mat& operator-=(const T o)
    {
        for (size_t i = 0; i < Rows; ++i)
            for (size_t j = 0; j < Cols; ++j)
                data[i][j] -= o;
        return *this;
    }

    /* ---------------- scalar ---------------- */

    constexpr Mat operator*(const T s) const
    {
        Mat r{};
        for (size_t i = 0; i < Rows; ++i)
            for (size_t j = 0; j < Cols; ++j)
                r[i][j] = data[i][j] * s;
        return r;
    }

    constexpr Mat operator/(const T s) const
    {
        Mat r{};
        for (size_t i = 0; i < Rows; ++i)
            for (size_t j = 0; j < Cols; ++j)
                r[i][j] = data[i][j] / s;
        return r;
    }

    /* ---------------- matrix multiply ---------------- */

    template <size_t OtherCols>
    constexpr Mat<T, Rows, OtherCols> operator*(const Mat<T, Cols, OtherCols>& o) const
    {
        Mat<T, Rows, OtherCols> r{};
        for (size_t i = 0; i < Rows; ++i)
            for (size_t j = 0; j < OtherCols; ++j)
            {
                r[i][j] = T(0);
                for (size_t k = 0; k < Cols; ++k)
                    r[i][j] += data[i][k] * o.data[k][j];
            }
        return r;
    }

    /* ---------------- matrix * vector ---------------- */

    constexpr Vec<T, Rows> operator*(const Vec<T, Cols>& v) const
    {
        Vec<T, Rows> r{};
        for (size_t i = 0; i < Rows; ++i)
        {
            r[i] = T(0);
            for (size_t j = 0; j < Cols; ++j)
                r[i] += data[i][j] * v[j];
        }
        return r;
    }

    /* ---------------- transpose ---------------- */

    constexpr Mat<T, Cols, Rows> transpose() const
    {
        Mat<T, Cols, Rows> r{};
        for (size_t i = 0; i < Rows; ++i)
            for (size_t j = 0; j < Cols; ++j)
                r[j][i] = data[i][j];
        return r;
    }

    /* ---------------- trace ---------------- */

    constexpr T trace() const
    {
        static_assert(Rows == Cols, "trace requires square matrix");
        T s = T(0);
        for (size_t i = 0; i < Rows; ++i)
            s += data[i][i];
        return s;
    }

    /* ---------------- determinant ---------------- */

    constexpr T det() const
    {
        static_assert(Rows == Cols, "det requires square matrix");

        if constexpr (Rows == 1)
        {
            return data[0][0];
        }
        else if constexpr (Rows == 2)
        {
            return data[0][0] * data[1][1] - data[0][1] * data[1][0];
        }
        else if constexpr (Rows == 3)
        {
            return data[0][0] * (data[1][1] * data[2][2] - data[1][2] * data[2][1]) -
                   data[0][1] * (data[1][0] * data[2][2] - data[1][2] * data[2][0]) +
                   data[0][2] * (data[1][0] * data[2][1] - data[1][1] * data[2][0]);
        }
        else if constexpr (Rows == 4)
        {
            // Laplace expansion, fully unrolled
            const T a0 = data[0][0];
            const T a1 = data[0][1];
            const T a2 = data[0][2];
            const T a3 = data[0][3];

            Mat<T, 3, 3> m0{ { { data[1][1], data[1][2], data[1][3] },
                               { data[2][1], data[2][2], data[2][3] },
                               { data[3][1], data[3][2], data[3][3] } } };
            Mat<T, 3, 3> m1{ { { data[1][0], data[1][2], data[1][3] },
                               { data[2][0], data[2][2], data[2][3] },
                               { data[3][0], data[3][2], data[3][3] } } };
            Mat<T, 3, 3> m2{ { { data[1][0], data[1][1], data[1][3] },
                               { data[2][0], data[2][1], data[2][3] },
                               { data[3][0], data[3][1], data[3][3] } } };
            Mat<T, 3, 3> m3{ { { data[1][0], data[1][1], data[1][2] },
                               { data[2][0], data[2][1], data[2][2] },
                               { data[3][0], data[3][1], data[3][2] } } };

            return a0 * m0.det() - a1 * m1.det() + a2 * m2.det() - a3 * m3.det();
        }
        else
        {
            static_assert(Rows <= 4, "det only specialized up to 4x4");
            return 0;
        }
    }

    /* ---------------- inverse ---------------- */
    constexpr Mat inverse() const
    {
        static_assert(Rows == Cols, "inverse requires square matrix");

        if constexpr (Rows == 1)
        {
            Mat r{};
            r[0][0] = T(1) / data[0][0];
            return r;
        }
        else if constexpr (Rows == 2)
        {
            T   d = det();
            Mat r{};
            r[0][0] = data[1][1] / d;
            r[0][1] = -data[0][1] / d;
            r[1][0] = -data[1][0] / d;
            r[1][1] = data[0][0] / d;
            return r;
        }
        else if constexpr (Rows == 3)
        {
            T   d = det();
            Mat r{};

            r[0][0] = (data[1][1] * data[2][2] - data[1][2] * data[2][1]) / d;
            r[0][1] = (data[0][2] * data[2][1] - data[0][1] * data[2][2]) / d;
            r[0][2] = (data[0][1] * data[1][2] - data[0][2] * data[1][1]) / d;

            r[1][0] = (data[1][2] * data[2][0] - data[1][0] * data[2][2]) / d;
            r[1][1] = (data[0][0] * data[2][2] - data[0][2] * data[2][0]) / d;
            r[1][2] = (data[0][2] * data[1][0] - data[0][0] * data[1][2]) / d;

            r[2][0] = (data[1][0] * data[2][1] - data[1][1] * data[2][0]) / d;
            r[2][1] = (data[0][1] * data[2][0] - data[0][0] * data[2][1]) / d;
            r[2][2] = (data[0][0] * data[1][1] - data[0][1] * data[1][0]) / d;

            return r;
        }
        else if constexpr (Rows == 4)
        {
            // adjugate / det
            T   d = det();
            Mat r{};

            for (size_t i = 0; i < 4; ++i)
                for (size_t j = 0; j < 4; ++j)
                {
                    Mat<T, 3, 3> m{};
                    size_t       r0 = 0;
                    for (size_t ii = 0; ii < 4; ++ii)
                    {
                        if (ii == i)
                            continue;
                        size_t c0 = 0;
                        for (size_t jj = 0; jj < 4; ++jj)
                        {
                            if (jj == j)
                                continue;
                            m[r0][c0++] = data[ii][jj];
                        }
                        ++r0;
                    }
                    T cofactor = (((i + j) & 1) ? -1 : 1) * m.det();
                    r[j][i]    = cofactor / d; // transpose here
                }

            return r;
        }
        else
        {
            return inverse_gj();
        }
    }
    constexpr Mat inverse_gj() const
    {
        static_assert(Rows == Cols, "inverse requires square matrix");

        Mat a   = *this;
        Mat inv = Mat::identity();

        for (size_t i = 0; i < Rows; ++i)
        {
            // pivot
            T piv = a[i][i];
            assert(std::abs(piv) > T(1e-9));

            T inv_piv = T(1) / piv;

            for (size_t j = 0; j < Rows; ++j)
            {
                a[i][j] *= inv_piv;
                inv[i][j] *= inv_piv;
            }

            for (size_t r = 0; r < Rows; ++r)
            {
                if (r == i)
                    continue;
                T f = a[r][i];
                for (size_t c = 0; c < Rows; ++c)
                {
                    a[r][c] -= f * a[i][c];
                    inv[r][c] -= f * inv[i][c];
                }
            }
        }
        return inv;
    }
};

namespace mat
{

template <typename T, size_t Dim> static constexpr Mat<T, Dim, Dim> diag(const Vec<T, Dim>& v)
{
    Mat m = Mat<T, Dim, Dim>::zero();
    for (size_t i = 0; i < Dim; ++i)
        m[i][i] = v[i];
    return m;
}
} // namespace mat

/* ---------------- common aliases ---------------- */

using Mat2f = Mat<float, 2, 2>;
using Mat3f = Mat<float, 3, 3>;
using Mat2d = Mat<double, 2, 2>;
using Mat3d = Mat<double, 3, 3>;

} // namespace math
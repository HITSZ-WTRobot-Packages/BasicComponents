/**
 * @file    EKF.hpp
 * @author  syhanjin
 * @date    2026-03-12
 */
#pragma once
#include "Mat.hpp"

namespace math::ekf
{

// TODO:优化计算性能

/**
 * 扩展卡尔曼滤波器
 * @ref
 * https://zh.wikipedia.org/wiki/%E6%93%B4%E5%B1%95%E5%8D%A1%E7%88%BE%E6%9B%BC%E6%BF%BE%E6%B3%A2%E5%99%A8#%E9%9B%A2%E6%95%A3%E6%99%82%E9%96%93%E9%87%8F%E6%B8%AC
 * @tparam T 数据类型
 * @tparam StateDim 状态维数
 */
template <typename T, size_t StateDim> struct EKF
{
public:
    using MatS = Mat<T, StateDim, StateDim>;
    using VecS = Vec<T, StateDim>;

    VecS state{}; // state
    MatS P{};     // 协方差矩阵
    MatS Q{};     // 过程噪声

    EKF() = default;

    EKF(const VecS& X_init, const MatS& P_init, const MatS& Q_init) :
        state(X_init), P(P_init), Q(Q_init)
    {
    }

    /* --- 预测 --- */
    /**
     * 预测
     * @tparam FFunc 预测函数模板类
     * @param predicted_x 预测步 x 结果
     * @param F Jacobian 矩阵，\partial{f}/\partial{x}
     */
    constexpr void predict(const VecS& predicted_x, const MatS& F)
    {
        state = std::move(predicted_x);
        P     = F * P * F.transpose() + Q;
    }

    /* 观测更新 */
    template <typename FFunc, size_t MeasDim>
    constexpr void update(const Vec<T, MeasDim>&           z,
                          FFunc                            h,
                          const Mat<T, MeasDim, StateDim>& H,
                          const Mat<T, MeasDim, MeasDim>&  R)
    {
        // innovation
        const auto y = z - h(state);

        // innovation covariance S = H^T * P^T * H + R
        const auto S = H * P * H.transpose() + R;

        // Kalman gain K = P * H * S^-1
        const auto K = P * H.transpose() * S.inverse();

        // state update
        state += K * y;

        P -= K * H * P;
    }
};
} // namespace math::ekf
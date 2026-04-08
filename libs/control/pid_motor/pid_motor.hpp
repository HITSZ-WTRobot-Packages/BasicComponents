/**
 * @file    pid_motor.hpp
 * @author  syhanjin
 * @date    2026-01-28
 * @brief   带输出限幅和抗积分饱和的电机 PID 控制器。
 *
 * 这个版本更适合电机速度环和位置环等场景：先计算 P / D 主体，再把剩余输出空间分配给 I 项，
 * 这样可以在保持响应速度的同时，尽量减少积分项把输出“顶满”的问题。
 */
#ifndef PID_MOTOR_HPP
#define PID_MOTOR_HPP
#include <cmath>

class PIDMotor
{
public:
    struct Config
    {
        float Kp;             ///< 比例系数
        float Ki;             ///< 积分系数
        float Kd;             ///< 微分系数
        float abs_output_max; ///< 输出绝对值上限
    };

    PIDMotor() = default;
    explicit PIDMotor(const Config& cfg) : cfg_(cfg) {}

    /**
     * @brief 计算一次 PID 输出。
     *
     * @param ref 目标值
     * @param fdb 反馈值
     * @return 限幅后的控制输出
     */
    float calc(const float& ref, const float& fdb);
    /**
     * @brief 更新控制参数。
     */
    void  setConfig(const Config& cfg) { cfg_ = cfg; }
    /**
     * @brief 直接修改输出上限。
     */
    void  setOutputMax(const float output_max) { cfg_.abs_output_max = std::fabsf(output_max); }
    /**
     * @brief 清空内部状态。
     */
    void  reset();

    [[nodiscard]] float getRef() const { return ref_; }
    [[nodiscard]] float getOutput() const { return output_; }

private:
    Config cfg_{};

    float ref_ = 0.0f; ///< 最近一次目标值
    float fdb_ = 0.0f; ///< 最近一次反馈值

    float error_      = 0.0f; ///< 当前误差
    float prev_error_ = 0.0f; ///< 上一次误差

    float integral_ = 0.0f; ///< 积分状态，已包含 Ki
    float output_   = 0.0f; ///< 最近一次输出
};

#endif // PID_MOTOR_HPP

/**
 * @file    pid_motor.hpp
 * @author  syhanjin
 * @date    2026-01-28
 * @brief   absolute pid with output
 */
#ifndef PID_MOTOR_HPP
#define PID_MOTOR_HPP
#include <cmath>

class PIDMotor
{
public:
    struct Config
    {
        float Kp;             //< 比例系数
        float Ki;             //< 积分系数
        float Kd;             //< 微分系数
        float abs_output_max; //< 输出限幅
    };

    PIDMotor() = default;
    explicit PIDMotor(const Config& cfg) : cfg_(cfg) {}

    float calc(const float& ref, const float& fdb);
    void  setConfig(const Config& cfg) { cfg_ = cfg; }
    void  setOutputMax(const float output_max) { cfg_.abs_output_max = std::fabsf(output_max); }
    void  reset();

    [[nodiscard]] float getRef() const { return ref_; }
    [[nodiscard]] float getOutput() const { return output_; }

private:
    Config cfg_{};

    float ref_ = 0.0f;
    float fdb_ = 0.0f; //< 反馈量

    float error_      = 0.0f; //< current error
    float prev_error_ = 0.0f; //< previous error

    float integral_ = 0.0f; //< integral state
    float output_   = 0.0f; //< output
};

#endif // PID_MOTOR_HPP

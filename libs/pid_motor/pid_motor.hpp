/**
 * @file    pid_motor.hpp
 * @author  syhanjin
 * @date    2026-01-28
 * @brief
 */
#ifndef PID_MOTOR_HPP
#define PID_MOTOR_HPP

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
    void  reset();

    float getRef() const { return ref_; }
    float getOutput() const { return output_; }

private:
    Config cfg_{};

    float fdb_ = 0; //< 反馈量

    float cur_error_   = 0; //< 当前误差
    float prev_error1_ = 0; //< 上一次误差
    float prev_error2_ = 0; //< 上上次误差

    float output_ = 0; //< 输出量
    float ref_    = 0; //< 目标值
};

#endif // PID_MOTOR_HPP

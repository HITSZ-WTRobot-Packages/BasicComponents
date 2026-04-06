/**
 * @file    pid_motor.cpp
 * @author  syhanjin
 * @date    2026-01-28
 * @brief   电机 PID 控制器实现。
 *
 * 这个版本在普通 PID 的基础上加了输出限幅和简单的 anti-windup 处理。
 * 代码逻辑保持得比较直白，目的是让控制初学者能直接对着公式和实现一一对应。
 */
#include "pid_motor.hpp"

#include <algorithm>

float PIDMotor::calc(const float& ref, const float& fdb)
{
    // 保存输入，便于上层调试和状态追踪。
    ref_ = ref;
    fdb_ = fdb;

    // 误差定义为目标减反馈。
    error_ = ref_ - fdb_;

    // 微分项基于误差变化量，抑制响应过冲。
    const float derivative = error_ - prev_error_;

    // 先计算 P+D，并按总输出幅值限幅。
    const float p = cfg_.Kp * error_;
    const float d = cfg_.Kd * derivative;

    const float output_pd = std::clamp(p + d, -cfg_.abs_output_max, cfg_.abs_output_max);

    // 留出给积分项的剩余空间，避免积分项把总输出再次推爆。
    float i_limit = cfg_.abs_output_max - std::fabsf(output_pd);
    if (i_limit < 0.0f)
        i_limit = 0.0f;

    // 积分项直接累计 Ki * error，方便后续做上限控制。
    integral_ += cfg_.Ki * error_;

    // 对积分项做独立限幅，相当于简单的 anti-windup。
    if (integral_ > i_limit)
        integral_ = i_limit;
    else if (integral_ < -i_limit)
        integral_ = -i_limit;

    // 最终输出 = P + D + I。
    output_ = output_pd + integral_;

    prev_error_ = error_;
    return output_;
}

void PIDMotor::reset()
{
    // 清空状态，防止累计积分在参数切换后残留。
    ref_        = 0.0f;
    fdb_        = 0.0f;
    error_      = 0.0f;
    prev_error_ = 0.0f;
    integral_   = 0.0f;
    output_     = 0.0f;
}

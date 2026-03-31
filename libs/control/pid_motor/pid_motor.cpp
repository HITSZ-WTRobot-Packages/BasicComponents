/**
 * @file    pid_motor.cpp
 * @author  syhanjin
 * @date    2026-01-28
 */
#include "pid_motor.hpp"

#include <algorithm>

float PIDMotor::calc(const float& ref, const float& fdb)
{
    ref_ = ref;
    fdb_ = fdb;

    error_ = ref_ - fdb_;

    /* derivative */
    const float derivative = error_ - prev_error_;

    /* P + D */
    const float p = cfg_.Kp * error_;
    const float d = cfg_.Kd * derivative;

    /* PD clamp */
    const float output_pd = std::clamp(p + d, -cfg_.abs_output_max, cfg_.abs_output_max);

    /* remaining margin for integral */
    float i_limit = cfg_.abs_output_max - std::fabsf(output_pd);
    if (i_limit < 0.0f)
        i_limit = 0.0f;

    /* integrate (integral_ already includes Ki) */
    integral_ += cfg_.Ki * error_;

    /* integral clamp */
    if (integral_ > i_limit)
        integral_ = i_limit;
    else if (integral_ < -i_limit)
        integral_ = -i_limit;

    /* final output */
    output_ = output_pd + integral_;

    prev_error_ = error_;
    return output_;
}

void PIDMotor::reset()
{
    ref_        = 0.0f;
    fdb_        = 0.0f;
    error_      = 0.0f;
    prev_error_ = 0.0f;
    integral_   = 0.0f;
    output_     = 0.0f;
}
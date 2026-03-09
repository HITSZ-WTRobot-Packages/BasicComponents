/**
 * @file    pid_motor.cpp
 * @author  syhanjin
 * @date    2026-01-28
 */
#include "pid_motor.hpp"

float PIDMotor::calc(const float& ref, const float& fdb)
{
    cur_error_ = ref - fdb;
    output_ += cfg_.Kp * (cur_error_ - prev_error1_) + cfg_.Ki * cur_error_ +
               cfg_.Kd * (cur_error_ - 2 * prev_error1_ + prev_error2_);
    if (output_ > cfg_.abs_output_max)
        output_ = cfg_.abs_output_max;
    if (output_ < -cfg_.abs_output_max)
        output_ = -cfg_.abs_output_max;

    prev_error2_ = prev_error1_;
    prev_error1_ = cur_error_;

    return output_;
}

void PIDMotor::reset()
{
    ref_         = 0.0f;
    fdb_         = 0.0f;
    cur_error_   = 0.0f;
    prev_error1_ = 0.0f;
    prev_error2_ = 0.0f;
    output_      = 0.0f;
}
/**
 * @file    mit_pd.h
 * @author  syhanjin
 * @date    2026-01-02
 * @brief   Brief description of the file
 *
 * Detailed description (optional).
 *
 */
#include "mit_pd.hpp"

float MITPD::calc(const float& p_ref, const float& p_fdb, const float& v_ref, const float& v_fdb)
{
    p_ref_ = p_ref;
    p_fdb_ = p_fdb;
    v_ref_ = v_ref;
    v_fdb_ = v_fdb;

    output_ = cfg_.Kp * (p_ref_ - p_fdb_) + cfg_.Kd * (v_ref_ - v_fdb_);
    if (output_ > cfg_.abs_output_max)
        output_ = cfg_.abs_output_max;
    if (output_ < -cfg_.abs_output_max)
        output_ = -cfg_.abs_output_max;
    return output_;
}

void MITPD::reset()
{
    p_ref_ = 0.0f;
    p_fdb_ = 0.0f;
    v_ref_ = 0.0f;
    v_fdb_ = 0.0f;
    output_ = 0.0f;
}


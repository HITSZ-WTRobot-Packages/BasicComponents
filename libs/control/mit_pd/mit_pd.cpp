/**
 * @file    mit_pd.cpp
 * @author  syhanjin
 * @date    2026-01-02
 * @brief   MIT 风格 PD 控制器实现。
 *
 * 这个实现把位置误差和速度误差分别映射到控制输出，适合电机伺服这种“位置 + 阻尼”联合调节的场景。
 * 它不是通用意义上的复杂控制器，只保留最核心的计算和限幅逻辑，方便新同学顺着代码快速理解。
 */
#include "mit_pd.hpp"

float MITPD::calc(const float& p_ref, const float& p_fdb, const float& v_ref, const float& v_fdb)
{
    // 保存本次输入，便于上层调试和状态追踪。
    p_ref_ = p_ref;
    p_fdb_ = p_fdb;
    v_ref_ = v_ref;
    v_fdb_ = v_fdb;

    // 控制律：位置误差提供主驱动力，速度误差提供阻尼。
    output_ = cfg_.Kp * (p_ref_ - p_fdb_) + cfg_.Kd * (v_ref_ - v_fdb_);
    // 输出限幅，防止下游驱动饱和。
    if (output_ > cfg_.abs_output_max)
        output_ = cfg_.abs_output_max;
    if (output_ < -cfg_.abs_output_max)
        output_ = -cfg_.abs_output_max;
    return output_;
}

void MITPD::reset()
{
    // 清空所有状态，下一次计算从零开始。
    p_ref_ = 0.0f;
    p_fdb_ = 0.0f;
    v_ref_ = 0.0f;
    v_fdb_ = 0.0f;
    output_ = 0.0f;
}

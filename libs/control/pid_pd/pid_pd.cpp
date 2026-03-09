/*******************************************************************************
 * @file    pid_pd.h
 * @author  syhanjin
 * @date    2025-11-08
 * @brief   PD 控制器
 *
 * 本库只包含基本的 比例 - 微分 控制以及抗饱和算法
 *
 * Detailed description (optional).
 *
 * --------------------------------------------------------------------------
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * Project repository: https://github.com/HITSZ-WTRobot/chassises_controller
 ******************************************************************************/
#include "pid_pd.hpp"

float PD::calc(const float& ref, const float& fdb)
{
    cur_error_ = ref - fdb;
    output_    = cfg_.Kp * cur_error_ + cfg_.Kd * (cur_error_ - prev_error_);
    if (output_ > cfg_.abs_output_max)
        output_ = cfg_.abs_output_max;
    if (output_ < -cfg_.abs_output_max)
        output_ = -cfg_.abs_output_max;

    prev_error_ = cur_error_;
    ref_        = ref;
    fdb_        = fdb;

    return output_;
}

void PD::reset()
{
    ref_        = 0.0f;
    fdb_        = 0.0f;
    cur_error_  = 0.0f;
    prev_error_ = 0.0f;
    output_     = 0.0f;
}

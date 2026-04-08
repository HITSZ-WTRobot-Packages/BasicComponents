/*******************************************************************************
 * @file    pid_pd.h
 * @author  syhanjin
 * @date    2025-11-08
 * @brief   PD 控制器
 *
 * 这个文件实现的是最基础的 PD 计算逻辑：误差、微分、限幅和状态更新。
 * 它适合用作理解控制器结构的起点，也适合那些不想引入积分项、但又希望控制量稳定的场景。
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
    // 当前误差 = 目标 - 反馈。
    cur_error_ = ref - fdb;
    // PD 主体：P 负责拉回，D 负责抑制误差变化速度。
    output_    = cfg_.Kp * cur_error_ + cfg_.Kd * (cur_error_ - prev_error_);
    // 输出限幅，防止驱动器进入过饱和区。
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
    // 清空历史状态，避免旧误差影响下一轮控制。
    ref_        = 0.0f;
    fdb_        = 0.0f;
    cur_error_  = 0.0f;
    prev_error_ = 0.0f;
    output_     = 0.0f;
}

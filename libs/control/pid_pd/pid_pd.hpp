/**
 * @file    pid_pd.hpp
 * @author  syhanjin
 * @date    2025-11-08
 * @brief   PD 控制器
 *
 * 本库提供最小可用的 PD 控制器：比例项负责快速跟踪，微分项用于抑制变化率。
 * 设计目标不是“功能多”，而是让新接触控制的小伙伴能够很快看懂输入、状态和输出分别代表什么。
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
 */
#ifndef PID_PD_HPP
#define PID_PD_HPP

class PD
{
public:
    struct Config
    {
        float Kp{ 0.0f };             ///< 比例系数
        float Kd{ 0.0f };             ///< 微分系数
        float abs_output_max{ 0.0f }; ///< 输出绝对值上限
    };

    PD() = default;
    explicit PD(const Config& cfg) : cfg_(cfg) {}

    /**
     * @brief 计算一次 PD 输出。
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
     * @brief 清空内部历史状态。
     */
    void  reset();

    float getRef() const { return ref_; }
    float getOutput() const { return output_; }

private:
    Config cfg_;

    float fdb_        = 0.0f; ///< 最近一次反馈值
    float cur_error_  = 0.0f; ///< 当前误差
    float prev_error_ = 0.0f; ///< 上一次误差
    float output_     = 0.0f; ///< 最近一次输出
    float ref_        = 0.0f; ///< 最近一次目标值
};

#endif // PID_PD_HPP

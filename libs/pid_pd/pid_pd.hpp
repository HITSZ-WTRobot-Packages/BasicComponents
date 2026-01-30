/**
 * @file    pid_pd.hpp
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
 */
#ifndef PID_PD_HPP
#define PID_PD_HPP

class PD
{
public:
    struct Config
    {
        float Kp{ 0.0f };
        float Kd{ 0.0f };
        float abs_output_max{ 0.0f };
    };

    PD() = default;
    explicit PD(const Config& cfg) : cfg_(cfg) {}

    float calc(const float& ref, const float& fdb);
    void  setConfig(const Config& cfg) { cfg_ = cfg; }
    void  reset();

    float getRef() const { return ref_; }
    float getOutput() const { return output_; }

private:
    Config cfg_;

    float fdb_        = 0.0f;
    float cur_error_  = 0.0f;
    float prev_error_ = 0.0f;
    float output_     = 0.0f;
    float ref_        = 0.0f;
};

#endif // PID_PD_HPP

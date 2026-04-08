/**
 * @file    mit_pd.hpp
 * @author  syhanjin
 * @date    2026-01-02
 * @brief   基于位置误差和速度误差的 MIT 风格 PD 控制器。
 *
 * 适合电机伺服场景：位置环给出位移误差，速度环给出阻尼项，输出做绝对值限幅。
 *
 */
#ifndef MIT_PD_HPP
#define MIT_PD_HPP

class MITPD
{
public:
    struct Config
    {
        float Kp{ 0.0f };             ///< 位置误差增益
        float Kd{ 0.0f };             ///< 速度误差增益
        float abs_output_max{ 0.0f }; ///< 输出绝对值上限
    };

    MITPD() = default;
    explicit MITPD(const Config& cfg) : cfg_(cfg) {}

    /**
     * @brief 计算一次 MIT 风格 PD 输出。
     *
     * @param p_ref 位置给定
     * @param p_fdb 位置反馈
     * @param v_ref 速度给定
     * @param v_fdb 速度反馈
     * @return 限幅后的控制输出
     */
    float calc(const float& p_ref, const float& p_fdb, const float& v_ref, const float& v_fdb);
    void  setConfig(const Config& cfg) { cfg_ = cfg; }
    void  reset();

    float getOutput() const { return output_; }

private:
    Config cfg_{};

    float p_ref_  = 0.0f; ///< 最近一次位置给定
    float p_fdb_  = 0.0f; ///< 最近一次位置反馈
    float v_ref_  = 0.0f; ///< 最近一次速度给定
    float v_fdb_  = 0.0f; ///< 最近一次速度反馈
    float output_ = 0.0f; ///< 最近一次控制输出
};

#endif // MIT_PD_HPP

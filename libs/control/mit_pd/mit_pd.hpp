/**
 * @file    mit_pd.hpp
 * @author  syhanjin
 * @date    2026-01-02
 * @brief   Brief description of the file
 *
 * Detailed description (optional).
 *
 */
#ifndef MIT_PD_HPP
#define MIT_PD_HPP

class MITPD
{
public:
    struct Config
    {
        float Kp{ 0.0f };
        float Kd{ 0.0f };
        float abs_output_max{ 0.0f };
    };

    MITPD() = default;
    explicit MITPD(const Config& cfg) : cfg_(cfg) {}

    float calc(const float& p_ref, const float& p_fdb, const float& v_ref, const float& v_fdb);
    void  setConfig(const Config& cfg) { cfg_ = cfg; }
    void  reset();

    float getOutput() const { return output_; }

private:
    Config cfg_{};

    float p_ref_  = 0.0f;
    float p_fdb_  = 0.0f;
    float v_ref_  = 0.0f;
    float v_fdb_  = 0.0f;
    float output_ = 0.0f;
};

#endif // MIT_PD_HPP

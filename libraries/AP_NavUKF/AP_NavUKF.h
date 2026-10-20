#pragma once

#include "AP_NavUKF_config.h"

#if HAL_NAVUKF_AVAILABLE

#include <AP_Param/AP_Param.h>

#include "AP_NavUKF_core.h"

class NavUKF {
public:
    NavUKF();

    CLASS_NO_COPY(NavUKF);

    static const struct AP_Param::GroupInfo var_info[];

    bool enabled() const { return _enable != 0; }

    // run one filter step; called by AP_AHRS after EKF3 and by Replay from the RFRF frame
    void UpdateFilter(void);

    // number of UpdateFilter calls that ran (stub check for Replay)
    uint32_t frame_count() const { return _frame_count; }

    // true once the primary core has aligned and initialised
    bool initialised() const { return cores[primary].initialised(); }

    // NIS monitoring after the settling time of core c, per measurement type (0 GPS pos, 1 GPS vel, 2 baro, 3 yaw, 4 airspeed, 5 sideslip, 6 GPS hgt)
    uint32_t nis_count(uint8_t t, uint8_t c = 0) const { return cores[c].nis_count(t); }
    uint32_t nis_inside(uint8_t t, uint8_t c = 0) const { return cores[c].nis_inside(t); }

    // redundancy (Step 19): cores, primary core, IMU and compass votes
    static constexpr uint8_t MAX_CORES = 3;
    bool core_enabled(uint8_t c) const { return c < MAX_CORES && (uint8_t(_imu_mask.get()) & (1U << c)) != 0; }
    bool core_initialised(uint8_t c) const { return c < MAX_CORES && cores[c].initialised(); }
    uint8_t primary_core() const { return primary; }
    uint16_t primary_switches() const { return switch_count; }
    uint8_t imu_voted_out_ever() const { return imu_out_ever; }
    uint16_t compass_isolations() const { return compass_isolation_count; }
    uint16_t yaw_suspensions() const { return yaw_suspension_count; }
    // compass i may be fused (compass voting has not isolated it)
    bool compass_allowed(uint8_t i) const { return i >= 8 || !(compass_isolated & (1U << i)); }

    // tuning values for the core
    UKFModel::ProcessNoise process_noise() const;
    ftype ut_alpha() const { return _ut_alpha.get(); }
    ftype ut_beta() const { return _ut_beta.get(); }
    ftype ut_kappa() const { return _ut_kappa.get(); }
    UKFModel::MeasNoise meas_noise() const;

    // sensor delays in ms; GPS -1 means the driver-reported lag
    int16_t gps_delay_ms() const { return _gps_delay_ms.get(); }
    int16_t baro_delay_ms() const { return _baro_delay_ms.get(); }
    int16_t mag_delay_ms() const { return _mag_delay_ms.get(); }
    int16_t gps_tofs_ms() const { return _gps_tofs_ms.get(); }
    int16_t arsp_delay_ms() const { return _arsp_delay_ms.get(); }
    bool sideslip_enabled() const { return _sslip_en != 0; }
    ftype arsp_scale() const { return _arsp_scale.get(); }

    // aiding sources switched on (UKF_MOD_MASK, bits as NavUKF_core::SRC_*)
    bool source_enabled(uint8_t bit) const { return (uint16_t(_mod_mask.get()) & (1U << bit)) != 0; }

    // horizontal acceleration above which horizontal accel bias is learned, m/s^2
    ftype abias_man() const { return _abias_man.get(); }

    // fusion horizon delay in ms chosen at initialisation
    uint16_t horizon_delay_ms() const { return cores[primary].horizon_delay_ms(); }

private:
    AP_Int8 _enable;
    AP_Float _gyr_noise;        // sigma_g, rad/s
    AP_Float _acc_noise;        // sigma_a, m/s^2
    AP_Float _gbias_noise;      // sigma_bg, rad/s per sqrt(s)
    AP_Float _abias_noise;      // sigma_ba, m/s^2 per sqrt(s)
    AP_Float _wind_noise;       // sigma_w, m/s per sqrt(s)
    AP_Float _ut_alpha;
    AP_Float _ut_beta;
    AP_Float _ut_kappa;
    AP_Float _gps_ph_noise;     // GPS horizontal position floor, m
    AP_Float _gps_pv_noise;     // GPS vertical position floor, m
    AP_Float _gps_v_noise;      // GPS velocity floor, m/s
    AP_Float _baro_noise;       // baro altitude, m
    AP_Float _bbias_noise;      // sigma_bb, m per sqrt(s)
    AP_Float _yaw_noise;        // compass yaw floor, rad
    AP_Int16 _gps_delay_ms;     // GPS delay, ms (-1: driver lag)
    AP_Int16 _baro_delay_ms;    // baro delay, ms
    AP_Int16 _mag_delay_ms;     // compass delay, ms
    AP_Float _abias_man;        // accel bias learning threshold, m/s^2
    AP_Int16 _gps_tofs_ms;      // GPS stamp offset, ms (positive: data newer than the stamp)
    AP_Float _arsp_noise;       // true airspeed noise, m/s
    AP_Int16 _arsp_delay_ms;    // airspeed delay, ms
    AP_Int8 _sslip_en;          // synthetic sideslip update on
    AP_Float _sslip_noise;      // lateral body airspeed noise, m/s
    AP_Float _arsp_scale;       // Pitot calibration factor on true airspeed
    AP_Int16 _mod_mask;         // aiding sources of the fusion modules
    AP_Int8 _imu_mask;          // IMUs that run a UKF core
    AP_Float _imu_gtol;         // gyro voting tolerance, rad/s
    AP_Float _imu_atol;         // accel voting tolerance, m/s^2
    AP_Float _mag_tol;          // compass voting tolerance, deg

    NavUKF_core cores[MAX_CORES] { {*this, 0}, {*this, 1}, {*this, 2} };

    uint32_t _frame_count;

    // IMU voting on the raw data (Step 19.2)
    void vote_imus(uint32_t now_ms);
    bool imu_usable[MAX_CORES];         // delivering valid data this frame
    Vector3F gyro_filt[MAX_CORES];      // low-pass filtered rate, rad/s
    Vector3F accel_filt[MAX_CORES];     // low-pass filtered specific force, m/s^2
    uint32_t imu_valid_since_ms[MAX_CORES];
    uint32_t imu_bad_since_ms[MAX_CORES];
    uint32_t imu_good_since_ms[MAX_CORES];
    uint8_t imu_out;                    // bit i: IMU i voted out (bad data or no data)
    uint8_t imu_out_ever;
    uint8_t core_needs_reset;           // bit i: core i must restart from the primary when its IMU returns

    // compass voting (Step 19.4)
    void vote_compasses(uint32_t now_ms);
    ftype mag_diff_filt[3];             // filtered |yaw difference|: compass 0-1, 0-prediction, 1-prediction, rad
    uint32_t mag_cond_since_ms;         // time the current voting outcome has held
    uint8_t mag_cond_pending;           // outcome being confirmed (isolated mask)
    uint8_t compass_isolated;           // bit i: compass i not fused
    uint16_t compass_isolation_count;
    uint16_t yaw_suspension_count;

    // core selection (Step 19.3)
    void select_primary(uint32_t now_ms);
    uint8_t primary;
    uint16_t switch_count;
    uint32_t better_since_ms;
    uint8_t better_core;
    uint8_t log_div;
    void Log_Write_Status(void) const;
};

#endif // HAL_NAVUKF_AVAILABLE

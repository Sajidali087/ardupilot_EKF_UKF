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

    // true once the core has aligned and initialised
    bool initialised() const { return core.initialised(); }

    // NIS monitoring after the settling time, per measurement type (0 GPS pos, 1 GPS vel, 2 baro, 3 yaw, 4 airspeed, 5 sideslip)
    uint32_t nis_count(uint8_t t) const { return core.nis_count(t); }
    uint32_t nis_inside(uint8_t t) const { return core.nis_inside(t); }

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
    uint16_t horizon_delay_ms() const { return core.horizon_delay_ms(); }

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

    NavUKF_core core{*this};

    uint32_t _frame_count;
};

#endif // HAL_NAVUKF_AVAILABLE

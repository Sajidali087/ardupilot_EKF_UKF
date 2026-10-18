#include "AP_NavUKF.h"

#if HAL_NAVUKF_AVAILABLE

#include <AP_DAL/AP_DAL.h>

const AP_Param::GroupInfo NavUKF::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Enable UKF shadow estimator
    // @Description: Runs the UKF next to EKF3 in SITL and Replay. It never controls the vehicle.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 0, NavUKF, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: GYR_NSE
    // @DisplayName: Gyro noise
    // @Description: Gyro white noise standard deviation on the IMU delta angles in flight (S1, IMU 0). Use 1.2e-3 for S2.
    // @Units: rad/s
    // @Range: 0.00001 0.1
    // @User: Advanced
    AP_GROUPINFO("GYR_NSE", 1, NavUKF, _gyr_noise, 3.9e-4f),

    // @Param: ACC_NSE
    // @DisplayName: Accelerometer noise
    // @Description: Accelerometer white noise standard deviation on the IMU delta velocities in flight (S1, IMU 0)
    // @Units: m/s/s
    // @Range: 0.00001 1.0
    // @User: Advanced
    AP_GROUPINFO("ACC_NSE", 2, NavUKF, _acc_noise, 1.1e-3f),

    // @Param: GBIAS_NSE
    // @DisplayName: Gyro bias process noise
    // @Description: Random walk of the gyro bias states (tuning choice, not measurable in SITL)
    // @Range: 0.0 0.01
    // @User: Advanced
    AP_GROUPINFO("GBIAS_NSE", 3, NavUKF, _gbias_noise, 1.0e-4f),

    // @Param: ABIAS_NSE
    // @DisplayName: Accel bias process noise
    // @Description: Random walk of the accel bias states (tuning choice, not measurable in SITL)
    // @Range: 0.0 0.1
    // @User: Advanced
    AP_GROUPINFO("ABIAS_NSE", 4, NavUKF, _abias_noise, 1.0e-3f),

    // @Param: WIND_NSE
    // @DisplayName: Wind process noise
    // @Description: Random walk of the wind states (not observed in Part 1)
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("WIND_NSE", 5, NavUKF, _wind_noise, 0.1f),

    // @Param: ALPHA
    // @DisplayName: UT alpha
    // @Description: Unscented transform spread parameter
    // @Range: 0.001 1.0
    // @User: Advanced
    AP_GROUPINFO("ALPHA", 6, NavUKF, _ut_alpha, 1.0f),

    // @Param: BETA
    // @DisplayName: UT beta
    // @Description: Unscented transform prior-distribution parameter (2 is optimal for Gaussian)
    // @Range: 0.0 4.0
    // @User: Advanced
    AP_GROUPINFO("BETA", 7, NavUKF, _ut_beta, 2.0f),

    // @Param: KAPPA
    // @DisplayName: UT kappa
    // @Description: Unscented transform secondary scaling parameter
    // @Range: -10.0 10.0
    // @User: Advanced
    AP_GROUPINFO("KAPPA", 8, NavUKF, _ut_kappa, 0.0f),

    // @Param: GPS_PH_NSE
    // @DisplayName: GPS horizontal position noise
    // @Description: Floor of the GPS horizontal position standard deviation; the receiver accuracy is used when larger. SITL horizontal noise is near zero; the error seen in flight (0.18-0.35 m on D2) is a 17 ms timing offset times speed.
    // @Units: m
    // @Range: 0.1 10.0
    // @User: Advanced
    AP_GROUPINFO("GPS_PH_NSE", 9, NavUKF, _gps_ph_noise, 0.3f),

    // @Param: GPS_PV_NSE
    // @DisplayName: GPS vertical position noise
    // @Description: Floor of the GPS vertical position standard deviation (static log); the receiver accuracy is used when larger
    // @Units: m
    // @Range: 0.1 10.0
    // @User: Advanced
    AP_GROUPINFO("GPS_PV_NSE", 10, NavUKF, _gps_pv_noise, 0.35f),

    // @Param: GPS_V_NSE
    // @DisplayName: GPS velocity noise
    // @Description: Floor of the GPS velocity standard deviation (D2 flight vs truth: 0.052 N, 0.054 E, 0.031 D m/s); the receiver speed accuracy is used when larger (SITL reports 0)
    // @Units: m/s
    // @Range: 0.01 5.0
    // @User: Advanced
    AP_GROUPINFO("GPS_V_NSE", 11, NavUKF, _gps_v_noise, 0.054f),

    // @Param: BARO_NSE
    // @DisplayName: Barometer noise
    // @Description: Barometer altitude standard deviation (D2 flight vs truth 0.117 m; static log 0.16 m)
    // @Units: m
    // @Range: 0.05 5.0
    // @User: Advanced
    AP_GROUPINFO("BARO_NSE", 12, NavUKF, _baro_noise, 0.12f),

    // @Param: BBIAS_NSE
    // @DisplayName: Baro bias process noise
    // @Description: Random walk of the baro bias state: (drift*60)/(3*sqrt(60)) with drift 38.55 m/h
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("BBIAS_NSE", 13, NavUKF, _bbias_noise, 0.028f),

    // @Param: YAW_NSE
    // @DisplayName: Compass yaw noise
    // @Description: Standard deviation of the tilt-compensated compass yaw measurement. D2: 0.0215 rad against truth with the true tilt; the yaw innovations with the estimated tilt spread 0.0325 rad, so 0.03 is used (NIS mean 1.16)
    // @Units: rad
    // @Range: 0.005 1.0
    // @User: Advanced
    AP_GROUPINFO("YAW_NSE", 14, NavUKF, _yaw_noise, 0.03f),

    // @Param: GPS_DELAY
    // @DisplayName: GPS measurement delay
    // @Description: Delay of the GPS data relative to the IMU. -1 uses the lag reported by the GPS driver, the same value EKF3 uses. The UKF fuses each measurement on a delayed fusion horizon at its time of validity.
    // @Units: ms
    // @Range: -1 250
    // @User: Advanced
    AP_GROUPINFO("GPS_DELAY", 15, NavUKF, _gps_delay_ms, -1),

    // @Param: BARO_DELAY
    // @DisplayName: Barometer measurement delay
    // @Description: Delay of the barometer data relative to the IMU (measured effective delay 0-10 ms in SITL, docs/noise_table.md)
    // @Units: ms
    // @Range: 0 250
    // @User: Advanced
    AP_GROUPINFO("BARO_DELAY", 16, NavUKF, _baro_delay_ms, 0),

    // @Param: MAG_DELAY
    // @DisplayName: Compass measurement delay
    // @Description: Delay of the compass data relative to the IMU (measured effective delay 45 ms in SITL against truth, docs/noise_table.md)
    // @Units: ms
    // @Range: 0 250
    // @User: Advanced
    AP_GROUPINFO("MAG_DELAY", 17, NavUKF, _mag_delay_ms, 45),

    // @Param: ABIAS_MAN
    // @DisplayName: Accel bias learning manoeuvre threshold
    // @Description: Horizontal accel bias is learned only while the filtered horizontal acceleration exceeds this value or the turn rate exceeds 0.1 rad/s; otherwise the body axes that are not near vertical keep their accel bias estimate (tilt and horizontal accel bias cannot be told apart at rest or in steady flight)
    // @Units: m/s/s
    // @Range: 0.0 10.0
    // @User: Advanced
    AP_GROUPINFO("ABIAS_MAN", 18, NavUKF, _abias_man, 1.0f),

    // @Param: GPS_TOFS
    // @DisplayName: GPS time stamp offset
    // @Description: Offset of the GPS data time from its time stamp, added to the time of validity after the delay. Positive when the data are newer than the stamp. In SITL the u-blox driver stamps each fix about 17 ms before the data it carries (docs/noise_table.md). 0 by default; used for the matched-delay sensitivity case.
    // @Units: ms
    // @Range: -50 50
    // @User: Advanced
    AP_GROUPINFO("GPS_TOFS", 19, NavUKF, _gps_tofs_ms, 0),

    AP_GROUPEND
};

NavUKF::NavUKF()
{
    AP_Param::setup_object_defaults(this, var_info);
}

UKFModel::ProcessNoise NavUKF::process_noise() const
{
    UKFModel::ProcessNoise q;
    q.gyro = _gyr_noise.get();
    q.accel = _acc_noise.get();
    q.gyro_bias = _gbias_noise.get();
    q.accel_bias = _abias_noise.get();
    q.baro_bias = _bbias_noise.get();
    q.wind = _wind_noise.get();
    return q;
}

UKFModel::MeasNoise NavUKF::meas_noise() const
{
    UKFModel::MeasNoise r;
    r.gps_pos_h = _gps_ph_noise.get();
    r.gps_pos_v = _gps_pv_noise.get();
    r.gps_vel = _gps_v_noise.get();
    r.baro = _baro_noise.get();
    r.yaw = _yaw_noise.get();
    return r;
}

void NavUKF::UpdateFilter(void)
{
    if (!enabled()) {
        // no frame bit when disabled, so logs stay identical to a build without the UKF
        return;
    }

    // adds the UKF bit to the frame EKF3 opened in this loop (no-op in Replay)
    AP::dal().start_frame(AP_DAL::FrameType::UpdateFilterUKF);

    core.update();

    _frame_count++;
}

#endif // HAL_NAVUKF_AVAILABLE

#include "AP_NavUKF.h"

#if HAL_NAVUKF_AVAILABLE

#include <AP_DAL/AP_DAL.h>

// baro bias process noise, m per sqrt(s): drift 38.55 m/h, (drift*60)/(3*sqrt(60)); parameter UKF_BBIAS_NSE comes with item 7
static constexpr float BARO_BIAS_NOISE = 0.028f;

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
    q.baro_bias = BARO_BIAS_NOISE;
    q.wind = _wind_noise.get();
    return q;
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

/*
  UKF shadow estimator for ArduPilot (libraries/AP_NavUKF): frontend (parameters, cores, votes, core selection)

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "AP_NavUKF.h"

#if HAL_NAVUKF_AVAILABLE

#include <AP_DAL/AP_DAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

// redundancy (Step 19, ukf_state_model.md Section 16)
static constexpr float VOTE_TAU = 0.5f;             // s, low-pass of the IMU data compared in the vote
static constexpr uint32_t VOTE_SETTLE_MS = 5000;    // IMU data needed before an IMU can be voted on
static constexpr uint32_t VOTE_BAD_MS = 1000;       // a difference above tolerance this long votes an IMU out
static constexpr uint32_t VOTE_GOOD_MS = 5000;      // back inside half the tolerance this long re-admits it
static constexpr uint32_t IMU_RETURN_MS = 2000;     // an IMU that lost its data is re-admitted after this long
static constexpr float MAG_TAU = 1.0f;              // s, low-pass of the compass yaw differences
static constexpr uint32_t MAG_CONFIRM_MS = 2000;    // a compass vote outcome must hold this long
static constexpr uint32_t MAG_RELEASE_MS = 5000;    // isolation ends when the compasses agree again this long
static constexpr float SWITCH_RATIO = 0.5f;         // another core's score must be below this fraction of the primary's
static constexpr uint32_t SWITCH_MS = 10000;        // for this long
static constexpr uint16_t TIMING_FRAMES = 50;      // UKFT: one message per 50 frames (1 s at 50 Hz)

// airspeed defaults measured in SITL against the simulator truth (docs/noise_table.md, Step 16)
#define ARSP_NSE_DEFAULT    0.05f   // m/s: Pitot TAS scatter 0.042 m/s at the matched delay (p2_w1_wind)
#define ARSP_DELAY_DEFAULT  200     // ms: effective Pitot delay (S1, p2_w1_wind)
#define ARSP_SCL_DEFAULT    1.034f  // SITL Pitot reads 1/EAS2TAS low: DAL airspeed x EAS2TAS = true EAS

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

    // @Param: ARSP_NSE
    // @DisplayName: Airspeed noise
    // @Description: Standard deviation of the true airspeed measurement (Pitot EAS times EAS2TAS)
    // @Units: m/s
    // @Range: 0.05 5.0
    // @User: Advanced
    AP_GROUPINFO("ARSP_NSE", 20, NavUKF, _arsp_noise, ARSP_NSE_DEFAULT),

    // @Param: ARSP_DELAY
    // @DisplayName: Airspeed measurement delay
    // @Description: Delay of the airspeed data relative to the IMU
    // @Units: ms
    // @Range: 0 250
    // @User: Advanced
    AP_GROUPINFO("ARSP_DELAY", 21, NavUKF, _arsp_delay_ms, ARSP_DELAY_DEFAULT),

    // @Param: SSLIP_EN
    // @DisplayName: Synthetic sideslip update
    // @Description: Fuse a zero lateral body airspeed with every airspeed sample (fixed wing in coordinated flight); on by default, as EKF3 does for planes
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("SSLIP_EN", 22, NavUKF, _sslip_en, 1),

    // @Param: SSLIP_NSE
    // @DisplayName: Synthetic sideslip noise
    // @Description: Standard deviation of the lateral body airspeed assumed by the synthetic sideslip update
    // @Units: m/s
    // @Range: 0.1 5.0
    // @User: Advanced
    AP_GROUPINFO("SSLIP_NSE", 23, NavUKF, _sslip_noise, 1.0f),

    // @Param: ARSP_SCL
    // @DisplayName: Airspeed scale
    // @Description: Factor applied to the true airspeed (Pitot EAS times EAS2TAS) before fusion, as a Pitot calibration (ARSPD_RATIO does the same in the airspeed library). In SITL the Pitot reads 1/EAS2TAS low (measured 1.034 on S1 and p2_w1_wind against the simulator truth); EKF3 does not apply it.
    // @Range: 0.8 1.2
    // @User: Advanced
    AP_GROUPINFO("ARSP_SCL", 24, NavUKF, _arsp_scale, ARSP_SCL_DEFAULT),

    // @Param: MOD_MASK
    // @DisplayName: Aiding sources of the fusion modules
    // @Description: Aiding measurements the UKF fuses, by fusion module (docs/part2/fusion_modules.md): position module GPS horizontal position; altitude module GPS height and barometer; velocity module GPS velocity and airspeed (with the synthetic sideslip); heading module compass. Clear a bit to switch that source off, e.g. for module tests.
    // @Bitmask: 0:GPS horizontal position,1:GPS height,2:GPS velocity,3:Airspeed,4:Barometer,5:Compass
    // @User: Advanced
    AP_GROUPINFO("MOD_MASK", 25, NavUKF, _mod_mask, 63),

    // @Param: IMU_MASK
    // @DisplayName: IMUs with a UKF core
    // @Description: One UKF core runs on each selected IMU; the primary core gives the output (UKFP)
    // @Bitmask: 0:IMU1,1:IMU2,2:IMU3
    // @User: Advanced
    AP_GROUPINFO("IMU_MASK", 26, NavUKF, _imu_mask, 7),

    // @Param: IMU_GTOL
    // @DisplayName: IMU vote gyro tolerance
    // @Description: An IMU whose filtered rate differs from the median of the three IMUs by more than this for 1 s is voted out
    // @Units: rad/s
    // @Range: 0.005 0.5
    // @User: Advanced
    AP_GROUPINFO("IMU_GTOL", 27, NavUKF, _imu_gtol, 0.02f),

    // @Param: IMU_ATOL
    // @DisplayName: IMU vote accel tolerance
    // @Description: An IMU whose filtered specific force differs from the median of the three IMUs by more than this for 1 s is voted out
    // @Units: m/s/s
    // @Range: 0.05 5.0
    // @User: Advanced
    AP_GROUPINFO("IMU_ATOL", 28, NavUKF, _imu_atol, 0.3f),

    // @Param: MAG_TOL
    // @DisplayName: Compass vote tolerance
    // @Description: Filtered yaw difference above which two compasses, or a compass and the primary core's predicted yaw, disagree
    // @Units: deg
    // @Range: 2 45
    // @User: Advanced
    AP_GROUPINFO("MAG_TOL", 29, NavUKF, _mag_tol, 10.0f),

    // @Param: FD_ENABLE
    // @DisplayName: Fault detection
    // @Description: Innovation gate, persistence, CUSUM and timeouts per sensor channel; faulty channels are isolated (not fused) and re-admitted after a consistent period (ukf_state_model.md Section 17)
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("FD_ENABLE", 30, NavUKF, _fd_enable, 1),

    // @Param: FD_GATE
    // @DisplayName: Fault detection gate scale
    // @Description: Scale on the per-channel innovation gates (in standard deviations; 1 = gates set on the fault-free logs)
    // @Range: 0.5 5
    // @User: Advanced
    AP_GROUPINFO("FD_GATE", 31, NavUKF, _fd_gate, 1.0f),

    // @Param: FD_K
    // @DisplayName: Fault detection persistence count
    // @Description: A channel is faulty when FD_K of the last FD_N samples were outside the gate
    // @Range: 1 16
    // @User: Advanced
    AP_GROUPINFO("FD_K", 32, NavUKF, _fd_k, 5),

    // @Param: FD_N
    // @DisplayName: Fault detection persistence window
    // @Description: Number of recent samples in the persistence test
    // @Range: 1 16
    // @User: Advanced
    AP_GROUPINFO("FD_N", 33, NavUKF, _fd_n, 10),

    // @Param: FD_CUSUM
    // @DisplayName: Fault detection CUSUM scale
    // @Description: Scale on the per-channel CUSUM thresholds (1 = thresholds set on the fault-free logs)
    // @Range: 0.2 10
    // @User: Advanced
    AP_GROUPINFO("FD_CUSUM", 34, NavUKF, _fd_cusum, 1.0f),

    // @Param: FD_RECOV
    // @DisplayName: Fault recovery period
    // @Description: Consistent innovations (computed but not fused) for this long re-admit an isolated channel; as long again in RECOVERING returns it to OK
    // @Units: s
    // @Range: 1 60
    // @User: Advanced
    AP_GROUPINFO("FD_RECOV", 35, NavUKF, _fd_recov, 5.0f),

    // @Param: FD_RESET
    // @DisplayName: Fault reset period
    // @Description: A channel isolated this long while its data keep arriving is re-admitted with a covariance reset (the state may have drifted while it was excluded)
    // @Units: s
    // @Range: 5 300
    // @User: Advanced
    AP_GROUPINFO("FD_RESET", 36, NavUKF, _fd_reset, 15.0f),

    // @Param: WIND_PSCALE
    // @DisplayName: Wind process noise height-rate scale
    // @Description: The wind process noise is scaled by (1 + WIND_PSCALE * |vertical speed|), as EKF3 does (EK3_WIND_PSCALE): wind changes with height (wind gradient), so the wind states must be free to change in climbs and descents
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("WIND_PSCALE", 37, NavUKF, _wind_pscale, 1.0f),

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
    r.airspeed = _arsp_noise.get();
    r.sideslip = _sslip_noise.get();
    return r;
}

void NavUKF::vote_imus(uint32_t now_ms)
{
    const auto &ins = AP::dal().ins();
    Vector3F rate[MAX_CORES], acc[MAX_CORES];
    uint8_t n_usable = 0;
    for (uint8_t i = 0; i < MAX_CORES; i++) {
        Vector3f dA, dV;
        float dtA = 0, dtV = 0;
        imu_usable[i] = i < ins.get_gyro_count() && i < ins.get_accel_count() &&
                        ins.use_gyro(i) && ins.use_accel(i) &&
                        ins.get_delta_angle(i, dA, dtA) && ins.get_delta_velocity(i, dV, dtV) &&
                        dtA > 1.0e-4f && dtV > 1.0e-4f;
        if (!imu_usable[i]) {
            // no data: voted out at once; its core restarts from the primary when the data returns
            if (!(imu_out & (1U << i)) && imu_valid_since_ms[i] != 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "UKF: IMU%u no data, voted out", unsigned(i));
            }
            imu_out |= (1U << i);
            core_needs_reset |= (1U << i);
            imu_valid_since_ms[i] = 0;
            continue;
        }
        rate[i] = dA.toftype() / ftype(dtA);
        acc[i] = dV.toftype() / ftype(dtV);
        if (imu_valid_since_ms[i] == 0) {
            imu_valid_since_ms[i] = now_ms;
            gyro_filt[i] = rate[i];
            accel_filt[i] = acc[i];
        }
        const ftype k = MIN(ftype(dtV) / ftype(VOTE_TAU), ftype(1));
        gyro_filt[i] += (rate[i] - gyro_filt[i]) * k;
        accel_filt[i] += (acc[i] - accel_filt[i]) * k;
        n_usable++;
    }

    for (uint8_t i = 0; i < MAX_CORES; i++) {
        if (!imu_usable[i]) {
            continue;
        }
        const bool settled = now_ms - imu_valid_since_ms[i] >= VOTE_SETTLE_MS;
        bool bad = false, good = true;
        if (n_usable == MAX_CORES && settled) {
            // each IMU against the median of the three, per axis
            Vector3F gmed, amed;
            for (uint8_t a = 0; a < 3; a++) {
                ftype g[3] = { gyro_filt[0][a], gyro_filt[1][a], gyro_filt[2][a] };
                ftype f[3] = { accel_filt[0][a], accel_filt[1][a], accel_filt[2][a] };
                gmed[a] = UKFModel::median3(g[0], g[1], g[2]);
                amed[a] = UKFModel::median3(f[0], f[1], f[2]);
            }
            const ftype gd = (gyro_filt[i] - gmed).length();
            const ftype ad = (accel_filt[i] - amed).length();
            bad = gd > ftype(_imu_gtol.get()) || ad > ftype(_imu_atol.get());
            good = gd < ftype(0.5) * ftype(_imu_gtol.get()) && ad < ftype(0.5) * ftype(_imu_atol.get());
        }
        if (bad) {
            imu_good_since_ms[i] = 0;
            if (imu_bad_since_ms[i] == 0) {
                imu_bad_since_ms[i] = now_ms;
            }
            if (!(imu_out & (1U << i)) && now_ms - imu_bad_since_ms[i] >= VOTE_BAD_MS) {
                imu_out |= (1U << i);
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "UKF: IMU%u voted out (differs from median)", unsigned(i));
            }
        } else {
            imu_bad_since_ms[i] = 0;
            if (imu_good_since_ms[i] == 0) {
                imu_good_since_ms[i] = now_ms;
            }
            // re-admit after a consistent period (data that had stopped: after IMU_RETURN_MS)
            const uint32_t need = (core_needs_reset & (1U << i)) ? IMU_RETURN_MS : VOTE_GOOD_MS;
            if ((imu_out & (1U << i)) && good && now_ms - imu_good_since_ms[i] >= need) {
                imu_out &= ~(1U << i);
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UKF: IMU%u re-admitted", unsigned(i));
            }
        }
    }
    imu_out_ever |= imu_out;
}

void NavUKF::vote_compasses(uint32_t now_ms)
{
    const auto &compass = AP::dal().compass();
    const NavUKF_core &pc = cores[primary];
    if (!pc.initialised()) {
        compass_isolated = 0;
        return;
    }
    ftype roll, pitch, yaw_pred;
    pc.output_euler(roll, pitch, yaw_pred);
    ftype yaw[2] {};
    bool have[2] {};
    for (uint8_t i = 0; i < 2 && i < compass.get_count(); i++) {
        if (compass.use_for_yaw(i) && compass.healthy(i)) {
            yaw[i] = UKFModel::yaw_from_mag(compass.get_field(i).toftype(), roll, pitch, compass.get_declination());
            have[i] = true;
        }
    }
    const ftype k = MIN(ftype(AP::dal().ins().get_loop_delta_t()) / ftype(MAG_TAU), ftype(1));
    const ftype d[3] = {
        (have[0] && have[1]) ? fabsF(wrap_PI(yaw[0] - yaw[1])) : ftype(0),
        have[0] ? fabsF(wrap_PI(yaw[0] - yaw_pred)) : ftype(0),
        have[1] ? fabsF(wrap_PI(yaw[1] - yaw_pred)) : ftype(0),
    };
    for (uint8_t j = 0; j < 3; j++) {
        mag_diff_filt[j] += (d[j] - mag_diff_filt[j]) * k;
    }
    const ftype tol = radians(ftype(_mag_tol.get()));
    const bool bad0 = have[0] && mag_diff_filt[1] > tol;
    const bool bad1 = have[1] && mag_diff_filt[2] > tol;
    uint8_t outcome = 0;
    if (have[0] && have[1]) {
        if (mag_diff_filt[0] > tol) {
            // the compasses disagree: the one further from the prediction is out (if it is outside the tolerance)
            if (mag_diff_filt[1] >= mag_diff_filt[2] && bad0) {
                outcome = 1;
            } else if (mag_diff_filt[2] > mag_diff_filt[1] && bad1) {
                outcome = 2;
            }
        } else if (bad0 && bad1) {
            // both agree with each other but not with the prediction: suspend yaw fusion
            outcome = 3;
        }
    } else if (bad0 || bad1) {
        outcome = have[0] ? 1 : 2;
    }
    // hysteresis: an isolation is released only when every available difference is inside half the tolerance
    // (a fixed body-frame offset changes the yaw error with heading, so it can dip below the tolerance in turns)
    const bool clear = mag_diff_filt[0] < ftype(0.5) * tol &&
                       (!have[0] || mag_diff_filt[1] < ftype(0.5) * tol) &&
                       (!have[1] || mag_diff_filt[2] < ftype(0.5) * tol);
    if (outcome == 0 && compass_isolated != 0 && !clear) {
        outcome = compass_isolated;
    }
    // an outcome must hold before it is applied; isolation ends only after a longer consistent period
    if (outcome != mag_cond_pending) {
        mag_cond_pending = outcome;
        mag_cond_since_ms = now_ms;
    }
    const uint32_t need = (outcome == 0) ? MAG_RELEASE_MS : MAG_CONFIRM_MS;
    if (outcome != compass_isolated && now_ms - mag_cond_since_ms >= need) {
        if (outcome == 3) {
            yaw_suspension_count++;
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "UKF: both compasses disagree with predicted yaw, yaw fusion suspended");
        } else if (outcome != 0) {
            compass_isolation_count++;
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "UKF: compass %u isolated", unsigned(outcome == 1 ? 1 : 2));
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UKF: compasses re-admitted");
        }
        compass_isolated = outcome;
    }
}

void NavUKF::select_primary(uint32_t now_ms)
{
    bool healthy[MAX_CORES];
    ftype score[MAX_CORES];
    for (uint8_t c = 0; c < MAX_CORES; c++) {
        healthy[c] = core_enabled(c) && cores[c].initialised() && imu_usable[c] && !(imu_out & (1U << c));
        score[c] = cores[c].score();
    }
    int8_t best;
    bool candidate;
    const bool at_once = UKFModel::primary_decision(primary, healthy, score, ftype(SWITCH_RATIO), best, candidate);
    if (best < 0) {
        return;
    }
    uint8_t next = primary;
    if (at_once) {
        // the primary's IMU is out: switch at once
        next = uint8_t(best);
    } else if (candidate) {
        // another core is clearly more consistent, for long enough
        if (better_core != uint8_t(best)) {
            better_core = uint8_t(best);
            better_since_ms = now_ms;
        } else if (now_ms - better_since_ms >= SWITCH_MS) {
            next = uint8_t(best);
        }
    } else {
        better_core = 0xFF;
    }
    if (next != primary) {
        primary = next;
        switch_count++;
        better_core = 0xFF;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "UKF: primary core %u", unsigned(primary));
    }
}

void NavUKF::UpdateFilter(void)
{
    if (!enabled()) {
        // no frame bit when disabled, so logs stay identical to a build without the UKF
        return;
    }

    // adds the UKF bit to the frame EKF3 opened in this loop (no-op in Replay)
    AP::dal().start_frame(AP_DAL::FrameType::UpdateFilterUKF);

    if (_frame_count == 0) {
        better_core = 0xFF;
        primary = 0;
        while (!core_enabled(primary) && primary < MAX_CORES - 1) {
            primary++;
        }
    }
    const uint32_t now_ms = AP::dal().millis();
    vote_imus(now_ms);
    vote_compasses(now_ms);

    for (uint8_t c = 0; c < MAX_CORES; c++) {
        if (!core_enabled(c)) {
            continue;
        }
        if ((core_needs_reset & (1U << c)) && imu_usable[c]) {
            // the IMU is back: restart this core from the primary, which kept navigating
            if (c != primary && cores[primary].initialised() && cores[c].initialised()) {
                cores[c].reset_from(cores[primary]);
            }
            core_needs_reset &= ~(1U << c);
        }
        cores[c].update(imu_usable[c], c == primary);
    }
    select_primary(now_ms);
    Log_Write_Vote_Events();

    if (cores[primary].initialised()) {
        cores[primary].Log_Write_Primary();
    }
    if (log_div == 0) {
        Log_Write_Status();
    }
    log_div = (log_div + 1) % 5;

    _frame_count++;
}

void NavUKF::Log_Write_Vote_Events(void)
{
#if HAL_LOGGING_ENABLED
    for (uint8_t i = 0; i < 5; i++) {
        const bool now_out = (i < 3) ? (imu_out & (1U << i)) : (compass_isolated & (1U << (i - 3)));
        const bool was_out = (i < 3) ? (imu_out_logged & (1U << i)) : (compass_logged & (1U << (i - 3)));
        if (now_out == was_out) {
            continue;
        }
        // the vote channels change between OK (0) and ISOLATED (3), test "vote" (5); written through the one UKFE
        // writer so that a log holds a single UKFE format
        NavUKF_core::Log_Write_Event(DAL_CORE(0), uint8_t(NavUKF_core::NUM_CH + i), uint8_t(was_out ? 3 : 0),
                                     uint8_t(now_out ? 3 : 0), uint8_t(5), 0.0f);
    }
#endif
    imu_out_logged = imu_out;
    compass_logged = compass_isolated;
}

void NavUKF::record_timing(uint32_t ekf3_ns, uint32_t ukf_ns)
{
#if HAL_LOGGING_ENABLED
    const uint32_t t[2] = { ekf3_ns, ukf_ns };
    for (uint8_t i = 0; i < 2; i++) {
        time_sum_ns[i] += t[i];
        time_max_ns[i] = MAX(time_max_ns[i], t[i]);
    }
    if (++time_count < TIMING_FRAMES) {
        return;
    }
#define UKFT_DEF "UKFT", "TimeUS,C,N,EAvg,EMax,UAvg,UMax", "s#-----", "F------", "QBHffff"
#if APM_BUILD_TYPE(APM_BUILD_Replay)
    AP_Logger::log_write_fmt *f = AP::logger().msg_fmt_for_name(UKFT_DEF, true);
    if (f != nullptr) {
        AP::logger().Safe_Write_Emit_FMT(f);
    }
#endif
    // @LoggerMessage: UKFT
    // @Description: Computation time per filter frame over N frames, measured in Replay (thread CPU time): EAvg, EMax EKF3 update (all EKF3 cores), UAvg, UMax UKF update (all UKF cores), microseconds
    // @Field: TimeUS: Time since system startup
    // @Field: C: core number base
    // @Field: N: frames in this record
    // @Field: EAvg: mean EKF3 update time, all EKF3 cores, microseconds
    // @Field: EMax: largest EKF3 update time, microseconds
    // @Field: UAvg: mean UKF update time, all UKF cores, microseconds
    // @Field: UMax: largest UKF update time, microseconds
    AP::logger().Write(UKFT_DEF, AP::dal().micros64(), DAL_CORE(0), time_count,
                       float(time_sum_ns[0] * 1.0e-3 / time_count), float(time_max_ns[0] * 1.0e-3),
                       float(time_sum_ns[1] * 1.0e-3 / time_count), float(time_max_ns[1] * 1.0e-3));
#endif
    memset(time_sum_ns, 0, sizeof(time_sum_ns));
    memset(time_max_ns, 0, sizeof(time_max_ns));
    time_count = 0;
}

void NavUKF::Log_Write_Status(void) const
{
#if HAL_LOGGING_ENABLED
#define UKFS_DEF "UKFS", "TimeUS,C,Pri,Sc0,Sc1,Sc2,IU,IV,MI,NSw,H0,H1,H2", "s#-----------", "F-00000000000", "QBBfffBBBHBBB"
#if APM_BUILD_TYPE(APM_BUILD_Replay)
    AP_Logger::log_write_fmt *f = AP::logger().msg_fmt_for_name(UKFS_DEF, true);
    if (f != nullptr) {
        AP::logger().Safe_Write_Emit_FMT(f);
    }
#endif
    uint8_t usable = 0;
    for (uint8_t i = 0; i < MAX_CORES; i++) {
        usable |= imu_usable[i] ? (1U << i) : 0;
    }
    // @LoggerMessage: UKFS
    // @Description: UKF redundancy at 10 Hz: Pri primary core, Sc0-Sc2 core scores (NIS per degree of freedom, filtered), IU IMUs delivering data, IV IMUs voted out, MI compass vote (0 both used, 1 compass 1 isolated, 2 compass 2 isolated, 3 yaw fusion suspended), NSw primary switches, H0-H2 health of cores 0-2 (bit ch: channel ch not OK: 0 GPS pos, 1 GPS vel, 2 baro, 3 compass yaw, 4 airspeed, 5 sideslip)
    // @Field: TimeUS: Time since system startup
    // @Field: C: core number base (0 live, 100 in Replay)
    // @Field: Pri: primary core
    // @Field: Sc0: score of core 0 (filtered NIS per degree of freedom)
    // @Field: Sc1: score of core 1
    // @Field: Sc2: score of core 2
    // @Field: IU: IMUs delivering data (bitmask)
    // @Field: IV: IMUs voted out (bitmask)
    // @Field: MI: compass vote: 0 both used, 1 compass 1 isolated, 2 compass 2 isolated, 3 yaw fusion suspended
    // @Field: NSw: primary core switches since start
    // @Field: H0: health of core 0: bit ch set when channel ch is not OK
    // @Field: H1: health of core 1
    // @Field: H2: health of core 2
    AP::logger().Write(UKFS_DEF,
                       AP::dal().micros64(), DAL_CORE(0), primary,
                       float(cores[0].score()), float(cores[1].score()), float(cores[2].score()),
                       usable, imu_out, compass_isolated, switch_count,
                       cores[0].health_mask(), cores[1].health_mask(), cores[2].health_mask());
#endif
}

#endif // HAL_NAVUKF_AVAILABLE

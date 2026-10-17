#include "AP_NavUKF_core.h"

#if HAL_NAVUKF_AVAILABLE

#include "AP_NavUKF.h"

#include <AP_DAL/AP_DAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

using namespace UKFMath;

// alignment window and at-rest limits (Step 11 item 4)
static constexpr float ALIGN_WINDOW_S = 1.0f;       // seconds of IMU data averaged for tilt
static constexpr float ALIGN_ACCEL_TOL = 0.5f;      // | |f| - g | allowed at rest, m/s^2
static constexpr float ALIGN_GYRO_MAX = 0.1f;       // mean rotation rate allowed at rest, rad/s

// initial standard deviations (ukf_state_model.md Section 7) and floors from the noise table
static constexpr float INIT_SIG_TILT = 0.05f;       // rad
static constexpr float INIT_SIG_YAW = 0.1f;         // rad, max(syaw = 0.05, 0.1)
static constexpr float INIT_SIG_GYRO_BIAS = 0.05f;  // rad/s
static constexpr float INIT_SIG_ACCEL_BIAS = 0.5f;  // m/s^2
static constexpr float INIT_SIG_BARO_BIAS = 1.0f;   // m
static constexpr float INIT_SIG_WIND = 5.0f;        // m/s
static constexpr float FLOOR_SIG_VEL = 0.1f;        // m/s, GPS velocity floor
static constexpr float FLOOR_SIG_POS_H = 0.5f;      // m, GPS horizontal floor
static constexpr float FLOOR_SIG_POS_V = 0.35f;     // m, GPS vertical floor

bool NavUKF_core::read_imu(Vector3F &dAng, ftype &dtA, Vector3F &dVel, ftype &dtV) const
{
    const auto &ins = AP::dal().ins();
    if (ins.get_gyro_count() < 1 || ins.get_accel_count() < 1) {
        return false;
    }
    Vector3f dAngF, dVelF;
    float dtAF, dtVF;
    if (!ins.get_delta_angle(0, dAngF, dtAF) || !ins.get_delta_velocity(0, dVelF, dtVF)) {
        return false;
    }
    if (dtAF < 1.0e-4f || dtVF < 1.0e-4f) {
        return false;
    }
    dAng = dAngF.toftype();
    dVel = dVelF.toftype();
    dtA = dtAF;
    dtV = dtVF;
    return true;
}

void NavUKF_core::reset_alignment(void)
{
    align_f_sum.zero();
    align_w_sum.zero();
    align_time = 0;
    align_count = 0;
}

void NavUKF_core::try_initialise(const Vector3F &dAng, ftype dtA, const Vector3F &dVel, ftype dtV)
{
    align_f_sum += dVel / dtV;
    align_w_sum += dAng / dtA;
    align_time += dtV;
    align_count++;
    if (align_time < ALIGN_WINDOW_S) {
        return;
    }

    const Vector3F f = align_f_sum / ftype(align_count);
    const Vector3F rate = align_w_sum / ftype(align_count);
    reset_alignment();

    // at rest: specific force close to g, little rotation
    if (fabsF(f.length() - ftype(GRAVITY_MSS)) > ALIGN_ACCEL_TOL || rate.length() > ALIGN_GYRO_MAX) {
        return;
    }

    // GPS 3D fix
    const auto &gps = AP::dal().gps();
    const uint8_t gi = gps.primary_sensor();
    if (gps.status(gi) < AP_DAL_GPS::GPS_OK_FIX_3D) {
        return;
    }

    // compasses used for yaw: mean corrected body-frame field
    const auto &compass = AP::dal().compass();
    Vector3F mag;
    uint32_t mag_us;
    if (!read_mag(mag, mag_us)) {
        return;
    }

    // attitude
    ftype roll, pitch;
    UKFModel::tilt_from_accel(f, roll, pitch);
    const ftype yaw = UKFModel::yaw_from_mag(mag, roll, pitch, compass.get_declination());
    QuaternionF q;
    q.from_euler(roll, pitch, yaw);

    // velocity from the GPS, position zero at the origin (this fix)
    origin = gps.location(gi);
    const Vector3F vel = gps.velocity(gi).toftype();

    memset(x, 0, sizeof(x));
    x[0] = q.q1;
    x[1] = q.q2;
    x[2] = q.q3;
    x[3] = q.q4;
    x[4] = vel.x;
    x[5] = vel.y;
    x[6] = vel.z;

    // initial covariance (diagonal)
    float spd_acc = 0, h_acc = 0, v_acc = 0;
    const ftype sig_v = gps.speed_accuracy(gi, spd_acc) ? MAX(spd_acc, FLOOR_SIG_VEL) : FLOOR_SIG_VEL;
    const ftype sig_ph = gps.horizontal_accuracy(gi, h_acc) ? MAX(h_acc, FLOOR_SIG_POS_H) : FLOOR_SIG_POS_H;
    const ftype sig_pv = gps.vertical_accuracy(gi, v_acc) ? MAX(v_acc, FLOOR_SIG_POS_V) : FLOOR_SIG_POS_V;
    const ftype sig[NUM_ERR] = {
        INIT_SIG_TILT, INIT_SIG_TILT, INIT_SIG_YAW,
        sig_v, sig_v, sig_v,
        sig_ph, sig_ph, sig_pv,
        INIT_SIG_GYRO_BIAS, INIT_SIG_GYRO_BIAS, INIT_SIG_GYRO_BIAS,
        INIT_SIG_ACCEL_BIAS, INIT_SIG_ACCEL_BIAS, INIT_SIG_ACCEL_BIAS,
        INIT_SIG_BARO_BIAS,
        INIT_SIG_WIND, INIT_SIG_WIND,
    };
    memset(P, 0, sizeof(P));
    for (uint8_t i = 0; i < NUM_ERR; i++) {
        P[i][i] = sig[i] * sig[i];
    }

    if (!compute_weights(frontend.ut_alpha(), frontend.ut_beta(), frontend.ut_kappa(), NUM_ERR, weights)) {
        return;
    }

    // the samples used here are not fused again; baro reference = altitude now, so b_baro starts at 0
    last_gps_ms = gps.last_message_time_ms(gi);
    last_mag_us = mag_us;
    const auto &baro = AP::dal().baro();
    const uint8_t bi = baro.get_primary();
    have_baro_ref = baro.healthy(bi);
    if (have_baro_ref) {
        baro_ref = baro.get_altitude(bi);
        last_baro_ms = baro.get_last_update(bi);
    }

    last_regularised = 0;
    sigma_fail_count = 0;
    _initialised = true;
}

bool NavUKF_core::read_mag(Vector3F &mag, uint32_t &update_us) const
{
    const auto &compass = AP::dal().compass();
    mag.zero();
    update_us = 0;
    uint8_t nmag = 0;
    for (uint8_t i = 0; i < compass.get_count(); i++) {
        if (compass.use_for_yaw(i) && compass.healthy(i)) {
            mag += compass.get_field(i).toftype();
            update_us = MAX(update_us, compass.last_update_usec(i));
            nmag++;
        }
    }
    if (nmag == 0) {
        return false;
    }
    mag /= ftype(nmag);
    return true;
}

bool NavUKF_core::fuse(UKFModel::Meas t, uint8_t m, const ftype z[MAX_MEAS], const ftype sd[MAX_MEAS])
{
    // fresh sigma points around the current estimate (ukf_state_model.md Section 4)
    if (!sigma_points(x, P, weights, chi, L, last_regularised)) {
        sigma_fail_count++;
        return false;
    }
    const bool vertical = (m == 3);
    for (uint8_t i = 0; i < NUM_SIGMA; i++) {
        UKFModel::measure(t, chi[i], Z[i], vertical);
    }
    ftype R[MAX_MEAS] {};
    for (uint8_t j = 0; j < m; j++) {
        R[j] = sd[j] * sd[j];
    }
    const uint8_t angle_mask = (t == UKFModel::Meas::YAW) ? 1 : 0;
    UpdateResult res;
    if (!unscented_update(x, P, weights, chi, Z, m, z, R, angle_mask, res)) {
        sigma_fail_count++;
        return false;
    }
    Log_Write_Innov(uint8_t(t), res);
    return true;
}

void NavUKF_core::fuse_gps(void)
{
    const auto &gps = AP::dal().gps();
    const uint8_t gi = gps.primary_sensor();
    if (gps.status(gi) < AP_DAL_GPS::GPS_OK_FIX_3D) {
        return;
    }
    const uint32_t t_ms = gps.last_message_time_ms(gi);
    if (t_ms == last_gps_ms) {
        return;
    }
    last_gps_ms = t_ms;

    const UKFModel::MeasNoise r = frontend.meas_noise();
    float h_acc = 0, v_acc = 0, s_acc = 0;
    const ftype sh = gps.horizontal_accuracy(gi, h_acc) ? MAX(ftype(h_acc), r.gps_pos_h) : r.gps_pos_h;
    const ftype sv = gps.vertical_accuracy(gi, v_acc) ? MAX(ftype(v_acc), r.gps_pos_v) : r.gps_pos_v;
    const ftype ss = gps.speed_accuracy(gi, s_acc) ? MAX(ftype(s_acc), r.gps_vel) : r.gps_vel;

    // position: NED offset of the GPS location from the origin (no lever arm, no delay compensation)
    const Vector3d d = origin.get_distance_NED_double(gps.location(gi));
    const ftype zp[MAX_MEAS] = { ftype(d.x), ftype(d.y), ftype(d.z) };
    const ftype sp[MAX_MEAS] = { sh, sh, sv };
    fuse(UKFModel::Meas::GPS_POS, 3, zp, sp);

    // velocity, vertical only if the receiver provides it
    const Vector3F v = gps.velocity(gi).toftype();
    const ftype zv[MAX_MEAS] = { v.x, v.y, v.z };
    const ftype sdv[MAX_MEAS] = { ss, ss, ss };
    fuse(UKFModel::Meas::GPS_VEL, gps.have_vertical_velocity(gi) ? 3 : 2, zv, sdv);
}

void NavUKF_core::fuse_baro(void)
{
    const auto &baro = AP::dal().baro();
    const uint8_t bi = baro.get_primary();
    if (!baro.healthy(bi)) {
        return;
    }
    const uint32_t t_ms = baro.get_last_update(bi);
    if (t_ms == last_baro_ms) {
        return;
    }
    last_baro_ms = t_ms;
    if (!have_baro_ref) {
        // baro was not healthy at initialisation: reference so that b_baro starts at 0
        baro_ref = baro.get_altitude(bi) + x[9];
        have_baro_ref = true;
        return;
    }
    const ftype z[MAX_MEAS] = { ftype(baro.get_altitude(bi)) - baro_ref, 0, 0 };
    const ftype sd[MAX_MEAS] = { frontend.meas_noise().baro, 0, 0 };
    fuse(UKFModel::Meas::BARO, 1, z, sd);
}

void NavUKF_core::fuse_yaw(void)
{
    Vector3F mag;
    uint32_t t_us;
    if (!read_mag(mag, t_us) || t_us == last_mag_us) {
        return;
    }
    last_mag_us = t_us;

    // tilt compensation with the current estimate (ukf_state_model.md 5.1)
    const QuaternionF q(x[0], x[1], x[2], x[3]);
    const ftype yaw = UKFModel::yaw_from_mag(mag, q.get_euler_roll(), q.get_euler_pitch(),
                                             AP::dal().compass().get_declination());
    const ftype z[MAX_MEAS] = { yaw, 0, 0 };
    const ftype sd[MAX_MEAS] = { frontend.meas_noise().yaw, 0, 0 };
    fuse(UKFModel::Meas::YAW, 1, z, sd);
}

void NavUKF_core::predict(const Vector3F &dAng, ftype dtA, const Vector3F &dVel, ftype dtV)
{
    if (!sigma_points(x, P, weights, chi, L, last_regularised)) {
        // P not factorizable: keep the state, skip this step
        sigma_fail_count++;
        return;
    }
    for (uint8_t i = 0; i < NUM_SIGMA; i++) {
        UKFModel::predict_state(chi[i], dAng, dtA, dVel, dtV, chi[i]);
    }
    unscented_mean(chi, weights, x);
    unscented_covariance(chi, x, weights, P);
    UKFModel::add_process_noise(P, frontend.process_noise(), dtA, dtV);
}

void NavUKF_core::update(void)
{
    Vector3F dAng, dVel;
    ftype dtA, dtV;
    if (!read_imu(dAng, dtA, dVel, dtV)) {
        return;
    }
    if (!_initialised) {
        try_initialise(dAng, dtA, dVel, dtV);
        if (_initialised) {
            Log_Write();
        }
        return;
    }
    predict(dAng, dtA, dVel, dtV);
    fuse_gps();
    fuse_baro();
    fuse_yaw();
    Log_Write();
}

#if HAL_LOGGING_ENABLED
// log message definitions: name, labels, units, multipliers, format
#define UKF1_DEF "UKF1", "TimeUS,C,Roll,Pitch,Yaw,VN,VE,VD,PN,PE,PD", "s#ddhnnnmmm", "F-000000000", "QBfffffffff"
#define UKF2_DEF "UKF2", "TimeUS,C,GX,GY,GZ,AX,AY,AZ,BB,WN,WE", "s#EEEooomnn", "F-000000000", "QBfffffffff"
#define UKF3_DEF "UKF3", "TimeUS,C,SR,SP,SY,SVN,SVE,SVD,SPN,SPE,SPD,Reg,Fail", "s#dddnnnmmm--", "F-00000000000", "QBfffffffffBI"
#define UKFN_DEF "UKFN", "TimeUS,C,T,I0,I1,I2,S0,S1,S2,NIS", "s#--------", "F-00000000", "QBBfffffff"

/*
  In Replay, AP_Logger does not emit the FMT record of a Write() message by itself
  (AP_Logger_Backend::ensure_format_emitted trusts Replay to do it), so the parser
  would drop the message. Emit it here, as AP_Scripting does; it is written once.
 */
static void emit_fmt_in_replay(const char *name, const char *labels, const char *units,
                               const char *mults, const char *fmt)
{
#if APM_BUILD_TYPE(APM_BUILD_Replay)
    AP_Logger::log_write_fmt *f = AP::logger().msg_fmt_for_name(name, labels, units, mults, fmt, true);
    if (f != nullptr) {
        AP::logger().Safe_Write_Emit_FMT(f);
    }
#else
    (void)name; (void)labels; (void)units; (void)mults; (void)fmt;
#endif
}
#endif // HAL_LOGGING_ENABLED

void NavUKF_core::Log_Write_Innov(uint8_t type, const UpdateResult &res) const
{
#if HAL_LOGGING_ENABLED
    emit_fmt_in_replay(UKFN_DEF);
    // @LoggerMessage: UKFN
    // @Description: UKF shadow estimator: innovations of one update (T: 0 GPS position, 1 GPS velocity, 2 baro, 3 yaw)
    ftype in[MAX_MEAS] {}, sd[MAX_MEAS] {};
    for (uint8_t j = 0; j < res.m; j++) {
        in[j] = res.innov[j];
        sd[j] = res.innov_sd[j];
    }
    AP::logger().Write(UKFN_DEF,
                       AP::dal().micros64(), DAL_CORE(0), type,
                       float(in[0]), float(in[1]), float(in[2]),
                       float(sd[0]), float(sd[1]), float(sd[2]),
                       float(res.nis));
#else
    (void)type; (void)res;
#endif
}

void NavUKF_core::Log_Write(void) const
{
#if HAL_LOGGING_ENABLED
    emit_fmt_in_replay(UKF1_DEF);
    emit_fmt_in_replay(UKF2_DEF);
    emit_fmt_in_replay(UKF3_DEF);

    const uint64_t time_us = AP::dal().micros64();
    const uint8_t core = DAL_CORE(0);
    const QuaternionF q(x[0], x[1], x[2], x[3]);

    // @LoggerMessage: UKF1
    // @Description: UKF shadow estimator: attitude, velocity, position (NED from the first GPS fix)
    AP::logger().Write(UKF1_DEF,
                       time_us, core,
                       float(degrees(q.get_euler_roll())),
                       float(degrees(q.get_euler_pitch())),
                       float(wrap_360(degrees(q.get_euler_yaw()))),
                       float(x[4]), float(x[5]), float(x[6]),
                       float(x[7]), float(x[8]), float(x[9]));

    // @LoggerMessage: UKF2
    // @Description: UKF shadow estimator: gyro bias, accel bias, baro bias, wind
    AP::logger().Write(UKF2_DEF,
                       time_us, core,
                       float(x[10]), float(x[11]), float(x[12]),
                       float(x[13]), float(x[14]), float(x[15]),
                       float(x[16]), float(x[17]), float(x[18]));

    // @LoggerMessage: UKF3
    // @Description: UKF shadow estimator: standard deviations from P and health counters
    AP::logger().Write(UKF3_DEF,
                       time_us, core,
                       float(degrees(sqrtF(P[0][0]))), float(degrees(sqrtF(P[1][1]))), float(degrees(sqrtF(P[2][2]))),
                       float(sqrtF(P[3][3])), float(sqrtF(P[4][4])), float(sqrtF(P[5][5])),
                       float(sqrtF(P[6][6])), float(sqrtF(P[7][7])), float(sqrtF(P[8][8])),
                       last_regularised, sigma_fail_count);
#endif
}

#endif // HAL_NAVUKF_AVAILABLE

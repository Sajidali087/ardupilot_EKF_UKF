#include "AP_NavUKF_core.h"

#if HAL_NAVUKF_AVAILABLE

#include "AP_NavUKF.h"

#include <AP_DAL/AP_DAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

using namespace UKFMath;

// NIS statistics start after this settling time (Step 12: "after 60 s")
static constexpr uint32_t NIS_SETTLE_US = 60000000U;

// alignment window and at-rest limits (Step 11 item 4)
static constexpr float ALIGN_WINDOW_S = 1.0f;       // seconds of IMU data averaged for tilt
static constexpr float ALIGN_ACCEL_TOL = 0.5f;      // | |f| - g | allowed at rest, m/s^2
static constexpr float ALIGN_GYRO_MAX = 0.1f;       // mean rotation rate allowed at rest, rad/s

// initial standard deviations (ukf_state_model.md Section 7) and floors from the noise table
static constexpr float INIT_SIG_TILT = 0.05f;       // rad
static constexpr float INIT_SIG_YAW = 0.1f;         // rad, max(syaw = 0.05, 0.1)
static constexpr float INIT_SIG_GYRO_BIAS = 0.05f;  // rad/s
static constexpr float INIT_SIG_ACCEL_BIAS_XY = 0.05f;  // m/s^2, body x and y (50 x the 0.001 m/s^2 static offset, noise_table.md)
static constexpr float INIT_SIG_ACCEL_BIAS_Z = 0.5f;    // m/s^2, body z (observable against GPS and baro height)
static constexpr float INIT_SIG_BARO_BIAS = 1.0f;   // m
static constexpr float INIT_SIG_WIND = 5.0f;        // m/s
static constexpr float FLOOR_SIG_VEL = 0.1f;        // m/s, GPS velocity floor
static constexpr float FLOOR_SIG_POS_H = 0.5f;      // m, GPS horizontal floor
static constexpr float FLOOR_SIG_POS_V = 0.35f;     // m, GPS vertical floor

// delayed fusion horizon (ukf_state_model.md Section 12)
static constexpr uint16_t MAX_DELAY_MS = 250;       // largest sensor delay accepted (EKF3 uses the same limit for the GPS)
// accel bias observability (ukf_state_model.md Section 13)
static constexpr float MAN_TAU = 1.0f;              // s, filter of the manoeuvre measures
static constexpr float MAN_RATE = 0.1f;             // rad/s, rotation rate that counts as manoeuvring

static constexpr uint16_t HORIZON_MARGIN_MS = 60;   // a sample reaches the DAL up to 57 ms after its stamp (GPS; baro and compass 21 ms; S1)
static constexpr float OUTPUT_TAU = 0.25f;          // s, output velocity and position tracking time constant (EKF3 EK3_TAU_OUTPUT default)

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

    // sensor delays (the driver-reported GPS lag is known once the receiver is configured)
    if (!setup_delays()) {
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
        INIT_SIG_ACCEL_BIAS_XY, INIT_SIG_ACCEL_BIAS_XY, INIT_SIG_ACCEL_BIAS_Z,
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
    init_time_us = AP::dal().micros64();

    // the state is valid now; the horizon falls behind while the IMU buffer fills
    horizon_us = init_time_us;
    imu_buf.reset();
    gps_buf.reset();
    baro_buf.reset();
    mag_buf.reset();
    dropped_count = 0;
    late_count = 0;
    man_accel = 0;
    man_rate = 0;
    abias_frozen = UKFModel::abias_frozen_axes(q, false);
    reset_output();
    memset(_nis_count, 0, sizeof(_nis_count));
    memset(_nis_inside, 0, sizeof(_nis_inside));
    _initialised = true;
}

bool NavUKF_core::setup_delays(void)
{
    uint16_t gps_ms;
    if (frontend.gps_delay_ms() < 0) {
        // same lag EKF3 uses (AP_NavEKF3_Measurements.cpp readGpsData)
        float lag_s = 0;
        const auto &gps = AP::dal().gps();
        if (!gps.get_lag(gps.primary_sensor(), lag_s)) {
            return false;
        }
        gps_ms = MIN(uint16_t(lag_s * 1000.0f + 0.5f), MAX_DELAY_MS);
    } else {
        gps_ms = MIN(uint16_t(frontend.gps_delay_ms()), MAX_DELAY_MS);
    }
    const uint16_t baro_ms = MIN(uint16_t(MAX(frontend.baro_delay_ms(), int16_t(0))), MAX_DELAY_MS);
    const uint16_t mag_ms = MIN(uint16_t(MAX(frontend.mag_delay_ms(), int16_t(0))), MAX_DELAY_MS);
    gps_delay_us = gps_ms * 1000U;
    baro_delay_us = baro_ms * 1000U;
    mag_delay_us = mag_ms * 1000U;
    // the horizon stays behind the time of validity of every sample still to arrive
    _horizon_delay_ms = MAX(gps_ms, MAX(baro_ms, mag_ms)) + HORIZON_MARGIN_MS;
    return true;
}

// time of validity in DAL microseconds of a sample received at t_ms (DAL milliseconds)
static uint64_t valid_from_ms(uint32_t t_ms, uint32_t delay_us)
{
    const uint64_t t_us = uint64_t(t_ms) * 1000U;
    return t_us > delay_us ? t_us - delay_us : 0;
}

void NavUKF_core::read_sensors(void)
{
    const uint64_t now_us = AP::dal().micros64();

    // GPS: new fix on the primary receiver
    const auto &gps = AP::dal().gps();
    const uint8_t gi = gps.primary_sensor();
    if (gps.status(gi) >= AP_DAL_GPS::GPS_OK_FIX_3D && gps.last_message_time_ms(gi) != last_gps_ms) {
        last_gps_ms = gps.last_message_time_ms(gi);
        const UKFModel::MeasNoise r = frontend.meas_noise();
        float h_acc = 0, v_acc = 0, s_acc = 0;
        GpsSample g;
        g.sd_pos_h = gps.horizontal_accuracy(gi, h_acc) ? MAX(ftype(h_acc), r.gps_pos_h) : r.gps_pos_h;
        g.sd_pos_v = gps.vertical_accuracy(gi, v_acc) ? MAX(ftype(v_acc), r.gps_pos_v) : r.gps_pos_v;
        g.sd_vel = gps.speed_accuracy(gi, s_acc) ? MAX(ftype(s_acc), r.gps_vel) : r.gps_vel;
        // position: NED offset of the GPS location from the origin (no lever arm)
        const Vector3d d = origin.get_distance_NED_double(gps.location(gi));
        g.pos[0] = d.x;
        g.pos[1] = d.y;
        g.pos[2] = d.z;
        const Vector3F v = gps.velocity(gi).toftype();
        g.vel[0] = v.x;
        g.vel[1] = v.y;
        g.vel[2] = v.z;
        g.have_vz = gps.have_vertical_velocity(gi);
        g.valid_us = valid_from_ms(last_gps_ms, gps_delay_us);
        if (!gps_buf.push(g)) {
            dropped_count++;
        }
    }

    // baro: new sample of the primary barometer
    const auto &baro = AP::dal().baro();
    const uint8_t bi = baro.get_primary();
    if (baro.healthy(bi) && baro.get_last_update(bi) != last_baro_ms) {
        last_baro_ms = baro.get_last_update(bi);
        BaroSample b;
        b.alt = baro.get_altitude(bi);
        b.valid_us = valid_from_ms(last_baro_ms, baro_delay_us);
        if (!baro_buf.push(b)) {
            dropped_count++;
        }
    }

    // compass: new mean field of the yaw compasses (32-bit microsecond stamp, converted wrap-safe)
    MagSample m;
    uint32_t t_us;
    if (read_mag(m.field, t_us) && t_us != last_mag_us) {
        last_mag_us = t_us;
        const uint64_t t64 = now_us - uint32_t(uint32_t(now_us) - t_us);
        m.valid_us = t64 > mag_delay_us ? t64 - mag_delay_us : 0;
        if (!mag_buf.push(m)) {
            dropped_count++;
        }
    }
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
    if (!unscented_update(x, P, weights, chi, Z, m, z, R, angle_mask, res, uint32_t(abias_frozen) << 12)) {
        sigma_fail_count++;
        return false;
    }
    const bool inside = UKFModel::nis_inside_bounds(res.nis, m);
    if (AP::dal().micros64() - init_time_us >= NIS_SETTLE_US) {
        _nis_count[uint8_t(t)]++;
        if (inside) {
            _nis_inside[uint8_t(t)]++;
        }
    }
    Log_Write_Innov(uint8_t(t), res, inside);
    return true;
}

void NavUKF_core::fuse_gps(const GpsSample &g)
{
    const ftype zp[MAX_MEAS] = { g.pos[0], g.pos[1], g.pos[2] };
    const ftype sp[MAX_MEAS] = { g.sd_pos_h, g.sd_pos_h, g.sd_pos_v };
    fuse(UKFModel::Meas::GPS_POS, 3, zp, sp);

    // velocity, vertical only if the receiver provides it
    const ftype zv[MAX_MEAS] = { g.vel[0], g.vel[1], g.vel[2] };
    const ftype sdv[MAX_MEAS] = { g.sd_vel, g.sd_vel, g.sd_vel };
    fuse(UKFModel::Meas::GPS_VEL, g.have_vz ? 3 : 2, zv, sdv);
}

void NavUKF_core::fuse_baro(const BaroSample &b)
{
    if (!have_baro_ref) {
        // baro was not healthy at initialisation: reference so that b_baro starts at 0
        baro_ref = b.alt + x[9];
        have_baro_ref = true;
        return;
    }
    const ftype z[MAX_MEAS] = { b.alt - baro_ref, 0, 0 };
    const ftype sd[MAX_MEAS] = { frontend.meas_noise().baro, 0, 0 };
    fuse(UKFModel::Meas::BARO, 1, z, sd);
}

void NavUKF_core::fuse_yaw(const MagSample &m)
{
    // tilt compensation with the horizon estimate (ukf_state_model.md 5.1)
    const QuaternionF q(x[0], x[1], x[2], x[3]);
    const ftype yaw = UKFModel::yaw_from_mag(m.field, q.get_euler_roll(), q.get_euler_pitch(),
                                             AP::dal().compass().get_declination());
    const ftype z[MAX_MEAS] = { yaw, 0, 0 };
    const ftype sd[MAX_MEAS] = { frontend.meas_noise().yaw, 0, 0 };
    fuse(UKFModel::Meas::YAW, 1, z, sd);
}

void NavUKF_core::count_late(uint64_t valid_us, uint32_t half_dt_us)
{
    if (horizon_us > valid_us + 3 * half_dt_us) {
        // the horizon had passed its time of validity by more than 1.5 steps: it arrived too late
        late_count++;
    }
}

void NavUKF_core::fuse_due(uint32_t half_dt_us)
{
    // same order as Part 1: GPS position and velocity, baro, yaw
    while (!gps_buf.empty() && UKFBuffer::meas_due(gps_buf.oldest().valid_us, horizon_us, half_dt_us)) {
        count_late(gps_buf.oldest().valid_us, half_dt_us);
        fuse_gps(gps_buf.oldest());
        gps_buf.pop();
    }
    while (!baro_buf.empty() && UKFBuffer::meas_due(baro_buf.oldest().valid_us, horizon_us, half_dt_us)) {
        count_late(baro_buf.oldest().valid_us, half_dt_us);
        fuse_baro(baro_buf.oldest());
        baro_buf.pop();
    }
    while (!mag_buf.empty() && UKFBuffer::meas_due(mag_buf.oldest().valid_us, horizon_us, half_dt_us)) {
        count_late(mag_buf.oldest().valid_us, half_dt_us);
        fuse_yaw(mag_buf.oldest());
        mag_buf.pop();
    }
}

void NavUKF_core::reset_output(void)
{
    memcpy(xout, x, sizeof(xout));
    out_dang_corr.zero();
    out_vel_int.zero();
    out_pos_int.zero();
}

void NavUKF_core::predict_output(const Vector3F &dAng, ftype dtA, const Vector3F &dVel, ftype dtV)
{
    // strapdown step of the output state with the horizon biases and the tracking correction
    ftype xo[NUM_STATES];
    memcpy(xo, xout, sizeof(xout));
    memcpy(&xo[10], &x[10], sizeof(ftype) * (NUM_STATES - 10));
    UKFModel::predict_state(xo, dAng + out_dang_corr, dtA, dVel, dtV, xo);
    memcpy(xout, xo, sizeof(xout));
}

void NavUKF_core::correct_output(const ImuSample &d)
{
    const ftype dt = d.dtV;

    // attitude error in the body frame of the output at the horizon time
    const QuaternionF qo(d.out[0], d.out[1], d.out[2], d.out[3]);
    const QuaternionF qh(x[0], x[1], x[2], x[3]);
    QuaternionF qe = qo.inverse() * qh;
    qe.normalize();
    const ftype sgn = qe.q1 >= 0 ? ftype(2) : ftype(-2);
    const Vector3F ang_err(sgn * qe.q2, sgn * qe.q3, sgn * qe.q4);

    // gain for a damping ratio of about 0.7 at this delay (EKF3 calcOutputStates)
    const ftype delay = MAX(ftype(AP::dal().micros64() - horizon_us) * ftype(1.0e-6), dt);
    out_dang_corr = ang_err * (ftype(0.5) / delay) * dt;

    // velocity and position: PI tracking with time constant OUTPUT_TAU
    const Vector3F vel_err(x[4] - d.out[4], x[5] - d.out[5], x[6] - d.out[6]);
    const Vector3F pos_err(x[7] - d.out[7], x[8] - d.out[8], x[9] - d.out[9]);
    const ftype gain = dt / constrain_ftype(ftype(OUTPUT_TAU), dt, ftype(10));
    out_vel_int += vel_err;
    out_pos_int += pos_err;
    const Vector3F vel_corr = vel_err * gain + out_vel_int * (sq(gain) * ftype(0.1));
    const Vector3F pos_corr = pos_err * gain + out_pos_int * (sq(gain) * ftype(0.1));

    // apply to the whole output history after the horizon and to the current output
    for (uint8_t i = 0; i < imu_buf.size(); i++) {
        ftype *o = imu_buf[i].out;
        for (uint8_t k = 0; k < 3; k++) {
            o[4 + k] += vel_corr[k];
            o[7 + k] += pos_corr[k];
        }
    }
    for (uint8_t k = 0; k < 3; k++) {
        xout[4 + k] += vel_corr[k];
        xout[7 + k] += pos_corr[k];
    }
}

void NavUKF_core::advance_horizon(void)
{
    const ImuSample s = imu_buf.oldest();
    imu_buf.pop();
    predict(s.dAng, s.dtA, s.dVel, s.dtV);
    horizon_us = s.time_us;
    fuse_due(uint32_t(s.dtV * ftype(0.5e6)));
    if (imu_buf.empty()) {
        // zero delay: the output is the horizon state
        reset_output();
    } else {
        correct_output(s);
    }
}

void NavUKF_core::update_abias_learning(const Vector3F &dAng, ftype dtA, const Vector3F &dVel, ftype dtV)
{
    // horizontal acceleration and rotation rate at the horizon, bias-corrected, low-pass filtered
    const QuaternionF q(x[0], x[1], x[2], x[3]);
    const Vector3F f_body = dVel / dtV - Vector3F(x[13], x[14], x[15]);
    const Vector3F a_ned = q * f_body + Vector3F(0, 0, ftype(UKFModel::GRAVITY_NED_D));
    const Vector3F rate = dAng / dtA - Vector3F(x[10], x[11], x[12]);
    const ftype k = MIN(dtV / ftype(MAN_TAU), ftype(1));
    man_accel += (a_ned.xy().length() - man_accel) * k;
    man_rate += (rate.length() - man_rate) * k;
    const bool manoeuvring = man_accel > frontend.abias_man() || man_rate > ftype(MAN_RATE);
    abias_frozen = UKFModel::abias_frozen_axes(q, manoeuvring);
}

void NavUKF_core::predict(const Vector3F &dAng, ftype dtA, const Vector3F &dVel, ftype dtV)
{
    update_abias_learning(dAng, dtA, dVel, dtV);
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
    UKFModel::add_process_noise(P, frontend.process_noise(), dtA, dtV, abias_frozen);
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
    const uint64_t now_us = AP::dal().micros64();

    // output at the current time, then the sample into the buffer with that output
    predict_output(dAng, dtA, dVel, dtV);
    if (imu_buf.full()) {
        // never drop an IMU sample: bring the horizon forward first
        advance_horizon();
    }
    ImuSample s;
    s.dAng = dAng;
    s.dVel = dVel;
    s.dtA = dtA;
    s.dtV = dtV;
    s.time_us = now_us;
    memcpy(s.out, xout, sizeof(s.out));
    imu_buf.push(s);

    read_sensors();

    // move the horizon to now minus the delay, fusing each measurement at its time of validity
    const uint32_t half_dt_us = uint32_t(dtV * ftype(0.5e6));
    while (!imu_buf.empty() &&
           UKFBuffer::imu_reached_horizon(imu_buf.oldest().time_us, now_us, _horizon_delay_ms * 1000U, half_dt_us)) {
        advance_horizon();
    }
    Log_Write();
}

#if HAL_LOGGING_ENABLED
// log message definitions: name, labels, units, multipliers, format
#define UKF1_DEF "UKF1", "TimeUS,C,Roll,Pitch,Yaw,VN,VE,VD,PN,PE,PD", "s#ddhnnnmmm", "F-000000000", "QBfffffffff"
#define UKF2_DEF "UKF2", "TimeUS,C,GX,GY,GZ,AX,AY,AZ,BB,WN,WE", "s#EEEooomnn", "F-000000000", "QBfffffffff"
#define UKF3_DEF "UKF3", "TimeUS,C,SR,SP,SY,SVN,SVE,SVD,SPN,SPE,SPD,Reg,Fail", "s#dddnnnmmm--", "F-00000000000", "QBfffffffffBI"
#define UKFD_DEF "UKFD", "TimeUS,C,Roll,Pitch,Yaw,VN,VE,VD,PN,PE,PD,Dly,Drop,Late", "s#ddhnnnmmm---", "F-000000000000", "QBfffffffffHII"
#define UKFH_DEF "UKFH", "TimeUS,C,ABF,AH", "s#-o", "F-00", "QBBf"
#define UKFN_DEF "UKFN", "TimeUS,C,T,I0,I1,I2,S0,S1,S2,NIS,In", "s#---------", "F-000000000", "QBBfffffffB"

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

void NavUKF_core::Log_Write_Innov(uint8_t type, const UpdateResult &res, bool inside) const
{
#if HAL_LOGGING_ENABLED
    emit_fmt_in_replay(UKFN_DEF);
    // @LoggerMessage: UKFN
    // @Description: UKF shadow estimator: innovations of one update (T: 0 GPS position, 1 GPS velocity, 2 baro, 3 yaw); In: 1 if NIS is inside the 95 % bounds
    ftype in[MAX_MEAS] {}, sd[MAX_MEAS] {};
    for (uint8_t j = 0; j < res.m; j++) {
        in[j] = res.innov[j];
        sd[j] = res.innov_sd[j];
    }
    AP::logger().Write(UKFN_DEF,
                       AP::dal().micros64(), DAL_CORE(0), type,
                       float(in[0]), float(in[1]), float(in[2]),
                       float(sd[0]), float(sd[1]), float(sd[2]),
                       float(res.nis), uint8_t(inside));
#else
    (void)type; (void)res; (void)inside;
#endif
}

void NavUKF_core::Log_Write(void) const
{
#if HAL_LOGGING_ENABLED
    emit_fmt_in_replay(UKF1_DEF);
    emit_fmt_in_replay(UKF2_DEF);
    emit_fmt_in_replay(UKF3_DEF);
    emit_fmt_in_replay(UKFD_DEF);
    emit_fmt_in_replay(UKFH_DEF);

    const uint64_t time_us = AP::dal().micros64();
    const uint8_t core = DAL_CORE(0);
    const QuaternionF qo(xout[0], xout[1], xout[2], xout[3]);
    const QuaternionF q(x[0], x[1], x[2], x[3]);

    // @LoggerMessage: UKF1
    // @Description: UKF shadow estimator output at the current time: attitude, velocity, position (NED from the first GPS fix)
    AP::logger().Write(UKF1_DEF,
                       time_us, core,
                       float(degrees(qo.get_euler_roll())),
                       float(degrees(qo.get_euler_pitch())),
                       float(wrap_360(degrees(qo.get_euler_yaw()))),
                       float(xout[4]), float(xout[5]), float(xout[6]),
                       float(xout[7]), float(xout[8]), float(xout[9]));

    // @LoggerMessage: UKFD
    // @Description: UKF shadow estimator state at the delayed fusion horizon: attitude, velocity, position; horizon delay (ms); buffered samples lost; measurements fused more than 1.5 steps after their time of validity
    AP::logger().Write(UKFD_DEF,
                       time_us, core,
                       float(degrees(q.get_euler_roll())),
                       float(degrees(q.get_euler_pitch())),
                       float(wrap_360(degrees(q.get_euler_yaw()))),
                       float(x[4]), float(x[5]), float(x[6]),
                       float(x[7]), float(x[8]), float(x[9]),
                       _horizon_delay_ms, dropped_count, late_count);

    // @LoggerMessage: UKFH
    // @Description: UKF shadow estimator observability and health: ABF accel bias axes not learned (bit 0 x, 1 y, 2 z); AH filtered horizontal acceleration
    AP::logger().Write(UKFH_DEF,
                       time_us, core,
                       abias_frozen, float(man_accel));

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

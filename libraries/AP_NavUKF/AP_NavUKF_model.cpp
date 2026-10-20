/*
  UKF shadow estimator for ArduPilot (libraries/AP_NavUKF): process and measurement models

  Copyright (C) 2026 the Client, the party that commissioned this work (see NOTICE in this directory).

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
#include "AP_NavUKF_config.h"

#if HAL_NAVUKF_AVAILABLE

#include "AP_NavUKF_model.h"

namespace UKFModel {

void tilt_from_accel(const Vector3F &f, ftype &roll, ftype &pitch)
{
    // at rest the accelerometer reads -g along the body down axis when level
    roll = atan2F(-f.y, -f.z);
    pitch = atan2F(f.x, sqrtF(f.y * f.y + f.z * f.z));
}

ftype yaw_from_mag(const Vector3F &mag, ftype roll, ftype pitch, ftype declination)
{
    const ftype cr = cosF(roll), sr = sinF(roll);
    const ftype cp = cosF(pitch), sp = sinF(pitch);
    // field rotated into the horizontal plane
    const ftype Xh = mag.x * cp + mag.y * sr * sp + mag.z * cr * sp;
    const ftype Yh = mag.y * cr - mag.z * sr;
    return wrap_PI(atan2F(-Yh, Xh) + declination);
}

void predict_state(const ftype xin[NUM_STATES], const Vector3F &dAng, ftype dtA,
                   const Vector3F &dVel, ftype dtV, ftype xout[NUM_STATES])
{
    const QuaternionF q(xin[0], xin[1], xin[2], xin[3]);
    const Vector3F v(xin[4], xin[5], xin[6]);
    const Vector3F p(xin[7], xin[8], xin[9]);
    const Vector3F bg(xin[10], xin[11], xin[12]);
    const Vector3F ba(xin[13], xin[14], xin[15]);

    // attitude: bias-corrected rotation increment, mid-step attitude for the velocity update
    const Vector3F dth = dAng - bg * dtA;
    const QuaternionF q_mid = q * UKFMath::quat_exp(dth * ftype(0.5));
    QuaternionF q_new = q * UKFMath::quat_exp(dth);
    q_new.normalize();

    // velocity and position
    const Vector3F f_body = dVel / dtV - ba;
    const Vector3F a_ned = q_mid * f_body + Vector3F(0, 0, ftype(GRAVITY_NED_D));
    const Vector3F v_new = v + a_ned * dtV;
    const Vector3F p_new = p + (v + v_new) * (ftype(0.5) * dtV);

    // biases and wind are random walks: copy before overwriting (xout may alias xin)
    for (uint8_t i = 10; i < NUM_STATES; i++) {
        xout[i] = xin[i];
    }
    xout[0] = q_new.q1;
    xout[1] = q_new.q2;
    xout[2] = q_new.q3;
    xout[3] = q_new.q4;
    xout[4] = v_new.x;
    xout[5] = v_new.y;
    xout[6] = v_new.z;
    xout[7] = p_new.x;
    xout[8] = p_new.y;
    xout[9] = p_new.z;
}

void add_process_noise(ftype P[NUM_ERR][NUM_ERR], const ProcessNoise &q, ftype dtA, ftype dtV, uint8_t abias_frozen)
{
    const ftype tau = ftype(NOISE_TAU);
    const ftype att = q.gyro * q.gyro * tau * dtA;
    const ftype vel = q.accel * q.accel * tau * dtV;
    const ftype pos = ftype(1e-9);  // numerical floor only
    const ftype gb = q.gyro_bias * q.gyro_bias * dtA;
    const ftype ab = q.accel_bias * q.accel_bias * dtV;
    const ftype bb = q.baro_bias * q.baro_bias * dtV;
    const ftype wn = q.wind * q.wind * dtV;
    for (uint8_t i = 0; i < 3; i++) {
        P[0 + i][0 + i] += att;
        P[3 + i][3 + i] += vel;
        P[6 + i][6 + i] += pos;
        P[9 + i][9 + i] += gb;
        if (!(abias_frozen & (1U << i))) {
            P[12 + i][12 + i] += ab;
        }
    }
    P[15][15] += bb;
    P[16][16] += wn;
    P[17][17] += wn;
}

uint8_t abias_frozen_axes(const QuaternionF &q, bool manoeuvring)
{
    if (manoeuvring) {
        return 0;
    }
    // row 2 of the body-to-NED rotation: down component of each body axis (EKF3 prevTnb[i][2])
    Matrix3F R;
    q.rotation_matrix(R);
    uint8_t frozen = 0;
    for (uint8_t i = 0; i < 3; i++) {
        if (fabsF(R.c[i]) <= ftype(0.8)) {
            frozen |= (1U << i);
        }
    }
    return frozen;
}

ftype median3(ftype a, ftype b, ftype c)
{
    return MAX(MIN(a, b), MIN(MAX(a, b), c));
}

bool primary_decision(uint8_t primary, const bool healthy[3], const ftype score[3], ftype ratio,
                      int8_t &best, bool &candidate)
{
    best = -1;
    for (uint8_t c = 0; c < 3; c++) {
        if (healthy[c] && (best < 0 || score[c] < score[best])) {
            best = int8_t(c);
        }
    }
    candidate = false;
    if (best < 0) {
        return false;
    }
    if (primary > 2 || !healthy[primary]) {
        return uint8_t(best) != primary;
    }
    candidate = uint8_t(best) != primary && score[best] < ratio * score[primary];
    return false;
}

bool nis_inside_bounds(ftype nis, uint8_t m)
{
    // two-sided 95 % bounds of chi-square with m degrees of freedom (2.5 % and 97.5 % points)
    static const float lo[] = { 0.0f, 0.000982f, 0.0506f, 0.216f };
    static const float hi[] = { 0.0f, 5.024f, 7.378f, 9.348f };
    if (m < 1 || m > 3) {
        return false;
    }
    return nis >= lo[m] && nis <= hi[m];
}

ftype yaw_of_state(const ftype x[NUM_STATES])
{
    const ftype qw = x[0], qx = x[1], qy = x[2], qz = x[3];
    return atan2F(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
}

uint8_t measure(Meas t, const ftype x[NUM_STATES], ftype zeta[UKFMath::MAX_MEAS], bool with_vertical)
{
    switch (t) {
    case Meas::GPS_POS:
        zeta[0] = x[7];
        zeta[1] = x[8];
        zeta[2] = x[9];
        return with_vertical ? 3 : 2;
    case Meas::GPS_VEL:
        zeta[0] = x[4];
        zeta[1] = x[5];
        zeta[2] = x[6];
        return with_vertical ? 3 : 2;
    case Meas::BARO:
        // baro altitude (up) relative to its value at initialisation
        zeta[0] = -x[9] + x[16];
        return 1;
    case Meas::YAW:
        zeta[0] = yaw_of_state(x);
        return 1;
    case Meas::GPS_HGT:
        zeta[0] = x[9];
        return 1;
    case Meas::AIRSPEED:
    case Meas::SIDESLIP: {
        // air-relative velocity in NED (wind horizontal, ukf_state_model.md Section 15)
        const Vector3F va(x[4] - x[17], x[5] - x[18], x[6]);
        if (t == Meas::AIRSPEED) {
            zeta[0] = va.length();
        } else {
            const QuaternionF q(x[0], x[1], x[2], x[3]);
            zeta[0] = (q.inverse() * va).y;
        }
        return 1;
    }
    }
    return 0;
}

} // namespace UKFModel

#endif // HAL_NAVUKF_AVAILABLE

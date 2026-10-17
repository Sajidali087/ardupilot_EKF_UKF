#pragma once

/*
  UKF models without sensor access, so they can be unit tested:
  initial alignment (Step 11 item 4) and strapdown process model (item 5).
  Equations: docs/ukf_state_model.md Sections 3, 3.1, 5.1 and 7.
 */

#include "AP_NavUKF_math.h"

namespace UKFModel {

using UKFMath::NUM_STATES;
using UKFMath::NUM_ERR;

// gravity in NED, m/s^2 (same constant the simulator and EKF3 use)
static constexpr float GRAVITY_NED_D = GRAVITY_MSS;

// interval over which the gyro and accel noise were measured: RISI delta interval (ukf_state_model.md 3.1)
static constexpr float NOISE_TAU = 0.02f;

// process noise settings (standard deviations, see ukf_state_model.md 3.1)
struct ProcessNoise {
    ftype gyro;         // sigma_g, rad/s
    ftype accel;        // sigma_a, m/s^2
    ftype gyro_bias;    // sigma_bg, rad/s per sqrt(s)
    ftype accel_bias;   // sigma_ba, m/s^2 per sqrt(s)
    ftype baro_bias;    // sigma_bb, m per sqrt(s)
    ftype wind;         // sigma_w, m/s per sqrt(s)
};

// roll and pitch (rad) from the mean specific force f (body, m/s^2) at rest
void tilt_from_accel(const Vector3F &f, ftype &roll, ftype &pitch);

// yaw (rad, -pi..pi) from the body-frame magnetic field, tilt-compensated, plus declination (rad, east positive)
ftype yaw_from_mag(const Vector3F &mag, ftype roll, ftype pitch, ftype declination);

/*
  strapdown step for one state vector (ukf_state_model.md Section 3):
  dAng, dVel are the IMU delta angle (rad) and delta velocity (m/s) over dtA, dtV seconds.
  xout may be the same array as xin.
 */
void predict_state(const ftype xin[NUM_STATES], const Vector3F &dAng, ftype dtA,
                   const Vector3F &dVel, ftype dtV, ftype xout[NUM_STATES]);

// add the diagonal process noise of one step to P (error-state covariance)
void add_process_noise(ftype P[NUM_ERR][NUM_ERR], const ProcessNoise &q, ftype dtA, ftype dtV);

} // namespace UKFModel

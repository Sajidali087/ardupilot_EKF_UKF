/*
  UKF shadow estimator for ArduPilot (libraries/AP_NavUKF): unit tests

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
/*
  unit tests for the UKF models (Step 11 items 4 and 5)
  checks planned in docs/ukf_state_model.md Sections 3 and 11:
  tilt and yaw alignment, hover with zero drift, constant yaw rate, constant acceleration
 */
#include <AP_gtest.h>

#include <AP_NavUKF/AP_NavUKF_model.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

using namespace UKFMath;
using namespace UKFModel;

static const ftype TOL = (sizeof(ftype) == 8) ? ftype(1e-9) : ftype(1e-4);

// full-precision angles (SITL compiles with -fsingle-precision-constant)
static const ftype HALF_PI = acosF(ftype(0));
static const ftype G = ftype(GRAVITY_MSS);

static void state_from_euler(ftype x[NUM_STATES], ftype roll, ftype pitch, ftype yaw)
{
    memset(x, 0, sizeof(ftype) * NUM_STATES);
    QuaternionF q;
    q.from_euler(roll, pitch, yaw);
    x[0] = q.q1; x[1] = q.q2; x[2] = q.q3; x[3] = q.q4;
}

// specific force the accelerometer reads at rest with this attitude
static Vector3F rest_specific_force(ftype roll, ftype pitch, ftype yaw)
{
    QuaternionF q;
    q.from_euler(roll, pitch, yaw);
    return q.inverse() * Vector3F(0, 0, -G);
}

TEST(UKFModel, TiltFromAccelRecoversRollPitch)
{
    const ftype cases[][2] = { {0, 0}, {ftype(0.3), 0}, {0, ftype(-0.4)}, {ftype(-0.7), ftype(0.5)}, {ftype(1.2), ftype(-1.0)} };
    for (const auto &c : cases) {
        ftype roll, pitch;
        tilt_from_accel(rest_specific_force(c[0], c[1], ftype(2.0)), roll, pitch);
        EXPECT_NEAR(roll, c[0], TOL);
        EXPECT_NEAR(pitch, c[1], TOL);
    }
}

TEST(UKFModel, YawFromMagRecoversYaw)
{
    // earth field: north and down components, declination 0.2 rad east
    const ftype decl = ftype(0.2);
    const Vector3F field_ned(ftype(0.25), 0, ftype(0.45));
    const ftype cases[][3] = { {0, 0, 0}, {ftype(0.3), ftype(-0.2), ftype(1.0)}, {ftype(-0.5), ftype(0.4), ftype(-2.5)}, {0, 0, ftype(3.0)} };
    for (const auto &c : cases) {
        QuaternionF q;
        q.from_euler(c[0], c[1], c[2]);
        // magnetic north is rotated by the declination from true north
        QuaternionF qd;
        qd.from_euler(0, 0, decl);
        const Vector3F mag_body = q.inverse() * (qd * field_ned);
        const ftype yaw = yaw_from_mag(mag_body, c[0], c[1], decl);
        EXPECT_NEAR(wrap_PI(yaw - c[2]), 0, TOL);
    }
}

TEST(UKFModel, HoverHasZeroDrift)
{
    // level, at rest: dv = -g dt along body down, no rotation -> state unchanged
    ftype x[NUM_STATES], y[NUM_STATES];
    state_from_euler(x, 0, 0, ftype(0.8));
    x[7] = 10; x[8] = -5; x[9] = -100;
    const ftype dt = ftype(0.02);
    memcpy(y, x, sizeof(x));
    for (uint16_t k = 0; k < 3000; k++) {     // 60 s
        predict_state(y, Vector3F(), dt, Vector3F(0, 0, -G * dt), dt, y);
    }
    for (uint8_t i = 0; i < NUM_STATES; i++) {
        EXPECT_NEAR(y[i], x[i], TOL * 100);
    }
}

TEST(UKFModel, TiltedRestHasZeroDrift)
{
    // tilted but at rest: the accelerometer reads the rotated -g, velocity stays zero
    ftype x[NUM_STATES];
    state_from_euler(x, ftype(0.4), ftype(-0.3), ftype(1.5));
    const Vector3F f = rest_specific_force(ftype(0.4), ftype(-0.3), ftype(1.5));
    const ftype dt = ftype(0.02);
    for (uint16_t k = 0; k < 3000; k++) {
        predict_state(x, Vector3F(), dt, f * dt, dt, x);
    }
    for (uint8_t i = 4; i < 10; i++) {
        EXPECT_NEAR(x[i], 0, TOL * 1000);
    }
}

TEST(UKFModel, ConstantYawRate)
{
    // level, 0.1 rad/s about body down for 10 s -> yaw +1 rad, roll and pitch stay 0
    ftype x[NUM_STATES];
    state_from_euler(x, 0, 0, 0);
    const ftype dt = ftype(0.02), rate = ftype(0.1);
    for (uint16_t k = 0; k < 500; k++) {
        predict_state(x, Vector3F(0, 0, rate * dt), dt, Vector3F(0, 0, -G * dt), dt, x);
    }
    // expected values use the elapsed time 500 * dt (dt is a float literal, not exactly 0.02)
    const ftype T = 500 * dt;
    const QuaternionF q(x[0], x[1], x[2], x[3]);
    EXPECT_NEAR(q.get_euler_yaw(), rate * T, TOL * 100);
    EXPECT_NEAR(q.get_euler_roll(), 0, TOL);
    EXPECT_NEAR(q.get_euler_pitch(), 0, TOL);
}

TEST(UKFModel, GyroBiasIsRemoved)
{
    // the gyro reads its bias only: with the bias in the state the attitude does not move
    ftype x[NUM_STATES];
    state_from_euler(x, 0, 0, 0);
    x[10] = ftype(0.01); x[11] = ftype(-0.005); x[12] = ftype(0.005);
    const ftype dt = ftype(0.02);
    const Vector3F dAng = Vector3F(x[10], x[11], x[12]) * dt;
    for (uint16_t k = 0; k < 3000; k++) {
        predict_state(x, dAng, dt, Vector3F(0, 0, -G * dt), dt, x);
    }
    EXPECT_NEAR(x[0], 1, TOL);
    EXPECT_NEAR(Vector3F(x[1], x[2], x[3]).length(), 0, TOL);
}

TEST(UKFModel, ConstantAcceleration)
{
    // yaw 90 deg (nose east), 2 m/s^2 forward for 10 s from 5 m/s north:
    // v = (5, 20, 0), p = (50, 100, 0)
    ftype x[NUM_STATES];
    state_from_euler(x, 0, 0, HALF_PI);
    x[4] = 5;
    const ftype dt = ftype(0.02), a = 2;
    for (uint16_t k = 0; k < 500; k++) {
        predict_state(x, Vector3F(), dt, Vector3F(a * dt, 0, -G * dt), dt, x);
    }
    // expected values use the elapsed time 500 * dt (dt is a float literal, not exactly 0.02)
    const ftype T = 500 * dt;
    EXPECT_NEAR(x[4], 5, TOL * 100);
    EXPECT_NEAR(x[5], a * T, TOL * 100);
    EXPECT_NEAR(x[6], 0, TOL * 100);
    EXPECT_NEAR(x[7], 5 * T, TOL * 1000);
    EXPECT_NEAR(x[8], ftype(0.5) * a * T * T, TOL * 1000);
    EXPECT_NEAR(x[9], 0, TOL * 1000);
}

TEST(UKFModel, ProcessNoiseDiagonal)
{
    static ftype P[NUM_ERR][NUM_ERR];
    memset(P, 0, sizeof(P));
    ProcessNoise q { ftype(3.9e-4), ftype(1.1e-3), ftype(1e-4), ftype(1e-3), ftype(0.028), ftype(0.1) };
    const ftype dt = ftype(0.02);
    add_process_noise(P, q, dt, dt);
    // with dt = tau the attitude and velocity rows are (sigma * dt)^2
    EXPECT_NEAR(P[0][0], q.gyro * dt * q.gyro * dt, TOL * TOL);
    EXPECT_NEAR(P[5][5], q.accel * dt * q.accel * dt, TOL * TOL);
    EXPECT_NEAR(P[9][9], q.gyro_bias * q.gyro_bias * dt, TOL * TOL);
    EXPECT_NEAR(P[15][15], q.baro_bias * q.baro_bias * dt, TOL * TOL);
    EXPECT_NEAR(P[17][17], q.wind * q.wind * dt, TOL * TOL);
    // nothing off the diagonal
    ftype off = 0;
    for (uint8_t i = 0; i < NUM_ERR; i++) {
        for (uint8_t j = 0; j < NUM_ERR; j++) {
            if (i != j) {
                off += fabsF(P[i][j]);
            }
        }
    }
    EXPECT_NEAR(off, 0, 0);
}

TEST(UKFModel, AccelBiasFreezeAndProcessNoise)
{
    QuaternionF q;
    // level: body x and y horizontal, frozen; z vertical, learned
    q.from_euler(ftype(0.1), ftype(-0.1), ftype(2.0));
    EXPECT_EQ(abias_frozen_axes(q, false), 0x3);
    EXPECT_EQ(abias_frozen_axes(q, true), 0);
    // nose straight down: body x vertical, y and z horizontal
    q.from_euler(0, -HALF_PI, ftype(1.0));
    EXPECT_EQ(abias_frozen_axes(q, false), 0x6);
    // frozen axes get no accel bias process noise
    static ftype P[NUM_ERR][NUM_ERR];
    memset(P, 0, sizeof(P));
    ProcessNoise pn { ftype(3.9e-4), ftype(1.1e-3), ftype(1e-4), ftype(1e-3), ftype(0.028), ftype(0.1) };
    const ftype dt = ftype(0.02);
    add_process_noise(P, pn, dt, dt, 0x3);
    EXPECT_NEAR(P[12][12], 0, 0);
    EXPECT_NEAR(P[13][13], 0, 0);
    EXPECT_NEAR(P[14][14], pn.accel_bias * pn.accel_bias * dt, TOL * TOL);
}

TEST(UKFModel, AirspeedAndSideslip)
{
    // flying north-east at 25 m/s with 2 m/s climb in an 8 m/s wind from the west (blowing east)
    ftype x[NUM_STATES];
    const ftype yaw = ftype(0.6);
    state_from_euler(x, ftype(0.1), ftype(0.05), yaw);
    const Vector3F va_ned(ftype(25) * cosF(yaw), ftype(25) * sinF(yaw), ftype(-2));
    x[17] = 0;          // wind north
    x[18] = 8;          // wind east
    x[4] = va_ned.x + x[17];
    x[5] = va_ned.y + x[18];
    x[6] = va_ned.z;
    ftype z[UKFMath::MAX_MEAS];
    EXPECT_EQ(measure(Meas::AIRSPEED, x, z, false), 1);
    EXPECT_NEAR(z[0], va_ned.length(), TOL);
    // air velocity along the heading: no lateral body component when the roll is about the air velocity
    // (coordinated); here roll is about the body x axis, which is not exactly along va, so build it level
    state_from_euler(x, 0, 0, yaw);
    x[17] = 0; x[18] = 8;
    x[4] = ftype(25) * cosF(yaw) + x[17];
    x[5] = ftype(25) * sinF(yaw) + x[18];
    x[6] = 0;
    EXPECT_EQ(measure(Meas::SIDESLIP, x, z, false), 1);
    EXPECT_NEAR(z[0], 0, TOL);
    // 10 deg of crab: lateral airspeed 25 sin(10 deg), positive to the right when the nose is left of the air velocity
    state_from_euler(x, 0, 0, yaw - HALF_PI / 9);
    x[17] = 0; x[18] = 8;
    x[4] = ftype(25) * cosF(yaw) + x[17];
    x[5] = ftype(25) * sinF(yaw) + x[18];
    x[6] = 0;
    measure(Meas::SIDESLIP, x, z, false);
    EXPECT_NEAR(z[0], ftype(25) * sinF(HALF_PI / 9), TOL * 100);
}

TEST(UKFModel, GpsHeightOnly)
{
    // Step 17: with GPS horizontal position switched off the GPS height is fused alone, h = p_D
    ftype x[NUM_STATES];
    state_from_euler(x, 0, 0, 0);
    x[7] = 12; x[8] = -3; x[9] = -95;
    ftype z[UKFMath::MAX_MEAS];
    EXPECT_EQ(measure(Meas::GPS_HGT, x, z, false), 1);
    EXPECT_NEAR(z[0], ftype(-95), TOL);
}

TEST(UKFModel, MeasurementModelsBasic)
{
    // GPS position and velocity (3D and 2D), baro with its bias, yaw of the attitude (Step 11 models)
    ftype x[NUM_STATES];
    state_from_euler(x, ftype(0.1), ftype(-0.2), ftype(2.5));
    x[4] = 21; x[5] = -4; x[6] = ftype(1.5);
    x[7] = 120; x[8] = -35; x[9] = -80;
    x[16] = ftype(1.25);
    ftype z[UKFMath::MAX_MEAS];
    EXPECT_EQ(measure(Meas::GPS_POS, x, z, true), 3);
    EXPECT_NEAR(z[0], ftype(120), TOL);
    EXPECT_NEAR(z[1], ftype(-35), TOL);
    EXPECT_NEAR(z[2], ftype(-80), TOL);
    EXPECT_EQ(measure(Meas::GPS_POS, x, z, false), 2);
    EXPECT_EQ(measure(Meas::GPS_VEL, x, z, true), 3);
    EXPECT_NEAR(z[0], ftype(21), TOL);
    EXPECT_NEAR(z[1], ftype(-4), TOL);
    EXPECT_NEAR(z[2], ftype(1.5), TOL);
    EXPECT_EQ(measure(Meas::GPS_VEL, x, z, false), 2);
    // baro altitude is up: -p_D plus the baro bias
    EXPECT_EQ(measure(Meas::BARO, x, z, false), 1);
    EXPECT_NEAR(z[0], ftype(81.25), TOL);
    // yaw of the attitude, independent of roll and pitch, wrapped to (-pi, pi]
    EXPECT_EQ(measure(Meas::YAW, x, z, false), 1);
    EXPECT_NEAR(z[0], ftype(2.5), TOL);
    EXPECT_NEAR(yaw_of_state(x), ftype(2.5), TOL);
    state_from_euler(x, ftype(0.3), ftype(0.2), ftype(-3));
    EXPECT_NEAR(yaw_of_state(x), ftype(-3), TOL);
}

TEST(UKFModel, MedianVote)
{
    // the median ignores one outlier whichever IMU it is
    EXPECT_NEAR(median3(ftype(0.01), ftype(0.012), ftype(0.04)), ftype(0.012), TOL);
    EXPECT_NEAR(median3(ftype(0.04), ftype(0.01), ftype(0.012)), ftype(0.012), TOL);
    EXPECT_NEAR(median3(ftype(0.012), ftype(0.04), ftype(0.01)), ftype(0.012), TOL);
    EXPECT_NEAR(median3(-1, 5, 2), 2, TOL);
    EXPECT_NEAR(median3(3, 3, -7), 3, TOL);
}

TEST(UKFModel, PrimarySelection)
{
    int8_t best;
    bool candidate;
    const ftype ratio = ftype(0.5);
    // all healthy, similar scores: stay
    bool h[3] = { true, true, true };
    ftype s[3] = { ftype(1.2), ftype(1.0), ftype(1.1) };
    EXPECT_FALSE(primary_decision(0, h, s, ratio, best, candidate));
    EXPECT_EQ(best, 1);
    EXPECT_FALSE(candidate);
    // another core clearly better: candidate (switch only after the hold time), not at once
    s[1] = ftype(0.5);
    s[0] = ftype(1.5);
    EXPECT_FALSE(primary_decision(0, h, s, ratio, best, candidate));
    EXPECT_TRUE(candidate);
    // primary unhealthy (IMU voted out): switch at once to the best healthy core, even with a better score
    h[0] = false;
    s[0] = ftype(0.1);
    EXPECT_TRUE(primary_decision(0, h, s, ratio, best, candidate));
    EXPECT_EQ(best, 1);
    // no healthy core: no decision
    h[1] = h[2] = false;
    EXPECT_FALSE(primary_decision(0, h, s, ratio, best, candidate));
    EXPECT_EQ(best, -1);
}

TEST(UKFModel, NisBounds)
{
    // two-sided 95 % chi-square bounds (ukf_state_model.md Section 8)
    EXPECT_TRUE(nis_inside_bounds(1, 1));
    EXPECT_FALSE(nis_inside_bounds(ftype(0.0005), 1));
    EXPECT_FALSE(nis_inside_bounds(ftype(5.1), 1));
    EXPECT_TRUE(nis_inside_bounds(3, 3));
    EXPECT_FALSE(nis_inside_bounds(ftype(0.2), 3));
    EXPECT_FALSE(nis_inside_bounds(ftype(9.4), 3));
    EXPECT_TRUE(nis_inside_bounds(2, 2));
    EXPECT_FALSE(nis_inside_bounds(1, 4));
}

AP_GTEST_MAIN()

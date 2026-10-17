/*
  unit tests for the UKF math helpers (Step 11 item 3)
  checks planned in docs/ukf_state_model.md Section 11
 */
#include <AP_gtest.h>

#include <AP_NavUKF/AP_NavUKF_math.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

using namespace UKFMath;

// comparison tolerance: SITL builds ftype as double
static const ftype TOL = (sizeof(ftype) == 8) ? ftype(1e-9) : ftype(1e-4);

// pi/2 in full ftype precision: SITL compiles with -fsingle-precision-constant,
// so the literal M_PI_2 is rounded to float (error about 4e-8)
static const ftype HALF_PI = acosF(ftype(0));

// deterministic uniform numbers in [-1, 1)
static uint32_t rng_state = 12345;
static ftype rnd()
{
    rng_state = rng_state * 1664525U + 1013904223U;
    return ftype(rng_state >> 8) / ftype(1U << 23) - 1;
}

static QuaternionF random_quat()
{
    return quat_exp(Vector3F(rnd(), rnd(), rnd()) * ftype(2.5));
}

static void random_state(ftype x[NUM_STATES])
{
    const QuaternionF q = random_quat();
    x[0] = q.q1; x[1] = q.q2; x[2] = q.q3; x[3] = q.q4;
    for (uint8_t i = 4; i < NUM_STATES; i++) {
        x[i] = 10 * rnd();
    }
}

// P = D (B B^T / NUM_ERR + I) D: symmetric positive definite with correlations, standard deviations sig
static void random_spd(ftype P[NUM_ERR][NUM_ERR], const ftype sig[NUM_ERR])
{
    static ftype B[NUM_ERR][NUM_ERR];
    for (uint8_t i = 0; i < NUM_ERR; i++) {
        for (uint8_t j = 0; j < NUM_ERR; j++) {
            B[i][j] = rnd();
        }
    }
    for (uint8_t i = 0; i < NUM_ERR; i++) {
        for (uint8_t j = 0; j < NUM_ERR; j++) {
            ftype s = (i == j) ? 1 : 0;
            for (uint8_t k = 0; k < NUM_ERR; k++) {
                s += B[i][k] * B[j][k] / NUM_ERR;
            }
            P[i][j] = sig[i] * s * sig[j];
        }
    }
}

// standard deviations in the units of each error state
static const ftype SIG[NUM_ERR] = {
    0.05, 0.05, 0.1,        // attitude, rad
    1, 1, 0.5,              // velocity, m/s
    5, 5, 3,                // position, m
    0.01, 0.01, 0.01,       // gyro bias, rad/s
    0.2, 0.2, 0.2,          // accel bias, m/s^2
    1,                      // baro bias, m
    3, 3,                   // wind, m/s
};

static ftype max_abs(const ftype *A, uint16_t count)
{
    ftype m = 0;
    for (uint16_t i = 0; i < count; i++) {
        m = MAX(m, fabsF(A[i]));
    }
    return m;
}

TEST(UKFMath, WeightsDefaultConstants)
{
    Weights w;
    ASSERT_TRUE(compute_weights(1, 2, 0, NUM_ERR, w));
    EXPECT_NEAR(w.lambda, 0, TOL);
    EXPECT_NEAR(w.wm0, 0, TOL);
    EXPECT_NEAR(w.wc0, 2, TOL);
    EXPECT_NEAR(w.wi, ftype(1) / 36, TOL);
    EXPECT_NEAR(w.gamma, sqrtF(18), TOL);
    EXPECT_NEAR(w.wm0 + 2 * NUM_ERR * w.wi, 1, TOL);
}

TEST(UKFMath, WeightsSumToOneGeneral)
{
    Weights w;
    ASSERT_TRUE(compute_weights(ftype(0.5), 2, 1, NUM_ERR, w));
    EXPECT_NEAR(w.wm0 + 2 * NUM_ERR * w.wi, 1, TOL);
    // n + lambda <= 0 is rejected
    EXPECT_FALSE(compute_weights(ftype(0.1), 2, -NUM_ERR, NUM_ERR, w));
}

TEST(UKFMath, CholeskyKnown3x3)
{
    const ftype A[9] = { 4, 12, -16,
                         12, 37, -43,
                         -16, -43, 98 };
    const ftype Lexp[9] = { 2, 0, 0,
                            6, 1, 0,
                            -8, 5, 3 };
    ftype L[9];
    ASSERT_TRUE(cholesky(A, L, 3));
    for (uint8_t i = 0; i < 9; i++) {
        EXPECT_NEAR(L[i], Lexp[i], TOL);
    }
}

TEST(UKFMath, CholeskyRandom18AndInPlace)
{
    static ftype A[NUM_ERR][NUM_ERR], L[NUM_ERR][NUM_ERR], LLt[NUM_ERR][NUM_ERR];
    random_spd(A, SIG);
    ASSERT_TRUE(cholesky(&A[0][0], &L[0][0], NUM_ERR));
    for (uint8_t i = 0; i < NUM_ERR; i++) {
        for (uint8_t j = 0; j < NUM_ERR; j++) {
            ftype s = 0;
            for (uint8_t k = 0; k < NUM_ERR; k++) {
                s += L[i][k] * L[j][k];
            }
            LLt[i][j] = s;
            if (j > i) {
                EXPECT_NEAR(L[i][j], 0, 0);    // upper triangle exactly zero
            }
        }
    }
    const ftype scale = max_abs(&A[0][0], NUM_ERR * NUM_ERR);
    for (uint8_t i = 0; i < NUM_ERR; i++) {
        for (uint8_t j = 0; j < NUM_ERR; j++) {
            EXPECT_NEAR(LLt[i][j], A[i][j], TOL * scale);
        }
    }
    // in place gives the same factor
    ASSERT_TRUE(cholesky(&A[0][0], &A[0][0], NUM_ERR));
    for (uint8_t i = 0; i < NUM_ERR; i++) {
        for (uint8_t j = 0; j < NUM_ERR; j++) {
            EXPECT_NEAR(A[i][j], L[i][j], TOL);
        }
    }
}

TEST(UKFMath, CholeskyRejectsIndefinite)
{
    const ftype A[4] = { 1, 2,
                         2, 1 };
    ftype L[4];
    EXPECT_FALSE(cholesky(A, L, 2));
}

TEST(UKFMath, QuatExpKnownRotations)
{
    const QuaternionF q0 = quat_exp(Vector3F());
    EXPECT_NEAR(q0.q1, 1, TOL);
    EXPECT_NEAR(q0.q2, 0, TOL);
    EXPECT_NEAR(q0.q3, 0, TOL);
    EXPECT_NEAR(q0.q4, 0, TOL);

    // +90 deg about down: yaw +90 deg, body x axis points east
    const QuaternionF qz = quat_exp(Vector3F(0, 0, HALF_PI));
    EXPECT_NEAR(qz.q1, cosF(HALF_PI / 2), TOL);
    EXPECT_NEAR(qz.q4, sinF(HALF_PI / 2), TOL);
    EXPECT_NEAR(qz.get_euler_yaw(), HALF_PI, TOL);
    const Vector3F east = qz * Vector3F(1, 0, 0);
    EXPECT_NEAR(east.x, 0, TOL);
    EXPECT_NEAR(east.y, 1, TOL);
    EXPECT_NEAR(east.z, 0, TOL);
}

TEST(UKFMath, QuatExpSeriesMatchesClosedForm)
{
    // the series branch (|r| < 1e-4) must agree with cos/sin evaluated directly
    for (const ftype a : { ftype(1e-12), ftype(1e-7), ftype(5e-5), ftype(2e-4) }) {
        const Vector3F u = Vector3F(rnd(), rnd(), rnd()).normalized();
        const QuaternionF q = quat_exp(u * a);
        EXPECT_NEAR(q.q1, cosF(a / 2), TOL);
        EXPECT_NEAR(q.q2, sinF(a / 2) * u.x, TOL);
        EXPECT_NEAR(q.q3, sinF(a / 2) * u.y, TOL);
        EXPECT_NEAR(q.q4, sinF(a / 2) * u.z, TOL);
    }
}

TEST(UKFMath, QuatLogInvertsExp)
{
    for (const ftype a : { ftype(0), ftype(1e-12), ftype(1e-7), ftype(1e-3), ftype(0.5), ftype(2), ftype(3.1) }) {
        const Vector3F u = Vector3F(rnd(), rnd(), rnd()).normalized();
        const Vector3F r = u * a;
        const QuaternionF q = quat_exp(r);
        const Vector3F back = quat_log(q);
        EXPECT_NEAR(back.x, r.x, TOL);
        EXPECT_NEAR(back.y, r.y, TOL);
        EXPECT_NEAR(back.z, r.z, TOL);
        // -q is the same rotation
        const QuaternionF qn(-q.q1, -q.q2, -q.q3, -q.q4);
        const Vector3F backn = quat_log(qn);
        EXPECT_NEAR((backn - r).length(), 0, TOL);
    }
}

TEST(UKFMath, StatePlusMinusRoundTrip)
{
    ftype x[NUM_STATES], y[NUM_STATES], dx[NUM_ERR], back[NUM_ERR];
    for (uint8_t trial = 0; trial < 20; trial++) {
        random_state(x);
        const Vector3F att = Vector3F(rnd(), rnd(), rnd()).normalized() * (ftype(3) * fabsF(rnd()));
        dx[0] = att.x; dx[1] = att.y; dx[2] = att.z;
        for (uint8_t i = 3; i < NUM_ERR; i++) {
            dx[i] = 5 * rnd();
        }
        state_plus(x, dx, y);
        const ftype qnorm = sqrtF(y[0] * y[0] + y[1] * y[1] + y[2] * y[2] + y[3] * y[3]);
        EXPECT_NEAR(qnorm, 1, TOL);
        state_minus(y, x, back);
        for (uint8_t i = 0; i < NUM_ERR; i++) {
            EXPECT_NEAR(back[i], dx[i], TOL * 10);
        }
        state_minus(x, x, back);
        EXPECT_NEAR(max_abs(back, NUM_ERR), 0, TOL);
    }
}

TEST(UKFMath, StatePlusAttitudeErrorIsBodyFrame)
{
    // yaw 90 deg, then a body-frame roll error of 0.1 rad: Euler roll 0.1, pitch 0, yaw 90 deg
    ftype x[NUM_STATES] = {};
    const QuaternionF q = quat_exp(Vector3F(0, 0, HALF_PI));
    x[0] = q.q1; x[1] = q.q2; x[2] = q.q3; x[3] = q.q4;
    ftype dx[NUM_ERR] = {};
    dx[0] = ftype(0.1);
    ftype y[NUM_STATES];
    state_plus(x, dx, y);
    const QuaternionF qy(y[0], y[1], y[2], y[3]);
    EXPECT_NEAR(qy.get_euler_roll(), ftype(0.1), TOL);
    EXPECT_NEAR(qy.get_euler_pitch(), 0, TOL);
    EXPECT_NEAR(qy.get_euler_yaw(), HALF_PI, TOL);
}

TEST(UKFMath, SigmaPointsRecoverMeanAndCovariance)
{
    // identity map: unscented mean and covariance must return x and P exactly
    static ftype x[NUM_STATES], P[NUM_ERR][NUM_ERR], L[NUM_ERR][NUM_ERR], chi[NUM_SIGMA][NUM_STATES], xbar[NUM_STATES], Pout[NUM_ERR][NUM_ERR];
    Weights w;
    ASSERT_TRUE(compute_weights(1, 2, 0, NUM_ERR, w));
    for (uint8_t trial = 0; trial < 5; trial++) {
        random_state(x);
        random_spd(P, SIG);
        uint8_t reg = 99;
        ASSERT_TRUE(sigma_points(x, P, w, chi, L, reg));
        EXPECT_EQ(reg, 0);
        unscented_mean(chi, w, xbar);
        ftype d[NUM_ERR];
        state_minus(xbar, x, d);
        EXPECT_NEAR(max_abs(d, NUM_ERR), 0, TOL);
        unscented_covariance(chi, xbar, w, Pout);
        const ftype scale = max_abs(&P[0][0], NUM_ERR * NUM_ERR);
        for (uint8_t i = 0; i < NUM_ERR; i++) {
            for (uint8_t j = 0; j < NUM_ERR; j++) {
                EXPECT_NEAR(Pout[i][j], P[i][j], TOL * scale);
            }
        }
    }
}

TEST(UKFMath, UnscentedLinearMapIsExact)
{
    // y = F(x): position += velocity * dt, wind *= 2, attitude unchanged
    static ftype x[NUM_STATES], P[NUM_ERR][NUM_ERR], L[NUM_ERR][NUM_ERR], chi[NUM_SIGMA][NUM_STATES], ybar[NUM_STATES], Py[NUM_ERR][NUM_ERR];
    static ftype F[NUM_ERR][NUM_ERR], FP[NUM_ERR][NUM_ERR], Pexp[NUM_ERR][NUM_ERR];
    const ftype dt = ftype(0.1);
    Weights w;
    ASSERT_TRUE(compute_weights(1, 2, 0, NUM_ERR, w));
    random_state(x);
    random_spd(P, SIG);
    uint8_t reg;
    ASSERT_TRUE(sigma_points(x, P, w, chi, L, reg));
    for (uint8_t i = 0; i < NUM_SIGMA; i++) {
        for (uint8_t k = 0; k < 3; k++) {
            chi[i][7 + k] += chi[i][4 + k] * dt;
        }
        chi[i][17] *= 2;
        chi[i][18] *= 2;
    }
    unscented_mean(chi, w, ybar);
    unscented_covariance(chi, ybar, w, Py);

    // expected mean
    ftype yexp[NUM_STATES];
    memcpy(yexp, x, sizeof(yexp));
    for (uint8_t k = 0; k < 3; k++) {
        yexp[7 + k] += x[4 + k] * dt;
    }
    yexp[17] *= 2;
    yexp[18] *= 2;
    ftype d[NUM_ERR];
    state_minus(ybar, yexp, d);
    EXPECT_NEAR(max_abs(d, NUM_ERR), 0, TOL * 10);

    // expected covariance F P F^T in error coordinates
    memset(F, 0, sizeof(F));
    for (uint8_t i = 0; i < NUM_ERR; i++) {
        F[i][i] = 1;
    }
    for (uint8_t k = 0; k < 3; k++) {
        F[6 + k][3 + k] = dt;
    }
    F[16][16] = 2;
    F[17][17] = 2;
    for (uint8_t i = 0; i < NUM_ERR; i++) {
        for (uint8_t j = 0; j < NUM_ERR; j++) {
            ftype s = 0;
            for (uint8_t k = 0; k < NUM_ERR; k++) {
                s += F[i][k] * P[k][j];
            }
            FP[i][j] = s;
        }
    }
    for (uint8_t i = 0; i < NUM_ERR; i++) {
        for (uint8_t j = 0; j < NUM_ERR; j++) {
            ftype s = 0;
            for (uint8_t k = 0; k < NUM_ERR; k++) {
                s += FP[i][k] * F[j][k];
            }
            Pexp[i][j] = s;
        }
    }
    const ftype scale = max_abs(&Pexp[0][0], NUM_ERR * NUM_ERR);
    for (uint8_t i = 0; i < NUM_ERR; i++) {
        for (uint8_t j = 0; j < NUM_ERR; j++) {
            EXPECT_NEAR(Py[i][j], Pexp[i][j], TOL * scale);
        }
    }
}

TEST(UKFMath, SigmaPointsRegulariseSingular)
{
    // all-zero P: Cholesky fails, jitter makes it factorizable, every point equals x
    static ftype x[NUM_STATES], P[NUM_ERR][NUM_ERR], L[NUM_ERR][NUM_ERR], chi[NUM_SIGMA][NUM_STATES];
    Weights w;
    ASSERT_TRUE(compute_weights(1, 2, 0, NUM_ERR, w));
    random_state(x);
    memset(P, 0, sizeof(P));
    uint8_t reg = 0;
    ASSERT_TRUE(sigma_points(x, P, w, chi, L, reg));
    EXPECT_GE(reg, 1);
    for (uint8_t i = 0; i < NUM_SIGMA; i++) {
        ftype d[NUM_ERR];
        state_minus(chi[i], x, d);
        EXPECT_NEAR(max_abs(d, NUM_ERR), 0, ftype(1e-6));
    }
}

TEST(UKFMath, SigmaPointsFailNegativeDefinite)
{
    static ftype x[NUM_STATES], P[NUM_ERR][NUM_ERR], L[NUM_ERR][NUM_ERR], chi[NUM_SIGMA][NUM_STATES];
    Weights w;
    ASSERT_TRUE(compute_weights(1, 2, 0, NUM_ERR, w));
    random_state(x);
    memset(P, 0, sizeof(P));
    for (uint8_t i = 0; i < NUM_ERR; i++) {
        P[i][i] = -1;
    }
    uint8_t reg = 0;
    EXPECT_FALSE(sigma_points(x, P, w, chi, L, reg));
}

TEST(UKFMath, UpdateLinearMatchesKalman)
{
    // GPS-like position measurement: for a linear h the UKF update equals the Kalman update
    static ftype x[NUM_STATES], P[NUM_ERR][NUM_ERR], L[NUM_ERR][NUM_ERR], chi[NUM_SIGMA][NUM_STATES];
    static ftype Z[NUM_SIGMA][MAX_MEAS], Pk[NUM_ERR][NUM_ERR];
    Weights w;
    ASSERT_TRUE(compute_weights(1, 2, 0, NUM_ERR, w));
    random_state(x);
    random_spd(P, SIG);
    uint8_t reg;
    ASSERT_TRUE(sigma_points(x, P, w, chi, L, reg));
    for (uint8_t i = 0; i < NUM_SIGMA; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            Z[i][j] = chi[i][7 + j];
        }
    }
    const ftype z[MAX_MEAS] = { x[7] + 2, x[8] - 3, x[9] + 1 };
    const ftype R[MAX_MEAS] = { ftype(0.25), ftype(0.25), ftype(0.1225) };

    // reference Kalman update with H selecting error states 6..8
    ftype S[3][3], Sinv[3][3], K[NUM_ERR][3], nu[3];
    for (uint8_t a = 0; a < 3; a++) {
        nu[a] = z[a] - x[7 + a];
        for (uint8_t b = 0; b < 3; b++) {
            S[a][b] = P[6 + a][6 + b] + ((a == b) ? R[a] : 0);
        }
    }
    const ftype det = S[0][0] * (S[1][1] * S[2][2] - S[1][2] * S[2][1])
                    - S[0][1] * (S[1][0] * S[2][2] - S[1][2] * S[2][0])
                    + S[0][2] * (S[1][0] * S[2][1] - S[1][1] * S[2][0]);
    for (uint8_t a = 0; a < 3; a++) {
        for (uint8_t b = 0; b < 3; b++) {
            // inverse by cofactors (S symmetric)
            const uint8_t r1 = (b + 1) % 3, r2 = (b + 2) % 3, c1 = (a + 1) % 3, c2 = (a + 2) % 3;
            Sinv[a][b] = (S[r1][c1] * S[r2][c2] - S[r1][c2] * S[r2][c1]) / det;
        }
    }
    ftype nis_ref = 0;
    for (uint8_t a = 0; a < 3; a++) {
        for (uint8_t b = 0; b < 3; b++) {
            nis_ref += nu[a] * Sinv[a][b] * nu[b];
        }
    }
    ftype corr[NUM_ERR];
    for (uint8_t r = 0; r < NUM_ERR; r++) {
        corr[r] = 0;
        for (uint8_t b = 0; b < 3; b++) {
            ftype s = 0;
            for (uint8_t k = 0; k < 3; k++) {
                s += P[r][6 + k] * Sinv[k][b];
            }
            K[r][b] = s;
            corr[r] += s * nu[b];
        }
    }
    for (uint8_t r = 0; r < NUM_ERR; r++) {
        for (uint8_t c = 0; c < NUM_ERR; c++) {
            ftype s = 0;
            for (uint8_t k = 0; k < 3; k++) {
                s += K[r][k] * P[c][6 + k];
            }
            Pk[r][c] = P[r][c] - s;
        }
    }

    ftype xprior[NUM_STATES];
    memcpy(xprior, x, sizeof(x));
    UpdateResult res;
    ASSERT_TRUE(unscented_update(x, P, w, chi, Z, 3, z, R, 0, res));
    EXPECT_NEAR(res.nis, nis_ref, TOL * 100);
    for (uint8_t j = 0; j < 3; j++) {
        EXPECT_NEAR(res.innov[j], nu[j], TOL);
    }
    ftype d[NUM_ERR];
    state_minus(x, xprior, d);
    for (uint8_t r = 0; r < NUM_ERR; r++) {
        EXPECT_NEAR(d[r], corr[r], TOL * 100);
    }
    const ftype scale = max_abs(&Pk[0][0], NUM_ERR * NUM_ERR);
    for (uint8_t r = 0; r < NUM_ERR; r++) {
        for (uint8_t c = 0; c < NUM_ERR; c++) {
            EXPECT_NEAR(P[r][c], Pk[r][c], TOL * scale);
        }
    }
}

TEST(UKFMath, UpdateYawWrapsAtPi)
{
    // estimate yaw +179 deg, measurement -179 deg: innovation is +2 deg, not -358 deg
    static ftype x[NUM_STATES], P[NUM_ERR][NUM_ERR], L[NUM_ERR][NUM_ERR], chi[NUM_SIGMA][NUM_STATES];
    static ftype Z[NUM_SIGMA][MAX_MEAS];
    Weights w;
    ASSERT_TRUE(compute_weights(1, 2, 0, NUM_ERR, w));
    const ftype deg = HALF_PI / 90;
    memset(x, 0, sizeof(x));
    QuaternionF q;
    q.from_euler(0, 0, 179 * deg);
    x[0] = q.q1; x[1] = q.q2; x[2] = q.q3; x[3] = q.q4;
    memset(P, 0, sizeof(P));
    for (uint8_t i = 0; i < NUM_ERR; i++) {
        P[i][i] = ftype(0.01);
    }
    uint8_t reg;
    ASSERT_TRUE(sigma_points(x, P, w, chi, L, reg));
    for (uint8_t i = 0; i < NUM_SIGMA; i++) {
        const QuaternionF qi(chi[i][0], chi[i][1], chi[i][2], chi[i][3]);
        Z[i][0] = qi.get_euler_yaw();
    }
    const ftype z[MAX_MEAS] = { -179 * deg, 0, 0 };
    const ftype R[MAX_MEAS] = { ftype(0.0025), 0, 0 };
    UpdateResult res;
    ASSERT_TRUE(unscented_update(x, P, w, chi, Z, 1, z, R, 1, res));
    EXPECT_NEAR(res.innov[0], 2 * deg, ftype(1e-3));
    // the estimate moves towards 180 deg, across the wrap
    const QuaternionF qn(x[0], x[1], x[2], x[3]);
    EXPECT_GT(fabsF(qn.get_euler_yaw()), 179 * deg);
}

TEST(UKFMath, UpdateRejectsInvalidS)
{
    // a negative measurement variance larger than the prediction spread makes S indefinite
    static ftype x[NUM_STATES], P[NUM_ERR][NUM_ERR], L[NUM_ERR][NUM_ERR], chi[NUM_SIGMA][NUM_STATES];
    static ftype Z[NUM_SIGMA][MAX_MEAS];
    Weights w;
    ASSERT_TRUE(compute_weights(1, 2, 0, NUM_ERR, w));
    random_state(x);
    random_spd(P, SIG);
    uint8_t reg;
    ASSERT_TRUE(sigma_points(x, P, w, chi, L, reg));
    for (uint8_t i = 0; i < NUM_SIGMA; i++) {
        Z[i][0] = chi[i][16];
    }
    ftype xprior[NUM_STATES];
    memcpy(xprior, x, sizeof(x));
    const ftype z[MAX_MEAS] = { x[16] + 1, 0, 0 };
    const ftype R[MAX_MEAS] = { -1000, 0, 0 };
    UpdateResult res;
    EXPECT_FALSE(unscented_update(x, P, w, chi, Z, 1, z, R, 0, res));
    for (uint8_t i = 0; i < NUM_STATES; i++) {
        EXPECT_NEAR(x[i], xprior[i], 0);
    }
}

AP_GTEST_MAIN()

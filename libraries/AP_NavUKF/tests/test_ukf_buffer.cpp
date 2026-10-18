/*
  unit tests for the delayed fusion horizon (Part 2 Step 15.1):
  ring buffer wrap and lookup; prediction over buffered samples equals direct prediction;
  a delayed position measurement on a constant-velocity track gives zero along-track bias
 */
#include <AP_gtest.h>

#include <AP_NavUKF/AP_NavUKF_buffer.h>
#include <AP_NavUKF/AP_NavUKF_model.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

using namespace UKFMath;
using namespace UKFModel;

static const ftype G = ftype(GRAVITY_MSS);

TEST(UKFBuffer, RingWrapAndLookup)
{
    UKFBuffer::Ring<int, 8> r;
    r.reset();
    EXPECT_TRUE(r.empty());
    for (int i = 1; i <= 8; i++) {
        EXPECT_TRUE(r.push(i));
    }
    EXPECT_TRUE(r.full());
    // overwriting the oldest is reported
    for (int i = 9; i <= 20; i++) {
        EXPECT_FALSE(r.push(i));
    }
    EXPECT_EQ(r.size(), 8);
    EXPECT_EQ(r.oldest(), 13);
    EXPECT_EQ(r.newest(), 20);
    for (uint8_t i = 0; i < r.size(); i++) {
        EXPECT_EQ(r[i], 13 + i);
    }
    // pop from the oldest end, across the wrap point
    r.pop();
    r.pop();
    EXPECT_EQ(r.size(), 6);
    EXPECT_EQ(r.oldest(), 15);
    EXPECT_TRUE(r.push(21));
    EXPECT_EQ(r.newest(), 21);
    EXPECT_EQ(r[6], 21);
    while (!r.empty()) {
        r.pop();
    }
    r.pop();    // no-op when empty
    EXPECT_EQ(r.size(), 0);
}

struct Imu {
    Vector3F dAng;
    Vector3F dVel;
    uint64_t time_us;
};

// level flight north with a constant yaw rate and forward acceleration, 50 Hz
static Imu make_sample(uint32_t k)
{
    const ftype dt = ftype(1) / ftype(50);   // exact (0.02 would be a float constant)
    Imu s;
    s.dAng = Vector3F(ftype(0.001) * ftype(k % 7), ftype(-0.0005), ftype(0.004));
    s.dVel = Vector3F(ftype(0.5), ftype(0.01) * ftype(k % 3), -G) * dt;
    s.time_us = 1000000ULL + 20000ULL * k;
    return s;
}

TEST(UKFBuffer, BufferedPredictionEqualsDirect)
{
    const ftype dt = ftype(1) / ftype(50);   // exact (0.02 would be a float constant)
    const uint32_t delay_us = 120000;
    const uint32_t half_dt_us = 10000;
    ftype x0[NUM_STATES] {};
    x0[0] = 1;
    x0[4] = 20;
    x0[10] = ftype(0.002);     // gyro bias
    x0[13] = ftype(0.05);      // accel bias

    // direct: every sample at once; history of states after each sample
    const uint32_t n = 60;
    ftype direct[n][NUM_STATES];
    ftype xd[NUM_STATES];
    memcpy(xd, x0, sizeof(xd));
    for (uint32_t k = 0; k < n; k++) {
        const Imu s = make_sample(k);
        predict_state(xd, s.dAng, dt, s.dVel, dt, xd);
        memcpy(direct[k], xd, sizeof(xd));
    }

    // buffered: samples enter the horizon 120 ms after they arrive
    UKFBuffer::Ring<Imu, 16> buf;
    buf.reset();
    ftype xb[NUM_STATES];
    memcpy(xb, x0, sizeof(xb));
    int32_t last = -1;
    for (uint32_t k = 0; k < n; k++) {
        const Imu s = make_sample(k);
        EXPECT_TRUE(buf.push(s));
        while (!buf.empty() && UKFBuffer::imu_reached_horizon(buf.oldest().time_us, s.time_us, delay_us, half_dt_us)) {
            predict_state(xb, buf.oldest().dAng, dt, buf.oldest().dVel, dt, xb);
            buf.pop();
            last++;
        }
        // the horizon runs exactly 6 samples (120 ms) behind
        if (k >= 6) {
            EXPECT_EQ(last, int32_t(k) - 6);
            EXPECT_EQ(buf.size(), 6);
        }
        if (last >= 0) {
            EXPECT_EQ(memcmp(xb, direct[last], sizeof(xb)), 0);
        }
    }
    // flush: the same final state, bit for bit
    while (!buf.empty()) {
        predict_state(xb, buf.oldest().dAng, dt, buf.oldest().dVel, dt, xb);
        buf.pop();
    }
    EXPECT_EQ(memcmp(xb, direct[n - 1], sizeof(xb)), 0);
}

TEST(UKFBuffer, DelayedPositionHasNoAlongTrackBias)
{
    // constant 20 m/s north, level; GPS at 5 Hz with a 120 ms delay, receive times not on the IMU grid;
    // a fix is seen in the first frame after its stamp, so the horizon keeps one frame of margin
    const ftype dt = ftype(1) / ftype(50);   // exact (0.02 would be a float constant)
    const ftype speed = 20;
    const uint32_t delay_us = 120000;
    const uint32_t horizon_us_delay = delay_us + 20000;
    const uint32_t half_dt_us = 10000;
    const Vector3F dAng(0, 0, 0);
    const Vector3F dVel = Vector3F(0, 0, -G) * dt;

    struct Fix {
        ftype pn;           // true north position at the time of validity
        uint64_t valid_us;
    };
    UKFBuffer::Ring<Imu, 16> imu;
    UKFBuffer::Ring<Fix, 8> gps;
    imu.reset();
    gps.reset();
    ftype x[NUM_STATES] {};
    x[0] = 1;
    x[4] = speed;
    uint64_t horizon_us = 0;
    uint64_t next_rx_us = 1000000;
    uint32_t nfix = 0, nrx = 0;
    ftype sum_comp = 0, sum_uncomp = 0, max_comp = 0;
    for (uint32_t k = 1; k <= 3000; k++) {
        const uint64_t now_us = 20000ULL * k;
        Imu s { dAng, dVel, now_us };
        imu.push(s);
        // fix received before this frame: valid delay_us earlier; offsets step through 0..19 ms
        if (now_us >= next_rx_us) {
            const uint64_t valid_us = next_rx_us - delay_us;
            gps.push(Fix { speed * ftype(valid_us) / ftype(1000000), valid_us });
            // without compensation the fix would be compared with the state now
            sum_uncomp += speed * ftype(valid_us) / ftype(1000000) - speed * ftype(now_us) / ftype(1000000);
            nrx++;
            next_rx_us += 200000 + 7000;
        }
        while (!imu.empty() && UKFBuffer::imu_reached_horizon(imu.oldest().time_us, now_us, horizon_us_delay, half_dt_us)) {
            predict_state(x, imu.oldest().dAng, dt, imu.oldest().dVel, dt, x);
            horizon_us = imu.oldest().time_us;
            imu.pop();
            while (!gps.empty() && UKFBuffer::meas_due(gps.oldest().valid_us, horizon_us, half_dt_us)) {
                const ftype innov = gps.oldest().pn - x[7];
                sum_comp += innov;
                max_comp = MAX(max_comp, fabsF(innov));
                nfix++;
                gps.pop();
            }
        }
    }
    ASSERT_GT(nfix, 250U);
    // fused at the nearest IMU sample: at most half a step of travel, mean close to zero
    EXPECT_LE(max_comp, speed * dt * ftype(0.5) + ftype(1e-6));
    EXPECT_NEAR(sum_comp / ftype(nfix), 0, ftype(0.02));
    // uncompensated: the fix lags by the delay plus the arrival offset (at least 2.4 m)
    EXPECT_LT(sum_uncomp / ftype(nrx), -ftype(2.4));
}
AP_GTEST_MAIN()

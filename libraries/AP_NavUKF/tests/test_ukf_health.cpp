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
  unit tests for the UKF channel health (Part 2 Step 20):
  CUSUM on a ramp and a step, state machine transitions, persistence counting, timeout, reset after isolation
 */
#include <AP_gtest.h>

#include <AP_NavUKF/AP_NavUKF_health.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

using namespace UKFHealth;

static const Config CFG { 25, 5, 10, 3, 30, 1000, 5000, 15000 };

// one 1-D sample with normalised innovation r (NIS = r^2)
static Decision sample(Channel &c, ftype r, uint32_t t)
{
    return c.on_sample(&r, 1, r * r, t, CFG);
}

TEST(UKFHealth, CleanDataStaysOk)
{
    Channel c;
    c.reset();
    for (uint32_t i = 0; i < 2000; i++) {
        const ftype r = ftype(int32_t((i * 7919) % 200) - 100) / ftype(50);   // within +-2 sigma
        EXPECT_EQ(sample(c, r, 100 * i), Decision::FUSE);
        c.on_tick(100 * i, true, CFG);
    }
    EXPECT_EQ(c.state(), State::OK);
    EXPECT_EQ(c.faults(), 0);
    EXPECT_EQ(c.warnings(), 0);
}

TEST(UKFHealth, PersistenceCountsKofN)
{
    Channel c;
    c.reset();
    uint32_t t = 0;
    // 4 gated samples spread over 9 samples: WARNING, no FAULT (k = 5)
    for (uint8_t i = 0; i < 9; i++) {
        const Decision d = sample(c, (i % 2 == 0 && i < 8) ? ftype(10) : ftype(0.5), t += 100);
        if (i % 2 == 0 && i < 8) {
            EXPECT_EQ(d, Decision::REJECT);     // a gated sample is never fused
        }
    }
    EXPECT_EQ(c.state(), State::WARNING);
    EXPECT_EQ(c.faults(), 0);
    // the fifth gated sample, still within the window of 10: FAULT
    sample(c, 10, t += 100);
    EXPECT_EQ(c.state(), State::FAULT);
    EXPECT_EQ(c.reason(), Reason::GATE);
}

TEST(UKFHealth, StepIsolatedThenRecovered)
{
    Channel c;
    c.reset();
    uint32_t t = 0;
    for (uint8_t i = 0; i < 20; i++) {
        sample(c, ftype(0.3), t += 100);
    }
    // step of 20 sigma: FAULT within the k = 5 gated samples (the CUSUM is faster here), then ISOLATED;
    // no step sample is fused
    uint8_t fault_at = 0;
    for (uint8_t i = 1; i <= 5 && fault_at == 0; i++) {
        EXPECT_EQ(sample(c, 20, t += 100), Decision::REJECT);
        if (c.state() == State::FAULT) {
            fault_at = i;
        }
    }
    EXPECT_GT(fault_at, 0);
    EXPECT_EQ(sample(c, 20, t += 100), Decision::REJECT);
    EXPECT_EQ(c.state(), State::ISOLATED);
    State prev;
    EXPECT_TRUE(c.take_change(prev));
    // step removed: consistent for 5 s -> RECOVERING (fused), another 5 s -> OK
    uint32_t t_rec = 0, t_ok = 0;
    for (uint16_t i = 0; i < 200; i++) {
        const Decision d = sample(c, ftype(0.3), t += 100);
        if (c.state() == State::RECOVERING && t_rec == 0) {
            t_rec = t;
            EXPECT_EQ(d, Decision::FUSE);
        }
        if (c.state() == State::OK && t_ok == 0) {
            t_ok = t;
        }
    }
    EXPECT_GT(t_rec, 0U);
    EXPECT_GE(t_ok - t_rec, CFG.recov_ms);
    EXPECT_EQ(c.state(), State::OK);
    EXPECT_EQ(c.faults(), 1);
}

TEST(UKFHealth, CusumCatchesRampBeforeGate)
{
    Channel c;
    c.reset();
    uint32_t t = 0;
    // innovation grows slowly: 0.05 sigma per sample, below the 5-sigma gate until sample 100
    ftype r = 0;
    uint16_t fault_at = 0;
    for (uint16_t i = 0; i < 200 && fault_at == 0; i++) {
        r += ftype(0.05);
        sample(c, r, t += 100);
        if (c.state() == State::FAULT) {
            fault_at = i;
        }
    }
    EXPECT_GT(fault_at, 0);
    EXPECT_LT(r, ftype(5));                 // detected while each sample was still inside the gate
    EXPECT_EQ(c.reason(), Reason::CUSUM);
}

TEST(UKFHealth, TimeoutThenResetOnReturn)
{
    Channel c;
    c.reset();
    uint32_t t = 0;
    for (uint8_t i = 0; i < 20; i++) {
        sample(c, ftype(0.2), t += 100);
        c.on_tick(t, true, CFG);
    }
    // no data for 2 s: FAULT (timeout), then ISOLATED
    for (uint8_t i = 0; i < 20; i++) {
        c.on_tick(t += 100, true, CFG);
    }
    EXPECT_EQ(c.state(), State::ISOLATED);
    EXPECT_EQ(c.reason(), Reason::TIMEOUT);
    // data back with a large innovation (dead reckoning drift): re-admitted at once with a covariance reset
    EXPECT_EQ(sample(c, 40, t += 100), Decision::RESET_AND_FUSE);
    EXPECT_EQ(c.state(), State::RECOVERING);
}

TEST(UKFHealth, LongIsolationResets)
{
    Channel c;
    c.reset();
    uint32_t t = 0;
    // a persistent offset: isolated, never consistent, re-admitted with a reset after 15 s
    bool reset = false;
    for (uint16_t i = 0; i < 300 && !reset; i++) {
        reset = sample(c, 30, t += 100) == Decision::RESET_AND_FUSE;
    }
    EXPECT_TRUE(reset);
    EXPECT_EQ(c.reason(), Reason::RESET);
}

TEST(UKFHealth, NoResetWhenDisabled)
{
    // reset_ms 0 (compass yaw and airspeed while GPS is healthy): a persistent offset stays isolated
    Config cfg = CFG;
    cfg.reset_ms = 0;
    Channel c;
    c.reset();
    uint32_t t = 0;
    for (uint16_t i = 0; i < 600; i++) {
        ftype r = 30;
        EXPECT_NE(c.on_sample(&r, 1, r * r, t += 100, cfg), Decision::RESET_AND_FUSE);
    }
    EXPECT_EQ(c.state(), State::ISOLATED);
    // the offset goes away: re-admitted by consistency (CUSUM capped at 2 h decays in about 20 samples, then 5 s)
    for (uint16_t i = 0; i < 100; i++) {
        ftype r = ftype(0.3);
        c.on_sample(&r, 1, r * r, t += 100, cfg);
    }
    EXPECT_EQ(c.state(), State::RECOVERING);
    EXPECT_EQ(c.reason(), Reason::CONSISTENT);
}

TEST(UKFHealth, QualityWarning)
{
    Channel c;
    c.reset();
    c.on_tick(100, false, CFG);
    EXPECT_EQ(c.state(), State::WARNING);
    EXPECT_EQ(c.reason(), Reason::QUALITY);
    // a warning is advisory: samples are still fused
    EXPECT_EQ(sample(c, ftype(0.5), 200), Decision::FUSE);
}

AP_GTEST_MAIN()

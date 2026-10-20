/*
  UKF shadow estimator for ArduPilot (libraries/AP_NavUKF): per-channel fault detection

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
#pragma once

/*
  sensor channel health for the UKF (Part 2 Step 20, ukf_state_model.md Section 17):
  per channel a state machine OK -> WARNING -> FAULT -> ISOLATED -> RECOVERING -> OK driven by
  an innovation gate, k-of-n persistence, a two-sided CUSUM on the normalised innovations and a data timeout.
  No sensor access, so it can be unit tested (tests/test_ukf_health.cpp).
 */

#include <AP_Math/AP_Math.h>

namespace UKFHealth {

enum class State : uint8_t { OK = 0, WARNING = 1, FAULT = 2, ISOLATED = 3, RECOVERING = 4 };

// the test that caused the last state change
enum class Reason : uint8_t { NONE = 0, GATE = 1, CUSUM = 2, TIMEOUT = 3, QUALITY = 4, VOTE = 5, CONSISTENT = 6, RESET = 7, DRIFT = 8 };

struct Config {
    ftype gate;             // NIS per degree of freedom above which a sample is gated (not fused)
    uint8_t k;              // FAULT when k of the last n samples were gated
    uint8_t n;              // window length, at most 16
    ftype cusum_k;          // CUSUM allowance, innovation standard deviations
    ftype cusum_h;          // CUSUM decision threshold
    uint32_t timeout_ms;    // no data for this long: FAULT (timeout)
    uint32_t recov_ms;      // consistent innovations this long: ISOLATED -> RECOVERING, RECOVERING -> OK
    uint32_t reset_ms;      // isolated this long with data: re-admitted with a covariance reset (0: never)
};

// what the caller does with the current sample
enum class Decision : uint8_t { REJECT = 0, FUSE = 1, RESET_AND_FUSE = 2 };

class Channel {
public:
    void reset(void);

    /*
      one measurement sample: r[0..m-1] innovation / its standard deviation, nis its NIS.
      Returns whether to fuse it; RESET_AND_FUSE: the channel is re-admitted after a long isolation or a timeout,
      the caller inflates the covariance of the states the channel observes before fusing.
     */
    Decision on_sample(const ftype *r, uint8_t m, ftype nis, uint32_t now_ms, const Config &cfg);

    // once per frame: data timeout and sensor-reported quality (quality_ok false: WARNING, still fused);
    // expected false: no data is due now (e.g. airspeed on the ground), no timeout
    void on_tick(uint32_t now_ms, bool quality_ok, const Config &cfg, bool expected = true,
                 Reason quality_reason = Reason::QUALITY);

    State state(void) const { return _state; }
    Reason reason(void) const { return _reason; }
    // CUSUM statistic of the last sample (largest of both sides and all components)
    ftype statistic(void) const { return _stat; }

    // a state change since the last call: returns true once per change with the previous state
    bool take_change(State &previous);

    // counters for the summary
    uint16_t faults(void) const { return _n_fault; }
    uint16_t warnings(void) const { return _n_warn; }
    uint32_t isolated_ms(void) const { return _isolated_ms; }

private:
    void set_state(State s, Reason why, uint32_t now_ms);
    uint8_t gated_in_window(void) const;

    State _state;
    Reason _reason;
    State _reported;            // state at the last take_change()
    uint16_t _window;           // bit i: sample i back was gated
    uint8_t _samples;           // samples seen (window fill)
    ftype _cus_hi[3];
    ftype _cus_lo[3];
    ftype _stat;
    uint32_t _state_since_ms;
    uint32_t _consistent_since_ms;
    uint32_t _last_data_ms;
    uint32_t _last_tick_ms;
    bool _timed_out;
    bool _quality_warn;
    Reason _quality_reason;
    uint16_t _n_fault;
    uint16_t _n_warn;
    uint32_t _isolated_ms;
};

} // namespace UKFHealth

/*
  UKF shadow estimator for ArduPilot (libraries/AP_NavUKF): per-channel fault detection

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

#include "AP_NavUKF_health.h"

namespace UKFHealth {

void Channel::reset(void)
{
    *this = Channel();
}

void Channel::set_state(State s, Reason why, uint32_t now_ms)
{
    if (s == _state) {
        return;
    }
    if (s == State::FAULT) {
        _n_fault++;
    } else if (s == State::WARNING) {
        _n_warn++;
    }
    _state = s;
    _reason = why;
    _state_since_ms = now_ms;
    _consistent_since_ms = 0;
}

uint8_t Channel::gated_in_window(void) const
{
    uint8_t c = 0;
    for (uint8_t i = 0; i < 16; i++) {
        c += (_window >> i) & 1U;
    }
    return c;
}

bool Channel::take_change(State &previous)
{
    if (_state == _reported) {
        return false;
    }
    previous = _reported;
    _reported = _state;
    return true;
}

Decision Channel::on_sample(const ftype *r, uint8_t m, ftype nis, uint32_t now_ms, const Config &cfg)
{
    const uint8_t n = MIN(cfg.n, uint8_t(16));
    const bool returning = _timed_out;
    _last_data_ms = now_ms;
    _timed_out = false;

    // gate and k-of-n window
    const bool gated = m > 0 && nis / ftype(m) > cfg.gate;
    _window = uint16_t((_window << 1) | (gated ? 1U : 0U)) & uint16_t((1U << n) - 1U);
    _samples = MIN(uint8_t(_samples + 1), n);
    const uint8_t count = gated_in_window();

    // two-sided CUSUM per component on the normalised innovation (drifts and ramps)
    _stat = 0;
    for (uint8_t j = 0; j < m && j < 3; j++) {
        // capped at twice the threshold: beyond it the sum carries no information, and the cap bounds the
        // time it needs to decay once the fault has gone (recovery)
        _cus_hi[j] = constrain_ftype(_cus_hi[j] + r[j] - cfg.cusum_k, 0, 2 * cfg.cusum_h);
        _cus_lo[j] = constrain_ftype(_cus_lo[j] - r[j] - cfg.cusum_k, 0, 2 * cfg.cusum_h);
        _stat = MAX(_stat, MAX(_cus_hi[j], _cus_lo[j]));
    }
    const bool cusum_fault = _stat > cfg.cusum_h;
    const bool consistent = !gated && _stat < ftype(0.25) * cfg.cusum_h;
    if (consistent) {
        if (_consistent_since_ms == 0) {
            _consistent_since_ms = now_ms;
        }
    } else {
        _consistent_since_ms = 0;
    }
    const bool consistent_long = _consistent_since_ms != 0 && now_ms - _consistent_since_ms >= cfg.recov_ms;

    switch (_state) {
    case State::OK:
    case State::WARNING:
        if (count >= cfg.k) {
            set_state(State::FAULT, Reason::GATE, now_ms);
        } else if (cusum_fault) {
            set_state(State::FAULT, Reason::CUSUM, now_ms);
        } else if (count >= 2 || _stat > ftype(0.5) * cfg.cusum_h || _quality_warn) {
            set_state(State::WARNING, _quality_warn ? _quality_reason : (count >= 2 ? Reason::GATE : Reason::CUSUM), now_ms);
        } else if (count == 0 && _stat < ftype(0.25) * cfg.cusum_h) {
            set_state(State::OK, Reason::CONSISTENT, now_ms);
        }
        break;
    case State::FAULT:
        // the sample after the detection: the channel is now excluded
        set_state(State::ISOLATED, _reason, now_ms);
        break;
    case State::ISOLATED:
        if (returning) {
            // data back after a timeout (e.g. a GPS outage): re-admit at once with a covariance reset
            set_state(State::RECOVERING, Reason::RESET, now_ms);
            memset(_cus_hi, 0, sizeof(_cus_hi));
            memset(_cus_lo, 0, sizeof(_cus_lo));
            _window = 0;
            return Decision::RESET_AND_FUSE;
        }
        if (consistent_long) {
            set_state(State::RECOVERING, Reason::CONSISTENT, now_ms);
            break;
        }
        if (cfg.reset_ms != 0 && now_ms - _state_since_ms >= cfg.reset_ms) {
            // isolated for long with data present: the state may have drifted while the sensor was excluded
            set_state(State::RECOVERING, Reason::RESET, now_ms);
            memset(_cus_hi, 0, sizeof(_cus_hi));
            memset(_cus_lo, 0, sizeof(_cus_lo));
            _window = 0;
            return Decision::RESET_AND_FUSE;
        }
        break;
    case State::RECOVERING:
        if (count >= cfg.k) {
            set_state(State::FAULT, Reason::GATE, now_ms);
        } else if (cusum_fault) {
            set_state(State::FAULT, Reason::CUSUM, now_ms);
        } else if (consistent_long) {
            set_state(State::OK, Reason::CONSISTENT, now_ms);
        }
        break;
    }

    const bool usable = _state == State::OK || _state == State::WARNING || _state == State::RECOVERING;
    return (usable && !gated) ? Decision::FUSE : Decision::REJECT;
}

void Channel::on_tick(uint32_t now_ms, bool quality_ok, const Config &cfg, bool expected, Reason quality_reason)
{
    _quality_reason = quality_reason;
    if (_last_tick_ms != 0 && (_state == State::FAULT || _state == State::ISOLATED)) {
        _isolated_ms += now_ms - _last_tick_ms;
    }
    _last_tick_ms = now_ms;
    _quality_warn = !quality_ok;
    if (_last_data_ms == 0 || (!expected && !_timed_out)) {
        _last_data_ms = now_ms;
    }
    if (!_timed_out && now_ms - _last_data_ms > cfg.timeout_ms) {
        _timed_out = true;
        if (_state != State::ISOLATED) {
            set_state(State::FAULT, Reason::TIMEOUT, now_ms);
        }
    } else if (_timed_out && _state == State::FAULT) {
        set_state(State::ISOLATED, Reason::TIMEOUT, now_ms);
    }
    if (_quality_warn && _state == State::OK) {
        set_state(State::WARNING, quality_reason, now_ms);
    }
}

} // namespace UKFHealth

#endif // HAL_NAVUKF_AVAILABLE

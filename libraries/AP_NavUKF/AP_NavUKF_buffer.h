/*
  UKF shadow estimator for ArduPilot (libraries/AP_NavUKF): delayed fusion horizon buffers

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
#pragma once

/*
  fixed-size FIFO ring buffers for the delayed fusion horizon (Part 2 Step 15.1):
  IMU samples between the fusion horizon and the current time, and sensor samples
  waiting for the horizon to reach their time of validity. No sensor access, so the
  buffers can be unit tested (tests/test_ukf_buffer.cpp).
 */

#include <stdint.h>

namespace UKFBuffer {

template <typename T, uint8_t N>
class Ring {
public:
    static_assert(N > 0, "empty ring");

    void reset(void) { _head = 0; _count = 0; }

    uint8_t size(void) const { return _count; }
    bool empty(void) const { return _count == 0; }
    bool full(void) const { return _count == N; }

    // append e as the newest element; when full the oldest is overwritten and false is returned
    bool push(const T &e) {
        const bool kept = !full();
        if (!kept) {
            pop();
        }
        _buf[(_head + _count) % N] = e;
        _count++;
        return kept;
    }

    // remove the oldest element (no-op when empty)
    void pop(void) {
        if (_count == 0) {
            return;
        }
        _head = (_head + 1) % N;
        _count--;
    }

    // i-th element counted from the oldest (0) to the newest (size() - 1); caller checks i < size()
    T &operator[](uint8_t i) { return _buf[(_head + i) % N]; }
    const T &operator[](uint8_t i) const { return _buf[(_head + i) % N]; }

    T &oldest(void) { return (*this)[0]; }
    const T &oldest(void) const { return (*this)[0]; }
    T &newest(void) { return (*this)[_count - 1]; }
    const T &newest(void) const { return (*this)[_count - 1]; }

private:
    T _buf[N];
    uint8_t _head;
    uint8_t _count;
};

/*
  timing rules of the fusion horizon (times in microseconds, same clock):
  - an IMU sample stamped t_imu enters the horizon once t_imu + delay <= t_now + dt/2
  - a measurement valid at t_valid is fused after the horizon prediction to t_horizon
    once t_valid <= t_horizon + dt/2, i.e. at the IMU sample nearest to its time of validity
    (EKF3 gets the same mean timing by subtracting dt/2 from the time of validity)
 */
inline bool imu_reached_horizon(uint64_t t_imu, uint64_t t_now, uint32_t delay_us, uint32_t half_dt_us)
{
    return t_imu + delay_us <= t_now + half_dt_us;
}

inline bool meas_due(uint64_t t_valid, uint64_t t_horizon, uint32_t half_dt_us)
{
    return t_valid <= t_horizon + half_dt_us;
}

} // namespace UKFBuffer

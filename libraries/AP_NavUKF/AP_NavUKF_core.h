/*
  UKF shadow estimator for ArduPilot (libraries/AP_NavUKF): one filter core

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

#include "AP_NavUKF_config.h"

#if HAL_NAVUKF_AVAILABLE

#include <AP_Common/Location.h>

#include "AP_NavUKF_math.h"
#include "AP_NavUKF_model.h"
#include "AP_NavUKF_buffer.h"
#include "AP_NavUKF_health.h"

class NavUKF;

/*
  one UKF core on one IMU (Step 19: up to three cores, one per IMU). All sensor data comes through the DAL.
  The filter runs on a delayed fusion horizon (now minus the largest sensor delay);
  an output predictor carries attitude, velocity and position to the current time.
 */
class NavUKF_core {
public:
    NavUKF_core(const NavUKF &_frontend, uint8_t _imu_index) : frontend(_frontend), imu_index(_imu_index) {}

    CLASS_NO_COPY(NavUKF_core);

    // one filter step, called once per DAL frame; imu_usable false: this core's IMU is out, skip the step;
    // primary: this core gives the output (its channel state changes are announced to the GCS)
    void update(bool imu_usable, bool primary);

    // fault detection (Step 20): sensor channels monitored by each core
    enum HealthChannel : uint8_t { CH_GPS_POS = 0, CH_GPS_VEL = 1, CH_BARO = 2, CH_YAW = 3, CH_AIRSPEED = 4, CH_SIDESLIP = 5, NUM_CH = 6 };
    static const char *channel_name(uint8_t ch);
    // the single UKFE writer (channel state changes of the cores and the votes of the frontend)
    static void Log_Write_Event(uint8_t core, uint8_t ch, uint8_t old_state, uint8_t new_state, uint8_t reason,
                                float stat);
    const UKFHealth::Channel &channel(uint8_t ch) const { return health[ch]; }
    // bit ch set: channel ch not OK
    uint8_t health_mask(void) const;

    bool initialised(void) const { return _initialised; }
    uint8_t imu(void) const { return imu_index; }

    // health score for core selection: innovation NIS / dimension, low-pass filtered over all updates (about 1 when consistent)
    ftype score(void) const { return nis_score; }

    // attitude of the output at the current time (rad)
    void output_euler(ftype &roll, ftype &pitch, ftype &yaw) const;

    // output at the current time for the primary-core log (UKFP)
    void Log_Write_Primary(void) const;

    // restart from another core's state when this core's IMU returns in flight: navigation states, covariance, buffers
    // and timing are copied; this IMU's gyro and accel biases restart at zero with their initial uncertainty
    void reset_from(const NavUKF_core &src);

    // NIS monitoring: updates of measurement type t (0..3) counted after the settling time, and how many had NIS inside the 95 % bounds
    uint32_t nis_count(uint8_t t) const { return t < NUM_MEAS_TYPES ? _nis_count[t] : 0; }
    uint32_t nis_inside(uint8_t t) const { return t < NUM_MEAS_TYPES ? _nis_inside[t] : 0; }
    static constexpr uint8_t NUM_MEAS_TYPES = 7;

    // aiding sources (UKF_MOD_MASK bits) and fusion modules (docs/part2/fusion_modules.md)
    enum Source : uint8_t { SRC_GPS_HPOS = 0, SRC_GPS_HGT = 1, SRC_GPS_VEL = 2, SRC_AIRSPEED = 3, SRC_BARO = 4, SRC_COMPASS = 5, NUM_SRC = 6 };
    enum Module : uint8_t { MOD_VELOCITY = 0, MOD_POSITION = 1, MOD_ALTITUDE = 2, MOD_HEADING = 3, NUM_MOD = 4 };
    enum ModuleStatus : uint8_t { MS_AIDED = 0, MS_DEGRADED = 1, MS_UNAIDED = 2 };
    // status of a module: all its switched-on sources fused within the last second, some, or none
    ModuleStatus module_status(Module mod, uint8_t &src_in_use) const;

    // fusion horizon delay chosen at initialisation, ms
    uint16_t horizon_delay_ms(void) const { return _horizon_delay_ms; }

private:
    const NavUKF &frontend;
    const uint8_t imu_index;
    ftype nis_score;
    bool is_primary;

    // fault detection (Step 20, ukf_state_model.md Section 17)
    UKFHealth::Channel health[NUM_CH];
    ftype baro_bias_prev;                   // baro bias at the previous health tick, m
    ftype baro_bias_rate;                   // its rate, low-pass filtered, m/s
    uint32_t health_tick_ms;
    UKFHealth::Config health_config(uint8_t ch) const;
    void health_tick(uint32_t now_ms);
    // inflate the covariance of the states channel ch observes by the squared innovations (re-admission with a reset)
    void inflate_for_channel(uint8_t ch, const UKFMath::UpdateResult &res, UKFModel::Meas t);
    void Log_Write_Health_Event(uint8_t ch, UKFHealth::State old_state) const;

    // nominal state (ukf_state_model.md Section 2) and error-state covariance
    ftype x[UKFMath::NUM_STATES];
    ftype P[UKFMath::NUM_ERR][UKFMath::NUM_ERR];

    // sigma point work space
    ftype chi[UKFMath::NUM_SIGMA][UKFMath::NUM_STATES];
    ftype L[UKFMath::NUM_ERR][UKFMath::NUM_ERR];
    ftype Z[UKFMath::NUM_SIGMA][UKFMath::MAX_MEAS];
    UKFMath::Weights weights;

    // sensor sample stamps already read into the buffers
    uint32_t last_gps_ms;
    uint32_t last_baro_ms;
    uint32_t last_mag_us;
    uint32_t last_arsp_ms;

    // delayed fusion horizon (ukf_state_model.md Section 12)
    struct ImuSample {
        Vector3F dAng;
        Vector3F dVel;
        ftype dtA;
        ftype dtV;
        uint64_t time_us;               // DAL frame time at the end of the delta interval
        ftype out[10];                  // output q, v, p after this sample (state indices 0-9)
    };
    struct GpsSample {
        ftype pos[3];                   // NED from the origin, m
        ftype vel[3];                   // NED, m/s
        ftype sd_pos_h, sd_pos_v, sd_vel;
        bool have_vz;
        uint64_t valid_us;              // time of validity
    };
    struct BaroSample {
        ftype alt;                      // baro altitude, m
        uint64_t valid_us;
    };
    struct MagSample {
        Vector3F field;                 // mean body-frame field of the yaw compasses
        uint64_t valid_us;
    };
    struct AirSample {
        ftype tas;                      // true airspeed, m/s
        uint64_t valid_us;
    };
    static constexpr uint8_t IMU_BUF_LEN = 20;     // 400 ms at 50 Hz: largest delay 250 ms plus the 60 ms arrival margin
    static constexpr uint8_t OBS_BUF_LEN = 8;      // 400 ms of 20 Hz data
    UKFBuffer::Ring<ImuSample, IMU_BUF_LEN> imu_buf;
    UKFBuffer::Ring<GpsSample, OBS_BUF_LEN> gps_buf;
    UKFBuffer::Ring<BaroSample, OBS_BUF_LEN> baro_buf;
    UKFBuffer::Ring<MagSample, OBS_BUF_LEN> mag_buf;
    UKFBuffer::Ring<AirSample, OBS_BUF_LEN> air_buf;
    int32_t gps_delay_us;               // GPS delay minus the stamp offset (negative: data newer than the stamp)
    uint32_t baro_delay_us;
    uint32_t mag_delay_us;
    uint32_t arsp_delay_us;
    uint16_t _horizon_delay_ms;
    uint64_t horizon_us;                // time of validity of x and P
    uint32_t dropped_count;             // buffered samples lost to a full buffer
    uint32_t late_count;                // measurements fused more than dt/2 after their time of validity

    // accel bias observability (ukf_state_model.md Section 13)
    ftype man_accel;                    // filtered horizontal acceleration, m/s^2
    ftype man_rate;                     // filtered rotation rate, rad/s
    uint8_t abias_frozen;               // bit i: body accel bias axis i is not learned

    // baro step detector (ukf_state_model.md Section 14): two-sided CUSUM of the normalised baro innovation
    ftype baro_cusum_hi;
    ftype baro_cusum_lo;
    uint16_t baro_step_count;

    // fusion modules: horizon time each aiding source was last fused
    uint64_t last_src_fuse_us[NUM_SRC];
    uint8_t module_log_div;                 // UKFM is written every 5th frame (10 Hz)

    // output predictor (EKF3-style complementary tracking of the horizon state)
    ftype xout[10];                     // q, v, p at the current time
    Vector3F out_dang_corr;             // delta angle correction per IMU step, rad
    Vector3F out_vel_int;               // velocity tracking error integral, m/s
    Vector3F out_pos_int;               // position tracking error integral, m
    ftype baro_ref;                 // baro altitude at initialisation, m
    bool have_baro_ref;

    bool _initialised;
    Location origin;                // first valid GPS fix: NED origin
    uint64_t init_time_us;          // DAL time of initialisation

    // NIS monitoring (ukf_state_model.md Section 8)
    uint32_t _nis_count[NUM_MEAS_TYPES];
    uint32_t _nis_inside[NUM_MEAS_TYPES];

    // alignment at rest: accelerometer and gyro means over a window
    Vector3F align_f_sum;
    Vector3F align_w_sum;
    ftype align_time;
    uint16_t align_count;

    // health counters
    uint8_t last_regularised;       // jitter retries in the last sigma point set
    uint32_t sigma_fail_count;      // sigma point sets or updates that could not be done

    // read IMU 0 deltas; false if not usable
    bool read_imu(Vector3F &dAng, ftype &dtA, Vector3F &dVel, ftype &dtV) const;

    // accumulate the alignment window and initialise when at rest with GPS 3D fix and compass
    void try_initialise(const Vector3F &dAng, ftype dtA, const Vector3F &dVel, ftype dtV);
    void reset_alignment(void);

    // sensor delays from the parameters (GPS -1: driver lag); false while the GPS lag is unknown
    bool setup_delays(void);

    // which accel bias axes are learned in this step (manoeuvre detection at the horizon)
    void update_abias_learning(const Vector3F &dAng, ftype dtA, const Vector3F &dVel, ftype dtV);

    // sigma point prediction with the IMU deltas, then process noise
    void predict(const Vector3F &dAng, ftype dtA, const Vector3F &dVel, ftype dtV);

    // new GPS, baro, compass and airspeed samples into their buffers, stamped with their time of validity
    void read_sensors(void);
    void read_airspeed(void);

    // predict the horizon state with the oldest buffered IMU sample, fuse what is due, correct the output
    void advance_horizon(void);

    // sequential measurement updates (items 6, 7, 8) on buffered samples
    void fuse_due(uint32_t half_dt_us);
    void count_late(uint64_t valid_us, uint32_t half_dt_us);
    void fuse_gps(const GpsSample &s);
    void fuse_baro(const BaroSample &s);
    void fuse_yaw(const MagSample &s);
    void fuse_airspeed(const AirSample &s);

    // output predictor: propagate with the newest IMU sample; track the horizon state at a delayed sample
    void predict_output(const Vector3F &dAng, ftype dtA, const Vector3F &dVel, ftype dtV);
    void correct_output(const ImuSample &delayed);
    void reset_output(void);

    // one unscented update of measurement t with fresh sigma points; logs UKFN
    // fuse one measurement; ch >= 0: through the health channel ch (gate, persistence, CUSUM, isolation);
    // returns true if it was fused
    bool fuse(UKFModel::Meas t, uint8_t m, const ftype z[UKFMath::MAX_MEAS], const ftype sd[UKFMath::MAX_MEAS],
              UKFMath::UpdateResult *result = nullptr, int8_t ch = -1);

    // CUSUM on a baro innovation; on a detected step the baro bias variance is inflated
    void baro_step_check(const UKFMath::UpdateResult &res);

    // mean body-frame field of the compasses used for yaw and the newest update time; false if none
    bool read_mag(Vector3F &mag, uint32_t &update_us) const;

    // UKF1 (output: attitude, velocity, position), UKFD (the same at the fusion horizon),
    // UKF2 (biases, wind), UKF3 (standard deviations, health)
    void Log_Write(void) const;

    // UKFN: innovations, their standard deviations and NIS of one update
    void Log_Write_Innov(uint8_t type, const UKFMath::UpdateResult &res, bool inside) const;
};

#endif // HAL_NAVUKF_AVAILABLE

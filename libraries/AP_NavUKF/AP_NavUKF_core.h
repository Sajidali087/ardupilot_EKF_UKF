#pragma once

#include "AP_NavUKF_config.h"

#if HAL_NAVUKF_AVAILABLE

#include <AP_Common/Location.h>

#include "AP_NavUKF_math.h"
#include "AP_NavUKF_model.h"

class NavUKF;

/*
  one UKF core on IMU 0 (Part 1). All sensor data comes through the DAL.
 */
class NavUKF_core {
public:
    NavUKF_core(const NavUKF &_frontend) : frontend(_frontend) {}

    CLASS_NO_COPY(NavUKF_core);

    // one filter step, called once per DAL frame
    void update(void);

    bool initialised(void) const { return _initialised; }

    // NIS monitoring: updates of measurement type t (0..3) counted after the settling time, and how many had NIS inside the 95 % bounds
    uint32_t nis_count(uint8_t t) const { return t < NUM_MEAS_TYPES ? _nis_count[t] : 0; }
    uint32_t nis_inside(uint8_t t) const { return t < NUM_MEAS_TYPES ? _nis_inside[t] : 0; }
    static constexpr uint8_t NUM_MEAS_TYPES = 4;

private:
    const NavUKF &frontend;

    // nominal state (ukf_state_model.md Section 2) and error-state covariance
    ftype x[UKFMath::NUM_STATES];
    ftype P[UKFMath::NUM_ERR][UKFMath::NUM_ERR];

    // sigma point work space
    ftype chi[UKFMath::NUM_SIGMA][UKFMath::NUM_STATES];
    ftype L[UKFMath::NUM_ERR][UKFMath::NUM_ERR];
    ftype Z[UKFMath::NUM_SIGMA][UKFMath::MAX_MEAS];
    UKFMath::Weights weights;

    // sensor sample stamps already fused
    uint32_t last_gps_ms;
    uint32_t last_baro_ms;
    uint32_t last_mag_us;
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

    // sigma point prediction with the IMU deltas, then process noise
    void predict(const Vector3F &dAng, ftype dtA, const Vector3F &dVel, ftype dtV);

    // sequential measurement updates on new sensor samples (items 6, 7, 8)
    void fuse_gps(void);
    void fuse_baro(void);
    void fuse_yaw(void);

    // one unscented update of measurement t with fresh sigma points; logs UKFN
    bool fuse(UKFModel::Meas t, uint8_t m, const ftype z[UKFMath::MAX_MEAS], const ftype sd[UKFMath::MAX_MEAS]);

    // mean body-frame field of the compasses used for yaw and the newest update time; false if none
    bool read_mag(Vector3F &mag, uint32_t &update_us) const;

    // UKF1 (attitude, velocity, position), UKF2 (biases, wind), UKF3 (standard deviations, health)
    void Log_Write(void) const;

    // UKFN: innovations, their standard deviations and NIS of one update
    void Log_Write_Innov(uint8_t type, const UKFMath::UpdateResult &res, bool inside) const;
};

#endif // HAL_NAVUKF_AVAILABLE

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

private:
    const NavUKF &frontend;

    // nominal state (ukf_state_model.md Section 2) and error-state covariance
    ftype x[UKFMath::NUM_STATES];
    ftype P[UKFMath::NUM_ERR][UKFMath::NUM_ERR];

    // sigma point work space
    ftype chi[UKFMath::NUM_SIGMA][UKFMath::NUM_STATES];
    ftype L[UKFMath::NUM_ERR][UKFMath::NUM_ERR];
    UKFMath::Weights weights;

    bool _initialised;
    Location origin;                // first valid GPS fix: NED origin

    // alignment at rest: accelerometer and gyro means over a window
    Vector3F align_f_sum;
    Vector3F align_w_sum;
    ftype align_time;
    uint16_t align_count;

    // health counters
    uint8_t last_regularised;       // jitter retries in the last sigma point set
    uint32_t sigma_fail_count;      // sigma point sets that could not be formed

    // read IMU 0 deltas; false if not usable
    bool read_imu(Vector3F &dAng, ftype &dtA, Vector3F &dVel, ftype &dtV) const;

    // accumulate the alignment window and initialise when at rest with GPS 3D fix and compass
    void try_initialise(const Vector3F &dAng, ftype dtA, const Vector3F &dVel, ftype dtV);
    void reset_alignment(void);

    // sigma point prediction with the IMU deltas, then process noise
    void predict(const Vector3F &dAng, ftype dtA, const Vector3F &dVel, ftype dtV);

    // UKF1 (attitude, velocity, position), UKF2 (biases, wind), UKF3 (standard deviations, health)
    void Log_Write(void) const;
};

#endif // HAL_NAVUKF_AVAILABLE

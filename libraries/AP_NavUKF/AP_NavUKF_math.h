#pragma once

/*
  UKF math helpers: Cholesky, quaternion exponential and logarithm,
  state (+)/(-) operators, sigma points, unscented mean and covariance.
  Model and conventions: docs/ukf_state_model.md (Sections 1, 2.1 and 4).
  All matrices are row-major arrays owned by the caller.
 */

#include <AP_Math/AP_Math.h>

namespace UKFMath {

// state sizes (ukf_state_model.md Section 2)
static constexpr uint8_t NUM_STATES = 19;             // nominal state: q(4) v(3) p(3) bg(3) ba(3) bbaro(1) wind(2)
static constexpr uint8_t NUM_ERR = 18;                 // error state: attitude error replaces the quaternion
static constexpr uint8_t NUM_SIGMA = 2 * NUM_ERR + 1;  // sigma points

// unscented transform weights for dimension n
struct Weights {
    ftype lambda;   // alpha^2 (n + kappa) - n
    ftype gamma;    // sqrt(n + lambda): sigma point spread
    ftype wm0;      // mean weight of the centre point
    ftype wc0;      // covariance weight of the centre point
    ftype wi;       // mean and covariance weight of every other point
};

// weights from alpha, beta, kappa; returns false if n + lambda <= 0
bool compute_weights(ftype alpha, ftype beta, ftype kappa, uint8_t n, Weights &w);

// A <- 0.5 (A + A^T), A is n x n
void symmetrize(ftype *A, uint8_t n);

// lower-triangular L with L L^T = A (A is n x n, symmetric); returns false if A is not positive definite
bool cholesky(const ftype *A, ftype *L, uint8_t n);

// unit quaternion of the rotation vector r (rad); exact near zero
QuaternionF quat_exp(const Vector3F &r);

// rotation vector of a unit quaternion, shortest rotation (angle in [0, pi])
Vector3F quat_log(const QuaternionF &q);

// out = x (+) dx: q <- normalize(q (x) exp(dx[0:3])), other state i <- x[i] + dx[i-1]
void state_plus(const ftype x[NUM_STATES], const ftype dx[NUM_ERR], ftype out[NUM_STATES]);

// dx = a (-) b: dx[0:3] = log(conj(q_b) (x) q_a), dx[i-1] = a[i] - b[i]
void state_minus(const ftype a[NUM_STATES], const ftype b[NUM_STATES], ftype dx[NUM_ERR]);

/*
  sigma points chi[0] = x, chi[1+i] = x (+) gamma L[:,i], chi[1+NUM_ERR+i] = x (+) -gamma L[:,i]
  with L L^T = P. P is symmetrized in a copy first. If the Cholesky fails, a small
  multiple of the identity is added and the factorization retried; regularised returns
  how many times that was needed. L is caller-owned work space (NUM_ERR x NUM_ERR).
  Returns false if P could not be factorized.
 */
bool sigma_points(const ftype x[NUM_STATES], const ftype P[NUM_ERR][NUM_ERR], const Weights &w,
                  ftype chi[NUM_SIGMA][NUM_STATES], ftype L[NUM_ERR][NUM_ERR], uint8_t &regularised);

/*
  weighted mean of sigma points: arithmetic for every state after the quaternion,
  iterative quaternion mean for the attitude, started from chi[0]
 */
void unscented_mean(const ftype chi[NUM_SIGMA][NUM_STATES], const Weights &w, ftype xbar[NUM_STATES]);

// P = sum Wc_i (chi_i (-) xbar)(chi_i (-) xbar)^T, symmetrized; process noise is added by the caller
void unscented_covariance(const ftype chi[NUM_SIGMA][NUM_STATES], const ftype xbar[NUM_STATES], const Weights &w,
                          ftype P[NUM_ERR][NUM_ERR]);

// largest measurement dimension of one sequential update (GPS position or velocity)
static constexpr uint8_t MAX_MEAS = 3;

// innovation statistics of one update
struct UpdateResult {
    uint8_t m;                  // measurement dimension
    ftype innov[MAX_MEAS];      // z - z_bar (wrapped for angles)
    ftype innov_sd[MAX_MEAS];   // sqrt of the diagonal of S
    ftype nis;                  // nu^T S^-1 nu
};

/*
  unscented measurement update (ukf_state_model.md Section 4).
  chi: sigma points formed from (x, P) with sigma_points(); Z[i][j] = h_j(chi_i).
  z: measurement, R: diagonal measurement variances, m <= MAX_MEAS.
  angle_mask bit j set: measurement j is an angle (circular mean, differences wrapped to [-pi, pi]).
  On success x <- x (+) K nu and P <- P - Pxz S^-1 Pxz^T (symmetrized).
  Returns false (x, P unchanged) if S is not positive definite.
 */
bool unscented_update(ftype x[NUM_STATES], ftype P[NUM_ERR][NUM_ERR], const Weights &w,
                      const ftype chi[NUM_SIGMA][NUM_STATES], const ftype Z[NUM_SIGMA][MAX_MEAS],
                      uint8_t m, const ftype z[MAX_MEAS], const ftype R[MAX_MEAS], uint8_t angle_mask,
                      UpdateResult &res);

} // namespace UKFMath

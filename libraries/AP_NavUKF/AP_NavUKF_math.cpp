#include "AP_NavUKF_config.h"

#if HAL_NAVUKF_AVAILABLE

#include "AP_NavUKF_math.h"

namespace UKFMath {

bool compute_weights(ftype alpha, ftype beta, ftype kappa, uint8_t n, Weights &w)
{
    const ftype nf = ftype(n);
    w.lambda = alpha * alpha * (nf + kappa) - nf;
    const ftype denom = nf + w.lambda;
    if (!(denom > 0)) {
        return false;
    }
    w.gamma = sqrtF(denom);
    w.wm0 = w.lambda / denom;
    w.wc0 = w.wm0 + (1 - alpha * alpha + beta);
    w.wi = 1 / (2 * denom);
    return true;
}

void symmetrize(ftype *A, uint8_t n)
{
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = i + 1; j < n; j++) {
            const ftype m = ftype(0.5) * (A[i * n + j] + A[j * n + i]);
            A[i * n + j] = m;
            A[j * n + i] = m;
        }
    }
}

/*
  Cholesky-Banachiewicz, column by column. Only the lower triangle of A is read,
  so A and L may be the same array (in-place).
 */
bool cholesky(const ftype *A, ftype *L, uint8_t n)
{
    for (uint8_t j = 0; j < n; j++) {
        ftype d = A[j * n + j];
        for (uint8_t k = 0; k < j; k++) {
            d -= L[j * n + k] * L[j * n + k];
        }
        if (!(d > 0) || !isfinite(d)) {
            return false;
        }
        const ftype ljj = sqrtF(d);
        L[j * n + j] = ljj;
        for (uint8_t i = j + 1; i < n; i++) {
            ftype s = A[i * n + j];
            for (uint8_t k = 0; k < j; k++) {
                s -= L[i * n + k] * L[j * n + k];
            }
            L[i * n + j] = s / ljj;
            L[j * n + i] = 0;   // upper triangle, never read again
        }
    }
    return true;
}

QuaternionF quat_exp(const Vector3F &r)
{
    const ftype theta2 = r * r;
    ftype c, s;     // q = [c, s * r]
    if (theta2 < ftype(1e-8)) {
        // series of cos(theta/2) and sin(theta/2)/theta, error below theta^6
        c = 1 - theta2 / 8 + theta2 * theta2 / 384;
        s = ftype(0.5) - theta2 / 48 + theta2 * theta2 / 3840;
    } else {
        const ftype theta = sqrtF(theta2);
        c = cosF(ftype(0.5) * theta);
        s = sinF(ftype(0.5) * theta) / theta;
    }
    QuaternionF q(c, s * r.x, s * r.y, s * r.z);
    q.normalize();
    return q;
}

Vector3F quat_log(const QuaternionF &q)
{
    // q and -q are the same rotation: use the one with w >= 0 (angle in [0, pi])
    const ftype sgn = (q.q1 < 0) ? -1 : 1;
    const ftype w = sgn * q.q1;
    const Vector3F v(sgn * q.q2, sgn * q.q3, sgn * q.q4);
    const ftype s = v.length();
    ftype k;        // r = k * v
    if (s < ftype(1e-6) && w > 0) {
        // 2 atan(s/w) / s, series in s/w
        const ftype t2 = (s * s) / (w * w);
        k = (2 / w) * (1 - t2 / 3 + t2 * t2 / 5);
    } else {
        k = 2 * atan2F(s, w) / s;
    }
    return v * k;
}

void state_plus(const ftype x[NUM_STATES], const ftype dx[NUM_ERR], ftype out[NUM_STATES])
{
    QuaternionF q(x[0], x[1], x[2], x[3]);
    q = q * quat_exp(Vector3F(dx[0], dx[1], dx[2]));
    q.normalize();
    out[0] = q.q1;
    out[1] = q.q2;
    out[2] = q.q3;
    out[3] = q.q4;
    for (uint8_t i = 4; i < NUM_STATES; i++) {
        out[i] = x[i] + dx[i - 1];
    }
}

void state_minus(const ftype a[NUM_STATES], const ftype b[NUM_STATES], ftype dx[NUM_ERR])
{
    const QuaternionF qa(a[0], a[1], a[2], a[3]);
    const QuaternionF qb(b[0], b[1], b[2], b[3]);
    const Vector3F r = quat_log(qb.inverse() * qa);
    dx[0] = r.x;
    dx[1] = r.y;
    dx[2] = r.z;
    for (uint8_t i = 4; i < NUM_STATES; i++) {
        dx[i - 1] = a[i] - b[i];
    }
}

bool sigma_points(const ftype x[NUM_STATES], const ftype P[NUM_ERR][NUM_ERR], const Weights &w,
                  ftype chi[NUM_SIGMA][NUM_STATES], ftype L[NUM_ERR][NUM_ERR], uint8_t &regularised)
{
    // jitter scale: mean diagonal of P, with a floor for an all-zero P
    ftype trace = 0;
    for (uint8_t i = 0; i < NUM_ERR; i++) {
        trace += P[i][i];
    }
    const ftype base = MAX(fabsF(trace) / NUM_ERR, ftype(1e-12));

    regularised = 0;
    ftype jitter = 0;
    bool ok = false;
    for (uint8_t attempt = 0; attempt < 7 && !ok; attempt++) {
        if (attempt > 0) {
            jitter = (attempt == 1) ? base * ftype(1e-9) : jitter * 10;
            regularised++;
        }
        // L <- 0.5 (P + P^T) + jitter I, then factorize in place
        for (uint8_t i = 0; i < NUM_ERR; i++) {
            for (uint8_t j = 0; j < NUM_ERR; j++) {
                L[i][j] = ftype(0.5) * (P[i][j] + P[j][i]);
            }
            L[i][i] += jitter;
        }
        ok = cholesky(&L[0][0], &L[0][0], NUM_ERR);
    }
    if (!ok) {
        return false;
    }

    memcpy(chi[0], x, sizeof(ftype) * NUM_STATES);
    ftype dx[NUM_ERR];
    for (uint8_t i = 0; i < NUM_ERR; i++) {
        for (uint8_t j = 0; j < NUM_ERR; j++) {
            dx[j] = w.gamma * L[j][i];
        }
        state_plus(x, dx, chi[1 + i]);
        for (uint8_t j = 0; j < NUM_ERR; j++) {
            dx[j] = -dx[j];
        }
        state_plus(x, dx, chi[1 + NUM_ERR + i]);
    }
    return true;
}

void unscented_mean(const ftype chi[NUM_SIGMA][NUM_STATES], const Weights &w, ftype xbar[NUM_STATES])
{
    // every state after the quaternion: weighted arithmetic mean
    for (uint8_t k = 4; k < NUM_STATES; k++) {
        ftype m = w.wm0 * chi[0][k];
        for (uint8_t i = 1; i < NUM_SIGMA; i++) {
            m += w.wi * chi[i][k];
        }
        xbar[k] = m;
    }

    // attitude: iterate q_bar <- q_bar (x) exp(sum Wm_i log(conj(q_bar) (x) q_i))
    const ftype tol = (sizeof(ftype) == 8) ? ftype(1e-9) : ftype(1e-6);
    QuaternionF qbar(chi[0][0], chi[0][1], chi[0][2], chi[0][3]);
    for (uint8_t iter = 0; iter < 5; iter++) {
        const QuaternionF qinv = qbar.inverse();
        Vector3F d;
        for (uint8_t i = 0; i < NUM_SIGMA; i++) {
            const QuaternionF qi(chi[i][0], chi[i][1], chi[i][2], chi[i][3]);
            const ftype wi = (i == 0) ? w.wm0 : w.wi;
            d += quat_log(qinv * qi) * wi;
        }
        qbar = qbar * quat_exp(d);
        qbar.normalize();
        if (d.length() < tol) {
            break;
        }
    }
    xbar[0] = qbar.q1;
    xbar[1] = qbar.q2;
    xbar[2] = qbar.q3;
    xbar[3] = qbar.q4;
}

void unscented_covariance(const ftype chi[NUM_SIGMA][NUM_STATES], const ftype xbar[NUM_STATES], const Weights &w,
                          ftype P[NUM_ERR][NUM_ERR])
{
    memset(P, 0, sizeof(ftype) * NUM_ERR * NUM_ERR);
    ftype dx[NUM_ERR];
    for (uint8_t i = 0; i < NUM_SIGMA; i++) {
        state_minus(chi[i], xbar, dx);
        const ftype wc = (i == 0) ? w.wc0 : w.wi;
        for (uint8_t r = 0; r < NUM_ERR; r++) {
            const ftype wr = wc * dx[r];
            for (uint8_t c = r; c < NUM_ERR; c++) {
                P[r][c] += wr * dx[c];
            }
        }
    }
    // copy the upper triangle to the lower one: exactly symmetric
    for (uint8_t r = 0; r < NUM_ERR; r++) {
        for (uint8_t c = 0; c < r; c++) {
            P[r][c] = P[c][r];
        }
    }
}

} // namespace UKFMath

#endif // HAL_NAVUKF_AVAILABLE

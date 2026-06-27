#include "batch_world.h"
#include <cmath>
#include <algorithm>
#include <cfloat>

#if defined(USE_AVX2)
#include <immintrin.h>
#endif

// ── Polynomial sincos approximation (6th order, ~1e-6 accuracy) ──

static inline void sincos_ps_simple(float a, float& s, float& c) {
    float q = std::round(a * (float)(M_1_PI * 0.5));
    float r = a - q * (float)(M_PI * 0.5);
    float u = r * r;
    float sin_approx = r * (1.0f - u * (1.0f/6.0f - u * (1.0f/120.0f - u * (1.0f/5040.0f))));
    float cos_approx = 1.0f - u * (0.5f - u * (1.0f/24.0f - u * (1.0f/720.0f - u * (1.0f/40320.0f))));
    int quad = ((int)q) & 3;
    if (quad == 1) { s = cos_approx; c = -sin_approx; }
    else if (quad == 2) { s = -sin_approx; c = -cos_approx; }
    else if (quad == 3) { s = -cos_approx; c = sin_approx; }
    else { s = sin_approx; c = cos_approx; }
}

#if defined(USE_AVX2)

static inline void sincos_ps_avx2(__m256 a, __m256& s, __m256& c) {
    static const __m256 pi_2 = _mm256_set1_ps(1.57079632679f);
    static const __m256 inv_pi_2 = _mm256_set1_ps(0.63661977236f);
    static const __m256 sign_mask = _mm256_castsi256_ps(_mm256_set1_epi32(0x80000000));

    __m256 q = _mm256_round_ps(_mm256_mul_ps(a, inv_pi_2), _MM_FROUND_TO_NEAREST_INT);
    __m256 r = _mm256_sub_ps(a, _mm256_mul_ps(q, pi_2));
    __m256 u = _mm256_mul_ps(r, r);

    static const __m256 sin_c1 = _mm256_set1_ps(-1.66666666667e-01f);
    static const __m256 sin_c2 = _mm256_set1_ps( 8.33333333333e-03f);
    static const __m256 sin_c3 = _mm256_set1_ps(-1.98412698413e-04f);
    static const __m256 cos_c1 = _mm256_set1_ps(-5.00000000000e-01f);
    static const __m256 cos_c2 = _mm256_set1_ps( 4.16666666667e-02f);
    static const __m256 cos_c3 = _mm256_set1_ps(-1.38888888889e-03f);
    static const __m256 cos_c4 = _mm256_set1_ps( 2.48015873016e-05f);

    __m256 sin_approx = _mm256_mul_ps(r, _mm256_add_ps(_mm256_set1_ps(1.0f),
        _mm256_mul_ps(u, _mm256_add_ps(sin_c1,
            _mm256_mul_ps(u, _mm256_add_ps(sin_c2,
                _mm256_mul_ps(u, sin_c3)))))));

    __m256 cos_approx = _mm256_add_ps(_mm256_set1_ps(1.0f),
        _mm256_mul_ps(u, _mm256_add_ps(cos_c1,
            _mm256_mul_ps(u, _mm256_add_ps(cos_c2,
                _mm256_mul_ps(u, _mm256_add_ps(cos_c3,
                    _mm256_mul_ps(u, cos_c4))))))));

    // Quadrant adjustment
    __m128i q_lo = _mm_cvtps_epi32(_mm256_castps256_ps128(q));
    __m128i q_hi = _mm_cvtps_epi32(_mm256_extractf128_ps(q, 1));
    __m256i qi = _mm256_insertf128_si256(_mm256_castsi128_si256(q_lo), q_hi, 1);
    __m256i quad = _mm256_and_si256(qi, _mm256_set1_epi32(3));

    __m256 sel_swap = _mm256_min_ps(
        _mm256_cvtepi32_ps(_mm256_and_si256(_mm256_set1_epi32(1), quad)),
        _mm256_set1_ps(1.0f));
    __m256 sin_swap = _mm256_add_ps(
        _mm256_mul_ps(sin_approx, _mm256_sub_ps(_mm256_set1_ps(1.0f), sel_swap)),
        _mm256_mul_ps(cos_approx, sel_swap));
    __m256 cos_swap = _mm256_add_ps(
        _mm256_mul_ps(cos_approx, _mm256_sub_ps(_mm256_set1_ps(1.0f), sel_swap)),
        _mm256_mul_ps(sin_approx, sel_swap));

    // Negate sin if quad == 2 or 3
    __m256 neg_sin_mask = _mm256_min_ps(
        _mm256_mul_ps(_mm256_cvtepi32_ps(
            _mm256_and_si256(_mm256_set1_epi32(2), quad)),
            _mm256_set1_ps(0.5f)),
        _mm256_set1_ps(1.0f));
    s = _mm256_add_ps(
        _mm256_mul_ps(sin_swap, _mm256_sub_ps(_mm256_set1_ps(1.0f), neg_sin_mask)),
        _mm256_mul_ps(_mm256_xor_ps(sin_swap, sign_mask), neg_sin_mask));

    // Negate cos if quad == 2
    __m256 neg_cos_mask = _mm256_min_ps(
        _mm256_mul_ps(_mm256_cvtepi32_ps(
            _mm256_and_si256(_mm256_andnot_si256(_mm256_set1_epi32(1), quad),
                             _mm256_set1_epi32(2))),
            _mm256_set1_ps(0.5f)),
        _mm256_set1_ps(1.0f));
    c = _mm256_add_ps(
        _mm256_mul_ps(cos_swap, _mm256_sub_ps(_mm256_set1_ps(1.0f), neg_cos_mask)),
        _mm256_mul_ps(_mm256_xor_ps(cos_swap, sign_mask), neg_cos_mask));
}

#endif // USE_AVX2

// ═════════════════════════════════════════════════════════════════
//  batch_clip_actions
// ═════════════════════════════════════════════════════════════════

void batch_clip_actions(BatchSimWorld& world,
                         const float* actions, int action_dim,
                         int start, int n) {
    int end = start + n;
    for (int i = start; i < end; i++) {
        float a0 = actions[i * action_dim];
        float vmin0 = world.vel_min_x(i);
        float vmax0 = world.vel_max_x(i);
        float acc0 = world.vel_acc_x(i) * world.dt();
        a0 = std::max(std::min(a0, vmax0), vmin0);
        a0 = std::max(std::min(a0, world.vx_data()[i] + acc0), world.vx_data()[i] - acc0);
        world.vx_data()[i] = a0;

        if (action_dim >= 2) {
            float a1 = actions[i * action_dim + 1];
            float vmin1 = world.vel_min_y(i);
            float vmax1 = world.vel_max_y(i);
            float acc1 = world.vel_acc_y(i) * world.dt();
            a1 = std::max(std::min(a1, vmax1), vmin1);
            a1 = std::max(std::min(a1, world.vy_data()[i] + acc1), world.vy_data()[i] - acc1);
            world.vy_data()[i] = a1;
        }
        if (action_dim >= 3) {
            float a2 = actions[i * action_dim + 2];
            float vmin2 = world.vel_min_omega(i);
            float vmax2 = world.vel_max_omega(i);
            float acc2 = world.vel_acc_omega(i) * world.dt();
            a2 = std::max(std::min(a2, vmax2), vmin2);
            a2 = std::max(std::min(a2, world.omega_data()[i] + acc2), world.omega_data()[i] - acc2);
            world.omega_data()[i] = a2;
        }
    }
}

// ═════════════════════════════════════════════════════════════════
//  batch_step_diff
// ═════════════════════════════════════════════════════════════════

void batch_step_diff(BatchSimWorld& world, int start, int n) {
    int end = start + n;
    float dt = world.dt();
    float* vx = world.vx_data();
    float* om = world.omega_data();
    float* ct = world.cos_theta_data();
    float* st = world.sin_theta_data();
    float* x  = world.x_data();
    float* y  = world.y_data();
    float* t  = world.theta_data();

#if defined(USE_AVX2)
    int simd_end = start + (n / SIMD_WIDTH) * SIMD_WIDTH;
    for (int i = start; i < simd_end; i += SIMD_WIDTH) {
        __m256 vv  = _mm256_loadu_ps(vx + i);
        __m256 w   = _mm256_loadu_ps(om + i);
        __m256 cct = _mm256_loadu_ps(ct + i);
        __m256 sst = _mm256_loadu_ps(st + i);
        __m256 xx  = _mm256_loadu_ps(x + i);
        __m256 yy  = _mm256_loadu_ps(y + i);
        __m256 tt  = _mm256_loadu_ps(t + i);
        __m256 ddt = _mm256_set1_ps(dt);

        xx = _mm256_add_ps(xx, _mm256_mul_ps(_mm256_mul_ps(vv, cct), ddt));
        yy = _mm256_add_ps(yy, _mm256_mul_ps(_mm256_mul_ps(vv, sst), ddt));
        tt = _mm256_add_ps(tt, _mm256_mul_ps(w, ddt));

        _mm256_storeu_ps(x + i, xx);
        _mm256_storeu_ps(y + i, yy);
        _mm256_storeu_ps(t + i, tt);
    }
    for (int i = simd_end; i < end; i++) {
        x[i] += vx[i] * ct[i] * dt;
        y[i] += vx[i] * st[i] * dt;
        t[i] += om[i] * dt;
    }
#else
    for (int i = start; i < end; i++) {
        x[i] += vx[i] * ct[i] * dt;
        y[i] += vx[i] * st[i] * dt;
        t[i] += om[i] * dt;
    }
#endif
}

// ═════════════════════════════════════════════════════════════════
//  batch_step_omni
// ═════════════════════════════════════════════════════════════════

void batch_step_omni(BatchSimWorld& world, int start, int n) {
    int end = start + n;
    float dt = world.dt();
    float* vx = world.vx_data();
    float* vy = world.vy_data();
    float* ct = world.cos_theta_data();
    float* st = world.sin_theta_data();
    float* x  = world.x_data();
    float* y  = world.y_data();

#if defined(USE_AVX2)
    int simd_end = start + (n / SIMD_WIDTH) * SIMD_WIDTH;
    for (int i = start; i < simd_end; i += SIMD_WIDTH) {
        __m256 vvx = _mm256_loadu_ps(vx + i);
        __m256 vvy = _mm256_loadu_ps(vy + i);
        __m256 cct = _mm256_loadu_ps(ct + i);
        __m256 sst = _mm256_loadu_ps(st + i);
        __m256 xx  = _mm256_loadu_ps(x + i);
        __m256 yy  = _mm256_loadu_ps(y + i);
        __m256 ddt = _mm256_set1_ps(dt);

        __m256 dx = _mm256_sub_ps(_mm256_mul_ps(vvx, cct), _mm256_mul_ps(vvy, sst));
        __m256 dy = _mm256_add_ps(_mm256_mul_ps(vvx, sst), _mm256_mul_ps(vvy, cct));

        xx = _mm256_add_ps(xx, _mm256_mul_ps(dx, ddt));
        yy = _mm256_add_ps(yy, _mm256_mul_ps(dy, ddt));

        _mm256_storeu_ps(x + i, xx);
        _mm256_storeu_ps(y + i, yy);
    }
    for (int i = simd_end; i < end; i++) {
        x[i] += (vx[i] * ct[i] - vy[i] * st[i]) * dt;
        y[i] += (vx[i] * st[i] + vy[i] * ct[i]) * dt;
    }
#else
    for (int i = start; i < end; i++) {
        x[i] += (vx[i] * ct[i] - vy[i] * st[i]) * dt;
        y[i] += (vx[i] * st[i] + vy[i] * ct[i]) * dt;
    }
#endif
}

// ═════════════════════════════════════════════════════════════════
//  batch_step_omni_angular
// ═════════════════════════════════════════════════════════════════

void batch_step_omni_angular(BatchSimWorld& world, int start, int n) {
    int end = start + n;
    float dt = world.dt();
    float* vx = world.vx_data();
    float* vy = world.vy_data();
    float* om = world.omega_data();
    float* x  = world.x_data();
    float* y  = world.y_data();
    float* t  = world.theta_data();

#if defined(USE_AVX2)
    int simd_end = start + (n / SIMD_WIDTH) * SIMD_WIDTH;
    for (int i = start; i < simd_end; i += SIMD_WIDTH) {
        __m256 vvx = _mm256_loadu_ps(vx + i);
        __m256 vvy = _mm256_loadu_ps(vy + i);
        __m256 w   = _mm256_loadu_ps(om + i);
        __m256 tt  = _mm256_loadu_ps(t + i);
        __m256 xx  = _mm256_loadu_ps(x + i);
        __m256 yy  = _mm256_loadu_ps(y + i);
        __m256 ddt = _mm256_set1_ps(dt);

        __m256 ct, st;
        sincos_ps_avx2(tt, st, ct);

        __m256 dx = _mm256_sub_ps(_mm256_mul_ps(vvx, ct), _mm256_mul_ps(vvy, st));
        __m256 dy = _mm256_add_ps(_mm256_mul_ps(vvx, st), _mm256_mul_ps(vvy, ct));

        xx = _mm256_add_ps(xx, _mm256_mul_ps(dx, ddt));
        yy = _mm256_add_ps(yy, _mm256_mul_ps(dy, ddt));
        tt = _mm256_add_ps(tt, _mm256_mul_ps(w, ddt));

        _mm256_storeu_ps(x + i, xx);
        _mm256_storeu_ps(y + i, yy);
        _mm256_storeu_ps(t + i, tt);
    }
    for (int i = simd_end; i < end; i++) {
        float ct, st;
        sincos_ps_simple(t[i], st, ct);
        x[i] += (vx[i] * ct - vy[i] * st) * dt;
        y[i] += (vx[i] * st + vy[i] * ct) * dt;
        t[i] += om[i] * dt;
    }
#else
    for (int i = start; i < end; i++) {
        float ct, st;
        sincos_ps_simple(t[i], st, ct);
        x[i] += (vx[i] * ct - vy[i] * st) * dt;
        y[i] += (vx[i] * st + vy[i] * ct) * dt;
        t[i] += om[i] * dt;
    }
#endif
}

// ═════════════════════════════════════════════════════════════════
//  batch_step_acker
// ═════════════════════════════════════════════════════════════════

void batch_step_acker(BatchSimWorld& world, int start, int n) {
    int end = start + n;
    float dt = world.dt();

    for (int i = start; i < end; i++) {
        float v = world.vx_data()[i];
        float desired_steer = world.vy_data()[i];

        float steer_min = world.vel_min_omega(i);
        float steer_max = world.vel_max_omega(i);
        float steer_rate = world.vel_acc_omega(i);

        float& steer = world.steer_angle(i);
        steer += std::max(-steer_rate * dt, std::min(desired_steer - steer, steer_rate * dt));
        steer = std::max(steer_min, std::min(steer, steer_max));

        float ct = world.cos_theta_data()[i];
        float st = world.sin_theta_data()[i];

        world.x_data()[i] += v * ct * dt;
        world.y_data()[i] += v * st * dt;

        if (std::abs(v) > 1e-6f && steer != 0) {
            float wb = 0.5f;
            world.theta_data()[i] += v * std::tan(steer) / wb * dt;
        }
    }
}

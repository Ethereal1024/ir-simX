#include "lidar.h"
#include <cmath>
#include <cstring>
#ifdef USE_AVX2
#include <immintrin.h>
#endif

// ═══════════════════════════════════════════════════════════════
//  Scalar LiDAR raycast
// ═══════════════════════════════════════════════════════════════

void lidar_raycast_scalar(
    Vec2 origin, float heading,
    const float* angles, int n_beams, float range_max,
    const Obstacle* obstacles, int n_obs,
    float* ranges_out)
{
    for (int i = 0; i < n_beams; i++) {
        float angle = angles[i] + heading;
        Vec2 dir{std::cos(angle), std::sin(angle)};
        float min_t = range_max;

        for (int j = 0; j < n_obs; j++) {
            float t;
            if (intersect_ray_obstacle(origin, dir, obstacles[j], t)) {
                if (t < min_t) min_t = t;
            }
        }

        ranges_out[i] = min_t;
    }
}

// ═══════════════════════════════════════════════════════════════
//  AVX2 accelerated LiDAR raycast
// ═══════════════════════════════════════════════════════════════

#ifdef USE_AVX2

// Compute 8 cos/sin values from 8 angles.
static inline void __attribute__((always_inline))
sincos_8(const float* angles, float* cos_out, float* sin_out)
{
    for (int k = 0; k < 8; k++) {
        cos_out[k] = std::cos(angles[k]);
        sin_out[k] = std::sin(angles[k]);
    }
}

void lidar_raycast_avx2(
    Vec2 origin, float heading,
    const float* angles, int n_beams, float range_max,
    const Obstacle* obstacles, int n_obs,
    float* ranges_out)
{
    for (int i = 0; i < n_beams; i++) ranges_out[i] = range_max;

    for (int j = 0; j < n_obs; j++) {
        const Obstacle& obs = obstacles[j];

        if (obs.type == ShapeType::CIRCLE || obs.type == ShapeType::RECT) {
            // ── Shape-specific SIMD constants ──
            __m256 rmax = _mm256_set1_ps(range_max);
            __m256 zero = _mm256_setzero_ps();
            __m256 sign_mask = _mm256_set1_ps(-0.0f);

            // Circle: (oc, r²)
            __m256 oc_x = {}, oc_y = {}, r2 = {};
            // Rect: AABB bounds
            __m256 ox = {}, oy = {};
            __m256 bxmin = {}, bxmax = {}, bymin = {}, bymax = {};
            __m256 eps = {}, one = {}, fmax = {}, fmin = {};

            if (obs.type == ShapeType::CIRCLE) {
                oc_x = _mm256_set1_ps(obs.center.x - origin.x);
                oc_y = _mm256_set1_ps(obs.center.y - origin.y);
                r2   = _mm256_set1_ps(obs.radius * obs.radius);
            } else {
                ox = _mm256_set1_ps(origin.x);
                oy = _mm256_set1_ps(origin.y);
                bxmin = _mm256_set1_ps(obs.aabb.min.x);
                bxmax = _mm256_set1_ps(obs.aabb.max.x);
                bymin = _mm256_set1_ps(obs.aabb.min.y);
                bymax = _mm256_set1_ps(obs.aabb.max.y);
                eps = _mm256_set1_ps(1e-12f);
                one = _mm256_set1_ps(1.0f);
                fmax = _mm256_set1_ps(FLT_MAX);
                fmin = _mm256_set1_ps(-FLT_MAX);
            }

            for (int i = 0; i < n_beams; i += 8) {
                int remaining = n_beams - i;
                int k = (remaining < 8) ? remaining : 8;

                // ── Common beam direction loading (shared by CIRCLE & RECT) ──
                float angle_full[8], cs_buf[8], sn_buf[8];
                for (int t = 0; t < k; t++) angle_full[t] = angles[i + t] + heading;
                sincos_8(angle_full, cs_buf, sn_buf);
                __m256 dx = _mm256_loadu_ps(cs_buf);
                __m256 dy = _mm256_loadu_ps(sn_buf);

                __m256 t_hit, hit_ok;

                if (obs.type == ShapeType::CIRCLE) {
                    // ── SIMD circle intersection ──
                    __m256 b = _mm256_fmadd_ps(dy, oc_y, _mm256_mul_ps(dx, oc_x));
                    b = _mm256_mul_ps(b, _mm256_set1_ps(-2.0f));
                    __m256 c_v = _mm256_fmadd_ps(oc_x, oc_x, _mm256_mul_ps(oc_y, oc_y));
                    c_v = _mm256_sub_ps(c_v, r2);
                    __m256 disc = _mm256_fmadd_ps(b, b, _mm256_mul_ps(c_v, _mm256_set1_ps(-4.0f)));
                    __m256 sqrt_disc = _mm256_sqrt_ps(_mm256_max_ps(disc, zero));
                    __m256 neg_b = _mm256_xor_ps(b, sign_mask);  // -b
                    __m256 t1 = _mm256_mul_ps(_mm256_sub_ps(neg_b, sqrt_disc), _mm256_set1_ps(0.5f));
                    __m256 t2 = _mm256_mul_ps(_mm256_add_ps(neg_b, sqrt_disc), _mm256_set1_ps(0.5f));
                    __m256 invalid = _mm256_cmp_ps(disc, zero, _CMP_LT_OS);
                    __m256 t1_ok = _mm256_cmp_ps(t1, zero, _CMP_GE_OS);
                    __m256 t2_ok = _mm256_cmp_ps(t2, zero, _CMP_GE_OS);
                    t_hit = _mm256_blendv_ps(t2, t1, t1_ok);
                    hit_ok = _mm256_andnot_ps(invalid, _mm256_or_ps(t1_ok, t2_ok));
                } else {
                    // ── SIMD AABB slab test ──
                    __m256 dx_abs = _mm256_andnot_ps(sign_mask, dx);
                    __m256 dy_abs = _mm256_andnot_ps(sign_mask, dy);
                    __m256 dx_ok = _mm256_cmp_ps(dx_abs, eps, _CMP_GT_OS);
                    __m256 dy_ok = _mm256_cmp_ps(dy_abs, eps, _CMP_GT_OS);

                    // X slab
                    __m256 inv_dx = _mm256_div_ps(one, _mm256_blendv_ps(one, dx, dx_ok));
                    __m256 tmin_x = _mm256_mul_ps(_mm256_sub_ps(bxmin, ox), inv_dx);
                    __m256 tmax_x = _mm256_mul_ps(_mm256_sub_ps(bxmax, ox), inv_dx);
                    {  // swap if needed
                        __m256 lo = _mm256_min_ps(tmin_x, tmax_x);
                        __m256 hi = _mm256_max_ps(tmin_x, tmax_x);
                        tmin_x = lo; tmax_x = hi;
                    }
                    // Y slab
                    __m256 inv_dy = _mm256_div_ps(one, _mm256_blendv_ps(one, dy, dy_ok));
                    __m256 tmin_y = _mm256_mul_ps(_mm256_sub_ps(bymin, oy), inv_dy);
                    __m256 tmax_y = _mm256_mul_ps(_mm256_sub_ps(bymax, oy), inv_dy);
                    {
                        __m256 lo = _mm256_min_ps(tmin_y, tmax_y);
                        __m256 hi = _mm256_max_ps(tmin_y, tmax_y);
                        tmin_y = lo; tmax_y = hi;
                    }

                    // Beams parallel to axis: if origin inside → unbounded; if outside → no hit
                    __m256 ox_in = _mm256_and_ps(_mm256_cmp_ps(ox, bxmin, _CMP_GE_OS),
                                                  _mm256_cmp_ps(ox, bxmax, _CMP_LE_OS));
                    __m256 oy_in = _mm256_and_ps(_mm256_cmp_ps(oy, bymin, _CMP_GE_OS),
                                                  _mm256_cmp_ps(oy, bymax, _CMP_LE_OS));
                    tmin_x = _mm256_blendv_ps(_mm256_blendv_ps(fmax, fmin, ox_in), tmin_x, dx_ok);
                    tmax_x = _mm256_blendv_ps(_mm256_blendv_ps(fmin, fmax, ox_in), tmax_x, dx_ok);
                    tmin_y = _mm256_blendv_ps(_mm256_blendv_ps(fmax, fmin, oy_in), tmin_y, dy_ok);
                    tmax_y = _mm256_blendv_ps(_mm256_blendv_ps(fmin, fmax, oy_in), tmax_y, dy_ok);

                    t_hit = _mm256_max_ps(tmin_x, tmin_y);
                    __m256 t_exit = _mm256_min_ps(tmax_x, tmax_y);
                    hit_ok = _mm256_cmp_ps(t_hit, t_exit, _CMP_LE_OS);
                    __m256 texit_ok = _mm256_cmp_ps(t_exit, zero, _CMP_GE_OS);
                    hit_ok = _mm256_and_ps(hit_ok, texit_ok);
                }

                // ── Common range update (shared by CIRCLE & RECT) ──
                t_hit = _mm256_max_ps(t_hit, zero);
                t_hit = _mm256_min_ps(t_hit, rmax);

                __m256 cur = _mm256_loadu_ps(ranges_out + i);
                __m256 closer = _mm256_cmp_ps(t_hit, cur, _CMP_LT_OS);
                __m256 mask = _mm256_and_ps(closer, hit_ok);
                __m256 new_val = _mm256_blendv_ps(cur, t_hit, mask);

                if (k < 8) {
                    int mask_int = (1 << k) - 1;
                    __m256i blend_mask = _mm256_set_epi32(
                        ((mask_int >> 7) & 1) ? -1 : 0, ((mask_int >> 6) & 1) ? -1 : 0,
                        ((mask_int >> 5) & 1) ? -1 : 0, ((mask_int >> 4) & 1) ? -1 : 0,
                        ((mask_int >> 3) & 1) ? -1 : 0, ((mask_int >> 2) & 1) ? -1 : 0,
                        ((mask_int >> 1) & 1) ? -1 : 0, ((mask_int >> 0) & 1) ? -1 : 0);
                    new_val = _mm256_blendv_ps(_mm256_loadu_ps(ranges_out + i),
                                                new_val, _mm256_castsi256_ps(blend_mask));
                }
                _mm256_storeu_ps(ranges_out + i, new_val);
            }
        } else {
            for (int i = 0; i < n_beams; i++) {
                float angle = angles[i] + heading;
                Vec2 dir{std::cos(angle), std::sin(angle)};
                float t;
                if (intersect_ray_obstacle(origin, dir, obs, t)) {
                    if (t < ranges_out[i]) ranges_out[i] = t;
                }
            }
        }
    }
}

// FMCW AVX2: same SIMD intersection + per-beam best obstacle index tracking
void fmcw_lidar_raycast_avx2(
    Vec2 origin, float heading,
    float sensor_vx, float sensor_vy, bool motion_compensate,
    const float* angles, int n_beams, float range_max,
    const Obstacle* obstacles, int n_obs,
    float* ranges_out, float* velocities_out)
{
    for (int i = 0; i < n_beams; i++) {
        ranges_out[i] = range_max;
        velocities_out[i] = 0.0f;
    }
    std::vector<int> best_obs(n_beams, -1);

    for (int j = 0; j < n_obs; j++) {
        const Obstacle& obs = obstacles[j];

        if (obs.type == ShapeType::CIRCLE || obs.type == ShapeType::RECT) {
            __m256 rmax = _mm256_set1_ps(range_max);
            __m256 zero = _mm256_setzero_ps();
            __m256 sign_mask = _mm256_set1_ps(-0.0f);

            __m256 oc_x = {}, oc_y = {}, r2 = {};
            __m256 ox = {}, oy = {};
            __m256 bxmin = {}, bxmax = {}, bymin = {}, bymax = {};
            __m256 eps = {}, one = {}, fmax = {}, fmin = {};

            if (obs.type == ShapeType::CIRCLE) {
                oc_x = _mm256_set1_ps(obs.center.x - origin.x);
                oc_y = _mm256_set1_ps(obs.center.y - origin.y);
                r2   = _mm256_set1_ps(obs.radius * obs.radius);
            } else {
                ox = _mm256_set1_ps(origin.x);
                oy = _mm256_set1_ps(origin.y);
                bxmin = _mm256_set1_ps(obs.aabb.min.x);
                bxmax = _mm256_set1_ps(obs.aabb.max.x);
                bymin = _mm256_set1_ps(obs.aabb.min.y);
                bymax = _mm256_set1_ps(obs.aabb.max.y);
                eps = _mm256_set1_ps(1e-12f);
                one = _mm256_set1_ps(1.0f);
                fmax = _mm256_set1_ps(FLT_MAX);
                fmin = _mm256_set1_ps(-FLT_MAX);
            }

            for (int i = 0; i < n_beams; i += 8) {
                int remaining = n_beams - i;
                int k = (remaining < 8) ? remaining : 8;

                float angle_full[8], cs_buf[8], sn_buf[8];
                for (int t = 0; t < k; t++) angle_full[t] = angles[i + t] + heading;
                sincos_8(angle_full, cs_buf, sn_buf);
                __m256 dx = _mm256_loadu_ps(cs_buf);
                __m256 dy = _mm256_loadu_ps(sn_buf);

                __m256 t_hit, hit_ok;

                if (obs.type == ShapeType::CIRCLE) {
                    __m256 b = _mm256_fmadd_ps(dy, oc_y, _mm256_mul_ps(dx, oc_x));
                    b = _mm256_mul_ps(b, _mm256_set1_ps(-2.0f));
                    __m256 c_v = _mm256_fmadd_ps(oc_x, oc_x, _mm256_mul_ps(oc_y, oc_y));
                    c_v = _mm256_sub_ps(c_v, r2);
                    __m256 disc = _mm256_fmadd_ps(b, b, _mm256_mul_ps(c_v, _mm256_set1_ps(-4.0f)));
                    __m256 sqrt_disc = _mm256_sqrt_ps(_mm256_max_ps(disc, zero));
                    __m256 neg_b = _mm256_xor_ps(b, sign_mask);
                    __m256 t1 = _mm256_mul_ps(_mm256_sub_ps(neg_b, sqrt_disc), _mm256_set1_ps(0.5f));
                    __m256 t2 = _mm256_mul_ps(_mm256_add_ps(neg_b, sqrt_disc), _mm256_set1_ps(0.5f));
                    __m256 invalid = _mm256_cmp_ps(disc, zero, _CMP_LT_OS);
                    __m256 t1_ok = _mm256_cmp_ps(t1, zero, _CMP_GE_OS);
                    __m256 t2_ok = _mm256_cmp_ps(t2, zero, _CMP_GE_OS);
                    t_hit = _mm256_blendv_ps(t2, t1, t1_ok);
                    hit_ok = _mm256_andnot_ps(invalid, _mm256_or_ps(t1_ok, t2_ok));
                } else {
                    __m256 dx_abs = _mm256_andnot_ps(sign_mask, dx);
                    __m256 dy_abs = _mm256_andnot_ps(sign_mask, dy);
                    __m256 dx_ok = _mm256_cmp_ps(dx_abs, eps, _CMP_GT_OS);
                    __m256 dy_ok = _mm256_cmp_ps(dy_abs, eps, _CMP_GT_OS);

                    __m256 inv_dx = _mm256_div_ps(one, _mm256_blendv_ps(one, dx, dx_ok));
                    __m256 tmin_x = _mm256_mul_ps(_mm256_sub_ps(bxmin, ox), inv_dx);
                    __m256 tmax_x = _mm256_mul_ps(_mm256_sub_ps(bxmax, ox), inv_dx);
                    {
                        __m256 lo = _mm256_min_ps(tmin_x, tmax_x);
                        __m256 hi = _mm256_max_ps(tmin_x, tmax_x);
                        tmin_x = lo; tmax_x = hi;
                    }
                    __m256 inv_dy = _mm256_div_ps(one, _mm256_blendv_ps(one, dy, dy_ok));
                    __m256 tmin_y = _mm256_mul_ps(_mm256_sub_ps(bymin, oy), inv_dy);
                    __m256 tmax_y = _mm256_mul_ps(_mm256_sub_ps(bymax, oy), inv_dy);
                    {
                        __m256 lo = _mm256_min_ps(tmin_y, tmax_y);
                        __m256 hi = _mm256_max_ps(tmin_y, tmax_y);
                        tmin_y = lo; tmax_y = hi;
                    }

                    __m256 ox_in = _mm256_and_ps(_mm256_cmp_ps(ox, bxmin, _CMP_GE_OS),
                                                  _mm256_cmp_ps(ox, bxmax, _CMP_LE_OS));
                    __m256 oy_in = _mm256_and_ps(_mm256_cmp_ps(oy, bymin, _CMP_GE_OS),
                                                  _mm256_cmp_ps(oy, bymax, _CMP_LE_OS));
                    tmin_x = _mm256_blendv_ps(_mm256_blendv_ps(fmax, fmin, ox_in), tmin_x, dx_ok);
                    tmax_x = _mm256_blendv_ps(_mm256_blendv_ps(fmin, fmax, ox_in), tmax_x, dx_ok);
                    tmin_y = _mm256_blendv_ps(_mm256_blendv_ps(fmax, fmin, oy_in), tmin_y, dy_ok);
                    tmax_y = _mm256_blendv_ps(_mm256_blendv_ps(fmin, fmax, oy_in), tmax_y, dy_ok);

                    t_hit = _mm256_max_ps(tmin_x, tmin_y);
                    __m256 t_exit = _mm256_min_ps(tmax_x, tmax_y);
                    hit_ok = _mm256_cmp_ps(t_hit, t_exit, _CMP_LE_OS);
                    hit_ok = _mm256_and_ps(hit_ok, _mm256_cmp_ps(t_exit, zero, _CMP_GE_OS));
                }

                t_hit = _mm256_max_ps(t_hit, zero);
                t_hit = _mm256_min_ps(t_hit, rmax);

                __m256 cur = _mm256_loadu_ps(ranges_out + i);
                __m256 closer = _mm256_cmp_ps(t_hit, cur, _CMP_LT_OS);
                __m256 mask = _mm256_and_ps(closer, hit_ok);
                __m256 new_val = _mm256_blendv_ps(cur, t_hit, mask);

                // Track best obstacle index per beam
                __m256i obs_j = _mm256_set1_epi32(j);
                __m256i mask_i = _mm256_castps_si256(mask);
                __m256i cur_best = _mm256_loadu_si256((__m256i*)(best_obs.data() + i));
                __m256i new_best = _mm256_blendv_epi8(cur_best, obs_j, mask_i);
                _mm256_storeu_si256((__m256i*)(best_obs.data() + i), new_best);

                if (k < 8) {
                    int mask_int = (1 << k) - 1;
                    __m256i blend_mask = _mm256_set_epi32(
                        ((mask_int >> 7) & 1) ? -1 : 0, ((mask_int >> 6) & 1) ? -1 : 0,
                        ((mask_int >> 5) & 1) ? -1 : 0, ((mask_int >> 4) & 1) ? -1 : 0,
                        ((mask_int >> 3) & 1) ? -1 : 0, ((mask_int >> 2) & 1) ? -1 : 0,
                        ((mask_int >> 1) & 1) ? -1 : 0, ((mask_int >> 0) & 1) ? -1 : 0);
                    new_val = _mm256_blendv_ps(_mm256_loadu_ps(ranges_out + i),
                                                new_val, _mm256_castsi256_ps(blend_mask));
                }
                _mm256_storeu_ps(ranges_out + i, new_val);
            }
        } else {
            for (int i = 0; i < n_beams; i++) {
                float angle = angles[i] + heading;
                Vec2 dir{std::cos(angle), std::sin(angle)};
                float t;
                if (intersect_ray_obstacle(origin, dir, obs, t)) {
                    if (t < ranges_out[i]) {
                        ranges_out[i] = t;
                        best_obs[i] = j;
                    }
                }
            }
        }
    }

    // Compute radial velocities from best obstacle indices
    for (int i = 0; i < n_beams; i++) {
        int bj = best_obs[i];
        if (bj < 0 || ranges_out[i] >= range_max) continue;
        const auto& obs = obstacles[bj];
        float ovx = obs.vx, ovy = obs.vy;
        if (!motion_compensate) { ovx -= sensor_vx; ovy -= sensor_vy; }
        float angle = angles[i] + heading;
        velocities_out[i] = ovx * std::cos(angle) + ovy * std::sin(angle);
    }
}

#else  // !USE_AVX2

void lidar_raycast_avx2(
    Vec2 origin, float heading,
    const float* angles, int n_beams, float range_max,
    const Obstacle* obstacles, int n_obs,
    float* ranges_out)
{
    lidar_raycast_scalar(origin, heading, angles, n_beams, range_max, obstacles, n_obs, ranges_out);
}

void fmcw_lidar_raycast_avx2(
    Vec2 origin, float heading,
    float sensor_vx, float sensor_vy, bool motion_compensate,
    const float* angles, int n_beams, float range_max,
    const Obstacle* obstacles, int n_obs,
    float* ranges_out, float* velocities_out)
{
    fmcw_lidar_raycast(origin, heading, sensor_vx, sensor_vy, motion_compensate,
                       angles, n_beams, range_max, obstacles, n_obs,
                       ranges_out, velocities_out);
}

#endif  // USE_AVX2

// ═══════════════════════════════════════════════════════════════
//  Auto-select: AVX2 (8-beam SIMD circle raycast) when available,
//  scalar fallback otherwise.
// ═══════════════════════════════════════════════════════════════

void lidar_raycast(
    Vec2 origin, float heading,
    const float* angles, int n_beams, float range_max,
    const Obstacle* obstacles, int n_obs,
    float* ranges_out)
{
#ifdef USE_AVX2
    lidar_raycast_avx2(origin, heading, angles, n_beams, range_max,
                       obstacles, n_obs, ranges_out);
#else
    lidar_raycast_scalar(origin, heading, angles, n_beams, range_max,
                         obstacles, n_obs, ranges_out);
#endif
}

// ═══════════════════════════════════════════════════════════════
//  FMCW LiDAR raycast — ranges + per-beam radial velocity
// ═══════════════════════════════════════════════════════════════

void fmcw_lidar_raycast(
    Vec2 origin, float heading,
    float sensor_vx, float sensor_vy,
    bool motion_compensate,
    const float* angles, int n_beams, float range_max,
    const Obstacle* obstacles, int n_obs,
    float* ranges_out,
    float* velocities_out)
{
#ifdef USE_AVX2
    fmcw_lidar_raycast_avx2(origin, heading, sensor_vx, sensor_vy, motion_compensate,
                            angles, n_beams, range_max, obstacles, n_obs,
                            ranges_out, velocities_out);
#else
    for (int i = 0; i < n_beams; i++) {
        float angle = angles[i] + heading;
        Vec2 dir{std::cos(angle), std::sin(angle)};
        float min_t = range_max;
        int best_j = -1;

        for (int j = 0; j < n_obs; j++) {
            float t;
            if (intersect_ray_obstacle(origin, dir, obstacles[j], t)) {
                if (t < min_t) {
                    min_t = t;
                    best_j = j;
                }
            }
        }

        ranges_out[i] = min_t;
        if (best_j >= 0) {
            float ovx = obstacles[best_j].vx;
            float ovy = obstacles[best_j].vy;
            if (!motion_compensate) {
                ovx -= sensor_vx;
                ovy -= sensor_vy;
            }
            velocities_out[i] = ovx * dir.x + ovy * dir.y;
        } else {
            velocities_out[i] = 0.0f;
        }
    }
#endif
}

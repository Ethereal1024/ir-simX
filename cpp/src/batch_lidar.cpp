#include "batch_world.h"
#include <cmath>
#include <cfloat>

#if defined(USE_AVX2)
#include <immintrin.h>
#endif

// ── Scalar ray-obstacle intersection for a single env ───────────

static float scalar_raycast_env(float ox, float oy, float heading,
                                 float cos_angle, float sin_angle,
                                 float range_max,
                                 const Obstacle* obstacles, int n_obs) {
    float dx = cos_angle * std::cos(heading) - sin_angle * std::sin(heading);
    float dy = cos_angle * std::sin(heading) + sin_angle * std::cos(heading);
    Vec2 o{ox, oy};
    Vec2 d{dx, dy};

    float best_t = range_max;
    for (int j = 0; j < n_obs; j++) {
        float t;
        if (intersect_ray_obstacle(o, d, obstacles[j], t)) {
            if (t < best_t) best_t = t;
        }
    }
    return best_t;
}

// ═════════════════════════════════════════════════════════════════
//  Mode B: per-environment LiDAR (scalar fallback)
// ═════════════════════════════════════════════════════════════════

void batch_lidar_raycast_per_env(BatchSimWorld& world,
    const float* cos_angles, const float* sin_angles,
    int n_beams, float range_max, float* ranges_out) {

    int bs = world.batch_size();
    for (int env = 0; env < bs; env++) {
        float ox = world.x_data()[env];
        float oy = world.y_data()[env];
        float heading = world.theta_data()[env];

        const auto& obs = world.obstacles_per_env(env);
        int n_obs = (int)obs.size();
        const Obstacle* obs_ptr = n_obs > 0 ? obs.data() : nullptr;

        for (int b = 0; b < n_beams; b++) {
            float t = scalar_raycast_env(ox, oy, heading,
                                          cos_angles[b], sin_angles[b],
                                          range_max, obs_ptr, n_obs);
            ranges_out[b * world.alloc_size() + env] = t;
        }
    }
}

// ═════════════════════════════════════════════════════════════════
//  Mode A: shared obstacles — cross-environment SIMD
// ═════════════════════════════════════════════════════════════════

#if defined(USE_AVX2)

static void batch_beam_avx2(BatchSimWorld& world,
                             float cos_angle, float sin_angle,
                             int start, int n, float range_max,
                             float* ranges_out) {
    const auto& obstacles = world.obstacles();
    int n_obs = (int)obstacles.size();

    __m256 best = _mm256_set1_ps(range_max);
    __m256 zero = _mm256_setzero_ps();

    __m256 ox = _mm256_loadu_ps(world.x_data() + start);
    __m256 oy = _mm256_loadu_ps(world.y_data() + start);
    __m256 bcos = _mm256_set1_ps(cos_angle);
    __m256 bsin = _mm256_set1_ps(sin_angle);

    __m256 ct = _mm256_loadu_ps(world.cos_theta_data() + start);
    __m256 st = _mm256_loadu_ps(world.sin_theta_data() + start);

    __m256 dx = _mm256_sub_ps(_mm256_mul_ps(bcos, ct), _mm256_mul_ps(bsin, st));
    __m256 dy = _mm256_add_ps(_mm256_mul_ps(bcos, st), _mm256_mul_ps(bsin, ct));

    for (int j = 0; j < n_obs; j++) {
        const auto& obs = obstacles[j];
        __m256 t;

        if (obs.type == ShapeType::CIRCLE) {
            __m256 cx = _mm256_set1_ps(obs.center.x);
            __m256 cy = _mm256_set1_ps(obs.center.y);
            __m256 rr = _mm256_set1_ps(obs.radius * obs.radius);

            __m256 ocx = _mm256_sub_ps(ox, cx);
            __m256 ocy = _mm256_sub_ps(oy, cy);

            __m256 b = _mm256_add_ps(_mm256_mul_ps(ocx, dx), _mm256_mul_ps(ocy, dy));
            __m256 cc = _mm256_sub_ps(
                _mm256_add_ps(_mm256_mul_ps(ocx, ocx), _mm256_mul_ps(ocy, ocy)), rr);

            __m256 disc = _mm256_sub_ps(_mm256_mul_ps(b, b), cc);
            __m256 mask = _mm256_cmp_ps(disc, zero, _CMP_GE_OQ);

            __m256 sqrt_disc = _mm256_sqrt_ps(disc);
            __m256 t1 = _mm256_sub_ps(_mm256_sub_ps(zero, b), sqrt_disc);
            __m256 t2 = _mm256_add_ps(_mm256_sub_ps(zero, b), sqrt_disc);
            // Pick smallest non-negative t
            __m256 t1_pos = _mm256_cmp_ps(t1, zero, _CMP_GE_OQ);
            t = _mm256_blendv_ps(t2, t1, t1_pos);
            // Both negative -> no valid hit
            __m256 t2_pos = _mm256_cmp_ps(t2, zero, _CMP_GE_OQ);
            __m256 any_pos = _mm256_or_ps(t1_pos, t2_pos);
            mask = _mm256_and_ps(mask, any_pos);
            t = _mm256_blendv_ps(_mm256_set1_ps(range_max), t, mask);

        } else if (obs.type == ShapeType::RECT) {
            __m256 box_min_x = _mm256_set1_ps(obs.aabb.min.x);
            __m256 box_max_x = _mm256_set1_ps(obs.aabb.max.x);
            __m256 box_min_y = _mm256_set1_ps(obs.aabb.min.y);
            __m256 box_max_y = _mm256_set1_ps(obs.aabb.max.y);

            __m256 tmin = _mm256_set1_ps(-FLT_MAX);
            __m256 tmax = _mm256_set1_ps(FLT_MAX);

            __m256 inv_dx = _mm256_div_ps(_mm256_set1_ps(1.0f), dx);
            __m256 t1x = _mm256_mul_ps(_mm256_sub_ps(box_min_x, ox), inv_dx);
            __m256 t2x = _mm256_mul_ps(_mm256_sub_ps(box_max_x, ox), inv_dx);
            tmin = _mm256_max_ps(tmin, _mm256_min_ps(t1x, t2x));
            tmax = _mm256_min_ps(tmax, _mm256_max_ps(t1x, t2x));

            __m256 inv_dy = _mm256_div_ps(_mm256_set1_ps(1.0f), dy);
            __m256 t1y = _mm256_mul_ps(_mm256_sub_ps(box_min_y, oy), inv_dy);
            __m256 t2y = _mm256_mul_ps(_mm256_sub_ps(box_max_y, oy), inv_dy);
            tmin = _mm256_max_ps(tmin, _mm256_min_ps(t1y, t2y));
            tmax = _mm256_min_ps(tmax, _mm256_max_ps(t1y, t2y));

            __m256 hit = _mm256_and_ps(
                _mm256_cmp_ps(tmin, tmax, _CMP_LE_OQ),
                _mm256_cmp_ps(tmax, zero, _CMP_GE_OQ));

            t = _mm256_max_ps(zero, tmin);
            t = _mm256_blendv_ps(_mm256_set1_ps(range_max), t, hit);

        }
        // POLYGON / LINESTRING handled by SpatialHashGrid below
    }

    // Phase 2: grid query for poly/linestring obstacles
    {
        _mm256_storeu_ps(ranges_out + start, best);
        const auto& obstacles = world.obstacles();
        const auto& lg = world.lidar_grid();
        if (!lg.empty()) {
            for (int ei = 0; ei < n; ei++) {
                int env_idx = start + ei;
                float cur_best = ranges_out[env_idx];
                Vec2 o_f{world.x_data()[env_idx], world.y_data()[env_idx]};
                float h_f = world.theta_data()[env_idx];
                float ddx = cos_angle * std::cos(h_f) - sin_angle * std::sin(h_f);
                float ddy = cos_angle * std::sin(h_f) + sin_angle * std::cos(h_f);
                Vec2 d_f{ddx, ddy};
                float t_f = lg.raycast(o_f, d_f, cur_best);
                if (t_f < cur_best) ranges_out[env_idx] = t_f;
            }
        } else {
            // No grid — scalar fallback per obstacle
            for (int j = 0; j < (int)obstacles.size(); j++) {
                const auto& obs = obstacles[j];
                if (obs.type == ShapeType::CIRCLE || obs.type == ShapeType::RECT) continue;
                for (int ei = 0; ei < n; ei++) {
                    int env_idx = start + ei;
                    float cur_best = ranges_out[env_idx];
                    float ox_f = world.x_data()[env_idx];
                    float oy_f = world.y_data()[env_idx];
                    float h_f = world.theta_data()[env_idx];
                    float ddx = cos_angle * std::cos(h_f) - sin_angle * std::sin(h_f);
                    float ddy = cos_angle * std::sin(h_f) + sin_angle * std::cos(h_f);
                    Vec2 o_f{ox_f, oy_f};
                    Vec2 d_f{ddx, ddy};
                    float t_f;
                    if (intersect_ray_obstacle(o_f, d_f, obs, t_f)) {
                        if (t_f < cur_best) ranges_out[env_idx] = t_f;
                    }
                }
            }
        }
        best = _mm256_loadu_ps(ranges_out + start);
    }

    _mm256_storeu_ps(ranges_out + start, best);
    if (n < SIMD_WIDTH) {
        for (int i = n; i < SIMD_WIDTH; i++) {
            ranges_out[start + i] = range_max;
        }
    }
}

#else // !USE_AVX2

static void batch_beam_scalar(BatchSimWorld& world,
                               float cos_angle, float sin_angle,
                               int start, int n, float range_max,
                               float* ranges_out) {
    int bs = world.batch_size();
    const auto& obstacles = world.obstacles();
    int n_obs = (int)obstacles.size();

    for (int ei = 0; ei < n; ei++) {
        int env_idx = start + ei;
        float ox = world.x_data()[env_idx];
        float oy = world.y_data()[env_idx];
        float heading = world.theta_data()[env_idx];

        float best_t = range_max;
        float ddx = cos_angle * std::cos(heading) - sin_angle * std::sin(heading);
        float ddy = cos_angle * std::sin(heading) + sin_angle * std::cos(heading);
        Vec2 o{ox, oy};
        Vec2 d{ddx, ddy};

        for (int j = 0; j < n_obs; j++) {
            float t;
            if (intersect_ray_obstacle(o, d, obstacles[j], t)) {
                if (t < best_t) best_t = t;
            }
        }
        ranges_out[env_idx] = best_t;
    }
}

#endif // USE_AVX2

// ═════════════════════════════════════════════════════════════════
//  batch_lidar_raycast_shared
// ═════════════════════════════════════════════════════════════════

void batch_lidar_raycast_shared(BatchSimWorld& world,
    const float* cos_angles, const float* sin_angles,
    int n_beams, float range_max, float* ranges_out) {

    int bs = world.batch_size();

    #pragma omp parallel for schedule(static, 4)
    for (int b = 0; b < n_beams; b++) {
        float* beam_out = ranges_out + b * world.alloc_size();
        float ca = cos_angles[b];
        float sa = sin_angles[b];

#if defined(USE_AVX2)
        int simd_bs = world.alloc_size();
        for (int env_start = 0; env_start < simd_bs; env_start += SIMD_WIDTH) {
            if (env_start >= bs) break;
            int n = std::min(SIMD_WIDTH, bs - env_start);
            batch_beam_avx2(world, ca, sa, env_start, n, range_max, beam_out);
        }
#else
        batch_beam_scalar(world, ca, sa, 0, bs, range_max, beam_out);
#endif
    }
}

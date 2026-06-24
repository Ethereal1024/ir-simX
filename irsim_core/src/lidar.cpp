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

// Compute 8 cos/sin values from 8 angles using Intel DSPL or approximation.
// We use a small polynomial approximation: fast, ~1e-5 accuracy.
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
    // Initialise all ranges to range_max
    for (int i = 0; i < n_beams; i++) ranges_out[i] = range_max;

    // We process obstacles one by one: for each obstacle, update ranges
    // where a closer hit is found.
    for (int j = 0; j < n_obs; j++) {
        const Obstacle& obs = obstacles[j];

        if (obs.type == ShapeType::CIRCLE) {
            // ── SIMD-hot path: CIRCLE obstacles ──
            // Ray-circle intersection: solve |o + t*d - c|^2 = r^2
            //   →  t^2 - 2*d·(c-o)*t + |c-o|^2 - r^2 = 0
            //   →  a=1, b=-2*d·(c-o), c_ = |c-o|^2 - r^2
            // We only need the smaller positive root.

            // oc = center - origin  (direction from origin to center)
            __m256 oc_x = _mm256_set1_ps(obs.center.x - origin.x);
            __m256 oc_y = _mm256_set1_ps(obs.center.y - origin.y);
            __m256 r2   = _mm256_set1_ps(obs.radius * obs.radius);
            __m256 rmax = _mm256_set1_ps(range_max);

            for (int i = 0; i < n_beams; i += 8) {
                int remaining = n_beams - i;
                int k = (remaining < 8) ? remaining : 8;

                float angle_full[8], cs_buf[8], sn_buf[8];
                for (int t = 0; t < k; t++) angle_full[t] = angles[i + t] + heading;
                sincos_8(angle_full, cs_buf, sn_buf);

                // Load direction vectors
                __m256 dx = _mm256_loadu_ps(cs_buf);
                __m256 dy = _mm256_loadu_ps(sn_buf);

                // b = -2*(dx*oc_x + dy*oc_y)
                __m256 b = _mm256_mul_ps(dx, oc_x);
                b = _mm256_fmadd_ps(dy, oc_y, b);
                b = _mm256_mul_ps(b, _mm256_set1_ps(-2.0f));

                // c_ = oc_x^2 + oc_y^2 - r^2
                __m256 c_v = _mm256_fmadd_ps(oc_x, oc_x, _mm256_mul_ps(oc_y, oc_y));
                c_v = _mm256_sub_ps(c_v, r2);

                // discriminant = b^2 - 4*c_
                __m256 disc = _mm256_fmadd_ps(b, b, _mm256_mul_ps(c_v, _mm256_set1_ps(-4.0f)));

                // sqrt(discriminant) using _mm256_sqrt_ps
                __m256 sqrt_disc = _mm256_sqrt_ps(_mm256_max_ps(disc, _mm256_setzero_ps()));

                // t1 = (-b - sqrt_disc) * 0.5
                __m256 t1 = _mm256_mul_ps(
                    _mm256_sub_ps(_mm256_xor_ps(b, _mm256_set1_ps(-0.0f)), sqrt_disc),
                    _mm256_set1_ps(0.5f));

                // t2 = (-b + sqrt_disc) * 0.5
                __m256 t2 = _mm256_mul_ps(
                    _mm256_add_ps(_mm256_xor_ps(b, _mm256_set1_ps(-0.0f)), sqrt_disc),
                    _mm256_set1_ps(0.5f));

                // invalid = disc < 0
                __m256 invalid_mask = _mm256_cmp_ps(disc, _mm256_setzero_ps(), _CMP_LT_OS);

                // valid if t >= 0
                __m256 zero = _mm256_setzero_ps();
                __m256 t1_ok = _mm256_cmp_ps(t1, zero, _CMP_GE_OS);
                __m256 t2_ok = _mm256_cmp_ps(t2, zero, _CMP_GE_OS);

                // choose t1 if t1 >= 0, else t2, else invalid
                __m256 t_hit = _mm256_blendv_ps(t2, t1, t1_ok);
                __m256 hit_ok = _mm256_or_ps(t1_ok, t2_ok);
                hit_ok = _mm256_andnot_ps(invalid_mask, hit_ok);

                // clamp to [0, range_max]
                t_hit = _mm256_max_ps(t_hit, zero);
                t_hit = _mm256_min_ps(t_hit, rmax);

                // load current ranges, update if closer
                __m256 cur = _mm256_loadu_ps(ranges_out + i);
                __m256 closer = _mm256_cmp_ps(t_hit, cur, _CMP_LT_OS);
                __m256 new_val = _mm256_blendv_ps(cur, t_hit, closer);
                // only update where hit_ok is true
                __m256 mask = _mm256_and_ps(closer, hit_ok);
                new_val = _mm256_blendv_ps(cur, t_hit, mask);

                // mask for remaining < 8
                if (k < 8) {
                    int mask_int = (1 << k) - 1;
                    __m256i blend_mask = _mm256_set_epi32(
                        (mask_int >> 7) & 1, (mask_int >> 6) & 1,
                        (mask_int >> 5) & 1, (mask_int >> 4) & 1,
                        (mask_int >> 3) & 1, (mask_int >> 2) & 1,
                        (mask_int >> 1) & 1, (mask_int >> 0) & 1);
                    new_val = _mm256_blendv_ps(
                        _mm256_loadu_ps(ranges_out + i),
                        new_val,
                        _mm256_castsi256_ps(blend_mask));
                }

                _mm256_storeu_ps(ranges_out + i, new_val);
            }
        } else {
            // ── Non-CIRCLE: scalar fallback ──
            // For each beam, check this obstacle
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

#else  // !USE_AVX2

void lidar_raycast_avx2(
    Vec2 origin, float heading,
    const float* angles, int n_beams, float range_max,
    const Obstacle* obstacles, int n_obs,
    float* ranges_out)
{
    // Fallback: just call scalar version
    lidar_raycast_scalar(origin, heading, angles, n_beams, range_max, obstacles, n_obs, ranges_out);
}

#endif  // USE_AVX2

// ═══════════════════════════════════════════════════════════════
//  Auto-select implementation
// ═══════════════════════════════════════════════════════════════

void lidar_raycast(
    Vec2 origin, float heading,
    const float* angles, int n_beams, float range_max,
    const Obstacle* obstacles, int n_obs,
    float* ranges_out)
{
#ifdef USE_AVX2
    // Use AVX2 if at least 8 beams and any CIRCLE obstacles benefit
    if (n_beams >= 8) {
        lidar_raycast_avx2(origin, heading, angles, n_beams, range_max, obstacles, n_obs, ranges_out);
        return;
    }
#endif
    lidar_raycast_scalar(origin, heading, angles, n_beams, range_max, obstacles, n_obs, ranges_out);
}

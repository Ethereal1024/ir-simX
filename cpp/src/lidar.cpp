#include "lidar.h"
#include <cmath>
#include <cstring>
#include <cfloat>
#include <algorithm>
#ifdef USE_AVX2
#include <immintrin.h>
#endif

// ═══════════════════════════════════════════════════════════════
//  SpatialHashGrid — build & raycast (unified: all shapes)
// ═══════════════════════════════════════════════════════════════

void SpatialHashGrid::build(const Obstacle* obstacles, int n_obs, bool include_cr) {
    shapes_.clear();
    cells_.clear();
    n_circle_ = 0;
    n_rect_ = 0;
    if (n_obs == 0) return;

    // 1. Collect all shapes and compute world AABB
    float min_x = FLT_MAX, min_y = FLT_MAX;
    float max_x = -FLT_MAX, max_y = -FLT_MAX;

    for (int j = 0; j < n_obs; j++) {
        const auto& obs = obstacles[j];

        if (obs.type == ShapeType::CIRCLE) {
            if (include_cr) {
                GridShape gs;
                gs.type = ShapeType::CIRCLE;
                gs.center = obs.center;
                gs.radius = obs.radius;
                shapes_.push_back(gs);
                n_circle_++;
                float cx = obs.center.x, cy = obs.center.y, r = obs.radius;
                min_x = std::min(min_x, cx - r);
                max_x = std::max(max_x, cx + r);
                min_y = std::min(min_y, cy - r);
                max_y = std::max(max_y, cy + r);
            } else {
                // Still need AABB for grid sizing even if not indexing
                float cx = obs.center.x, cy = obs.center.y, r = obs.radius;
                min_x = std::min(min_x, cx - r);
                max_x = std::max(max_x, cx + r);
                min_y = std::min(min_y, cy - r);
                max_y = std::max(max_y, cy + r);
            }

        } else if (obs.type == ShapeType::RECT) {
            if (include_cr) {
                // Store as 4 segments (matching Python obj_to_c_dict behavior)
                float c = std::cos(obs.theta), si = std::sin(obs.theta);
                float cx = obs.center.x, cy = obs.center.y;
                float hw = obs.half_w, hh = obs.half_h;
                Vec2 v0{cx + hw * c - hh * si, cy + hw * si + hh * c};
                Vec2 v1{cx - hw * c - hh * si, cy - hw * si + hh * c};
                Vec2 v2{cx - hw * c + hh * si, cy - hw * si - hh * c};
                Vec2 v3{cx + hw * c + hh * si, cy + hw * si - hh * c};
                Vec2 edges[4][2] = {{v0, v1}, {v1, v2}, {v2, v3}, {v3, v0}};
                for (int e = 0; e < 4; e++) {
                    GridShape gs;
                    gs.type = ShapeType::LINESTRING;
                    gs.a = edges[e][0];
                    gs.b = edges[e][1];
                    shapes_.push_back(gs);
                }
                n_rect_++;
            }
            min_x = std::min(min_x, obs.aabb.min.x);
            max_x = std::max(max_x, obs.aabb.max.x);
            min_y = std::min(min_y, obs.aabb.min.y);
            max_y = std::max(max_y, obs.aabb.max.y);

        } else if ((obs.type == ShapeType::POLYGON ||
                     obs.type == ShapeType::LINESTRING) &&
                    obs.verts && obs.n_verts >= 2) {
            bool is_line = (obs.type == ShapeType::LINESTRING);
            int n_seg = is_line ? obs.n_verts - 1 : obs.n_verts;
            for (int v = 0; v < n_seg; v++) {
                const Vec2& a = obs.verts[v];
                const Vec2& b = obs.verts[(v + 1) % obs.n_verts];
                GridShape gs;
                gs.type = ShapeType::LINESTRING;  // reuse as "segment"
                gs.a = a;
                gs.b = b;
                shapes_.push_back(gs);
                min_x = std::min({min_x, a.x, b.x});
                max_x = std::max({max_x, a.x, b.x});
                min_y = std::min({min_y, a.y, b.y});
                max_y = std::max({max_y, a.y, b.y});
            }
        }
    }

    if (shapes_.empty()) return;

    // 2. Allocate grid
    float margin = cell_size_;
    ox_ = min_x - margin;
    oy_ = min_y - margin;
    ow_ = (max_x - min_x) + 2 * margin;
    oh_ = (max_y - min_y) + 2 * margin;
    nx_ = std::max(1, (int)std::ceil(ow_ / cell_size_));
    ny_ = std::max(1, (int)std::ceil(oh_ / cell_size_));
    cells_.resize(nx_ * ny_);

    // 3. Insert shapes into cells by their AABB
    for (size_t si = 0; si < shapes_.size(); si++) {
        AABB saabb;
        const auto& s = shapes_[si];
        if (s.type == ShapeType::CIRCLE) {
            saabb = AABB{s.center - Vec2{s.radius, s.radius},
                          s.center + Vec2{s.radius, s.radius}};
        } else if (s.type == ShapeType::RECT) {
            saabb = s.aabb;
        } else {
            // SEGMENT
            float sx = std::min(s.a.x, s.b.x);
            float sy = std::min(s.a.y, s.b.y);
            float ex = std::max(s.a.x, s.b.x);
            float ey = std::max(s.a.y, s.b.y);
            saabb = AABB{Vec2{sx, sy}, Vec2{ex, ey}};
        }
        insert_shape((int)si, saabb);
    }
}

inline void SpatialHashGrid::insert_shape(int shape_idx, const AABB& saabb) {
    int cx1 = std::max(0, (int)((saabb.min.x - ox_) / cell_size_));
    int cy1 = std::max(0, (int)((saabb.min.y - oy_) / cell_size_));
    int cx2 = std::min(nx_ - 1, (int)((saabb.max.x - ox_) / cell_size_));
    int cy2 = std::min(ny_ - 1, (int)((saabb.max.y - oy_) / cell_size_));

    for (int cy = cy1; cy <= cy2; cy++) {
        for (int cx = cx1; cx <= cx2; cx++) {
            cells_[cy * nx_ + cx].push_back(shape_idx);
        }
    }
}

float SpatialHashGrid::raycast(Vec2 o, Vec2 d, float limit) const {
    if (shapes_.empty()) return limit;

    float inv_dx = (std::abs(d.x) > 1e-12f) ? 1.0f / d.x : 0;
    float inv_dy = (std::abs(d.y) > 1e-12f) ? 1.0f / d.y : 0;

    float cx_f = (o.x - ox_) / cell_size_;
    float cy_f = (o.y - oy_) / cell_size_;
    int cx = std::max(0, std::min(nx_ - 1, (int)std::floor(cx_f)));
    int cy = std::max(0, std::min(ny_ - 1, (int)std::floor(cy_f)));

    int step_x = (d.x > 0) ? 1 : (d.x < 0) ? -1 : 0;
    int step_y = (d.y > 0) ? 1 : (d.y < 0) ? -1 : 0;

    float t_max_x, t_max_y, t_delta_x, t_delta_y;

    if (std::abs(d.x) > 1e-12f) {
        float x_bound = (d.x > 0) ? (cx + 1) * cell_size_ + ox_ : cx * cell_size_ + ox_;
        t_max_x = (x_bound - o.x) * inv_dx;
        t_delta_x = std::abs(cell_size_ * inv_dx);
    } else {
        t_max_x = FLT_MAX; t_delta_x = FLT_MAX;
    }

    if (std::abs(d.y) > 1e-12f) {
        float y_bound = (d.y > 0) ? (cy + 1) * cell_size_ + oy_ : cy * cell_size_ + oy_;
        t_max_y = (y_bound - o.y) * inv_dy;
        t_delta_y = std::abs(cell_size_ * inv_dy);
    } else {
        t_max_y = FLT_MAX; t_delta_y = FLT_MAX;
    }

    int max_steps = nx_ + ny_ + 4;

    for (int step = 0; step < max_steps; step++) {
        int cell_idx = cy * nx_ + cx;
        if (cell_idx >= 0 && cell_idx < (int)cells_.size()) {
            for (int si : cells_[cell_idx]) {
                const auto& s = shapes_[si];
                float t_val;

                if (s.type == ShapeType::CIRCLE) {
                    Vec2 oc = o - s.center;
                    float b = 2.0f * d.dot(oc);
                    float cc = oc.len2() - s.radius * s.radius;
                    float disc = b * b - 4.0f * cc;
                    if (disc < 0) continue;
                    float sqrt_disc = std::sqrt(disc);
                    float t1 = (-b - sqrt_disc) * 0.5f;
                    float t2 = (-b + sqrt_disc) * 0.5f;
                    t_val = (t1 >= 0) ? t1 : t2;
                    if (t_val < 0) continue;

                } else if (s.type == ShapeType::RECT) {
                    // RECT stored as 4 segments; this branch is no longer reached.
                    // Safety fallback: Cyrus-Beck on the 4 actual vertices.
                    float c = std::cos(s.theta), si = std::sin(s.theta);
                    float hw = s.half_w, hh = s.half_h;
                    float cx = s.center.x, cy = s.center.y;
                    Vec2 rverts[4] = {
                        {cx + hw * c - hh * si, cy + hw * si + hh * c},
                        {cx - hw * c - hh * si, cy - hw * si + hh * c},
                        {cx - hw * c + hh * si, cy - hw * si - hh * c},
                        {cx + hw * c + hh * si, cy + hw * si - hh * c},
                    };
                    if (!intersect_ray_convex_polygon(o, d, rverts, 4, t_val))
                        continue;

                } else {
                    // SEGMENT (polygon/linestring edge)
                    Vec2 ab = s.b - s.a;
                    float denom = d.cross(ab);
                    if (std::abs(denom) < 1e-12f) continue;
                    Vec2 ao = s.a - o;
                    t_val = ao.cross(ab) / denom;
                    float u = ao.cross(d) / denom;
                    // Valid intersection: segment is in front (t_val > 0)
                    // and hit point is within segment bounds (u in [0, 1])
                    if (t_val > 1e-6f && u >= -1e-6f && u <= 1.0f + 1e-6f) {
                        // normal hit
                    } else if (std::abs(t_val) <= 1e-6f && u >= -1e-6f && u <= 1.0f + 1e-6f) {
                        // origin on segment
                        t_val = 0;
                    } else {
                        continue;
                    }
                }

                if (t_val < limit) {
                    limit = t_val;
                    if (limit <= 0) return 0;
                }
            }
        }

        float next_t = std::min(t_max_x, t_max_y);
        if (limit <= next_t + 1e-8f) break;

        if (t_max_x < t_max_y) {
            cx += step_x;
            if (cx < 0 || cx >= nx_) break;
            t_max_x += t_delta_x;
        } else {
            cy += step_y;
            if (cy < 0 || cy >= ny_) break;
            t_max_y += t_delta_y;
        }
    }

    return std::max(0.0f, limit);
}

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
    float* ranges_out,
    const SpatialHashGrid* grid)
{
    for (int i = 0; i < n_beams; i++) ranges_out[i] = range_max;

    // Count CIRCLE + RECT for dispatch decision
    int n_cr = 0;
    for (int j = 0; j < n_obs; j++) {
        auto tt = obstacles[j].type;
        if (tt == ShapeType::CIRCLE || tt == ShapeType::RECT) n_cr++;
    }

    // Grid covers C/R → use grid for everything
    bool grid_has_cr = (grid && grid->num_circle_rect() > 0);
    // Grid is polyline-only + C/R count is small → do SIMD for C/R
    bool simd_cr = (!grid_has_cr && n_cr > 0 &&
                    n_cr <= SpatialHashGrid::CR_SIMD_THRESHOLD);

    if (simd_cr) {
        // Phase 1a: per-obstacle SIMD for CIRCLE, scalar Cyrus-Beck for RECT
        for (int j = 0; j < n_obs; j++) {
            const Obstacle& obs = obstacles[j];
            if (!(obs.type == ShapeType::CIRCLE || obs.type == ShapeType::RECT)) continue;

            if (obs.type == ShapeType::RECT) {
                // Rotated rect: scalar per-beam Cyrus-Beck (SIMD doesn't apply)
                float c = std::cos(obs.theta), si = std::sin(obs.theta);
                float cx = obs.center.x, cy = obs.center.y;
                float hw = obs.half_w, hh = obs.half_h;
                Vec2 rverts[4] = {
                    {cx + hw * c - hh * si, cy + hw * si + hh * c},
                    {cx - hw * c - hh * si, cy - hw * si + hh * c},
                    {cx - hw * c + hh * si, cy - hw * si - hh * c},
                    {cx + hw * c + hh * si, cy + hw * si - hh * c},
                };
                for (int i = 0; i < n_beams; i++) {
                    float angle = angles[i] + heading;
                    Vec2 dir{std::cos(angle), std::sin(angle)};
                    float t;
                    if (intersect_ray_convex_polygon(origin, dir, rverts, 4, t)) {
                        if (t < ranges_out[i]) ranges_out[i] = t;
                    }
                }
                continue;
            }

            __m256 rmax = _mm256_set1_ps(range_max);
            __m256 zero = _mm256_setzero_ps();
            __m256 sign_mask = _mm256_set1_ps(-0.0f);

            __m256 oc_x = {}, oc_y = {}, r2 = {};

            oc_x = _mm256_set1_ps(obs.center.x - origin.x);
            oc_y = _mm256_set1_ps(obs.center.y - origin.y);
            r2   = _mm256_set1_ps(obs.radius * obs.radius);

            for (int i = 0; i < n_beams; i += 8) {
                int remaining = n_beams - i;
                int k = (remaining < 8) ? remaining : 8;

                float angle_full[8], cs_buf[8], sn_buf[8];
                for (int t = 0; t < k; t++) angle_full[t] = angles[i + t] + heading;
                sincos_8(angle_full, cs_buf, sn_buf);
                __m256 dx = _mm256_loadu_ps(cs_buf);
                __m256 dy = _mm256_loadu_ps(sn_buf);

                __m256 t_hit, hit_ok;
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
        }
    }
    if (grid && !grid->empty()) {
        for (int i = 0; i < n_beams; i++) {
            float angle = angles[i] + heading;
            Vec2 dir{std::cos(angle), std::sin(angle)};
            float best = ranges_out[i];
            float t = grid->raycast(origin, dir, best);
            if (t < best) ranges_out[i] = t;
        }
    } else {
        // No grid — scalar fallback for poly/linestring only (C/R already done by SIMD)
        for (int j = 0; j < n_obs; j++) {
            const auto& obs = obstacles[j];
            if (obs.type == ShapeType::CIRCLE || obs.type == ShapeType::RECT) continue;
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
    float* ranges_out,
    const SpatialHashGrid* grid)
{
    // Non-AVX2 fallback: use grid if available, otherwise scalar
    if (grid && !grid->empty()) {
        for (int i = 0; i < n_beams; i++) {
            float angle = angles[i] + heading;
            Vec2 dir{std::cos(angle), std::sin(angle)};
            ranges_out[i] = grid->raycast(origin, dir, range_max);
        }
    } else {
        lidar_raycast_scalar(origin, heading, angles, n_beams, range_max,
                             obstacles, n_obs, ranges_out);
    }
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
    // Count CIRCLE+RECT to decide grid build mode
    int n_cr = 0;
    for (int j = 0; j < n_obs; j++) {
        auto tt = obstacles[j].type;
        if (tt == ShapeType::CIRCLE || tt == ShapeType::RECT) n_cr++;
    }
    bool build_full_grid = (n_cr > SpatialHashGrid::CR_SIMD_THRESHOLD);
    SpatialHashGrid grid;
    grid.build(obstacles, n_obs, build_full_grid);
    lidar_raycast_avx2(origin, heading, angles, n_beams, range_max,
                       obstacles, n_obs, ranges_out,
                       grid.empty() ? nullptr : &grid);
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

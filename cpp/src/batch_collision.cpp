#include "batch_world.h"
#include <cmath>
#include <cfloat>

#if defined(USE_AVX2)
#include <immintrin.h>
#endif

void batch_detect_collisions(BatchSimWorld& world) {
    int bs = world.batch_size();
    if (bs == 0) return;

    // 1. Compute per-environment robot AABBs
    std::vector<AABB> robot_aabbs(bs);
    for (int i = 0; i < bs; i++) {
        world.compute_aabb_scalar(i, robot_aabbs[i]);
    }

    const auto& obstacles = world.obstacles();
    int n_obs = world.share_obstacles() ? (int)obstacles.size() : 0;

    // 2. For each obstacle, check overlap
    if (world.share_obstacles() && n_obs > 0) {
        for (int j = 0; j < n_obs; j++) {
            const auto& obs = obstacles[j];
            AABB oaabb = obs.aabb;

#if defined(USE_AVX2)
            int simd_bs = world.alloc_size();
            __m256 o_min_x = _mm256_set1_ps(oaabb.min.x);
            __m256 o_max_x = _mm256_set1_ps(oaabb.max.x);
            __m256 o_min_y = _mm256_set1_ps(oaabb.min.y);
            __m256 o_max_y = _mm256_set1_ps(oaabb.max.y);

            for (int i = 0; i < simd_bs; i += SIMD_WIDTH) {
                if (i >= bs) break;
                int n = std::min(SIMD_WIDTH, bs - i);

                alignas(32) float r_min_x[SIMD_WIDTH], r_max_x[SIMD_WIDTH];
                alignas(32) float r_min_y[SIMD_WIDTH], r_max_y[SIMD_WIDTH];
                for (int k = 0; k < n; k++) {
                    r_min_x[k] = robot_aabbs[i + k].min.x;
                    r_max_x[k] = robot_aabbs[i + k].max.x;
                    r_min_y[k] = robot_aabbs[i + k].min.y;
                    r_max_y[k] = robot_aabbs[i + k].max.y;
                }
                for (int k = n; k < SIMD_WIDTH; k++) {
                    r_min_x[k] = r_max_x[k] = r_min_y[k] = r_max_y[k] = 0;
                }

                __m256 r_minx = _mm256_loadu_ps(r_min_x);
                __m256 r_maxx = _mm256_loadu_ps(r_max_x);
                __m256 r_miny = _mm256_loadu_ps(r_min_y);
                __m256 r_maxy = _mm256_loadu_ps(r_max_y);

                __m256 overlap_x = _mm256_and_ps(
                    _mm256_cmp_ps(r_maxx, o_min_x, _CMP_GE_OQ),
                    _mm256_cmp_ps(o_max_x, r_minx, _CMP_GE_OQ));
                __m256 overlap_y = _mm256_and_ps(
                    _mm256_cmp_ps(r_maxy, o_min_y, _CMP_GE_OQ),
                    _mm256_cmp_ps(o_max_y, r_miny, _CMP_GE_OQ));
                __m256 overlap = _mm256_and_ps(overlap_x, overlap_y);

                int mask = _mm256_movemask_ps(overlap);
                if (mask == 0) continue;

                for (int k = 0; k < n; k++) {
                    if (!(mask & (1 << k))) continue;
                    int env_idx = i + k;
                    // Check if already collided via the public is_collision method
                    // (we write collision_flags_ directly via friend)
                    const auto& wv = world.world_vertices(env_idx);
                    if (check_robot_obstacle_collision(
                            wv.data(), world.n_verts(), obs)) {
                        world.set_collision(env_idx, true);
                    }
                }
            }
#else  // !USE_AVX2
            for (int i = 0; i < bs; i++) {
                if (world.collision(i)) continue;
                if (!robot_aabbs[i].overlaps(oaabb)) continue;
                const auto& wv = world.world_vertices(i);
                if (check_robot_obstacle_collision(
                        wv.data(), world.n_verts(), obs)) {
                    world.set_collision(i, true);
                }
            }
#endif  // USE_AVX2
        }
    } else if (!world.share_obstacles()) {
        for (int i = 0; i < bs; i++) {
            if (world.collision(i)) continue;
            const auto& env_obs = world.obstacles_per_env(i);
            const auto& wv = world.world_vertices(i);
            for (const auto& obs : env_obs) {
                if (!robot_aabbs[i].overlaps(obs.aabb)) continue;
                if (check_robot_obstacle_collision(
                        wv.data(), world.n_verts(), obs)) {
                    world.set_collision(i, true);
                    break;
                }
            }
        }
    }
}

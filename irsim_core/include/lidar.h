#pragma once
#include "geometry.h"
#include <cstdint>

// ═══════════════════════════════════════════════════════════════
//  LiDAR raycasting — scalar and SIMD accelerated
// ═══════════════════════════════════════════════════════════════

// Scalar raycast: for each beam, test all obstacles, keep nearest hit.
// angles: array of n_beams angles in radians (relative to robot heading)
// ranges_out: output array, filled with hit distances (range_max if no hit)
void lidar_raycast_scalar(
    Vec2 origin, float heading,
    const float* angles, int n_beams, float range_max,
    const Obstacle* obstacles, int n_obs,
    float* ranges_out);

// AVX2 batch raycast: process 8 beams at once using SIMD.
// Only benefits from CIRCLE obstacles (vectorized quadratic solve).
// Falls back to scalar for other types.
void lidar_raycast_avx2(
    Vec2 origin, float heading,
    const float* angles, int n_beams, float range_max,
    const Obstacle* obstacles, int n_obs,
    float* ranges_out);

// Auto-select implementation based on CPU features.
void lidar_raycast(
    Vec2 origin, float heading,
    const float* angles, int n_beams, float range_max,
    const Obstacle* obstacles, int n_obs,
    float* ranges_out);

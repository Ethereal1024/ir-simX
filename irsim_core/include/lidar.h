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

// FMCW LiDAR raycast: returns ranges + per-beam radial velocity.
// sensor_vx, sensor_vy: velocity of the sensor (robot) in world frame.
// motion_compensate: if true, use obstacle world velocity directly;
//   if false, subtract sensor velocity (relative velocity in sensor frame).
// velocities_out: radial velocity per beam (dot(rel_vel, beam_dir)).
//   Set to 0 for beams with no hit.
void fmcw_lidar_raycast(
    Vec2 origin, float heading,
    float sensor_vx, float sensor_vy,
    bool motion_compensate,
    const float* angles, int n_beams, float range_max,
    const Obstacle* obstacles, int n_obs,
    float* ranges_out,
    float* velocities_out);

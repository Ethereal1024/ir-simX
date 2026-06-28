#pragma once
#include "geometry.h"
#include <cstdint>
#include <vector>
#include <algorithm>

// ═══════════════════════════════════════════════════════════════
//  SpatialHashGrid — spatial grid for accelerating ray
//  intersection queries for all obstacle types.
//
//  All obstacle shapes are binned into grid cells by AABB.
//  DDA ray traversal visits only cells the ray passes through,
//  enabling early termination at O(√N) instead of O(N).
// ═══════════════════════════════════════════════════════════════

struct GridShape {
    ShapeType type;
    // CIRCLE
    Vec2 center;
    float radius;
    // RECT
    float half_w, half_h;
    float theta;  // rotation angle
    AABB aabb;
    // SEGMENT (from polygon/linestring edges)
    Vec2 a, b;
};

class SpatialHashGrid {
public:
    static constexpr float DEFAULT_CELL_SIZE = 0.5f;

    SpatialHashGrid() = default;

    // Build from obstacle list.  All shape types are indexed.
    void build(const Obstacle* obstacles, int n_obs, bool include_cr = true);

    // Raycast: returns nearest intersection distance (limit if no hit).
    float raycast(Vec2 o, Vec2 d, float limit) const;

    bool empty() const { return shapes_.empty(); }
    int num_circle_rect() const { return n_circle_ + n_rect_; }

    // C/R → SIMD threshold for AVX2 single-env path.
    // When CIRCLE+RECT count ≤ this value, they use per-obstacle SIMD
    // instead of the grid (which has overhead for small counts).
    static constexpr int CR_SIMD_THRESHOLD = 50;

private:
    float cell_size_ = DEFAULT_CELL_SIZE;
    int nx_ = 0, ny_ = 0;
    float ox_ = 0, oy_ = 0;
    float ow_ = 0, oh_ = 0;

    std::vector<GridShape> shapes_;
    std::vector<std::vector<int>> cells_;  // cells_[y * nx + x] → shape indices
    int n_circle_ = 0, n_rect_ = 0;

    void insert_shape(int shape_idx, const AABB& shape_aabb);
};

// ═══════════════════════════════════════════════════════════════
//  LiDAR raycasting — scalar and SIMD accelerated
// ═══════════════════════════════════════════════════════════════

// Scalar raycast: for each beam, test all obstacles, keep nearest hit.
void lidar_raycast_scalar(
    Vec2 origin, float heading,
    const float* angles, int n_beams, float range_max,
    const Obstacle* obstacles, int n_obs,
    float* ranges_out);

// AVX2 batch raycast: process 8 beams at once using SIMD.
// Accelerates CIRCLE (quadratic solve) and RECT (AABB slab test).
// Uses SpatialHashGrid for POLYGON/LINESTRING obstacles.
// When grid covers CIRCLE/RECT (i.e. built with all shapes),
// the grid is queried for all obstacles; otherwise per-obstacle
// SIMD is used for CIRCLE/RECT and grid for polygon/linestring.
void lidar_raycast_avx2(
    Vec2 origin, float heading,
    const float* angles, int n_beams, float range_max,
    const Obstacle* obstacles, int n_obs,
    float* ranges_out,
    const SpatialHashGrid* grid = nullptr);

// FMCW AVX2 variant: same SIMD intersection + per-beam best obstacle
// index tracking for radial velocity computation.
void fmcw_lidar_raycast_avx2(
    Vec2 origin, float heading,
    float sensor_vx, float sensor_vy,
    bool motion_compensate,
    const float* angles, int n_beams, float range_max,
    const Obstacle* obstacles, int n_obs,
    float* ranges_out,
    float* velocities_out);

// Auto-select implementation based on CPU features.
void lidar_raycast(
    Vec2 origin, float heading,
    const float* angles, int n_beams, float range_max,
    const Obstacle* obstacles, int n_obs,
    float* ranges_out);

// FMCW LiDAR raycast: returns ranges + per-beam radial velocity.
void fmcw_lidar_raycast(
    Vec2 origin, float heading,
    float sensor_vx, float sensor_vy,
    bool motion_compensate,
    const float* angles, int n_beams, float range_max,
    const Obstacle* obstacles, int n_obs,
    float* ranges_out,
    float* velocities_out);

#pragma once
#include "simd_config.h"
#include "geometry.h"
#include "collision.h"
#include "kinematics.h"
#include <deque>
#include <vector>
#include <cstring>
#include <cfloat>

// ═══════════════════════════════════════════════════════════════
//  BatchSimWorld — SIMD multi-environment simulation world
//  SoA (Structure of Arrays) layout for SIMD-friendly access.
// ═══════════════════════════════════════════════════════════════

class BatchSimWorld {
public:
    explicit BatchSimWorld(const BatchConfig& cfg);

    // ── Setup ──
    void set_step_time(float dt) { dt_ = dt; }
    float step_time() const { return dt_; }
    int batch_size() const { return cfg_.batch_size; }
    int alloc_size() const { return alloc_size_; }

    // Robot kinematics type (same for all environments in batch)
    void set_robot_kinematics(KinematicsType kin) { kin_type_ = kin; }
    KinematicsType robot_kinematics() const { return kin_type_; }

    // Robot shape vertices (local frame, same shape for all envs)
    void set_robot_vertices(const float* verts, int n);
    void set_robot_limits(const float* vel_min, const float* vel_max,
                          const float* vel_acc);

    // Set initial poses for all environments in the batch
    // poses: flat array [batch_size * 3] = [x0,y0,theta0, x1,y1,theta1, ...]
    void set_initial_poses(const float* poses);

    // ── Obstacles ──
    // Mode A (share_obstacles): single obstacle set for all envs
    void add_obstacle(const Obstacle& obs);
    // Mode B (per-environment): add obstacle to specific env
    void add_obstacle_per_env(int env_id, const Obstacle& obs);

    int num_obstacles() const;
    int num_obstacles_per_env(int env_id) const;

    // Polygon and linestring obstacles (vertex data persisted internally)
    void add_polygon_obstacle(const std::vector<Vec2>& verts);
    void add_linestring_obstacle(const std::vector<Vec2>& verts);
    const std::deque<std::vector<Vec2>>& polygon_vertices() const { return polygon_vertices_; }

    // ── Step ──
    // actions: flat array [action_dim * batch_size]
    // Each robot's action occupies action_dim contiguous floats
    void step(const float* actions, int action_dim);

    // ── LiDAR ──
    // All environments share the same beam angles.
    // ranges_out: [n_beams * batch_size], beam-major layout
    //   (beam0_env0, beam0_env1, ..., beam0_envN, beam1_env0, ...)
    // Mode A (share_obstacles): cross-environment SIMD
    // Mode B (per-env): scalar per-environment fallback
    void batch_raycast(const float* angles, int n_beams, float range_max,
                       float* ranges_out);

    // ── Bulk getters (C++ → Python) ──
    // out: length = batch_size * 3 (interleaved: x,y,theta per env)
    void get_all_poses(float* out) const;
    void get_all_velocities(float* out) const;
    // out: length = batch_size (bool as float 0/1 for numpy compat)
    void get_all_collisions(float* out) const;

    // ── SoA raw data access (for SIMD kernel friends) ──
    float* x_data()             { return x_.data(); }
    float* y_data()             { return y_.data(); }
    float* theta_data()         { return theta_.data(); }
    float* vx_data()            { return vx_.data(); }
    float* vy_data()            { return vy_.data(); }
    float* omega_data()         { return omega_.data(); }
    float* sin_theta_data()     { return sin_theta_.data(); }
    float* cos_theta_data()     { return cos_theta_.data(); }
    const float* x_data() const       { return x_.data(); }
    const float* y_data() const       { return y_.data(); }
    const float* theta_data() const   { return theta_.data(); }

    float& vel_min_x(int i)           { return vel_min_x_[i]; }
    float& vel_min_y(int i)           { return vel_min_y_[i]; }
    float& vel_min_omega(int i)       { return vel_min_omega_[i]; }
    float& vel_max_x(int i)           { return vel_max_x_[i]; }
    float& vel_max_y(int i)           { return vel_max_y_[i]; }
    float& vel_max_omega(int i)       { return vel_max_omega_[i]; }
    float& vel_acc_x(int i)           { return vel_acc_x_[i]; }
    float& vel_acc_y(int i)           { return vel_acc_y_[i]; }
    float& vel_acc_omega(int i)       { return vel_acc_omega_[i]; }

    int n_verts() const               { return n_verts_; }
    const Vec2* local_verts() const   { return local_vertices_.data(); }
    const std::vector<Vec2>& world_vertices(int i) const { return world_vertices_[i]; }

    const std::vector<Obstacle>& obstacles() const { return obstacles_; }
    const std::vector<Obstacle>& obstacles_per_env(int i) const { return per_env_obstacles_[i]; }
    bool share_obstacles() const { return share_obstacles_; }
    float steer_angle(int i) const { return steer_angles_[i]; }
    float& steer_angle(int i) { return steer_angles_[i]; }

    float dt() const { return dt_; }
    bool collision(int i) const { return collision_flags_[i]; }
    void set_collision(int i, bool val) { collision_flags_[i] = val; }

private:
    BatchConfig cfg_;
    int alloc_size_ = 0;    // padded to SIMD_WIDTH boundary
    float dt_ = 0.1f;
    KinematicsType kin_type_ = KinematicsType::DIFF;

    // ── SoA core ──────────────────────────────────────────────
    // Each vector length = alloc_size_ (padded, aligned to SIMD_WIDTH)
    std::vector<float> x_, y_, theta_;
    std::vector<float> vx_, vy_, omega_;
    std::vector<float> sin_theta_, cos_theta_;

    // Per-environment velocity/acceleration limits (3 dims each)
    std::vector<float> vel_min_x_, vel_min_y_, vel_min_omega_;
    std::vector<float> vel_max_x_, vel_max_y_, vel_max_omega_;
    std::vector<float> vel_acc_x_, vel_acc_y_, vel_acc_omega_;

    // ── Shared robot geometry ─────────────────────────────────
    // All environments share the same robot shape
    std::vector<Vec2> local_vertices_;
    int n_verts_ = 0;

    // Per-environment world-space vertices (updated each step)
    // world_vertices_[env] = vector<Vec2>
    std::vector<std::vector<Vec2>> world_vertices_;

    // ── Obstacles ─────────────────────────────────────────────
    bool share_obstacles_ = true;
    std::vector<Obstacle> obstacles_;                     // mode A: shared
    std::vector<std::vector<Obstacle>> per_env_obstacles_; // mode B
    std::deque<std::vector<Vec2>> polygon_vertices_;      // persistent vertex storage

    // ── State tracking ───────────────────────────────────────
    std::vector<bool> collision_flags_;
    std::vector<float> steer_angles_;  // for ACKER kinematics

    // ── Internal helpers ──
    int simd_round_up(int n) const;

    void alloc_arrays();
    void update_trig_arrays(int start, int n);
    void find_world_vertices_scalar(int env_id);
    void compute_aabb_scalar(int env_id, AABB& out) const;

    // SIMD kernels (implemented in batch_kinematics.cpp)
    friend void batch_clip_actions(BatchSimWorld& world,
                                   const float* actions, int action_dim,
                                   int start, int n);
    friend void batch_step_diff(BatchSimWorld& world, int start, int n);
    friend void batch_step_omni(BatchSimWorld& world, int start, int n);
    friend void batch_step_omni_angular(BatchSimWorld& world, int start, int n);
    friend void batch_step_acker(BatchSimWorld& world, int start, int n);

    // LiDAR (implemented in batch_lidar.cpp)
    friend void batch_lidar_raycast_shared(BatchSimWorld& world,
        const float* cos_angles, const float* sin_angles,
        int n_beams, float range_max, float* ranges_out);
    friend void batch_lidar_raycast_per_env(BatchSimWorld& world,
        const float* cos_angles, const float* sin_angles,
        int n_beams, float range_max, float* ranges_out);

    // Collision (implemented in batch_collision.cpp)
    friend void batch_detect_collisions(BatchSimWorld& world);
};

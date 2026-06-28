#include "batch_world.h"
#include <algorithm>
#include <cstring>
#include <cfloat>
#include <cmath>

#if defined(USE_AVX2)
#include <immintrin.h>
#endif

// ── Construction ────────────────────────────────────────────────

BatchSimWorld::BatchSimWorld(const BatchConfig& cfg)
    : cfg_(cfg)
    , share_obstacles_(cfg.share_obstacles)
{
    alloc_size_ = simd_round_up(cfg.batch_size);
    alloc_arrays();
}

int BatchSimWorld::simd_round_up(int n) const {
    if (SIMD_WIDTH <= 1) return n;
    return ((n + SIMD_WIDTH - 1) / SIMD_WIDTH) * SIMD_WIDTH;
}

void BatchSimWorld::alloc_arrays() {
    int n = alloc_size_;
    x_.assign(n, 0.0f);
    y_.assign(n, 0.0f);
    theta_.assign(n, 0.0f);
    vx_.assign(n, 0.0f);
    vy_.assign(n, 0.0f);
    omega_.assign(n, 0.0f);
    sin_theta_.assign(n, 0.0f);
    cos_theta_.assign(n, 1.0f);

    vel_min_x_.assign(n, -FLT_MAX);
    vel_min_y_.assign(n, -FLT_MAX);
    vel_min_omega_.assign(n, -FLT_MAX);
    vel_max_x_.assign(n, FLT_MAX);
    vel_max_y_.assign(n, FLT_MAX);
    vel_max_omega_.assign(n, FLT_MAX);
    vel_acc_x_.assign(n, FLT_MAX);
    vel_acc_y_.assign(n, FLT_MAX);
    vel_acc_omega_.assign(n, FLT_MAX);

    world_vertices_.resize(cfg_.batch_size);
    collision_flags_.assign(cfg_.batch_size, false);
    steer_angles_.assign(n, 0.0f);

    if (!share_obstacles_) {
        per_env_obstacles_.resize(cfg_.batch_size);
    }
}

// ── Robot setup ─────────────────────────────────────────────────

void BatchSimWorld::set_robot_vertices(const float* verts, int n) {
    local_vertices_.resize(n);
    for (int i = 0; i < n; i++) {
        local_vertices_[i] = {verts[i * 2], verts[i * 2 + 1]};
    }
    n_verts_ = n;

    // Init per-environment world vertices
    for (int i = 0; i < cfg_.batch_size; i++) {
        world_vertices_[i].resize(n);
    }
}

void BatchSimWorld::set_robot_limits(const float* vel_min,
                                      const float* vel_max,
                                      const float* vel_acc) {
    for (int i = 0; i < cfg_.batch_size; i++) {
        vel_min_x_[i] = vel_min[0];
        vel_min_y_[i] = vel_min[1];
        vel_min_omega_[i] = vel_min[2];
        vel_max_x_[i] = vel_max[0];
        vel_max_y_[i] = vel_max[1];
        vel_max_omega_[i] = vel_max[2];
        vel_acc_x_[i] = vel_acc[0];
        vel_acc_y_[i] = vel_acc[1];
        vel_acc_omega_[i] = vel_acc[2];
    }
}

void BatchSimWorld::set_initial_poses(const float* poses) {
    for (int i = 0; i < cfg_.batch_size; i++) {
        x_[i] = poses[i * 3];
        y_[i] = poses[i * 3 + 1];
        theta_[i] = poses[i * 3 + 2];
    }
    update_trig_arrays(0, cfg_.batch_size);
}

// ── Obstacles ───────────────────────────────────────────────────

void BatchSimWorld::add_obstacle(const Obstacle& obs) {
    if (share_obstacles_) {
        obstacles_.push_back(obs);
        rebuild_lidar_grid();
    }
}

void BatchSimWorld::add_polygon_obstacle(const std::vector<Vec2>& verts) {
    if (!share_obstacles_) return;
    polygon_vertices_.push_back(verts);
    Obstacle o;
    o.type = ShapeType::POLYGON;
    o.n_verts = (int)verts.size();
    o.verts = polygon_vertices_.back().data();
    o.center = {0, 0};
    for (const auto& v : verts) { o.center.x += v.x; o.center.y += v.y; }
    o.center.x /= verts.size();
    o.center.y /= verts.size();
    o.compute_aabb();
    obstacles_.push_back(o);
}

void BatchSimWorld::add_linestring_obstacle(const std::vector<Vec2>& verts) {
    if (!share_obstacles_ || verts.size() < 2) return;
    polygon_vertices_.push_back(verts);
    Obstacle o;
    o.type = ShapeType::LINESTRING;
    o.n_verts = (int)verts.size();
    o.verts = polygon_vertices_.back().data();
    o.compute_aabb();
    obstacles_.push_back(o);
}

void BatchSimWorld::add_obstacle_per_env(int env_id, const Obstacle& obs) {
    if (!share_obstacles_ && env_id >= 0 && env_id < cfg_.batch_size) {
        per_env_obstacles_[env_id].push_back(obs);
    }
}

int BatchSimWorld::num_obstacles() const {
    return (int)obstacles_.size();
}

int BatchSimWorld::num_obstacles_per_env(int env_id) const {
    if (share_obstacles_) return (int)obstacles_.size();
    if (env_id >= 0 && env_id < (int)per_env_obstacles_.size())
        return (int)per_env_obstacles_[env_id].size();
    return 0;
}

// ── Trig helpers ────────────────────────────────────────────────

void BatchSimWorld::update_trig_arrays(int start, int n) {
    for (int i = start; i < start + n; i++) {
        cos_theta_[i] = std::cos(theta_[i]);
        sin_theta_[i] = std::sin(theta_[i]);
    }
}

// ── World vertex transform (per environment, scalar) ────────────

void BatchSimWorld::find_world_vertices_scalar(int env_id) {
    float c = cos_theta_[env_id];
    float s = sin_theta_[env_id];
    float cx = x_[env_id];
    float cy = y_[env_id];
    auto& wv = world_vertices_[env_id];
    for (int j = 0; j < n_verts_; j++) {
        const auto& lv = local_vertices_[j];
        wv[j].x = cx + lv.x * c - lv.y * s;
        wv[j].y = cy + lv.x * s + lv.y * c;
    }
}

void BatchSimWorld::compute_aabb_scalar(int env_id, AABB& out) const {
    out = AABB();
    const auto& wv = world_vertices_[env_id];
    for (int j = 0; j < n_verts_; j++) {
        out.expand(wv[j]);
    }
}

// ── Step (OpenMP + SIMD) ───────────────────────────────────────

void BatchSimWorld::step(const float* actions, int action_dim) {
    int bs = cfg_.batch_size;

    // 1. Clip actions (SIMD per-chunk, sequentially for now)
    batch_clip_actions(*this, actions, action_dim, 0, bs);

    // 2. Step kinematics (SIMD per-chunk, sequentially for now)
    switch (kin_type_) {
    case KinematicsType::DIFF:
        batch_step_diff(*this, 0, bs);
        break;
    case KinematicsType::OMNI:
        batch_step_omni(*this, 0, bs);
        break;
    case KinematicsType::OMNI_ANGULAR:
        batch_step_omni_angular(*this, 0, bs);
        break;
    case KinematicsType::ACKER:
        batch_step_acker(*this, 0, bs);
        break;
    }

    // 3. Update trig arrays (OpenMP parallel for)
    #pragma omp parallel for schedule(static, 64)
    for (int i = 0; i < bs; i++) {
        cos_theta_[i] = std::cos(theta_[i]);
        sin_theta_[i] = std::sin(theta_[i]);
    }

    // 4. Find world vertices (OpenMP parallel for)
    if (n_verts_ > 0) {
        #pragma omp parallel for schedule(static, 8)
        for (int i = 0; i < bs; i++) {
            find_world_vertices_scalar(i);
        }
    }

    // 5. Detect collisions (OpenMP parallel over environments)
    collision_flags_.assign(bs, false);
    if (n_verts_ > 0) {
        const auto& obstacles = obstacles_;
        int n_obs = share_obstacles_ ? (int)obstacles.size() : 0;

        if (share_obstacles_ && n_obs > 0) {
            #pragma omp parallel for schedule(dynamic, 32)
            for (int i = 0; i < bs; i++) {
                AABB env_aabb;
                compute_aabb_scalar(i, env_aabb);
                for (int j = 0; j < n_obs; j++) {
                    if (!env_aabb.overlaps(obstacles[j].aabb)) continue;
                    const auto& wv = world_vertices_[i];
                    if (check_robot_obstacle_collision(
                            wv.data(), n_verts_, obstacles[j])) {
                        collision_flags_[i] = true;
                        break;
                    }
                }
            }
        } else if (!share_obstacles_) {
            #pragma omp parallel for schedule(dynamic, 32)
            for (int i = 0; i < bs; i++) {
                const auto& env_obs = per_env_obstacles_[i];
                const auto& wv = world_vertices_[i];
                for (const auto& obs : env_obs) {
                    AABB env_aabb;
                    compute_aabb_scalar(i, env_aabb);
                    if (!env_aabb.overlaps(obs.aabb)) continue;
                    if (check_robot_obstacle_collision(
                            wv.data(), n_verts_, obs)) {
                        collision_flags_[i] = true;
                        break;
                    }
                }
            }
        }
    }
}

// ── LiDAR ───────────────────────────────────────────────────────

void BatchSimWorld::rebuild_lidar_grid() {
    lidar_grid_.build(obstacles_.data(), (int)obstacles_.size());
}

void BatchSimWorld::batch_raycast(const float* angles, int n_beams,
                                   float range_max, float* ranges_out) {
    // Precompute beam direction cos/sin
    std::vector<float> cos_angles(n_beams);
    std::vector<float> sin_angles(n_beams);
    for (int b = 0; b < n_beams; b++) {
        cos_angles[b] = std::cos(angles[b]);
        sin_angles[b] = std::sin(angles[b]);
    }

    if (share_obstacles_) {
        // Mode A: cross-environment SIMD
        batch_lidar_raycast_shared(*this, cos_angles.data(), sin_angles.data(),
                                    n_beams, range_max, ranges_out);
    } else {
        // Mode B: per-environment scalar
        batch_lidar_raycast_per_env(*this, cos_angles.data(), sin_angles.data(),
                                     n_beams, range_max, ranges_out);
    }
}

// ── Bulk getters ────────────────────────────────────────────────

void BatchSimWorld::get_all_poses(float* out) const {
    for (int i = 0; i < cfg_.batch_size; i++) {
        out[i * 3]     = x_[i];
        out[i * 3 + 1] = y_[i];
        out[i * 3 + 2] = theta_[i];
    }
}

void BatchSimWorld::get_all_velocities(float* out) const {
    for (int i = 0; i < cfg_.batch_size; i++) {
        out[i * 3]     = vx_[i];
        out[i * 3 + 1] = vy_[i];
        out[i * 3 + 2] = omega_[i];
    }
}

void BatchSimWorld::get_all_collisions(float* out) const {
    for (int i = 0; i < cfg_.batch_size; i++) {
        out[i] = collision_flags_[i] ? 1.0f : 0.0f;
    }
}

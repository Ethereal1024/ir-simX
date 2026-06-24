#pragma once
#include "geometry.h"
#include "lidar.h"
#include "collision.h"
#include "kinematics.h"
#include "astar.h"
#include <vector>
#include <cstring>

// ═══════════════════════════════════════════════════════════════
//  Robot state
// ═══════════════════════════════════════════════════════════════

struct RobotState {
    float x = 0, y = 0, theta = 0;     // pose
    float vx = 0, vy = 0, omega = 0;   // velocity
    KinematicsType kin = KinematicsType::DIFF;
    std::vector<Vec2> local_vertices;    // shape vertices in robot frame (never modified)
    std::vector<Vec2> world_vertices;    // shape vertices in world frame (updated each step)

    // Velocity and acceleration limits
    float vel_min[3] = {-1.0f, -1.0f, -1.0f};
    float vel_max[3] = { 1.0f,  1.0f,  1.0f};
    float vel_acc[3] = { 1.0f,  1.0f,  1.0f};  // acceleration limit (max delta per step)

    // Collision flags
    bool collision = false;
    bool arrived = false;

    // ID for Python sync
    int id = 0;
};

// ═══════════════════════════════════════════════════════════════
//  Core simulation world
// ═══════════════════════════════════════════════════════════════

class SimWorld {
public:
    // ── Setup ──
    void set_step_time(float dt) { dt_ = dt; }
    float step_time() const { return dt_; }

    // Add objects (called from Python during init)
    int add_robot(KinematicsType kin, float x, float y, float theta,
                  const float* vel_min = nullptr,
                  const float* vel_max = nullptr,
                  const float* vel_acc = nullptr);
    void set_robot_vertices(int robot_id, const Vec2* verts, int n);
    int add_obstacle(const Obstacle& obs);
    int add_polygon_obstacle(const std::vector<Vec2>& verts);

    // ── Step ──
    // Advance all robots by one step with given actions.
    // actions: flat array [action_dim * n_robots]
    void step(const float* actions, int action_dim);

    // LiDAR: cast from a robot's position
    void raycast(int robot_id,
                 const float* angles, int n_beams, float range_max,
                 float* ranges_out);

    // Collision check for a specific robot
    bool check_robot_collision(int robot_id);

    // ── Access ──
    int num_robots() const { return (int)robots_.size(); }
    int num_obstacles() const { return (int)obstacles_.size(); }

    const RobotState& robot(int id) const { return robots_[id]; }
    RobotState& robot(int id) { return robots_[id]; }

    const std::vector<Obstacle>& obstacles() const { return obstacles_; }
    AStarPlanner& astar() { return astar_; }

    const std::vector<std::vector<Vec2>>& polygon_vertices() const { return polygon_vertices_; }

private:
    float dt_ = 0.1f;
    std::vector<RobotState> robots_;
    std::vector<Obstacle> obstacles_;
    std::vector<std::vector<Vec2>> polygon_vertices_;  // persistent storage for polygon obs
    AStarPlanner astar_;
    int next_id_ = 0;

    void update_robot_aabb(RobotState& robot, const Vec2* verts, int n);
};

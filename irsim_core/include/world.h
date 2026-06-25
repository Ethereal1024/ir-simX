#pragma once
#include "geometry.h"
#include "lidar.h"
#include "collision.h"
#include "kinematics.h"
#include "astar.h"
#include <vector>
#include <deque>
#include <cstring>

// ═══════════════════════════════════════════════════════════════
//  Robot state
// ═══════════════════════════════════════════════════════════════

struct RobotState {
    float x = 0, y = 0, theta = 0;     // pose
    float vx = 0, vy = 0, omega = 0;   // velocity
    float steer_angle = 0;              // current steer angle (ACKER only)
    float wheelbase = 0.5f;             // wheelbase (ACKER only, default 0.5)
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
//  Dynamic obstacle state (has kinematics, moves each step)
// ═══════════════════════════════════════════════════════════════

struct DynamicObstacle {
    float x = 0, y = 0, theta = 0;
    float vx = 0, vy = 0, omega = 0;
    float steer_angle = 0;              // current steer angle (ACKER only)
    float wheelbase = 0.5f;             // wheelbase (ACKER only)
    KinematicsType kin = KinematicsType::DIFF;
    ShapeType shape_type = ShapeType::CIRCLE;

    // For circle
    float radius = 0.5f;

    // For rect
    float half_w = 0.5f, half_h = 0.5f;

    // For polygon: vertices stored persistently
    std::vector<Vec2> local_vertices;     // vertices relative to initial center
    float init_center_x = 0, init_center_y = 0;  // initial absolute center

    // For linestring: vertices stored persistently (same as polygon but open chain)
    std::vector<Vec2> local_linestring_verts;

    // Collision flag (set by detect_collisions when overlapping another object)
    bool collision = false;

    // Velocity and acceleration limits
    float vel_min[3] = {-1.0f, -1.0f, -1.0f};
    float vel_max[3] = { 1.0f,  1.0f,  1.0f};
    float vel_acc[3] = { 1.0f,  1.0f,  1.0f};
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
                  const float* vel_acc = nullptr,
                  float wheelbase = 0.5f);
    void set_robot_vertices(int robot_id, const Vec2* verts, int n);
    int add_obstacle(const Obstacle& obs);
    int add_polygon_obstacle(const std::vector<Vec2>& verts);
    int add_linestring_obstacle(const std::vector<Vec2>& verts);

    // Add a dynamic obstacle (has kinematics, moves each step).
    // The obstacle geometry is also added to obstacles_ for collision.
    // Returns the dynamic obstacle id.
    int add_dynamic_obstacle(KinematicsType kin, float x, float y, float theta,
                             float radius,  // for circle
                             const float* vel_min = nullptr,
                             const float* vel_max = nullptr,
                             const float* vel_acc = nullptr);

    int add_dynamic_rect_obstacle(KinematicsType kin, float x, float y, float theta,
                                   float half_w, float half_h,
                                   const float* vel_min = nullptr,
                                   const float* vel_max = nullptr,
                                   const float* vel_acc = nullptr);

    // Add a dynamic polygon obstacle.
    // verts: absolute world-coordinate vertices (same as add_polygon_obstacle).
    int add_dynamic_polygon_obstacle(KinematicsType kin, float x, float y, float theta,
                                     const std::vector<Vec2>& verts,
                                     const float* vel_min = nullptr,
                                     const float* vel_max = nullptr,
                                     const float* vel_acc = nullptr);

    // Add a dynamic linestring obstacle.
    // verts: absolute world-coordinate vertices (open chain, n ≥ 2).
    int add_dynamic_linestring_obstacle(KinematicsType kin, float x, float y, float theta,
                                        const std::vector<Vec2>& verts,
                                        const float* vel_min = nullptr,
                                        const float* vel_max = nullptr,
                                        const float* vel_acc = nullptr);

    // ── Step ──
    // Advance all robots by one step with given actions.
    // actions: flat array [action_dim * n_robots]
    void step(const float* actions, int action_dim);

    // Advance all dynamic obstacles with given behavior velocities.
    // obs_actions: flat array [action_dim * n_dynamic_obs]
    void step_dynamic_obstacles(const float* obs_actions, int action_dim);

    // LiDAR: cast from a robot's position
    void raycast(int robot_id,
                 const float* angles, int n_beams, float range_max,
                 float* ranges_out);

    // Collision check for a specific robot
    bool check_robot_collision(int robot_id);

    // ── Access ──
    int num_robots() const { return (int)robots_.size(); }
    int num_obstacles() const { return (int)obstacles_.size(); }
    int num_dynamic_obstacles() const { return (int)dyn_obstacles_.size(); }

    const RobotState& robot(int id) const { return robots_[id]; }
    RobotState& robot(int id) { return robots_[id]; }

    const DynamicObstacle& dynamic_obstacle(int id) const { return dyn_obstacles_[id]; }
    DynamicObstacle& dynamic_obstacle(int id) { return dyn_obstacles_[id]; }

    void set_obstacle_velocity(int id, float vx, float vy) {
        if (id >= 0 && id < (int)dyn_obstacles_.size()) {
            dyn_obstacles_[id].vx = vx;
            dyn_obstacles_[id].vy = vy;
        }
    }

    const std::vector<Obstacle>& obstacles() const { return obstacles_; }
    AStarPlanner& astar() { return astar_; }

    const std::deque<std::vector<Vec2>>& polygon_vertices() const { return polygon_vertices_; }

private:
    float dt_ = 0.1f;
    std::vector<RobotState> robots_;
    std::vector<DynamicObstacle> dyn_obstacles_;
    std::vector<Obstacle> obstacles_;
    std::deque<std::vector<Vec2>> polygon_vertices_;  // persistent storage (deque: no reallocation on push)
    AStarPlanner astar_;
    int next_id_ = 0;

    void update_robot_aabb(RobotState& robot, const Vec2* verts, int n);

    // Run collision detection for all robots against all obstacles
    void detect_collisions();
};

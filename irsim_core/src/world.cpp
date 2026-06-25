#include "world.h"
#include <cmath>
#include <cstring>

int SimWorld::add_robot(KinematicsType kin, float x, float y, float theta,
                        const float* vel_min, const float* vel_max,
                        const float* vel_acc) {
    RobotState r;
    r.x = x; r.y = y; r.theta = theta;
    r.kin = kin;
    r.id = next_id_++;

    if (vel_min) for (int i = 0; i < 3; i++) r.vel_min[i] = vel_min[i];
    if (vel_max) for (int i = 0; i < 3; i++) r.vel_max[i] = vel_max[i];
    if (vel_acc) for (int i = 0; i < 3; i++) r.vel_acc[i] = vel_acc[i];

    // Default rectangle local vertices (0.32 x 0.24)
    float hw = 0.16f, hh = 0.12f;
    r.local_vertices = {{-hw, -hh}, {hw, -hh}, {hw, hh}, {-hw, hh}};
    r.world_vertices = r.local_vertices;  // initial pose has theta=0
    robots_.push_back(r);
    return r.id;
}

void SimWorld::set_robot_vertices(int robot_id, const Vec2* verts, int n) {
    if (robot_id < 0 || robot_id >= (int)robots_.size()) return;
    auto& r = robots_[robot_id];
    r.local_vertices.assign(verts, verts + n);
    r.world_vertices = r.local_vertices;
}

int SimWorld::add_obstacle(const Obstacle& obs) {
    Obstacle o = obs;
    if (o.type == ShapeType::POLYGON) {
        // Vertices must be stored persistently in polygon_vertices_
        // before calling compute_aabb (which uses verts pointer)
    }
    o.compute_aabb();
    obstacles_.push_back(o);
    return (int)obstacles_.size() - 1;
}

int SimWorld::add_polygon_obstacle(const std::vector<Vec2>& verts) {
    Obstacle o;
    o.type = ShapeType::POLYGON;
    o.n_verts = (int)verts.size();
    polygon_vertices_.push_back(verts);
    o.verts = polygon_vertices_.back().data();
    o.center = {0, 0};  // polygons use absolute vertices, center not relevant
    for (const auto& v : verts) {
        o.center.x += v.x; o.center.y += v.y;
    }
    o.center.x /= verts.size();
    o.center.y /= verts.size();
    o.compute_aabb();
    obstacles_.push_back(o);
    return (int)obstacles_.size() - 1;
}

int SimWorld::add_dynamic_obstacle(KinematicsType kin, float x, float y, float theta,
                                    float radius,
                                    const float* vel_min,
                                    const float* vel_max,
                                    const float* vel_acc)
{
    DynamicObstacle dob;
    dob.x = x; dob.y = y; dob.theta = theta;
    dob.kin = kin;

    if (vel_min) for (int i = 0; i < 3; i++) dob.vel_min[i] = vel_min[i];
    if (vel_max) for (int i = 0; i < 3; i++) dob.vel_max[i] = vel_max[i];
    if (vel_acc) for (int i = 0; i < 3; i++) dob.vel_acc[i] = vel_acc[i];

    // Add corresponding collision geometry to obstacles_
    Obstacle obs;
    obs.type = ShapeType::CIRCLE;
    obs.center = {x, y};
    obs.radius = radius;
    obs.compute_aabb();
    obstacles_.push_back(obs);
    dob.obs_index = (int)obstacles_.size() - 1;

    int id = (int)dyn_obstacles_.size();
    dyn_obstacles_.push_back(dob);
    return id;
}

void SimWorld::update_dyn_obs_geometry(int dyn_id) {
    auto& dob = dyn_obstacles_[dyn_id];
    auto& obs = obstacles_[dob.obs_index];
    obs.center = {dob.x, dob.y};
    obs.compute_aabb();
}

void SimWorld::step_dynamic_obstacles(const float* obs_actions, int action_dim) {
    for (int i = 0; i < (int)dyn_obstacles_.size(); i++) {
        auto& dob = dyn_obstacles_[i];
        const float* act = obs_actions + i * action_dim;

        // Clip action the same way as robots
        float clipped[3];
        float cur_vel[3] = {dob.vx, dob.vy, dob.omega};
        for (int j = 0; j < action_dim && j < 3; j++) {
            float lo = cur_vel[j] - dob.vel_acc[j] * dt_;
            float hi = cur_vel[j] + dob.vel_acc[j] * dt_;
            if (lo < dob.vel_min[j]) lo = dob.vel_min[j];
            if (hi > dob.vel_max[j]) hi = dob.vel_max[j];
            float a = act[j];
            if (a < lo) a = lo;
            if (a > hi) a = hi;
            clipped[j] = a;
        }

        step_kinematics(dob.kin, dob.x, dob.y, dob.theta, clipped, dt_);

        dob.vx = clipped[0];
        dob.vy = (action_dim >= 2) ? clipped[1] : 0;
        dob.omega = (action_dim >= 3) ? clipped[2] : (action_dim >= 2 ? clipped[1] : 0);

        // Keep obstacle collision geometry in sync
        update_dyn_obs_geometry(i);
    }
}

// ── Helper: transform robot local vertices to world ────────────
static void transform_vertices(const Vec2* local, int n,
                               float x, float y, float theta,
                               Vec2* world_out)
{
    float c = std::cos(theta), s = std::sin(theta);
    for (int i = 0; i < n; i++) {
        world_out[i].x = x + local[i].x * c - local[i].y * s;
        world_out[i].y = y + local[i].x * s + local[i].y * c;
    }
}

void SimWorld::step(const float* actions, int action_dim) {
    for (int i = 0; i < (int)robots_.size(); i++) {
        auto& r = robots_[i];
        const float* act = actions + i * action_dim;

        // Clip action: acceleration limits first (mirrors Python get_vel_range),
        // then velocity limits.
        float clipped[3];
        float cur_vel[3] = {r.vx, r.vy, r.omega};
        for (int j = 0; j < action_dim && j < 3; j++) {
            // Acceleration clip: current +- accel*dt
            float dt = dt_;
            float lo = cur_vel[j] - r.vel_acc[j] * dt;
            float hi = cur_vel[j] + r.vel_acc[j] * dt;
            // Velocity clip
            if (lo < r.vel_min[j]) lo = r.vel_min[j];
            if (hi > r.vel_max[j]) hi = r.vel_max[j];
            float a = act[j];
            if (a < lo) a = lo;
            if (a > hi) a = hi;
            clipped[j] = a;
        }

        step_kinematics(r.kin, r.x, r.y, r.theta, clipped, dt_);

        r.vx = clipped[0];
        r.vy = (action_dim >= 2) ? clipped[1] : 0;
        r.omega = (action_dim >= 3) ? clipped[2] : (action_dim >= 2 ? clipped[1] : 0);

        // Transform local vertices to world frame (fresh each step, no accumulation)
        float c = std::cos(r.theta), s = std::sin(r.theta);
        r.world_vertices.resize(r.local_vertices.size());
        for (size_t vi = 0; vi < r.local_vertices.size(); vi++) {
            r.world_vertices[vi].x = r.x + r.local_vertices[vi].x * c - r.local_vertices[vi].y * s;
            r.world_vertices[vi].y = r.y + r.local_vertices[vi].x * s + r.local_vertices[vi].y * c;
        }
    }

    // Collision detection
    for (auto& r : robots_) {
        r.collision = false;
        for (const auto& obs : obstacles_) {
            if (check_robot_obstacle_collision(
                    r.world_vertices.data(), (int)r.world_vertices.size(), obs))
            {
                r.collision = true;
                break;
            }
        }
    }
}

void SimWorld::raycast(int robot_id,
                       const float* angles, int n_beams, float range_max,
                       float* ranges_out)
{
    if (robot_id < 0 || robot_id >= (int)robots_.size()) {
        std::memset(ranges_out, 0, n_beams * sizeof(float));
        return;
    }
    const auto& r = robots_[robot_id];
    Vec2 origin{r.x, r.y};

    lidar_raycast(origin, r.theta, angles, n_beams, range_max,
                  obstacles_.data(), (int)obstacles_.size(),
                  ranges_out);
}

bool SimWorld::check_robot_collision(int robot_id) {
    if (robot_id < 0 || robot_id >= (int)robots_.size()) return false;
    return robots_[robot_id].collision;
}

#include "world.h"
#include <cmath>
#include <cstring>

int SimWorld::add_robot(KinematicsType kin, float x, float y, float theta,
                        const float* vel_min, const float* vel_max,
                        const float* vel_acc, float wheelbase) {
    RobotState r;
    r.x = x; r.y = y; r.theta = theta;
    r.kin = kin;
    r.wheelbase = wheelbase;
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
    rebuild_lidar_grid();
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
    rebuild_lidar_grid();
    return (int)obstacles_.size() - 1;
}

int SimWorld::add_linestring_obstacle(const std::vector<Vec2>& verts) {
    if (verts.size() < 2) return -1;
    Obstacle o;
    o.type = ShapeType::LINESTRING;
    o.n_verts = (int)verts.size();
    polygon_vertices_.push_back(verts);
    o.verts = polygon_vertices_.back().data();
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
    dob.shape_type = ShapeType::CIRCLE;
    dob.radius = radius;

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
    dob.obs_index = obstacles_.size() - 1;

    int id = (int)dyn_obstacles_.size();
    dyn_obstacles_.push_back(dob);
    return id;
}

int SimWorld::add_dynamic_rect_obstacle(KinematicsType kin, float x, float y, float theta,
                                          float half_w, float half_h,
                                          const float* vel_min,
                                          const float* vel_max,
                                          const float* vel_acc)
{
    DynamicObstacle dob;
    dob.x = x; dob.y = y; dob.theta = theta;
    dob.kin = kin;
    dob.shape_type = ShapeType::RECT;
    dob.half_w = half_w;
    dob.half_h = half_h;

    if (vel_min) for (int i = 0; i < 3; i++) dob.vel_min[i] = vel_min[i];
    if (vel_max) for (int i = 0; i < 3; i++) dob.vel_max[i] = vel_max[i];
    if (vel_acc) for (int i = 0; i < 3; i++) dob.vel_acc[i] = vel_acc[i];

    Obstacle obs;
    obs.type = ShapeType::RECT;
    obs.center = {x, y};
    obs.half_w = half_w;
    obs.half_h = half_h;
    obs.theta = theta;
    obs.compute_aabb();
    obstacles_.push_back(obs);
    dob.obs_index = obstacles_.size() - 1;

    int id = (int)dyn_obstacles_.size();
    dyn_obstacles_.push_back(dob);
    return id;
}

int SimWorld::add_dynamic_polygon_obstacle(KinematicsType kin, float x, float y, float theta,
                                            const std::vector<Vec2>& verts,
                                            const float* vel_min,
                                            const float* vel_max,
                                            const float* vel_acc)
{
    DynamicObstacle dob;
    dob.x = x; dob.y = y; dob.theta = theta;
    dob.kin = kin;
    dob.shape_type = ShapeType::POLYGON;

    if (vel_min) for (int i = 0; i < 3; i++) dob.vel_min[i] = vel_min[i];
    if (vel_max) for (int i = 0; i < 3; i++) dob.vel_max[i] = vel_max[i];
    if (vel_acc) for (int i = 0; i < 3; i++) dob.vel_acc[i] = vel_acc[i];

    // Store polygon vertices persistently
    dob.local_vertices = verts;
    dob.init_center_x = x;
    dob.init_center_y = y;

    // Add obstacle geometry to obstacles_
    polygon_vertices_.push_back(verts);
    dob.poly_verts_index = polygon_vertices_.size() - 1;
    Obstacle obs;
    obs.type = ShapeType::POLYGON;
    obs.n_verts = (int)verts.size();
    obs.verts = polygon_vertices_.back().data();
    obs.center = {0, 0};
    for (const auto& v : verts) {
        obs.center.x += v.x; obs.center.y += v.y;
    }
    obs.center.x /= (float)verts.size();
    obs.center.y /= (float)verts.size();
    obs.compute_aabb();
    obstacles_.push_back(obs);
    dob.obs_index = obstacles_.size() - 1;

    int id = (int)dyn_obstacles_.size();
    dyn_obstacles_.push_back(dob);
    return id;
}

int SimWorld::add_dynamic_linestring_obstacle(KinematicsType kin, float x, float y, float theta,
                                                const std::vector<Vec2>& verts,
                                                const float* vel_min,
                                                const float* vel_max,
                                                const float* vel_acc)
{
    if (verts.size() < 2) return -1;

    DynamicObstacle dob;
    dob.x = x; dob.y = y; dob.theta = theta;
    dob.kin = kin;
    dob.shape_type = ShapeType::LINESTRING;

    if (vel_min) for (int i = 0; i < 3; i++) dob.vel_min[i] = vel_min[i];
    if (vel_max) for (int i = 0; i < 3; i++) dob.vel_max[i] = vel_max[i];
    if (vel_acc) for (int i = 0; i < 3; i++) dob.vel_acc[i] = vel_acc[i];

    // Store linestring vertices persistently
    dob.local_linestring_verts = verts;
    dob.init_center_x = x;
    dob.init_center_y = y;

    // Add obstacle geometry to obstacles_
    polygon_vertices_.push_back(verts);
    dob.poly_verts_index = polygon_vertices_.size() - 1;
    Obstacle obs;
    obs.type = ShapeType::LINESTRING;
    obs.n_verts = (int)verts.size();
    obs.verts = polygon_vertices_.back().data();
    obs.compute_aabb();
    obstacles_.push_back(obs);
    dob.obs_index = obstacles_.size() - 1;

    int id = (int)dyn_obstacles_.size();
    dyn_obstacles_.push_back(dob);
    return id;
}

void SimWorld::step_dynamic_obstacles(const float* obs_actions, int action_dim) {
    for (int i = 0; i < (int)dyn_obstacles_.size(); i++) {
        auto& dob = dyn_obstacles_[i];

        if (dob.obs_index == SIZE_MAX || dob.obs_index >= obstacles_.size()) continue;
        auto& obs = obstacles_[dob.obs_index];

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

        step_kinematics(dob.kin, dob.x, dob.y, dob.theta, &dob.steer_angle, dob.wheelbase, clipped, dt_);

        dob.vx = clipped[0];
        dob.vy = (action_dim >= 2) ? clipped[1] : 0;
        dob.omega = (action_dim >= 3) ? clipped[2] : (action_dim >= 2 ? clipped[1] : 0);

        // Keep obstacle collision geometry in sync via stored obs_index
        if (dob.shape_type == ShapeType::CIRCLE) {
            obs.center = {dob.x, dob.y};
        } else if (dob.shape_type == ShapeType::RECT) {
            obs.center = {dob.x, dob.y};
            obs.theta = dob.theta;
        } else if (dob.shape_type == ShapeType::POLYGON) {
            if (dob.poly_verts_index == SIZE_MAX) continue;
            float dx = dob.x - dob.init_center_x;
            float dy = dob.y - dob.init_center_y;
            float c = std::cos(dob.theta), s = std::sin(dob.theta);
            auto& pv = polygon_vertices_[dob.poly_verts_index];
            pv.resize(dob.local_vertices.size());
            obs.verts = pv.data();
            obs.n_verts = (int)dob.local_vertices.size();
            for (size_t vi = 0; vi < dob.local_vertices.size(); vi++) {
                float rx = dob.local_vertices[vi].x - dob.init_center_x;
                float ry = dob.local_vertices[vi].y - dob.init_center_y;
                pv[vi].x = dob.x + rx * c - ry * s;
                pv[vi].y = dob.y + rx * s + ry * c;
            }
            obs.center = {dob.x, dob.y};
        } else if (dob.shape_type == ShapeType::LINESTRING) {
            if (dob.poly_verts_index == SIZE_MAX) continue;
            float dx = dob.x - dob.init_center_x;
            float dy = dob.y - dob.init_center_y;
            float c = std::cos(dob.theta), s = std::sin(dob.theta);
            auto& pv = polygon_vertices_[dob.poly_verts_index];
            pv.resize(dob.local_linestring_verts.size());
            obs.verts = pv.data();
            obs.n_verts = (int)dob.local_linestring_verts.size();
            for (size_t vi = 0; vi < dob.local_linestring_verts.size(); vi++) {
                float rx = dob.local_linestring_verts[vi].x - dob.init_center_x;
                float ry = dob.local_linestring_verts[vi].y - dob.init_center_y;
                pv[vi].x = dob.x + rx * c - ry * s;
                pv[vi].y = dob.y + rx * s + ry * c;
            }
        }
        obs.compute_aabb();
    }

    // Re-run collision detection with updated obstacle positions
    detect_collisions();
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

        step_kinematics(r.kin, r.x, r.y, r.theta, &r.steer_angle, r.wheelbase, clipped, dt_);

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

    detect_collisions();
}

void SimWorld::detect_collisions() {
    // Reset all robot collision flags
    for (auto& r : robots_) {
        r.collision = false;
    }

    // Robot vs robot collision (SAT on world vertices)
    for (size_t ri = 0; ri < robots_.size(); ri++) {
        for (size_t rj = ri + 1; rj < robots_.size(); rj++) {
            if (sat_intersect(robots_[ri].world_vertices.data(),
                              (int)robots_[ri].world_vertices.size(),
                              robots_[rj].world_vertices.data(),
                              (int)robots_[rj].world_vertices.size())) {
                robots_[ri].collision = true;
                robots_[rj].collision = true;
            }
        }
    }

    // Robot collisions (robot vs all obstacles)
    for (auto& r : robots_) {
        for (size_t oi = 0; oi < obstacles_.size(); oi++) {
            const auto& obs = obstacles_[oi];
            if (check_robot_obstacle_collision(
                    r.world_vertices.data(), (int)r.world_vertices.size(), obs))
            {
                r.collision = true;
                break;
            }
        }
    }
    // Dynamic obstacle collisions (obstacle vs robot, obstacle vs obstacle)
    for (size_t di = 0; di < dyn_obstacles_.size(); di++) {
        auto& dob = dyn_obstacles_[di];
        if (dob.obs_index >= obstacles_.size()) continue;
        const auto& obs = obstacles_[dob.obs_index];
        dob.collision = false;
        // Check vs robots
        for (const auto& r : robots_) {
            if (check_robot_obstacle_collision(
                    r.world_vertices.data(), (int)r.world_vertices.size(), obs))
            {
                dob.collision = true;
                break;
            }
        }
        if (dob.collision) continue;
        // Check vs other dynamic obstacles (all shape pairs)
        for (size_t oj = 0; oj < dyn_obstacles_.size(); oj++) {
            if (di == oj) continue;
            auto& dob_oj = dyn_obstacles_[oj];
            if (dob_oj.obs_index >= obstacles_.size()) continue;
            if (check_obstacle_obstacle_collision(
                    obstacles_[dob.obs_index], obstacles_[dob_oj.obs_index]))
            {
                dob.collision = true;
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

#ifdef USE_AVX2
    lidar_raycast_avx2(origin, r.theta, angles, n_beams, range_max,
                       obstacles_.data(), (int)obstacles_.size(),
                       ranges_out,
                       lidar_grid_.empty() ? nullptr : &lidar_grid_);
#else
    lidar_raycast_scalar(origin, r.theta, angles, n_beams, range_max,
                         obstacles_.data(), (int)obstacles_.size(),
                         ranges_out);
#endif
}

void SimWorld::rebuild_lidar_grid() {
    lidar_grid_.build(obstacles_.data(), (int)obstacles_.size());
}

bool SimWorld::check_robot_collision(int robot_id) {
    if (robot_id < 0 || robot_id >= (int)robots_.size()) return false;
    return robots_[robot_id].collision;
}

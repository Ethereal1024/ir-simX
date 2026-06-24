#include "collision.h"
#include "geometry.h"
#include <algorithm>
#include <cmath>
#include <cfloat>

// ── Helper: project polygon onto axis, return [min, max] ──────
static void project_polygon(const Vec2* verts, int n, Vec2 axis, float& min, float& max) {
    min = max = verts[0].dot(axis);
    for (int i = 1; i < n; i++) {
        float proj = verts[i].dot(axis);
        min = std::min(min, proj);
        max = std::max(max, proj);
    }
}

// ── Helper: get edge normals for a polygon ────────────────────
// Returns the normal (perpendicular) for each edge.
static void get_edge_normals(const Vec2* verts, int n, Vec2* normals_out) {
    for (int i = 0; i < n; i++) {
        const Vec2& a = verts[i];
        const Vec2& b = verts[(i + 1) % n];
        Vec2 edge = b - a;
        // Outward normal: right-hand perpendicular
        normals_out[i] = {edge.y, -edge.x};
        float len = normals_out[i].len();
        if (len > 1e-8f) normals_out[i] = normals_out[i] / len;
    }
}

bool sat_intersect(const Vec2* verts_a, int n_a, const Vec2* verts_b, int n_b) {
    // Collect all potential separating axes (edge normals from both polygons)
    int max_axes = n_a + n_b;
    Vec2* axes = (Vec2*)alloca(max_axes * sizeof(Vec2));
    get_edge_normals(verts_a, n_a, axes);
    get_edge_normals(verts_b, n_b, axes + n_a);

    for (int i = 0; i < max_axes; i++) {
        Vec2 axis = axes[i];
        float min_a, max_a, min_b, max_b;
        project_polygon(verts_a, n_a, axis, min_a, max_a);
        project_polygon(verts_b, n_b, axis, min_b, max_b);
        if (max_a < min_b || max_b < min_a)
            return false;  // Separating axis found → no collision
    }
    return true;  // No separating axis → collision
}

bool point_in_rect(Vec2 p, Vec2 center, float half_w, float half_h) {
    return std::abs(p.x - center.x) <= half_w &&
           std::abs(p.y - center.y) <= half_h;
}

bool check_robot_obstacle_collision(
    const Vec2* robot_verts, int n_robot,
    const Obstacle& obs)
{
    switch (obs.type) {
    case ShapeType::CIRCLE: {
        // Find closest point on robot polygon to circle center
        float min_d2 = FLT_MAX;
        for (int i = 0; i < n_robot; i++) {
            const Vec2& a = robot_verts[i];
            const Vec2& b = robot_verts[(i + 1) % n_robot];
            Vec2 ab = b - a;
            float t = (obs.center - a).dot(ab) / ab.len2();
            t = std::max(0.0f, std::min(1.0f, t));
            Vec2 closest = a + ab * t;
            float d2 = (closest - obs.center).len2();
            if (d2 < min_d2) min_d2 = d2;
        }
        return min_d2 <= obs.radius * obs.radius;
    }
    case ShapeType::RECT: {
        // Use SAT for rectangle obstacles (convex, reliable).
        AABB rbox{obs.center - Vec2{obs.half_w, obs.half_h},
                   obs.center + Vec2{obs.half_w, obs.half_h}};
        // Quick check: any robot vertex inside rect?
        for (int i = 0; i < n_robot; i++) {
            if (robot_verts[i].x >= rbox.min.x && robot_verts[i].x <= rbox.max.x &&
                robot_verts[i].y >= rbox.min.y && robot_verts[i].y <= rbox.max.y)
                return true;
        }
        // SAT for edge-edge intersection
        Vec2 rverts[4] = {
            {rbox.min.x, rbox.min.y}, {rbox.max.x, rbox.min.y},
            {rbox.max.x, rbox.max.y}, {rbox.min.x, rbox.max.y}};
        return sat_intersect(robot_verts, n_robot, rverts, 4);
    }
    case ShapeType::POLYGON:
        if (obs.verts && obs.n_verts >= 3) {
            if (is_convex_polygon(obs.verts, obs.n_verts))
                return sat_intersect(robot_verts, n_robot, obs.verts, obs.n_verts);
            // Concave polygon: check robot-obstacle intersection per edge
            // using SAT on each edge's triangles
            for (int i = 0; i < obs.n_verts; i++) {
                const Vec2& a = obs.verts[i];
                const Vec2& b = obs.verts[(i + 1) % obs.n_verts];
                // Check if any robot vertex is inside this edge's inward half-plane
                // For a CCW polygon, the interior is to the LEFT of each edge
                Vec2 edge = b - a;
                float nx = -edge.y, ny = edge.x;  // left normal (interior side)
                bool all_inside = true;
                for (int j = 0; j < n_robot; j++) {
                    Vec2 rv = robot_verts[j] - a;
                    if (rv.dot({nx, ny}) < 0) { all_inside = false; break; }
                }
                if (all_inside) return true;  // robot fully inside this edge
            }
            // Fallback: check robot center-to-edge distance
            for (int i = 0; i < obs.n_verts; i++) {
                const Vec2& a = obs.verts[i];
                const Vec2& b = obs.verts[(i + 1) % obs.n_verts];
                Vec2 ab = b - a;
                // Project robot center onto edge
                float cx = 0, cy = 0;
                for (int j = 0; j < n_robot; j++) { cx += robot_verts[j].x; cy += robot_verts[j].y; }
                cx /= n_robot; cy /= n_robot;
                Vec2 rc = {cx - a.x, cy - a.y};
                float t = rc.dot(ab) / ab.len2();
                if (t >= 0 && t <= 1.0f) {
                    Vec2 closest = a + ab * t;
                    float d2 = (cx - closest.x) * (cx - closest.x) + (cy - closest.y) * (cy - closest.y);
                    // Approximate robot as circle with radius = half of max extent
                    float r = 0;
                    for (int j = 0; j < n_robot; j++) {
                        float dx = robot_verts[j].x - cx, dy = robot_verts[j].y - cy;
                        r = std::max(r, dx * dx + dy * dy);
                    }
                    if (d2 <= r * 1.5f) return true;
                }
            }
            return false;
        }
        return false;
    default:
        return false;
    }
}

bool batch_collision_check(
    const Vec2* robot_verts, const int* robot_nverts, int n_robots,
    const Obstacle* obstacles, int n_obs)
{
    // For each obstacle, check each robot
    int vert_offset = 0;
    for (int ri = 0; ri < n_robots; ri++) {
        int nv = robot_nverts[ri];
        for (int oj = 0; oj < n_obs; oj++) {
            if (check_robot_obstacle_collision(robot_verts + vert_offset, nv, obstacles[oj]))
                return true;
        }
        vert_offset += nv;
    }
    return false;
}

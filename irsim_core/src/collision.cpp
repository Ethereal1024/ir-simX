#include "collision.h"
#include "geometry.h"
#include <algorithm>
#include <cmath>
#include <cfloat>
#include <alloca.h>

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
    int n_axes = 0;
    auto add_normals = [&](const Vec2* verts, int n) {
        for (int i = 0; i < n; i++) {
            const Vec2& a = verts[i];
            const Vec2& b = verts[(i + 1) % n];
            Vec2 edge = b - a;
            if (edge.len2() < 1e-12f) continue;  // skip degenerate edges
            axes[n_axes] = {edge.y, -edge.x};
            float len = axes[n_axes].len();
            if (len > 1e-8f) axes[n_axes] = axes[n_axes] / len;
            n_axes++;
        }
    };
    add_normals(verts_a, n_a);
    add_normals(verts_b, n_b);

    for (int i = 0; i < n_axes; i++) {
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
        // If the rect is rotated (theta != 0), rotate robot vertices into rect's
        // local frame so the axis-aligned check is correct.
        float c = 1.0f, s = 0.0f;
        bool rotated = std::abs(obs.theta) > 1e-6f;
        if (rotated) {
            c = std::cos(-obs.theta);
            s = std::sin(-obs.theta);
        }

        Vec2* check_verts = const_cast<Vec2*>(robot_verts);
        Vec2* tmp = nullptr;
        if (rotated) {
            tmp = (Vec2*)alloca(n_robot * sizeof(Vec2));
            float cx = obs.center.x, cy = obs.center.y;
            for (int i = 0; i < n_robot; i++) {
                float dx = robot_verts[i].x - cx;
                float dy = robot_verts[i].y - cy;
                tmp[i].x = cx + dx * c - dy * s;
                tmp[i].y = cy + dx * s + dy * c;
            }
            check_verts = tmp;
        }

        AABB rbox{obs.center - Vec2{obs.half_w, obs.half_h},
                   obs.center + Vec2{obs.half_w, obs.half_h}};
        // Quick check: any robot vertex inside rect?
        for (int i = 0; i < n_robot; i++) {
            if (check_verts[i].x >= rbox.min.x && check_verts[i].x <= rbox.max.x &&
                check_verts[i].y >= rbox.min.y && check_verts[i].y <= rbox.max.y)
                return true;
        }
        // SAT for edge-edge intersection
        Vec2 rverts[4] = {
            {rbox.min.x, rbox.min.y}, {rbox.max.x, rbox.min.y},
            {rbox.max.x, rbox.max.y}, {rbox.min.x, rbox.max.y}};
        return sat_intersect(check_verts, n_robot, rverts, 4);
    }
    case ShapeType::POLYGON:
        if (obs.verts && obs.n_verts >= 3) {
            if (is_convex_polygon(obs.verts, obs.n_verts))
                return sat_intersect(robot_verts, n_robot, obs.verts, obs.n_verts);
            // Concave polygon: check robot center-to-edge distance
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
                    if (d2 <= r * 1.01f) return true;
                }
            }
            return false;
        }
        return false;
    default:
        return false;
    }
}

bool check_obstacle_obstacle_collision(const Obstacle& a, const Obstacle& b) {
    // Quick AABB reject
    if (!a.aabb.overlaps(b.aabb)) return false;

    // Dispatch by shape pair type
    auto check_circle_circle = [](const Obstacle& ca, const Obstacle& cb) -> bool {
        float dx = ca.center.x - cb.center.x;
        float dy = ca.center.y - cb.center.y;
        float r_sum = ca.radius + cb.radius;
        return dx * dx + dy * dy <= r_sum * r_sum;
    };

    auto check_circle_rect = [](const Obstacle& circle, const Obstacle& rect) -> bool {
        float cx = circle.center.x, cy = circle.center.y;
        float rx = rect.center.x, ry = rect.center.y;
        float hw = rect.half_w, hh = rect.half_h;
        float closest_x = std::max(rx - hw, std::min(cx, rx + hw));
        float closest_y = std::max(ry - hh, std::min(cy, ry + hh));
        float dx = cx - closest_x, dy = cy - closest_y;
        return dx * dx + dy * dy <= circle.radius * circle.radius;
    };

    auto check_rect_rect = [](const Obstacle& ra, const Obstacle& rb) -> bool {
        return std::abs(ra.center.x - rb.center.x) <= ra.half_w + rb.half_w &&
               std::abs(ra.center.y - rb.center.y) <= ra.half_h + rb.half_h;
    };

    auto obs_to_quad = [](const Obstacle& obs, Vec2* quad) {
        if (obs.type == ShapeType::CIRCLE) {
            float r = obs.radius;
            quad[0] = {obs.center.x - r, obs.center.y - r};
            quad[1] = {obs.center.x + r, obs.center.y - r};
            quad[2] = {obs.center.x + r, obs.center.y + r};
            quad[3] = {obs.center.x - r, obs.center.y + r};
        } else if (obs.type == ShapeType::RECT) {
            quad[0] = {obs.center.x - obs.half_w, obs.center.y - obs.half_h};
            quad[1] = {obs.center.x + obs.half_w, obs.center.y - obs.half_h};
            quad[2] = {obs.center.x + obs.half_w, obs.center.y + obs.half_h};
            quad[3] = {obs.center.x - obs.half_w, obs.center.y + obs.half_h};
        } else if (obs.type == ShapeType::POLYGON && obs.verts) {
            // Use actual polygon vertices
            for (int i = 0; i < std::min(obs.n_verts, 64); i++)
                quad[i] = obs.verts[i];
        }
    };

    // Handle pairs that need SAT (polygon involved)
    auto needs_sat = [](ShapeType t) { return t == ShapeType::POLYGON; };

    if (needs_sat(a.type) || needs_sat(b.type)) {
        Vec2 va[64], vb[64];
        int na = std::min(a.n_verts, 64);
        int nb = std::min(b.n_verts, 64);
        if (a.type == ShapeType::POLYGON && a.verts) {
            na = std::min(a.n_verts, 64);
            for (int i = 0; i < na; i++) va[i] = a.verts[i];
        } else {
            na = 4;
            obs_to_quad(a, va);
        }
        if (b.type == ShapeType::POLYGON && b.verts) {
            nb = std::min(b.n_verts, 64);
            for (int i = 0; i < nb; i++) vb[i] = b.verts[i];
        } else {
            nb = 4;
            obs_to_quad(b, vb);
        }
        return sat_intersect(va, na, vb, nb);
    }

    // Exact checks for circle/rect pairs
    if (a.type == ShapeType::CIRCLE && b.type == ShapeType::CIRCLE)
        return check_circle_circle(a, b);
    if (a.type == ShapeType::CIRCLE && b.type == ShapeType::RECT)
        return check_circle_rect(a, b);
    if (a.type == ShapeType::RECT && b.type == ShapeType::CIRCLE)
        return check_circle_rect(b, a);
    if (a.type == ShapeType::RECT && b.type == ShapeType::RECT)
        return check_rect_rect(a, b);

    return false;
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

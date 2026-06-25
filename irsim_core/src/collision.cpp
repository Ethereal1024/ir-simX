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

bool sat_intersect(const Vec2* verts_a, int n_a, const Vec2* verts_b, int n_b) {
    if (n_a > 256 || n_b > 256) return false;

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
            // Concave polygon: ear-clip triangulation + SAT per triangle
            auto triangles = ear_clip_triangulate(obs.verts, obs.n_verts);
            for (const auto& tri : triangles) {
                Vec2 tri_verts[3] = {obs.verts[tri[0]], obs.verts[tri[1]], obs.verts[tri[2]]};
                if (sat_intersect(robot_verts, n_robot, tri_verts, 3))
                    return true;
            }
            return false;
        }
        return false;
    case ShapeType::LINESTRING:
        if (obs.verts && obs.n_verts >= 2) {
            // Treat each segment as a thin oriented rectangle (OBB) and SAT.
            for (int i = 0; i < obs.n_verts - 1; i++) {
                Vec2 a = obs.verts[i], b = obs.verts[i + 1];
                Vec2 dir = (b - a).normalized();
                float half_thick = obs.linestring_half_thickness;
                Vec2 perp{ -dir.y * half_thick, dir.x * half_thick};
                Vec2 seg_quad[4] = {
                    {a.x - perp.x, a.y - perp.y},
                    {b.x - perp.x, b.y - perp.y},
                    {b.x + perp.x, b.y + perp.y},
                    {a.x + perp.x, a.y + perp.y}
                };
                if (sat_intersect(robot_verts, n_robot, seg_quad, 4))
                    return true;
            }
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
        Vec2 q[4];
        float c = std::cos(rect.theta), s = std::sin(rect.theta);
        float hw = rect.half_w, hh = rect.half_h;
        q[0] = rect.center + Vec2{ hw * c - hh * s,  hw * s + hh * c};
        q[1] = rect.center + Vec2{-hw * c - hh * s, -hw * s + hh * c};
        q[2] = rect.center + Vec2{-hw * c + hh * s, -hw * s - hh * c};
        q[3] = rect.center + Vec2{ hw * c + hh * s,  hw * s - hh * c};
        float min_d2 = FLT_MAX;
        for (int i = 0; i < 4; i++) {
            Vec2 a = q[i], b = q[(i + 1) % 4];
            Vec2 ab = b - a;
            float t = (circle.center - a).dot(ab) / ab.len2();
            t = std::max(0.0f, std::min(1.0f, t));
            Vec2 closest = a + ab * t;
            float d2 = (closest - circle.center).len2();
            if (d2 < min_d2) min_d2 = d2;
        }
        return min_d2 <= circle.radius * circle.radius;
    };

    auto check_rect_rect = [](const Obstacle& ra, const Obstacle& rb) -> bool {
        Vec2 qa[4], qb[4];
        {
            float c = std::cos(ra.theta), s = std::sin(ra.theta);
            float hw = ra.half_w, hh = ra.half_h;
            qa[0] = ra.center + Vec2{ hw * c - hh * s,  hw * s + hh * c};
            qa[1] = ra.center + Vec2{-hw * c - hh * s, -hw * s + hh * c};
            qa[2] = ra.center + Vec2{-hw * c + hh * s, -hw * s - hh * c};
            qa[3] = ra.center + Vec2{ hw * c + hh * s,  hw * s - hh * c};
        }
        {
            float c = std::cos(rb.theta), s = std::sin(rb.theta);
            float hw = rb.half_w, hh = rb.half_h;
            qb[0] = rb.center + Vec2{ hw * c - hh * s,  hw * s + hh * c};
            qb[1] = rb.center + Vec2{-hw * c - hh * s, -hw * s + hh * c};
            qb[2] = rb.center + Vec2{-hw * c + hh * s, -hw * s - hh * c};
            qb[3] = rb.center + Vec2{ hw * c + hh * s,  hw * s - hh * c};
        }
        return sat_intersect(qa, 4, qb, 4);
    };

    auto obs_to_quad = [](const Obstacle& obs, Vec2* quad) {
        if (obs.type == ShapeType::CIRCLE) {
            float r = obs.radius;
            quad[0] = {obs.center.x - r, obs.center.y - r};
            quad[1] = {obs.center.x + r, obs.center.y - r};
            quad[2] = {obs.center.x + r, obs.center.y + r};
            quad[3] = {obs.center.x - r, obs.center.y + r};
        } else if (obs.type == ShapeType::RECT) {
            float c = std::cos(obs.theta), s = std::sin(obs.theta);
            float hw = obs.half_w, hh = obs.half_h;
            quad[0] = obs.center + Vec2{ hw * c - hh * s,  hw * s + hh * c};
            quad[1] = obs.center + Vec2{-hw * c - hh * s, -hw * s + hh * c};
            quad[2] = obs.center + Vec2{-hw * c + hh * s, -hw * s - hh * c};
            quad[3] = obs.center + Vec2{ hw * c + hh * s,  hw * s - hh * c};
        } else if (obs.type == ShapeType::POLYGON && obs.verts) {
            // Use actual polygon vertices
            for (int i = 0; i < std::min(obs.n_verts, 64); i++)
                quad[i] = obs.verts[i];
        }
    };

    // Handle pairs that need SAT (polygon, linestring, or rotated rect involved)
    auto needs_sat = [](const Obstacle& a, const Obstacle& b) -> bool {
        auto check_one = [](ShapeType t, float theta) {
            return t == ShapeType::POLYGON || t == ShapeType::LINESTRING ||
                   (t == ShapeType::RECT && std::abs(theta) > 1e-6f);
        };
        return check_one(a.type, a.theta) || check_one(b.type, b.theta);
    };

    if (needs_sat(a, b)) {
        // Build flat convex-shape lists from each obstacle.
        // Convex polygons → single entry; concave → triangulated.
        // Circle/rect → 4-point quad.
        struct ConvexShape { Vec2 verts[64]; int n; };
        auto to_convex_shapes = [&obs_to_quad](const Obstacle& o, ConvexShape shapes[64]) -> int {
            if (o.type == ShapeType::POLYGON && o.verts) {
                if (is_convex_polygon(o.verts, o.n_verts)) {
                    int n = std::min(o.n_verts, 64);
                    for (int i = 0; i < n; i++) shapes[0].verts[i] = o.verts[i];
                    shapes[0].n = n;
                    return 1;
                }
                auto tris = ear_clip_triangulate(o.verts, o.n_verts);
                int cnt = 0;
                for (const auto& t : tris) {
                    if (cnt >= 64) break;
                    shapes[cnt].verts[0] = o.verts[t[0]];
                    shapes[cnt].verts[1] = o.verts[t[1]];
                    shapes[cnt].verts[2] = o.verts[t[2]];
                    shapes[cnt].n = 3;
                    cnt++;
                }
                return cnt;
            }
            if (o.type == ShapeType::LINESTRING && o.verts && o.n_verts >= 2) {
                int cnt = 0;
                float half_thick = o.linestring_half_thickness;
                for (int i = 0; i < o.n_verts - 1 && cnt < 64; i++) {
                    Vec2 a = o.verts[i], b = o.verts[i + 1];
                    Vec2 dir = (b - a).normalized();
                    Vec2 perp{ -dir.y * half_thick, dir.x * half_thick};
                    shapes[cnt].verts[0] = {a.x - perp.x, a.y - perp.y};
                    shapes[cnt].verts[1] = {b.x - perp.x, b.y - perp.y};
                    shapes[cnt].verts[2] = {b.x + perp.x, b.y + perp.y};
                    shapes[cnt].verts[3] = {a.x + perp.x, a.y + perp.y};
                    shapes[cnt].n = 4;
                    cnt++;
                }
                return cnt;
            }
            Vec2 q[4];
            obs_to_quad(o, q);
            for (int i = 0; i < 4; i++) shapes[0].verts[i] = q[i];
            shapes[0].n = 4;
            return 1;
        };
        ConvexShape shapes_a[64], shapes_b[64];
        int na = to_convex_shapes(a, shapes_a);
        int nb = to_convex_shapes(b, shapes_b);
        for (int i = 0; i < na; i++)
            for (int j = 0; j < nb; j++)
                if (sat_intersect(shapes_a[i].verts, shapes_a[i].n,
                                  shapes_b[j].verts, shapes_b[j].n))
                    return true;
        return false;
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

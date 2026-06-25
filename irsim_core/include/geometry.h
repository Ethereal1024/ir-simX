#pragma once
#include <cmath>
#include <cstdint>
#include <vector>
#include <array>
#include <list>
#include <algorithm>
#include <cfloat>

// ═══════════════════════════════════════════════════════════════
//  Geometric primitives and intersection functions
// ═══════════════════════════════════════════════════════════════

struct Vec2 {
    float x, y;

    Vec2() : x(0), y(0) {}
    Vec2(float x_, float y_) : x(x_), y(y_) {}

    Vec2 operator+(Vec2 rhs) const { return {x + rhs.x, y + rhs.y}; }
    Vec2 operator-(Vec2 rhs) const { return {x - rhs.x, y - rhs.y}; }
    Vec2 operator*(float s) const { return {x * s, y * s}; }
    Vec2 operator/(float s) const { return {x / s, y / s}; }
    Vec2& operator+=(Vec2 rhs) { x += rhs.x; y += rhs.y; return *this; }

    float dot(Vec2 v) const { return x * v.x + y * v.y; }
    float cross(Vec2 v) const { return x * v.y - y * v.x; }
    float len2() const { return x * x + y * y; }
    float len() const { return std::sqrt(len2()); }
    Vec2 normalized() const { float l = len(); return l > 1e-8f ? *this / l : Vec2{0,0}; }

    // Rotate by angle
    Vec2 rotated(float angle) const {
        float c = std::cos(angle), s = std::sin(angle);
        return {x * c - y * s, x * s + y * c};
    }
};

struct AABB {
    Vec2 min, max;
    AABB() : min{FLT_MAX, FLT_MAX}, max{-FLT_MAX, -FLT_MAX} {}
    AABB(Vec2 min_, Vec2 max_) : min(min_), max(max_) {}

    void expand(Vec2 p) {
        min.x = std::min(min.x, p.x); min.y = std::min(min.y, p.y);
        max.x = std::max(max.x, p.x); max.y = std::max(max.y, p.y);
    }

    bool overlaps(const AABB& other) const {
        return min.x <= other.max.x && max.x >= other.min.x &&
               min.y <= other.max.y && max.y >= other.min.y;
    }
};

// ── Obstacle types ────────────────────────────────────────────
enum class ShapeType : uint8_t {
    CIRCLE = 0,
    RECT   = 1,
    POLYGON = 2,
    LINESTRING = 3,  // for map grid edges
};

struct Obstacle {
    ShapeType type;
    Vec2 center;            // for CIRCLE
    float radius;           // for CIRCLE
    float half_w, half_h;   // for RECT (axis-aligned)
    float theta = 0;        // rotation for RECT (dynamic obstacles)
    // For POLYGON: vertices are stored externally, use n_verts + get_vert()
    const Vec2* verts = nullptr;
    int n_verts = 0;
    AABB aabb;

    void compute_aabb() {
        aabb = AABB();
        if (type == ShapeType::CIRCLE) {
            aabb = AABB{center - Vec2{radius, radius}, center + Vec2{radius, radius}};
        } else if (type == ShapeType::RECT) {
            aabb = AABB{center - Vec2{half_w, half_h}, center + Vec2{half_w, half_h}};
        } else if (type == ShapeType::POLYGON && verts) {
            for (int i = 0; i < n_verts; i++) aabb.expand(verts[i]);
        }
    }
};

// ═══════════════════════════════════════════════════════════════
//  Intersection functions
// ═══════════════════════════════════════════════════════════════

// Ray-circle intersection.
// Returns true if hit, t = distance along ray (t >= 0).
// ray: origin o, unit direction d.
inline bool intersect_ray_circle(
    Vec2 o, Vec2 d, Vec2 c, float r, float& t_out)
{
    Vec2 oc = o - c;
    float b = 2.0f * d.dot(oc);
    float c_ = oc.len2() - r * r;
    float disc = b * b - 4.0f * c_;
    if (disc < 0) return false;
    float sqrt_disc = std::sqrt(disc);
    float t1 = (-b - sqrt_disc) * 0.5f;
    float t2 = (-b + sqrt_disc) * 0.5f;
    // Find nearest positive t
    float t = (t1 >= 0) ? t1 : t2;
    if (t < 0) return false;
    t_out = t;
    return true;
}

// Ray-AABB intersection (slab method).
inline bool intersect_ray_aabb(Vec2 o, Vec2 d, const AABB& box, float& t_out) {
    float tmin = -FLT_MAX, tmax = FLT_MAX;
    if (std::abs(d.x) > 1e-12f) {
        float t1 = (box.min.x - o.x) / d.x;
        float t2 = (box.max.x - o.x) / d.x;
        tmin = std::max(tmin, std::min(t1, t2));
        tmax = std::min(tmax, std::max(t1, t2));
    } else if (o.x < box.min.x || o.x > box.max.x) return false;

    if (std::abs(d.y) > 1e-12f) {
        float t1 = (box.min.y - o.y) / d.y;
        float t2 = (box.max.y - o.y) / d.y;
        tmin = std::max(tmin, std::min(t1, t2));
        tmax = std::min(tmax, std::max(t1, t2));
    } else if (o.y < box.min.y || o.y > box.max.y) return false;

    if (tmax < 0) return false;
    t_out = std::max(0.0f, tmin);
    return tmin <= tmax;
}

// ── Convexity check ──────────────────────────────────────────
inline bool is_convex_polygon(const Vec2* verts, int n) {
    if (n < 3) return false;
    int sign = 0;
    for (int i = 0; i < n; i++) {
        const Vec2& a = verts[i];
        const Vec2& b = verts[(i + 1) % n];
        const Vec2& c = verts[(i + 2) % n];
        float cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
        if (std::abs(cross) < 1e-10f) continue;
        if (sign == 0) sign = (cross > 0) ? 1 : -1;
        else if ((cross > 0 && sign < 0) || (cross < 0 && sign > 0))
            return false;
    }
    return true;
}

// ── Point-in-triangle test (barycentric) ─────────────────────
inline bool point_in_triangle(Vec2 p, Vec2 a, Vec2 b, Vec2 c) {
    float d1 = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
    float d2 = (c.x - b.x) * (p.y - b.y) - (c.y - b.y) * (p.x - b.x);
    float d3 = (a.x - c.x) * (p.y - c.y) - (a.y - c.y) * (p.x - c.x);
    return (d1 >= 0 && d2 >= 0 && d3 >= 0) || (d1 <= 0 && d2 <= 0 && d3 <= 0);
}

// ── Ear clipping triangulation for concave/self-intersecting polygons ──
// Input: vertices (CCW or CW), n ≥ 3.
// Returns: list of (i1,i2,i3) triangle vertex indices.
inline std::vector<std::array<int,3>> ear_clip_triangulate(
    const Vec2* verts, int n)
{
    std::vector<std::array<int,3>> triangles;
    if (n < 3) return triangles;
    if (n == 3) { triangles.push_back({0, 1, 2}); return triangles; }

    // Build doubly-linked list of remaining vertex indices
    struct Node { int idx; Node* prev; Node* next; };
    std::vector<Node> nodes(n);
    for (int i = 0; i < n; i++) {
        nodes[i].idx = i;
        nodes[i].prev = &nodes[(i + n - 1) % n];
        nodes[i].next = &nodes[(i + 1) % n];
    }

    int remaining = n;
    Node* cur = &nodes[0];
    int max_iter = n * n;  // safety limit
    int convex_sign = 0;   // winding: +1 CCW, -1 CW

    while (remaining > 3 && --max_iter > 0) {
        Node* a = cur;
        Node* b = a->next;
        Node* c = b->next;
        const Vec2& va = verts[a->idx];
        const Vec2& vb = verts[b->idx];
        const Vec2& vc = verts[c->idx];

        // Check if (a,b,c) is convex (same winding as polygon)
        float cross = (vb.x - va.x) * (vc.y - vb.y) - (vb.y - va.y) * (vc.x - vb.x);
        bool is_ear = false;
        // Ear vertex must have the same winding direction as the polygon.
        // We determine winding from the initial cross product check; once
        // winding is known, only corners matching that sign are convex ears.
        if (cross != 0) {
            if (convex_sign == 0)
                convex_sign = (cross > 0) ? 1 : -1;
            is_ear = (cross > 0) == (convex_sign > 0);
        }
        if (is_ear) {
            // Ear = no other vertex inside triangle (a,b,c)
            Node* test = c->next;
            while (test != a) {
                if (point_in_triangle(verts[test->idx], va, vb, vc)) {
                    is_ear = false;
                    break;
                }
                test = test->next;
            }
        }

        if (is_ear) {
            triangles.push_back({a->idx, b->idx, c->idx});
            // Clip ear: remove b from linked list
            a->next = c;
            c->prev = a;
            remaining--;
            cur = c;  // continue from c
        } else {
            cur = b;  // try next vertex
        }
    }

    // Last 3 vertices form final triangle
    Node* a = cur;
    Node* b = a->next;
    Node* c = b->next;
    triangles.push_back({a->idx, b->idx, c->idx});

    return triangles;
}

// ── Ray-convex-polygon intersection (Cyrus-Beck) ──────────────
// Works with both CW and CCW vertex winding.
inline bool intersect_ray_convex_polygon(
    Vec2 o, Vec2 d, const Vec2* verts, int n, float& t_out)
{
    // Determine winding via signed area
    float area = 0;
    for (int i = 0; i < n; i++)
        area += verts[i].cross(verts[(i + 1) % n]);
    bool ccw = area > 0;

    float t_enter = 0, t_exit = FLT_MAX;
    for (int i = 0; i < n; i++) {
        const Vec2& a = verts[i];
        const Vec2& b = verts[(i + 1) % n];
        Vec2 edge = b - a;
        float nx = edge.y, ny = -edge.x;
        if (!ccw) { nx = -nx; ny = -ny; }
        float denom = d.dot({nx, ny});
        if (std::abs(denom) < 1e-12f) continue;
        Vec2 ao = a - o;
        float t = ao.dot({nx, ny}) / denom;
        if (std::abs(t) < 1e-10f) continue;  // ignore edge-on intersections at origin
        if (denom > 0) {
            if (t < t_exit) t_exit = t;
        } else {
            if (t > t_enter) t_enter = t;
        }
    }
    if (t_enter > t_exit) return false;
    if (t_exit < 0) return false;
    t_out = (t_enter > 0) ? t_enter : 0;
    return true;
}

// ── Ray-polygon intersection (convex OR concave) ──────────────
// For convex polygons: Cyrus-Beck.
// Concave polygon: edge-by-edge, return first intersection (entry point).
inline bool intersect_ray_polygon(
    Vec2 o, Vec2 d, const Vec2* verts, int n, float& t_out)
{
    if (n < 3) return false;
    if (is_convex_polygon(verts, n))
        return intersect_ray_convex_polygon(o, d, verts, n, t_out);

    // Find the nearest ray-segment intersection.
    // For a ray starting outside a simple polygon, the first intersection
    // is always the entry point.
    float min_t = FLT_MAX;
    for (int i = 0; i < n; i++) {
        const Vec2& a = verts[i];
        const Vec2& b = verts[(i + 1) % n];
        Vec2 ab = b - a;
        float denom = d.cross(ab);
        if (std::abs(denom) < 1e-12f) continue;

        Vec2 ao = a - o;
        float t = ao.cross(ab) / denom;
        float u = ao.cross(d) / denom;

        // origin exactly on edge → distance 0
        if (std::abs(t) <= 1e-6f && u >= -1e-6f && u <= 1.0f + 1e-6f) {
            t_out = 0.0f; return true;
        }
        if (t > 1e-6f && u > 1e-6f && u < 1.0f - 1e-6f) {
            if (t < min_t) { min_t = t; }
        }
    }
    if (min_t < FLT_MAX) { t_out = min_t; return true; }
    return false;
}

// Ray-AABB intersection for axis-aligned rectangle.
inline bool intersect_ray_rect(Vec2 o, Vec2 d, Vec2 c, float hw, float hh, float& t_out) {
    AABB box{c - Vec2{hw, hh}, c + Vec2{hw, hh}};
    return intersect_ray_aabb(o, d, box, t_out);
}

// Point-in-convex-polygon test.
inline bool point_in_polygon(Vec2 p, const Vec2* verts, int n) {
    for (int i = 0; i < n; i++) {
        const Vec2& a = verts[i];
        const Vec2& b = verts[(i + 1) % n];
        if ((b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x) < 0)
            return false;
    }
    return true;
}

// Generic ray-obstacle intersection dispatcher.
inline bool intersect_ray_obstacle(
    Vec2 o, Vec2 d, const Obstacle& obs, float& t_out)
{
    // First quick AABB reject
    if (!intersect_ray_aabb(o, d, obs.aabb, t_out)) return false;

    switch (obs.type) {
    case ShapeType::CIRCLE:
        return intersect_ray_circle(o, d, obs.center, obs.radius, t_out);
    case ShapeType::RECT:
        return intersect_ray_rect(o, d, obs.center, obs.half_w, obs.half_h, t_out);
    case ShapeType::POLYGON:
        if (obs.verts)
            return intersect_ray_polygon(o, d, obs.verts, obs.n_verts, t_out);
        return false;
    default:
        return false;
    }
}

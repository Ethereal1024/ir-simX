#pragma once
#include "geometry.h"

// ═══════════════════════════════════════════════════════════════
//  SAT-based collision detection
// ═══════════════════════════════════════════════════════════════

// Check if two convex polygons intersect.
// verts_a/b: vertices in world space, n_a/b: vertex counts.
// Uses Separating Axis Theorem; works for any convex polygons.
bool sat_intersect(const Vec2* verts_a, int n_a, const Vec2* verts_b, int n_b);

// Check if a point is inside a rectangle defined by center + half extents.
bool point_in_rect(Vec2 p, Vec2 center, float half_w, float half_h);

// Check collision between robot polygon and obstacle.
// Robot assumed convex polygon.
bool check_robot_obstacle_collision(
    const Vec2* robot_verts, int n_robot,
    const Obstacle& obs);

// Batch collision check: returns true if any robot-obstacle pair collides.
// robot_verts: per-robot vertex arrays (packed flat).
// robot_nverts: number of vertices per robot.
// n_robots: number of robots.
bool batch_collision_check(
    const Vec2* robot_verts, const int* robot_nverts, int n_robots,
    const Obstacle* obstacles, int n_obs);

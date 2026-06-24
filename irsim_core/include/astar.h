#pragma once
#include <cstdint>
#include <vector>

// ═══════════════════════════════════════════════════════════════
//  A* grid path planner
// ═══════════════════════════════════════════════════════════════

struct AStarPlanner {
    // grid: occupancy grid (0 = free, >50 = obstacle), row-major
    // width, height: grid dimensions in cells
    // resolution: meters per cell
    int width, height;
    float resolution;
    std::vector<uint8_t> grid;  // 0=free, 1=occupied

    AStarPlanner() : width(0), height(0), resolution(0.1f) {}

    // Load grid from raw data
    void set_grid(const uint8_t* data, int w, int h, float res);

    // Plan A* path from start to goal (world coordinates).
    // Returns path as interleaved (x,y) world coordinates.
    // Returns empty vector if no path exists.
    std::vector<float> plan(float sx, float sy, float gx, float gy);

private:
    // Heuristic: Euclidean distance
    float heuristic(int ax, int ay, int bx, int by) const;
};

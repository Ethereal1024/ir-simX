#include "astar.h"
#include "geometry.h"
#include <cmath>
#include <queue>
#include <algorithm>
#include <cstring>
#include <cfloat>
#include <vector>

void AStarPlanner::set_grid(const uint8_t* data, int w, int h, float res) {
    width = w; height = h; resolution = res;
    grid.assign(data, data + w * h);
}

float AStarPlanner::heuristic(int ax, int ay, int bx, int by) const {
    float dx = float(ax - bx);
    float dy = float(ay - by);
    return std::sqrt(dx * dx + dy * dy) * resolution;
}

std::vector<float> AStarPlanner::plan(float sx, float sy, float gx, float gy) {
    std::vector<float> result;

    if (width <= 0 || height <= 0) return result;

    // Convert world to grid coordinates
    int start_x = std::max(0, std::min(width - 1, int(sx / resolution)));
    int start_y = std::max(0, std::min(height - 1, int(sy / resolution)));
    int goal_x  = std::max(0, std::min(width - 1, int(gx / resolution)));
    int goal_y  = std::max(0, std::min(height - 1, int(gy / resolution)));

    // If start or goal is blocked, search nearest free neighbour
    if (grid[start_y * width + start_x]) {
        int best_d2 = width * width + height * height;
        bool found_start = false;
        for (int dy = -3; dy <= 3 && !found_start; dy++) {
            for (int dx = -3; dx <= 3 && !found_start; dx++) {
                int ny = start_y + dy, nx = start_x + dx;
                if (ny < 0 || ny >= height || nx < 0 || nx >= width) continue;
                if (grid[ny * width + nx]) continue;
                int d2 = dy * dy + dx * dx;
                if (d2 < best_d2) { best_d2 = d2; start_y = ny; start_x = nx; found_start = true; }
            }
        }
        if (!found_start) return result;
    }
    if (grid[goal_y * width + goal_x]) {
        int best_d2 = width * width + height * height;
        bool found_goal = false;
        for (int dy = -3; dy <= 3 && !found_goal; dy++) {
            for (int dx = -3; dx <= 3 && !found_goal; dx++) {
                int ny = goal_y + dy, nx = goal_x + dx;
                if (ny < 0 || ny >= height || nx < 0 || nx >= width) continue;
                if (grid[ny * width + nx]) continue;
                int d2 = dy * dy + dx * dx;
                if (d2 < best_d2) { best_d2 = d2; goal_y = ny; goal_x = nx; found_goal = true; }
            }
        }
        if (!found_goal) return result;
    }

    // A* state
    struct Node {
        int x, y;
        float g, f;
        bool operator>(const Node& o) const { return f > o.f; }
    };

    std::vector<float> g_score(width * height, FLT_MAX);
    std::vector<int> came_from(width * height, -1);

    // Priority queue
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;

    int start_idx = start_y * width + start_x;
    g_score[start_idx] = 0;
    pq.push({start_x, start_y, 0.0f, heuristic(start_x, start_y, goal_x, goal_y)});

    // 8-direction neighbours
    const int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    const int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    const float move_cost[8] = {1.0f, 1.414f, 1.0f, 1.414f, 1.0f, 1.414f, 1.0f, 1.414f};

    bool found = false;
    int goal_idx = goal_y * width + goal_x;

    while (!pq.empty()) {
        Node cur = pq.top(); pq.pop();
        int cur_idx = cur.y * width + cur.x;

        if (cur.g > g_score[cur_idx]) continue;  // stale entry

        if (cur.x == goal_x && cur.y == goal_y) {
            found = true;
            break;
        }

        for (int i = 0; i < 8; i++) {
            int nx = cur.x + dx[i];
            int ny = cur.y + dy[i];
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
            if (grid[ny * width + nx]) continue;  // obstacle

            float ng = cur.g + move_cost[i] * resolution;
            int nidx = ny * width + nx;
            if (ng < g_score[nidx]) {
                g_score[nidx] = ng;
                came_from[nidx] = cur_idx;
                pq.push({nx, ny, ng, ng + heuristic(nx, ny, goal_x, goal_y)});
            }
        }
    }

    if (!found) return result;

    // Reconstruct path
    std::vector<Vec2> path_world;
    int idx = goal_idx;
    while (idx >= 0) {
        int px = idx % width;
        int py = idx / width;
        path_world.push_back({(px + 0.5f) * resolution, (py + 0.5f) * resolution});
        idx = came_from[idx];
    }
    std::reverse(path_world.begin(), path_world.end());

    // Interleave to flat array
    result.reserve(path_world.size() * 2);
    for (const auto& p : path_world) {
        result.push_back(p.x);
        result.push_back(p.y);
    }
    return result;
}

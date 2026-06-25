"""Tests for the C++ accelerated core (irsim_core).

Requires the C++ extension to be built (``pip install -e .``).
Tests are skipped automatically if ``irsim_core`` is not importable.
"""

import math

import numpy as np
import pytest

_cc = pytest.importorskip("irsim_core", reason="C++ core not built; skipping")


# =========================================================================
# Utilities
# =========================================================================


def _dict_circle(x: float, y: float, radius: float) -> dict:
    return {"type": "circle", "x": x, "y": y, "radius": radius}


def _dict_rect(x: float, y: float, half_w: float, half_h: float) -> dict:
    return {"type": "rect", "x": x, "y": y, "half_w": half_w, "half_h": half_h}


def _dict_polygon(verts: list[list[float]]) -> dict:
    return {"type": "polygon", "x": 0, "y": 0, "vertices": verts}


# =========================================================================
# SimWorld — construction and basic properties
# =========================================================================


class TestSimWorldBasics:
    def test_create(self):
        w = _cc.SimWorld()
        assert w.num_robots() == 0
        assert w.num_obstacles() == 0

    def test_step_time(self):
        w = _cc.SimWorld()
        w.set_step_time(0.1)
        assert w.step_time() == pytest.approx(0.1)

    def test_add_robot(self):
        w = _cc.SimWorld()
        rid = w.add_robot(0, 1.0, 2.0, 0.5)
        assert rid == 0
        assert w.num_robots() == 1
        px, py, pt = w.get_robot_pose(rid)
        assert px == pytest.approx(1.0)
        assert py == pytest.approx(2.0)
        assert pt == pytest.approx(0.5)

    def test_add_robot_with_limits(self):
        w = _cc.SimWorld()
        vmin = np.array([-2.0, -1.0], dtype=np.float32)
        vmax = np.array([2.0, 1.0], dtype=np.float32)
        vacc = np.array([1.0, 0.5], dtype=np.float32)
        rid = w.add_robot(0, 0.0, 0.0, 0.0, vmin, vmax, vacc)
        assert rid == 0


# =========================================================================
# SimWorld — obstacle management
# =========================================================================


class TestSimWorldObstacles:
    def test_add_circle(self):
        w = _cc.SimWorld()
        oid = w.add_obstacle(_dict_circle(5.0, 5.0, 1.0))
        assert oid >= 0
        assert w.num_obstacles() == 1

    def test_add_rect(self):
        w = _cc.SimWorld()
        oid = w.add_obstacle(_dict_rect(3.0, 4.0, 2.0, 1.0))
        assert oid >= 0
        assert w.num_obstacles() == 1

    def test_add_polygon(self):
        w = _cc.SimWorld()
        verts = [[0.0, 0.0], [2.0, 0.0], [2.0, 1.0], [0.0, 1.0]]
        oid = w.add_obstacle(_dict_polygon(verts))
        assert oid >= 0
        assert w.num_obstacles() == 1

    def test_add_multiple_mixed(self):
        w = _cc.SimWorld()
        w.add_obstacle(_dict_circle(1.0, 1.0, 0.5))
        w.add_obstacle(_dict_rect(3.0, 3.0, 1.0, 1.0))
        w.add_obstacle(_dict_polygon([[5.0, 5.0], [7.0, 5.0], [6.0, 7.0]]))
        assert w.num_obstacles() == 3


# =========================================================================
# SimWorld — dynamic obstacles
# =========================================================================


class TestSimWorldDynamicObstacles:
    def test_add_dynamic_circle(self):
        w = _cc.SimWorld()
        did = w.add_dynamic_obstacle(0, 5.0, 5.0, 0.0, 0.5)
        assert did == 0
        assert w.num_dynamic_obstacles() == 1
        assert w.num_obstacles() == 1  # geometry also added

    def test_get_obstacle_pose(self):
        w = _cc.SimWorld()
        w.add_dynamic_obstacle(0, 3.0, 4.0, 1.0, 0.5)
        px, py, pt = w.get_obstacle_pose(0)
        assert px == pytest.approx(3.0)
        assert py == pytest.approx(4.0)
        assert pt == pytest.approx(1.0)

    def test_get_obstacle_velocity(self):
        w = _cc.SimWorld()
        w.add_dynamic_obstacle(0, 0.0, 0.0, 0.0, 0.5)
        vx, vy, omega = w.get_obstacle_velocity(0)
        assert vx == pytest.approx(0.0)
        assert vy == pytest.approx(0.0)
        assert omega == pytest.approx(0.0)

    def test_step_dynamic_diff_moves(self):
        w = _cc.SimWorld()
        w.set_step_time(0.1)
        w.add_dynamic_obstacle(0, 0.0, 0.0, 0.0, 0.5)
        # Add a robot to the world too (step requires at least one robot action)
        w.add_robot(0, 10.0, 10.0, 0.0)
        robot_actions = np.zeros(3, dtype=np.float32)
        w.step(robot_actions, 3)
        # Step the obstacle with v=1, omega=0
        obs_actions = np.array([1.0, 0.0, 0.0], dtype=np.float32)
        w.step_dynamic_obstacles(obs_actions, 3)
        px, _py, pt = w.get_obstacle_pose(0)
        # With accel lim=1, dt=0.1: clipped v = 0.1, dx = 0.1*0.1 = 0.01
        assert px == pytest.approx(0.01, abs=0.001)
        assert pt == pytest.approx(0.0, abs=0.001)

    def test_step_dynamic_omni_moves(self):
        w = _cc.SimWorld()
        w.set_step_time(0.1)
        w.add_dynamic_obstacle(1, 0.0, 0.0, 0.0, 0.5)  # kin=1 = OMNI
        w.add_robot(0, 10.0, 10.0, 0.0)
        robot_actions = np.zeros(3, dtype=np.float32)
        w.step(robot_actions, 3)
        obs_actions = np.array([0.0, 1.0, 0.0], dtype=np.float32)
        w.step_dynamic_obstacles(obs_actions, 3)
        _px, py, _pt = w.get_obstacle_pose(0)
        assert py == pytest.approx(0.01, abs=0.001)

    def test_dynamic_obstacle_collision_geometry_updates(self):
        w = _cc.SimWorld()
        w.set_step_time(0.1)
        w.add_dynamic_obstacle(0, 0.0, 0.0, 0.0, 0.5)
        w.add_robot(0, 5.0, 5.0, 0.0)
        robot_actions = np.zeros(3, dtype=np.float32)
        w.step(robot_actions, 3)
        obs_actions = np.array([10.0, 0.0, 0.0], dtype=np.float32)
        # After many steps the obstacle should have moved
        for _ in range(50):
            w.step(robot_actions, 3)
            w.step_dynamic_obstacles(obs_actions, 3)
        px, _, _ = w.get_obstacle_pose(0)
        # Should have moved forward (clipped by accel)
        assert px > 0.1

    def test_multiple_dynamic_obstacles(self):
        w = _cc.SimWorld()
        w.set_step_time(0.1)
        w.add_dynamic_obstacle(0, 0.0, 0.0, 0.0, 0.5)
        w.add_dynamic_obstacle(0, 5.0, 5.0, 0.0, 0.3)
        assert w.num_dynamic_obstacles() == 2
        assert w.num_obstacles() == 2
        px0, _, _ = w.get_obstacle_pose(0)
        px1, _, _ = w.get_obstacle_pose(1)
        assert px0 == pytest.approx(0.0)
        assert px1 == pytest.approx(5.0)

    def test_add_dynamic_polygon(self):
        w = _cc.SimWorld()
        verts = [[5.0, 5.0], [7.0, 5.0], [7.0, 6.0], [5.0, 6.0]]
        did = w.add_dynamic_polygon_obstacle(0, 6.0, 5.5, 0.0, verts)
        assert did == 0
        assert w.num_dynamic_obstacles() == 1
        assert w.num_obstacles() == 1

    def test_step_dynamic_polygon_moves(self):
        w = _cc.SimWorld()
        w.set_step_time(0.1)
        verts = [[5.0, 5.0], [7.0, 5.0], [7.0, 6.0], [5.0, 6.0]]
        w.add_dynamic_polygon_obstacle(0, 6.0, 5.5, 0.0, verts)
        w.add_robot(0, 10.0, 10.0, 0.0)
        robot_actions = np.zeros(3, dtype=np.float32)
        w.step(robot_actions, 3)
        obs_actions = np.array([1.0, 0.0, 0.0], dtype=np.float32)
        for _ in range(5):
            w.step(robot_actions, 3)
            w.step_dynamic_obstacles(obs_actions, 3)
        px, py, _pt = w.get_obstacle_pose(0)
        # Should have moved right (clipped via accel)
        assert px > 6.0
        assert py == pytest.approx(5.5, abs=0.01)

    def test_dynamic_polygon_collision_detected(self):
        w = _cc.SimWorld()
        w.set_step_time(0.1)
        verts = [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]]
        w.add_dynamic_polygon_obstacle(0, 0.0, 0.0, 0.0, verts)
        w.add_robot(0, 0.0, 0.0, 0.0)
        # Robot overlaps with polygon at origin -> collision
        robot_actions = np.zeros(3, dtype=np.float32)
        w.step(robot_actions, 3)
        assert w.check_robot_collision(0)


# =========================================================================
# SimWorld — kinematics stepping
# =========================================================================


class TestSimWorldStep:
    # Default limits: vel_acc=[1,1,1], vel_max=[1,1,1], vel_min=[-1,-1,-1]
    # With dt=0.1, accel clips action to ±0.1 per step

    def test_diff_moves_forward(self):
        w = _cc.SimWorld()
        w.set_step_time(0.1)
        w.add_robot(0, 0.0, 0.0, 0.0)
        actions = np.array([1.0, 0.0, 0.0], dtype=np.float32)
        w.step(actions, 3)
        px, py, pt = w.get_robot_pose(0)
        assert px == pytest.approx(0.01, abs=0.001)
        assert py == pytest.approx(0.0, abs=0.001)
        assert pt == pytest.approx(0.0, abs=0.001)

    def test_diff_turns(self):
        w = _cc.SimWorld()
        w.set_step_time(0.1)
        w.add_robot(0, 0.0, 0.0, 0.0)
        # diff: action[0]=v, action[1]=omega
        actions = np.array([0.0, 1.0, 0.0], dtype=np.float32)
        w.step(actions, 3)
        _, _, pt = w.get_robot_pose(0)
        assert pt == pytest.approx(0.01, abs=0.001)

    def test_diff_negative_velocity(self):
        w = _cc.SimWorld()
        w.set_step_time(0.1)
        w.add_robot(0, 5.0, 5.0, 0.0)
        actions = np.array([-1.0, 0.0, 0.0], dtype=np.float32)
        w.step(actions, 3)
        px, _, _ = w.get_robot_pose(0)
        assert px == pytest.approx(4.99, abs=0.001)

    def test_multiple_robots(self):
        w = _cc.SimWorld()
        w.set_step_time(0.1)
        w.add_robot(0, 0.0, 0.0, 0.0)
        w.add_robot(0, 1.0, 1.0, 0.0)
        actions = np.array([1.0, 0.0, 0.0, 1.0, 0.0, 0.0], dtype=np.float32)
        w.step(actions, 3)
        px0, _, _ = w.get_robot_pose(0)
        px1, _, _ = w.get_robot_pose(1)
        assert px0 == pytest.approx(0.01, abs=0.001)
        assert px1 == pytest.approx(1.01, abs=0.001)

    def test_acceleration_clipping(self):
        w = _cc.SimWorld()
        w.set_step_time(0.1)
        # Set high vel limits so accel is the bottleneck
        vmax = np.array([10.0, 10.0], dtype=np.float32)
        vmin = np.array([-10.0, -10.0], dtype=np.float32)
        vacc = np.array([0.5, 0.5], dtype=np.float32)
        w.add_robot(0, 0.0, 0.0, 0.0, vmin, vmax, vacc)
        actions = np.array([10.0, 0.0, 0.0], dtype=np.float32)
        w.step(actions, 3)
        px, _, _ = w.get_robot_pose(0)
        assert px == pytest.approx(0.005, abs=0.001)

    def test_velocity_limit_clipping(self):
        w = _cc.SimWorld()
        w.set_step_time(0.1)
        vmax = np.array([0.5, 0.5], dtype=np.float32)
        vmin = np.array([-0.5, -0.5], dtype=np.float32)
        vacc = np.array([100.0, 100.0], dtype=np.float32)  # accel won't limit
        w.add_robot(0, 0.0, 0.0, 0.0, vmin, vmax, vacc)
        actions = np.array([10.0, 0.0, 0.0], dtype=np.float32)
        w.step(actions, 3)
        px, _, _ = w.get_robot_pose(0)
        assert px == pytest.approx(0.05, abs=0.001)

    def test_zero_action_no_movement(self):
        w = _cc.SimWorld()
        w.set_step_time(0.1)
        w.add_robot(0, 3.0, 4.0, 1.0)
        actions = np.zeros(3, dtype=np.float32)
        w.step(actions, 3)
        px, py, pt = w.get_robot_pose(0)
        assert px == pytest.approx(3.0, abs=0.001)
        assert py == pytest.approx(4.0, abs=0.001)
        assert pt == pytest.approx(1.0, abs=0.001)


# =========================================================================
# SimWorld — collision detection
# =========================================================================


class TestSimWorldCollision:
    def test_no_collision(self):
        w = _cc.SimWorld()
        w.set_step_time(0.1)
        w.add_robot(0, 0.0, 0.0, 0.0)
        w.add_obstacle(_dict_circle(10.0, 10.0, 0.5))
        actions = np.zeros(3, dtype=np.float32)
        w.step(actions, 3)
        assert not w.check_robot_collision(0)

    def test_collision_circle(self):
        w = _cc.SimWorld()
        w.set_step_time(0.1)
        w.add_robot(0, 0.0, 0.0, 0.0)
        w.add_obstacle(_dict_circle(0.1, 0.0, 1.0))
        actions = np.zeros(3, dtype=np.float32)
        w.step(actions, 3)
        assert w.check_robot_collision(0)

    def test_collision_rect(self):
        w = _cc.SimWorld()
        w.set_step_time(0.1)
        w.add_robot(0, 0.0, 0.0, 0.0)
        w.add_obstacle(_dict_rect(0.1, 0.0, 2.0, 2.0))
        actions = np.zeros(3, dtype=np.float32)
        w.step(actions, 3)
        assert w.check_robot_collision(0)

    def test_collision_polygon(self):
        w = _cc.SimWorld()
        w.set_step_time(0.1)
        w.add_robot(0, 0.0, 0.0, 0.0)
        verts = [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]]
        w.add_obstacle(_dict_polygon(verts))
        actions = np.zeros(3, dtype=np.float32)
        w.step(actions, 3)
        assert w.check_robot_collision(0)

    def test_collision_drives_into_obstacle(self):
        w = _cc.SimWorld()
        w.set_step_time(0.1)
        w.add_robot(0, 0.0, 0.0, 0.0)
        w.add_obstacle(_dict_circle(0.5, 0.0, 0.3))
        # Drive toward the obstacle
        actions = np.array([1.0, 0.0, 0.0], dtype=np.float32)
        for _ in range(20):
            w.step(actions, 3)
            if w.check_robot_collision(0):
                break
        assert w.check_robot_collision(0)

    def test_collision_against_obstacle_is_detected(self):
        w = _cc.SimWorld()
        w.set_step_time(0.1)
        w.add_robot(0, 0.0, 0.0, 0.0)
        w.add_obstacle(_dict_circle(0.1, 0.0, 1.0))
        actions = np.ones(3, dtype=np.float32)
        w.step(actions, 3)
        assert w.check_robot_collision(0)
        # Collision flag is set but velocity is NOT zeroed (handled by Python layer)
        vx, _vy, _omega = w.get_robot_velocity(0)
        assert vx != pytest.approx(0.0, abs=1e-6)


# =========================================================================
# SimWorld — LiDAR raycasting
# =========================================================================


class TestSimWorldLidar:
    def test_raycast_no_obstacles(self):
        w = _cc.SimWorld()
        w.set_step_time(0.1)
        w.add_robot(0, 0.0, 0.0, 0.0)
        angles = np.array([0.0], dtype=np.float32)
        ranges = w.raycast(0, angles, 10.0)
        assert len(ranges) == 1
        assert ranges[0] == pytest.approx(10.0, abs=0.01)

    def test_raycast_hits_circle(self):
        w = _cc.SimWorld()
        w.set_step_time(0.1)
        w.add_robot(0, 0.0, 0.0, 0.0)
        w.add_obstacle(_dict_circle(3.0, 0.0, 0.5))
        angles = np.array([0.0], dtype=np.float32)
        ranges = w.raycast(0, angles, 10.0)
        assert len(ranges) == 1
        assert ranges[0] < 10.0

    def test_raycast_multiple_angles(self):
        w = _cc.SimWorld()
        w.set_step_time(0.1)
        w.add_robot(0, 0.0, 0.0, 0.0)
        w.add_obstacle(_dict_circle(3.0, 0.0, 0.5))
        angles = np.array([-1.0, 0.0, 1.0], dtype=np.float32)
        ranges = w.raycast(0, angles, 10.0)
        assert len(ranges) == 3
        assert ranges[1] < ranges[0]
        assert ranges[1] < ranges[2]

    def test_raycast_polygon_obstacle(self):
        w = _cc.SimWorld()
        w.set_step_time(0.1)
        w.add_robot(0, 0.0, 0.0, 0.0)
        verts = [[2.0, -0.5], [3.0, -0.5], [3.0, 0.5], [2.0, 0.5]]
        w.add_obstacle(_dict_polygon(verts))
        angles = np.array([0.0], dtype=np.float32)
        ranges = w.raycast(0, angles, 10.0)
        assert ranges[0] < 10.0
        assert 1.5 < ranges[0] < 3.5

    def test_raycast_rect_obstacle(self):
        w = _cc.SimWorld()
        w.set_step_time(0.1)
        w.add_robot(0, 0.0, 0.0, 0.0)
        w.add_obstacle(_dict_rect(3.0, 0.0, 0.5, 2.0))
        angles = np.array([0.0], dtype=np.float32)
        ranges = w.raycast(0, angles, 10.0)
        assert ranges[0] < 10.0

    def test_raycast_180_degrees(self):
        w = _cc.SimWorld()
        w.set_step_time(0.1)
        w.add_robot(0, 0.0, 0.0, 0.0)
        w.add_obstacle(_dict_circle(3.0, 0.0, 0.5))
        n = 181
        angles = np.linspace(-math.pi / 2, math.pi / 2, n, dtype=np.float32)
        ranges = w.raycast(0, angles, 10.0)
        assert len(ranges) == n
        assert np.all(ranges >= 0.0)
        assert np.all(ranges <= 10.0)


# =========================================================================
# SimWorld — A* path planning
# =========================================================================


class TestSimWorldAStar:
    def test_astar_planner_found(self):
        w = _cc.SimWorld()
        ap = w.astar()
        grid = np.zeros((10, 10), dtype=np.uint8)
        ap.set_grid(grid, 10, 10, 0.1)
        path = ap.plan(0.5, 0.5, 8.5, 8.5)
        assert len(path) > 0

    def test_astar_planner_no_path(self):
        w = _cc.SimWorld()
        ap = w.astar()
        grid = np.zeros((10, 10), dtype=np.uint8)
        grid[:, :] = 255  # all occupied
        ap.set_grid(grid, 10, 10, 0.1)
        path = ap.plan(0.5, 0.5, 8.5, 8.5)
        assert len(path) == 0

    def test_astar_planner_obstacle_in_middle(self):
        w = _cc.SimWorld()
        ap = w.astar()
        # 30x30 grid at 0.05 m/cell = 1.5 x 1.5 m world
        grid = np.zeros((30, 30), dtype=np.uint8)
        grid[10:20, 10:20] = 255  # wall in center, leaves room on all sides
        ap.set_grid(grid, 30, 30, 0.05)
        path = ap.plan(0.1, 0.1, 1.4, 1.4)
        assert len(path) > 0
        path_arr = np.array(path).reshape(-1, 2)
        assert len(path_arr) >= 2
        np.testing.assert_allclose(path_arr[0], [0.1, 0.1], atol=0.1)
        np.testing.assert_allclose(path_arr[-1], [1.4, 1.4], atol=0.1)

    def test_astar_same_start_goal(self):
        w = _cc.SimWorld()
        ap = w.astar()
        grid = np.zeros((10, 10), dtype=np.uint8)
        ap.set_grid(grid, 10, 10, 0.1)
        path = ap.plan(5.0, 5.0, 5.0, 5.0)
        assert len(path) > 0


# =========================================================================
# Standalone AStarPlanner
# =========================================================================


class TestStandaloneAStar:
    def test_plan_open_grid(self):
        ap = _cc.AStarPlanner()
        grid = np.zeros((20, 20), dtype=np.uint8)
        ap.set_grid(grid, 20, 20, 0.05)
        path = ap.plan(0.5, 0.5, 18.5, 18.5)
        assert len(path) > 0
        path_arr = np.array(path).reshape(-1, 2)
        assert len(path_arr) >= 2

    def test_plan_blocked_start(self):
        ap = _cc.AStarPlanner()
        grid = np.zeros((10, 10), dtype=np.uint8)
        grid[0, 0] = 255  # start cell occupied
        ap.set_grid(grid, 10, 10, 0.1)
        path = ap.plan(0.05, 0.05, 8.5, 8.5)
        assert len(path) == 0


# =========================================================================
# Standalone kinematics functions
# =========================================================================


class TestStandaloneKinematics:
    def test_step_diff_forward(self):
        nx, ny, nt = _cc.step_diff(0.0, 0.0, 0.0, 1.0, 0.0, 0.1)
        assert nx == pytest.approx(0.1, abs=1e-6)
        assert ny == pytest.approx(0.0, abs=1e-6)
        assert nt == pytest.approx(0.0, abs=1e-6)

    def test_step_diff_rotate(self):
        nx, _ny, nt = _cc.step_diff(0.0, 0.0, 0.0, 0.0, math.pi, 0.1)
        assert nx == pytest.approx(0.0, abs=1e-6)
        assert nt == pytest.approx(0.1 * math.pi, abs=1e-6)

    def test_step_diff_arc(self):
        nx, ny, nt = _cc.step_diff(0.0, 0.0, 0.0, 1.0, 0.5, 0.1)
        assert abs(nx) > 0.0
        assert abs(ny) > 0.0
        assert abs(nt) > 0.0

    def test_step_diff_zero_dt(self):
        nx, ny, nt = _cc.step_diff(1.0, 2.0, 0.5, 1.0, 0.0, 0.0)
        assert nx == pytest.approx(1.0, abs=1e-6)
        assert ny == pytest.approx(2.0, abs=1e-6)
        assert nt == pytest.approx(0.5, abs=1e-6)

    def test_step_omni_forward(self):
        nx, ny, _ = _cc.step_omni(0.0, 0.0, 0.0, 1.0, 0.0, 0.1)
        assert nx == pytest.approx(0.1, abs=1e-6)
        assert ny == pytest.approx(0.0, abs=1e-6)

    def test_step_omni_lateral(self):
        nx, ny, _ = _cc.step_omni(0.0, 0.0, 0.0, 0.0, 1.0, 0.1)
        assert nx == pytest.approx(0.0, abs=1e-6)
        assert ny == pytest.approx(0.1, abs=1e-6)

    def test_step_omni_diagonal(self):
        nx, ny, _ = _cc.step_omni(0.0, 0.0, 0.0, 1.0, 1.0, 0.1)
        assert nx == pytest.approx(0.1, abs=1e-6)
        assert ny == pytest.approx(0.1, abs=1e-6)

    def test_step_omni_rotated_frame(self):
        nx, ny, _ = _cc.step_omni(0.0, 0.0, math.pi / 2, 1.0, 0.0, 0.1)
        assert nx == pytest.approx(0.0, abs=1e-6)
        assert ny == pytest.approx(0.1, abs=1e-6)


# =========================================================================
# Standalone LiDAR raycast
# =========================================================================


class TestStandaloneLidar:
    def test_no_obstacles(self):
        angles = np.array([0.0], dtype=np.float32)
        ranges = _cc.lidar_raycast(0.0, 0.0, 0.0, angles, 10.0, [])
        assert ranges[0] == pytest.approx(10.0, abs=0.01)

    def test_scalar_no_obstacles(self):
        angles = np.array([0.0], dtype=np.float32)
        ranges = _cc.lidar_raycast_scalar(0.0, 0.0, 0.0, angles, 10.0, [])
        assert ranges[0] == pytest.approx(10.0, abs=0.01)

    def test_circle_hit(self):
        angles = np.array([0.0], dtype=np.float32)
        obs = [_dict_circle(3.0, 0.0, 0.5)]
        ranges = _cc.lidar_raycast(0.0, 0.0, 0.0, angles, 10.0, obs)
        assert 2.0 < ranges[0] < 4.0

    def test_polygon_hit(self):
        angles = np.array([0.0], dtype=np.float32)
        verts = [[2.0, -0.5], [4.0, -0.5], [4.0, 0.5], [2.0, 0.5]]
        obs = [_dict_polygon(verts)]
        ranges = _cc.lidar_raycast(0.0, 0.0, 0.0, angles, 10.0, obs)
        assert 1.5 < ranges[0] < 3.5

    def test_rect_hit(self):
        angles = np.array([0.0], dtype=np.float32)
        obs = [_dict_rect(3.0, 0.0, 0.5, 2.0)]
        ranges = _cc.lidar_raycast(0.0, 0.0, 0.0, angles, 10.0, obs)
        assert 2.0 < ranges[0] < 4.0

    def test_miss(self):
        angles = np.array([math.pi / 2], dtype=np.float32)
        obs = [_dict_circle(3.0, 0.0, 0.5)]
        ranges = _cc.lidar_raycast(0.0, 0.0, 0.0, angles, 10.0, obs)
        assert ranges[0] == pytest.approx(10.0, abs=0.01)

    def test_multiple_obstacles_closest_wins(self):
        angles = np.array([0.0], dtype=np.float32)
        obs = [
            _dict_circle(5.0, 0.0, 0.5),
            _dict_circle(2.0, 0.0, 0.3),
        ]
        ranges = _cc.lidar_raycast(0.0, 0.0, 0.0, angles, 10.0, obs)
        # Closer obstacle should be hit
        assert ranges[0] < 3.0

    def test_beam_misses_circle(self):
        # Circle at (3,0) r=0.5 is hit when |angle| < 0.167 rad
        angles = np.array([0.3], dtype=np.float32)
        obs = [_dict_circle(3.0, 0.0, 0.5)]
        ranges = _cc.lidar_raycast(0.0, 0.0, 0.0, angles, 10.0, obs)
        assert ranges[0] == pytest.approx(10.0, abs=0.01)

    def test_scalar_matches_auto(self):
        angles = np.array([0.0, 0.5, 1.0], dtype=np.float32)
        obs = [_dict_circle(3.0, 0.0, 0.5), _dict_rect(5.0, 2.0, 1.0, 0.5)]
        r1 = _cc.lidar_raycast(0.0, 0.0, 0.0, angles, 10.0, obs)
        r2 = _cc.lidar_raycast_scalar(0.0, 0.0, 0.0, angles, 10.0, obs)
        np.testing.assert_allclose(r1, r2, atol=0.01)

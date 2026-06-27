"""C++ BatchSimWorld integration bridge — build, step, and sync for N environments."""

from typing import Any

import numpy as np

try:
    import cpp as _cc

    HAS_C_CORE = hasattr(_cc, "BatchSimWorld")
except Exception:
    HAS_C_CORE = False


class BatchCppSim:
    """Manages C++ BatchSimWorld lifecycle and bulk step delegation."""

    def __init__(self, envs: list[Any], share_obstacles: bool = True):
        self._envs = envs
        self._batch_size = len(envs)
        self._share_obstacles = share_obstacles
        self._w = None

    def build(self) -> None:
        """Build or rebuild the C++ BatchSimWorld from Python objects."""
        if not HAS_C_CORE:
            self._w = None
            return

        import cpp as _cc

        config = _cc.BatchConfig()
        config.batch_size = self._batch_size
        config.share_obstacles = self._share_obstacles
        w = _cc.BatchSimWorld(config)
        master = self._envs[0]
        w.set_step_time(master._world.step_time)

        # ── Robots ──────────────────────────────────────────────
        robots = [obj for obj in master.objects if obj.role == "robot"]
        if not robots:
            self._w = None
            return

        first_robot = robots[0]
        kin = getattr(first_robot, "kinematics", "diff")
        kin_map = {"diff": 0, "omni": 1, "acker": 2, "omni_angular": 3}
        w.set_robot_kinematics(kin_map.get(kin, 0))

        # Limits from master env (same for all)
        vmin = (
            getattr(first_robot, "vel_min", np.array([-1.0, -1.0]))
            .ravel()
            .astype(np.float32)
        )
        vmax = (
            getattr(first_robot, "vel_max", np.array([1.0, 1.0]))
            .ravel()
            .astype(np.float32)
        )
        info = getattr(first_robot, "info", None)
        vacc = (
            info.acce.ravel().astype(np.float32)
            if info is not None
            else np.array([1.0, 1.0], dtype=np.float32)
        )
        vmin3 = np.full(3, -np.inf, dtype=np.float32)
        vmin3[: len(vmin)] = vmin
        vmax3 = np.full(3, np.inf, dtype=np.float32)
        vmax3[: len(vmax)] = vmax
        vacc3 = np.full(3, np.inf, dtype=np.float32)
        vacc3[: len(vacc)] = vacc
        w.set_robot_limits(vmin3, vmax3, vacc3)

        # Robot shape vertices (same for all envs)
        orig_verts = getattr(first_robot, "original_vertices", None)
        if orig_verts is not None and orig_verts.shape[1] >= 3:
            flat = orig_verts.T.flatten().astype(np.float32)
            w.set_robot_vertices(flat)

        # Collect initial poses from all envs
        poses = np.zeros((self._batch_size, 3), dtype=np.float32)
        for i, env in enumerate(self._envs):
            r = next(obj for obj in env.objects if obj.role == "robot")
            s = r.state
            poses[i, 0] = float(s[0, 0])
            poses[i, 1] = float(s[1, 0])
            poses[i, 2] = float(s[2, 0])
        w.set_initial_poses(poses.ravel())

        # ── Obstacles (from master env, mode A) ────────────────
        if self._share_obstacles:
            for obj in master.objects:
                if obj.role != "obstacle" or obj.unobstructed:
                    continue
                shape = getattr(obj, "shape", None)
                pos = getattr(obj, "position", None)
                if pos is None or pos.size < 2:
                    continue
                x, y = float(pos[0, 0]), float(pos[1, 0])
                gf = getattr(obj, "gf", None)

                if shape == "circle":
                    if gf is None:
                        continue
                    radius = float(getattr(gf, "radius", 0.5))
                    w.add_obstacle(
                        {
                            "type": "circle",
                            "x": x,
                            "y": y,
                            "radius": radius,
                        }
                    )
                elif shape == "rectangle" and obj.static:
                    if gf is None:
                        continue
                    verts = getattr(gf, "vertices", None)
                    if verts is not None and verts.shape[1] == 4:
                        w.add_obstacle(
                            {
                                "type": "polygon",
                                "x": 0,
                                "y": 0,
                                "vertices": [
                                    [float(verts[0, i]), float(verts[1, i])]
                                    for i in range(verts.shape[1])
                                ],
                            }
                        )
                elif shape == "polygon" and obj.static:
                    verts = getattr(obj, "vertices", None)
                    if verts is not None and verts.shape[1] >= 3:
                        w.add_obstacle(
                            {
                                "type": "polygon",
                                "x": 0,
                                "y": 0,
                                "vertices": [
                                    [float(verts[0, i]), float(verts[1, i])]
                                    for i in range(verts.shape[1])
                                ],
                            }
                        )
                elif shape == "map":
                    self._add_map_obstacles(w, obj)

        self._w = w

    def _add_map_obstacles(self, w: Any, obj: Any) -> None:
        grid = getattr(obj, "grid_map", None)
        if grid is None or grid.size == 0:
            return
        reso = getattr(obj, "grid_reso", None)
        if reso is None or reso.size < 2:
            return
        rx, ry = float(reso[0, 0]), float(reso[1, 0])
        if rx <= 0 or ry <= 0:
            return
        offset = getattr(obj, "world_offset", [0.0, 0.0])
        ox, oy = float(offset[0]), float(offset[1])
        gw, gh = grid.shape
        visited = [[False] * gh for _ in range(gw)]

        for gi in range(gw):
            for gj in range(gh):
                if visited[gi][gj] or grid[gi, gj] <= 50:
                    continue
                gje = gj
                while gje + 1 < gh and grid[gi, gje + 1] > 50:
                    gje += 1
                gie = gi
                while gie + 1 < gw:
                    all_occ = True
                    for j in range(gj, gje + 1):
                        if grid[gie + 1, j] <= 50:
                            all_occ = False
                            break
                    if not all_occ:
                        break
                    gie += 1
                for i in range(gi, gie + 1):
                    for j in range(gj, gje + 1):
                        visited[i][j] = True
                cx = ox + (gi + gie + 1) * rx * 0.5
                cy = oy + (gj + gje + 1) * ry * 0.5
                hw = (gie - gi + 1) * rx * 0.5
                hh = (gje - gj + 1) * ry * 0.5
                w.add_obstacle(
                    {
                        "type": "rect",
                        "x": cx,
                        "y": cy,
                        "half_w": hw,
                        "half_h": hh,
                    }
                )

    def step(self, action: np.ndarray) -> None:
        """Run one simulation step for all environments in batch.

        Args:
            action: (batch_size, action_dim) numpy array
        """
        w = self._w
        if w is None:
            return

        # Build flat action array (interleaved per environment)
        act_list = np.zeros(self._batch_size * 3, dtype=np.float32)
        for i in range(self._batch_size):
            env = self._envs[i]
            robots = [obj for obj in env.objects if obj.role == "robot"]
            if not robots:
                continue
            obj = robots[0]

            if getattr(obj, "stop_flag", False) or getattr(obj, "static", False):
                continue  # already zero

            a = action[i] if i < len(action) else None
            if a is None:
                a = obj.gen_behavior_vel(None)
                if a is None or np.all(a == 0):
                    a = getattr(obj, "_velocity", np.zeros((2, 1)))
            else:
                min_vel, max_vel = obj.get_vel_range()
                a = np.asarray(a).ravel()
                a = np.clip(a.reshape(-1, 1), min_vel, max_vel).ravel()

            a = np.asarray(a).ravel()
            act_list[i * 3 : i * 3 + min(len(a), 3)] = a[:3]

        # Bulk step
        w.step(act_list, 3)

        # Bulk sync back to Python envs
        self._sync()

        # LiDAR (delegated to each robot's sensor)
        for env in self._envs:
            env._objects_sensor_step()

        # Post-process
        for env in self._envs:
            for obj in env.objects:
                if not obj.static and hasattr(obj, "post_process"):
                    obj.post_process()
            env._status_step()
            env._world.step()

    def _sync(self) -> None:
        """Write C++ batch simulation results back to Python objects."""
        w = self._w
        if w is None:
            return

        poses = w.get_all_poses().reshape(self._batch_size, 3)
        collided = w.get_all_collisions()

        for i, env in enumerate(self._envs):
            robots = [obj for obj in env.objects if obj.role == "robot"]
            if not robots:
                continue
            py_obj = robots[0]

            state = py_obj.state
            if state.dtype.kind in ("i", "u"):
                state = state.astype(np.float64)
                py_obj._state = state
            if state.shape[0] >= 3:
                state[0, 0] = poses[i, 0]
                state[1, 0] = poses[i, 1]
                state[2, 0] = poses[i, 2]

            py_obj.collision_flag = bool(collided[i])

            processed = py_obj.mid_process(py_obj.state.copy())
            if processed is not None:
                py_obj._state = processed

            if hasattr(py_obj, "trajectory") and isinstance(py_obj.trajectory, list):
                py_obj.trajectory.append(py_obj.state.copy())

            if py_obj.gf is not None:
                py_obj._geometry = py_obj.gf.step(py_obj.state)
                import shapely

                py_obj._geometry_valid = shapely.is_valid(py_obj._geometry)
            py_obj._invalidate_reactive_cache()

    def batch_raycast(self, angles: np.ndarray, range_max: float) -> np.ndarray:
        """Run LiDAR for all environments.

        Returns:
            (batch_size, n_beams) array.
        """
        w = self._w
        if w is None:
            bs = self._batch_size
            return np.full((bs, len(angles)), range_max, dtype=np.float32)

        raw = w.batch_raycast(angles.astype(np.float32), float(range_max))
        # raw is beam-major: (n_beams * alloc_size)
        # Reshape to (n_beams, batch_size) then transpose to (batch_size, n_beams)
        n_beams = len(angles)
        return raw.reshape(n_beams, -1)[:, : self._batch_size].T.copy()

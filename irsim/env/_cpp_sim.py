"""C++ SimWorld integration bridge — build, step, and sync."""

from typing import Any

import numpy as np

try:
    import cpp as _cc

    HAS_C_CORE = hasattr(_cc, "SimWorld")
except Exception:
    HAS_C_CORE = False


class CppSim:
    """Manages C++ SimWorld lifecycle and step delegation."""

    def __init__(self, env):
        self._env = env
        self._w = None

    def build(self) -> None:
        """Build or rebuild the C++ SimWorld from current Python objects."""
        if not HAS_C_CORE:
            self._w = None
            return

        w = _cc.SimWorld()
        w.set_step_time(self._env._world.step_time)

        for obj in self._env.objects:
            obj._cpp_id = None

        # Add robots
        for obj in self._env.objects:
            if obj.role != "robot":
                continue
            s = obj.state
            kin = getattr(obj, "kinematics", "diff")
            kin_map = {"diff": 0, "omni": 1, "acker": 2, "omni_angular": 3}
            kid = kin_map.get(kin, 0)
            vmin = (
                getattr(obj, "vel_min", np.array([-1.0, -1.0]))
                .ravel()
                .astype(np.float32)
            )
            vmax = (
                getattr(obj, "vel_max", np.array([1.0, 1.0])).ravel().astype(np.float32)
            )
            info = getattr(obj, "info", None)
            vacc = (
                info.acce.ravel().astype(np.float32)
                if info is not None
                else np.array([1.0, 1.0], dtype=np.float32)
            )
            # Pad to 3 elements for C++ (omni_angular uses 3);
            # unused dims get inf so they don't constrain
            vmin3 = np.full(3, -np.inf, dtype=np.float32)
            vmin3[: len(vmin)] = vmin
            vmax3 = np.full(3, np.inf, dtype=np.float32)
            vmax3[: len(vmax)] = vmax
            vacc3 = np.full(3, np.inf, dtype=np.float32)
            vacc3[: len(vacc)] = vacc
            wb = getattr(obj, "wheelbase", None)
            wheelbase = float(wb) if wb is not None else 0.5
            rid = w.add_robot(
                kid, float(s[0, 0]), float(s[1, 0]), float(s[2, 0]), vmin3, vmax3, vacc3, wheelbase
            )
            obj._cpp_id = rid

            # Set robot shape vertices for collision (use original_vertices = local frame)
            orig_verts = getattr(obj, "original_vertices", None)
            if orig_verts is not None and orig_verts.shape[1] >= 3:
                # C++ expects [x0,y0, x1,y1, ...]; transpose from (2,N) row-major
                flat = orig_verts.T.flatten().astype(np.float32)
                w.set_robot_vertices(rid, flat)
            else:
                # Fallback to default 0.32x0.24 rectangle
                import warnings
                warnings.warn(
                    f"Robot {getattr(obj, 'name', 'unknown')} has no original_vertices; "
                    "using default 0.32x0.24 collision shape.",
                    RuntimeWarning,
                    stacklevel=2,
                )

        # Add all obstacles (static geometry only, or static + dynamic)
        dyn_obs_map = {}  # obj -> dynamic obstacle id

        for obj in self._env.objects:
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
                if obj.static:
                    w.add_obstacle(
                        {
                            "type": "circle",
                            "x": x,
                            "y": y,
                            "radius": radius,
                        }
                    )
                if not obj.static:
                    did = self._add_dynamic_obstacle(w, obj, x, y, "circle", radius)
                    if did >= 0:
                        dyn_obs_map[id(obj)] = did
                        obj._cpp_id = did
            elif shape == "rectangle":
                if gf is None:
                    continue
                verts = getattr(gf, "vertices", None)
                if obj.static:
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
                    else:
                        w.add_obstacle(
                            {
                                "type": "rect",
                                "x": x,
                                "y": y,
                                "half_w": float(getattr(gf, "half_w", 0.5)),
                                "half_h": float(getattr(gf, "half_h", 0.5)),
                            }
                        )
                if not obj.static:
                    hw = float(getattr(gf, "half_w", 0.5))
                    hh = float(getattr(gf, "half_h", 0.5))
                    did = self._add_dynamic_obstacle(
                        w, obj, x, y, "rectangle", hw, hh
                    )
                    if did >= 0:
                        dyn_obs_map[id(obj)] = did
                        obj._cpp_id = did
            elif shape == "polygon":
                # Polygon obstacle: pass vertices to C++ (supports concave via ear-clip)
                verts = getattr(obj, "vertices", None)
                if verts is None or verts.shape[1] < 3:
                    continue
                if obj.static:
                    vlist = [
                        {
                            "type": "polygon",
                            "x": 0,
                            "y": 0,
                            "vertices": [
                                [float(verts[0, i]), float(verts[1, i])]
                                for i in range(verts.shape[1])
                            ],
                        }
                    ]
                    for vd in vlist:
                        w.add_obstacle(vd)
                if not obj.static:
                    # Add as a dynamic polygon obstacle
                    vlist_flat = [[float(verts[0, i]), float(verts[1, i])]
                                  for i in range(verts.shape[1])]
                    kin_map = {"diff": 0, "omni": 1, "acker": 2, "omni_angular": 3}
                    kid = kin_map.get(getattr(obj, "kinematics", "diff"), 0)
                    vmin = getattr(obj, "vel_min", np.array([-1.0, -1.0])).ravel().astype(np.float32)
                    vmax = getattr(obj, "vel_max", np.array([1.0, 1.0])).ravel().astype(np.float32)
                    info = getattr(obj, "info", None)
                    vacc = info.acce.ravel().astype(np.float32) if info is not None else np.array([1.0, 1.0], dtype=np.float32)
                    vmin3 = np.full(3, -np.inf, dtype=np.float32)
                    vmin3[: len(vmin)] = vmin
                    vmax3 = np.full(3, np.inf, dtype=np.float32)
                    vmax3[: len(vmax)] = vmax
                    vacc3 = np.full(3, np.inf, dtype=np.float32)
                    vacc3[: len(vacc)] = vacc
                    s = getattr(obj, "state", None)
                    theta = float(s[2, 0]) if s is not None and s.shape[0] >= 3 else 0.0
                    did = w.add_dynamic_polygon_obstacle(kid, x, y, theta, vlist_flat, vmin3, vmax3, vacc3)
                    if did >= 0:
                        dyn_obs_map[id(obj)] = did
                        obj._cpp_id = did
            elif shape == "linestring":
                # Approximate linestring as thin rect for collision
                verts = getattr(obj, "vertices", None)
                if verts is not None and verts.shape[1] >= 2:
                    cx = float(np.mean(verts[0, :]))
                    cy = float(np.mean(verts[1, :]))
                    half_len = float(np.max(np.abs(verts[0, :] - cx))) * 0.5
                    w.add_obstacle(
                        {
                            "type": "rect",
                            "x": cx,
                            "y": cy,
                            "half_w": max(half_len, 0.05),
                            "half_h": 0.05,
                        }
                    )
                    if not obj.static:
                        did = self._add_dynamic_linestring_obstacle(
                            w, obj, x, y, verts
                        )
                        if did >= 0:
                            dyn_obs_map[id(obj)] = did
                            obj._cpp_id = did
            elif shape == "map":
                grid = getattr(obj, "grid_map", None)
                if grid is None or grid.size == 0:
                    continue
                reso = getattr(obj, "grid_reso", None)
                if reso is None or reso.size < 2:
                    continue
                rx, ry = float(reso[0, 0]), float(reso[1, 0])
                if rx <= 0 or ry <= 0:
                    continue
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
                            {"type": "rect", "x": cx, "y": cy,
                             "half_w": hw, "half_h": hh})

        self._w = w

    def _add_dynamic_obstacle(
        self, w: Any, obj: Any, x: float, y: float, shape_name: str,
        dim1: float, dim2: float | None = None
    ) -> int:
        """Add a dynamic obstacle to the C++ SimWorld.

        Args:
            shape_name: "circle" or "rectangle"
            dim1: radius for circle, half_w for rectangle
            dim2: half_h for rectangle (None for circle)
        """
        kin_map = {"diff": 0, "omni": 1, "acker": 2, "omni_angular": 3}
        kid = kin_map.get(getattr(obj, "kinematics", "diff"), 0)
        vmin = (
            getattr(obj, "vel_min", np.array([-1.0, -1.0])).ravel().astype(np.float32)
        )
        vmax = getattr(obj, "vel_max", np.array([1.0, 1.0])).ravel().astype(np.float32)
        info = getattr(obj, "info", None)
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
        s = getattr(obj, "state", None)
        theta = float(s[2, 0]) if s is not None and s.shape[0] >= 3 else 0.0
        if shape_name == "rectangle":
            hw = dim1
            hh = dim2 if dim2 is not None else dim1
            return w.add_dynamic_rect_obstacle(kid, x, y, theta, hw, hh, vmin3, vmax3, vacc3)
        return w.add_dynamic_obstacle(kid, x, y, theta, dim1, vmin3, vmax3, vacc3)

    def _add_dynamic_linestring_obstacle(
        self, w: Any, obj: Any, x: float, y: float, verts: np.ndarray
    ) -> int:
        """Add a dynamic linestring obstacle to the C++ SimWorld.

        Args:
            verts: (2, N) numpy array of world-frame vertices.
        """
        kin_map = {"diff": 0, "omni": 1, "acker": 2, "omni_angular": 3}
        kid = kin_map.get(getattr(obj, "kinematics", "diff"), 0)
        vmin = (
            getattr(obj, "vel_min", np.array([-1.0, -1.0])).ravel().astype(np.float32)
        )
        vmax = (
            getattr(obj, "vel_max", np.array([1.0, 1.0])).ravel().astype(np.float32)
        )
        info = getattr(obj, "info", None)
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
        s = getattr(obj, "state", None)
        theta = float(s[2, 0]) if s is not None and s.shape[0] >= 3 else 0.0
        vlist_flat = [[float(verts[0, i]), float(verts[1, i])]
                       for i in range(verts.shape[1])]
        return w.add_dynamic_linestring_obstacle(kid, x, y, theta, vlist_flat,
                                                  vmin3, vmax3, vacc3)

    def step(self, action: list[Any]) -> None:
        """Run one simulation step via C++ SimWorld."""
        w = self._w
        if w is None:
            return

        # Run Python pre-processing for all robots (wander goal renewal, etc.)
        for obj in self._env.objects:
            if obj.role == "robot":
                obj.pre_process()

        # Build flat action array; pad each robot to 3 elements for fixed-stride C++ parsing
        act_list = []
        for obj in self._env.objects:
            if obj.role != "robot":
                continue
            # Respect stop_flag / static (mirrors obj.step() early-return behavior)
            if getattr(obj, "stop_flag", False) or getattr(obj, "static", False):
                padded = np.zeros(3, dtype=np.float32)
                act_list.extend(padded)
                continue
            a = action[obj._id] if obj._id < len(action) else None
            if a is None:
                # Generate velocity from behavior (dash, rvo, etc.)
                a = obj.gen_behavior_vel(None)
                if a is None or np.all(a == 0):
                    a = getattr(obj, "_velocity", np.zeros((2, 1)))
            else:
                # Clip user-provided action to velocity limits (gen_behavior_vel does its own clipping)
                min_vel, max_vel = obj.get_vel_range()
                a = np.asarray(a).ravel()
                a = np.clip(a.reshape(-1, 1), min_vel, max_vel).ravel()
            a = np.asarray(a).ravel()
            padded = np.zeros(3, dtype=np.float32)
            padded[: min(len(a), 3)] = a[:3]
            act_list.extend(padded)

        # Step physics & collision via C++
        try:
            w.step(np.array(act_list, dtype=np.float32), 3)
        except Exception as e:
            self._env.logger.error(f"C++ step failed: {e}")
            raise

        # Step dynamic obstacles: generate behavior velocities, then C++ step.
        #
        # Mirrors object_base.py:508-516.  Three gates before normal stepping:
        #   G1 — stop_flag / goal arrival  →  zero C++ velocity + zero action
        #   G2 — unsupported shape          →  zero action (pad for index alignment)
        #   G3 — normal step                →  pre_process + gen_behavior_vel
        # C++ step_dynamic_obstacles reapplies acc clipping internally; we
        # neutralise that by pre-writing the behaviour velocity into the C++
        # state so that cur_vel == act and the acc window trivially includes act.
        obs_act_list = []
        dyn_obstacles = [obj for obj in self._env.objects if obj.role == "obstacle" and not obj.static]
        for did, obj in enumerate(dyn_obstacles):
            # ── G1: immobilised obstacles  ──
            # stop_flag:   collision-triggered halt
            # check_arrive: OmniDash returns [0,0] inside goal_threshold, but
            #               gen_behavior_vel's acc clip would push it to a
            #               non-zero residual.  Bypass early.
            if getattr(obj, "stop_flag", False) or obj.check_arrive(obj.goal):
                obj.pre_process()
                w.set_obstacle_velocity(did, 0.0, 0.0)
                obj._velocity = np.zeros(obj.vel_shape)
                obs_act_list.extend([0.0, 0.0, 0.0])
                continue

            # ── G2: shape not handled by C++ (still pad for index alignment)
            shape = getattr(obj, "shape", None)
            if shape not in ("circle", "rectangle", "polygon", "linestring"):
                obs_act_list.extend([0.0, 0.0, 0.0])
                continue

            # ── G3: normal step  ──
            obj.pre_process()
            a = obj.gen_behavior_vel(None)
            if a is None or np.all(a == 0):
                a = getattr(obj, "_velocity", np.zeros((2, 1)))
            a = np.asarray(a).ravel()

            # Pre-write desired velocity so C++ acc clip is a no-op.
            w.set_obstacle_velocity(did, float(a[0]),
                                    float(a[1]) if len(a) > 1 else 0.0)

            padded = np.zeros(3, dtype=np.float32)
            padded[: min(len(a), 3)] = a[:3]
            obs_act_list.extend(padded)

        if obs_act_list:
            try:
                w.step_dynamic_obstacles(np.array(obs_act_list, dtype=np.float32), 3)
            except Exception as e:
                self._env.logger.error(f"C++ obstacle step failed: {e}")
                raise

        # Sync C++ results back to Python objects
        self._sync()

        # LiDAR (delegated to each robot's sensor step, which may use C++ in lidar2d.py)
        self._env._objects_sensor_step()

        # Run post_process on all non-static objects (mirrors obj.step())
        for obj in self._env.objects:
            if not obj.static and hasattr(obj, 'post_process'):
                obj.post_process()

        # Status update
        self._env._status_step()
        self._env._world.step()

    def _sync(self) -> None:
        """Write C++ simulation results back to Python objects."""
        w = self._w
        if w is None:
            return
        py_robots = [obj for obj in self._env.objects if obj.role == "robot"]
        for py_obj in py_robots:
            rid = getattr(py_obj, "_cpp_id", None)
            if rid is None or rid >= w.num_robots():
                continue
            pose = w.get_robot_pose(rid)
            vel = w.get_robot_velocity(rid)
            collided = w.get_robot_collision(rid)

            state = py_obj.state
            # Ensure float dtype before writing (YAML may give int)
            if state.dtype.kind in ("i", "u"):
                state = state.astype(np.float64)
                py_obj._state = state
            if state.shape[0] >= 3:
                state[0, 0] = pose[0]
                state[1, 0] = pose[1]
                state[2, 0] = pose[2]
            vel_py = py_obj.velocity
            if vel_py.dtype.kind in ("i", "u"):
                vel_py = vel_py.astype(np.float64)
                py_obj._velocity = vel_py
            if vel_py.shape[0] >= 2:
                vel_py[0, 0] = vel[0]
                vel_py[1, 0] = vel[1] if vel[1] is not None else 0.0

            py_obj.collision_flag = collided

            # mid_process: wrap angle, pad/trim to state_dim (mirrors obj.step())
            processed = py_obj.mid_process(py_obj.state.copy())
            if processed is not None:
                py_obj._state = processed

            # Record trajectory (mirrors obj.step() appending state)
            if hasattr(py_obj, "trajectory") and isinstance(py_obj.trajectory, list):
                py_obj.trajectory.append(py_obj.state.copy())

            # Update geometry for rendering (mirrors obj.step())
            if py_obj.gf is not None:
                py_obj._geometry = py_obj.gf.step(py_obj.state)
                import shapely
                py_obj._geometry_valid = shapely.is_valid(py_obj._geometry)
            py_obj._invalidate_reactive_cache()
        py_obstacles = [
            obj for obj in self._env.objects if obj.role == "obstacle" and not obj.static
        ]
        for py_obj in py_obstacles:
            did = getattr(py_obj, "_cpp_id", None)
            if did is None or did >= w.num_dynamic_obstacles():
                continue
            pose = w.get_obstacle_pose(did)
            vel = w.get_obstacle_velocity(did)
            collided = w.get_obstacle_collision(did)

            state = py_obj.state
            if state.dtype.kind in ("i", "u"):
                state = state.astype(np.float64)
                py_obj._state = state
            if state.shape[0] >= 3:
                state[0, 0] = pose[0]
                state[1, 0] = pose[1]
                state[2, 0] = pose[2]
            vel_py = py_obj.velocity
            if vel_py.dtype.kind in ("i", "u"):
                vel_py = vel_py.astype(np.float64)
                py_obj._velocity = vel_py
            if vel_py.shape[0] >= 2:
                vel_py[0, 0] = vel[0]
                vel_py[1, 0] = vel[1] if vel[1] is not None else 0.0

            processed = py_obj.mid_process(py_obj.state.copy())
            if processed is not None:
                py_obj._state = processed

            if hasattr(py_obj, "trajectory") and isinstance(py_obj.trajectory, list):
                py_obj.trajectory.append(py_obj.state.copy())

            if py_obj.gf is not None:
                py_obj._geometry = py_obj.gf.step(py_obj.state)
                import shapely
                py_obj._geometry_valid = shapely.is_valid(py_obj._geometry)
            py_obj.collision_flag = collided

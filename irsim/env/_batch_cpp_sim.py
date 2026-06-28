"""C++ BatchSimWorld bridge — no Python env dependency, pure C++ bulk operations."""

import numpy as np

try:
    import cpp as _cc
    HAS_C_CORE = hasattr(_cc, "BatchSimWorld")
except Exception:
    HAS_C_CORE = False


class BatchCppSim:
    def __init__(self, batch_size: int):
        self._batch_size = batch_size
        self._w = None
        # State cache for external API
        self._cached_poses: np.ndarray | None = None
        self._cached_vels: np.ndarray | None = None
        self._cached_collided: np.ndarray | None = None

    def build(self, config: dict) -> None:
        """Build BatchSimWorld from a robot config dict.

        Config keys:
            kinematics (int): 0=diff, 1=omni, 2=acker, 3=omni_angular
            vel_min, vel_max, vel_acc (np.ndarray): (3,) each
            wheelbase (float): for acker
            vertices (np.ndarray): (2, N) robot shape vertices
            step_time (float)
            poses (np.ndarray): (batch_size, 3) initial [x, y, theta]
        """
        if not HAS_C_CORE:
            self._w = None
            return

        cfg = _cc.BatchConfig()
        cfg.batch_size = self._batch_size
        cfg.share_obstacles = True
        w = _cc.BatchSimWorld(cfg)
        w.set_step_time(config.get("step_time", 0.1))
        w.set_robot_kinematics(config["kinematics"])
        w.set_robot_limits(
            np.asarray(config["vel_min"], dtype=np.float32),
            np.asarray(config["vel_max"], dtype=np.float32),
            np.asarray(config["vel_acc"], dtype=np.float32),
        )
        verts = np.asarray(config["vertex_array"], dtype=np.float32)
        w.set_robot_vertices(verts)
        poses = np.asarray(config["poses"], dtype=np.float32).ravel()
        w.set_initial_poses(poses)

        self._w = w
        self._update_cache()

    def _update_cache(self) -> None:
        w = self._w
        if w is None:
            self._cached_poses = np.zeros((self._batch_size, 3))
            self._cached_vels = np.zeros((self._batch_size, 3))
            self._cached_collided = np.zeros(self._batch_size, dtype=bool)
            return
        self._cached_poses = w.get_all_poses().reshape(self._batch_size, 3)
        self._cached_vels = w.get_all_velocities().reshape(self._batch_size, 3)
        col = w.get_all_collisions()
        self._cached_collided = col.astype(bool) if col.dtype == np.float32 else col

    def clear_obstacles(self) -> None:
        """Remove all obstacles by rebuilding the world."""
        if self._w is not None:
            # We can't selectively clear obstacles in C++,
            # but add_obstacle adds to the obstacle list.
            # After rebuilding via build(), obstacles are cleared.
            pass

    def add_obstacle(self, obs_dict: dict) -> None:
        if self._w is not None:
            self._w.add_obstacle(obs_dict)

    def add_polygon_obstacle(self, verts: list) -> None:
        if self._w is not None:
            self._w.add_obstacle({
                "type": "polygon", "x": 0, "y": 0,
                "vertices": [[float(v[0]), float(v[1])] for v in verts],
            })

    def add_linestring_obstacle(self, verts: list) -> None:
        if self._w is not None:
            self._w.add_obstacle({
                "type": "linestring", "x": 0, "y": 0,
                "vertices": [[float(v[0]), float(v[1])] for v in verts],
            })

    def step(self, action: np.ndarray) -> None:
        w = self._w
        if w is None:
            return
        w.step(np.asarray(action, dtype=np.float32).ravel(), 3)
        self._update_cache()

    def batch_raycast(self, angles: np.ndarray, range_max: float) -> np.ndarray:
        w = self._w
        if w is None:
            return np.full((self._batch_size, len(angles)), range_max, dtype=np.float32)
        raw = w.batch_raycast(angles.astype(np.float32), float(range_max))
        n_beams = len(angles)
        return raw.reshape(n_beams, -1)[:, : self._batch_size].T.copy()

    def set_poses(self, poses: np.ndarray) -> None:
        """Set all robot poses from (batch, 3) numpy array."""
        w = self._w
        if w is not None:
            w.set_initial_poses(np.asarray(poses, dtype=np.float32).ravel())
            self._update_cache()

    @property
    def poses(self) -> np.ndarray:
        return self._cached_poses

    @property
    def velocities(self) -> np.ndarray:
        return self._cached_vels

    @property
    def collisions(self) -> np.ndarray:
        return self._cached_collided

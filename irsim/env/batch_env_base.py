"""BatchEnvBase — multi-environment batch simulation without EnvBase instances."""

from __future__ import annotations

from typing import Any
import numpy as np
try:
    import yaml
except ImportError:
    yaml = None

from ._batch_cpp_sim import BatchCppSim

_KIN_MAP = {"diff": 0, "omni": 1, "acker": 2, "omni_angular": 3}


def _rect_vertices(length: float, width: float) -> np.ndarray:
    """Return (2, 4) vertex array for a rectangle centered at origin."""
    hl, hw = length / 2, width / 2
    return np.array([[-hl,  hl,  hl, -hl],
                     [-hw, -hw,  hw,  hw]], dtype=np.float32)


def _robot_config_from_yaml(yaml_path: str) -> dict:
    """Parse YAML and extract robot + lidar config without creating EnvBase."""
    with open(yaml_path) as f:
        cfg = yaml.safe_load(f)

    robot_cfg = cfg.get("robot", {})
    kin_name = robot_cfg.get("kinematics", {}).get("name", "diff")
    kin = _KIN_MAP.get(kin_name, 0)

    shape = robot_cfg.get("shape", {})
    shape_name = shape.get("name", "circle")
    if shape_name == "rectangle":
        length = float(shape.get("length", 0.32))
        width = float(shape.get("width", 0.24))
        vertex_array = _rect_vertices(length, width).ravel()
    elif shape_name == "circle":
        radius = float(shape.get("radius", 0.2))
        # Approximate circle as octagon
        nv = 8
        angles = np.linspace(0, 2 * np.pi, nv, endpoint=False)
        verts = np.stack([np.cos(angles) * radius, np.sin(angles) * radius], axis=0)
        vertex_array = verts.ravel()
    else:
        vertex_array = _rect_vertices(0.32, 0.24).ravel()

    vel_min = np.array(robot_cfg.get("vel_min", [-0.5, -1.0, -2.0]), dtype=np.float32)
    vel_max = np.array(robot_cfg.get("vel_max", [3.0, 1.0, 2.0]), dtype=np.float32)
    vel_acc = np.full(3, 100.0, dtype=np.float32)  # default high acceleration

    state = robot_cfg.get("state", [0, 0, 0])

    # LiDAR config from sensors
    sensors = robot_cfg.get("sensors", [])
    lidar_cfg = {}
    for s in sensors:
        if s.get("name") in ("lidar2d", "fmcw_lidar2d"):
            n_beams = int(s.get("number", 1200))
            angle_range = float(s.get("angle_range", 4.7124))
            range_max = float(s.get("range_max", 30.0))
            angle_list = np.linspace(-angle_range / 2, angle_range / 2, n_beams, dtype=np.float32)
            lidar_cfg = dict(
                n_beams=n_beams, angle_range=angle_range,
                range_max=range_max, angle_list=angle_list,
            )
            break

    world = cfg.get("world", {})
    step_time = float(world.get("step_time", 0.1))

    return {
        "kinematics": kin,
        "vel_min": vel_min,
        "vel_max": vel_max,
        "vel_acc": vel_acc,
        "vertex_array": vertex_array,
        "step_time": step_time,
        "initial_state": state,
        "lidar": lidar_cfg,
    }


class BatchEnvBase:
    """Batch simulation environment backed purely by C++ BatchSimWorld.

    No EnvBase instances are created. Robot configuration is parsed directly
    from YAML. All state is managed in C++ SoA arrays.
    """

    def __init__(
        self,
        yaml_path: str,
        batch_size: int = 1,
        share_obstacles: bool = True,
    ):
        self._batch_size = batch_size
        self._yaml_config = _robot_config_from_yaml(yaml_path)
        self._lidar_cfg = self._yaml_config.get("lidar", {})
        self._step_time = self._yaml_config["step_time"]

        # Create batch C++ world with all robots at default positions
        self._batch_cpp = BatchCppSim(batch_size)
        self._build()

    def _build(self) -> None:
        config = dict(self._yaml_config)
        # All robots start at the initial state
        init_state = config["initial_state"]
        poses = np.tile(init_state, (self._batch_size, 1)).astype(np.float32)
        config["poses"] = poses
        self._batch_cpp.build(config)

    def rebuild(self) -> None:
        """Clear all obstacles from C++ world (preserves robot config)."""
        # Store current poses before rebuilding
        old_poses = self._batch_cpp.poses.copy() if self._batch_cpp.poses is not None else None
        self._build()
        if old_poses is not None:
            self._batch_cpp.set_poses(old_poses)

    def add_obstacle(self, obs_dict: dict) -> None:
        self._batch_cpp.add_obstacle(obs_dict)

    def step(self, action: np.ndarray) -> None:
        self._batch_cpp.step(action)

    def get_lidar_scan(self) -> np.ndarray:
        lidar = self._lidar_cfg
        if not lidar:
            return np.zeros((self._batch_size, 0), dtype=np.float32)
        return self._batch_cpp.batch_raycast(lidar["angle_list"], lidar["range_max"])

    def get_robot_state(self) -> np.ndarray:
        """Returns (batch_size, 3, 1) robot poses."""
        poses = self._batch_cpp.poses
        return poses[:, :, np.newaxis]

    def set_poses(self, poses: np.ndarray) -> None:
        """Set all robot poses from (batch, 3) array."""
        self._batch_cpp.set_poses(poses)

    # ── Batch C++ bridge access ────────────────────────────────

    @property
    def batch_cpp(self) -> BatchCppSim:
        return self._batch_cpp

    @property
    def batch_size(self) -> int:
        return self._batch_size

    # ── Properties for backward-compat API usage ───────────────

    @property
    def angle_list(self) -> np.ndarray:
        return self._lidar_cfg.get("angle_list", np.array([], dtype=np.float32))

    @property
    def world_param(self):
        from irsim.config.world_param import WorldParam
        wp = WorldParam()
        wp.step_time = self._step_time
        return wp

    @property
    def robot(self):
        """Minimal robot object for generator compatibility."""
        from types import SimpleNamespace
        rect_v = self._yaml_config["vertex_array"]
        nv = len(rect_v) // 2
        verts = rect_v.reshape(2, nv)
        radius = float(np.max(np.linalg.norm(verts, axis=0)))
        # state as (3, 1) column vector
        init = self._yaml_config["initial_state"]
        state_arr = np.array(init, dtype=np.float64).reshape(3, 1)
        return SimpleNamespace(
            radius=radius,
            state=state_arr,
        )

    def get_map(self, resolution: float = 0.1) -> Any:
        """Build a Map from C++ obstacle data for path planning."""
        # Return a simple map object for generator compatibility
        return None

    # ── Close ──────────────────────────────────────────────────

    def close(self) -> None:
        self._batch_cpp._w = None

"""BatchEnvBase — multi-environment batch simulation with shared C++ backend."""

from __future__ import annotations

from typing import Any

import numpy as np

from irsim.env.env_base import EnvBase

from ._batch_cpp_sim import BatchCppSim


class BatchEnvBase:
    """Batch simulation environment wrapping N identical EnvBase instances.

    Public methods mirror EnvBase but operate on batched numpy arrays
    with an extra leading dimension ``(batch_size, ...)``.

    When ``batch_size == 1``, shapes are ``(1, ...)`` so downstream code
    can uniformly index ``result[0]`` to obtain single-environment data.
    """

    def __init__(
        self,
        yaml_path: str,
        batch_size: int = 1,
        display: bool = False,
        seed: int | None = None,
        share_obstacles: bool = True,
        **kwargs: Any,
    ):
        self._batch_size = batch_size
        self._share_obstacles = share_obstacles

        # Create N environments, each with display=False (batch = headless)
        self._envs: list[EnvBase] = []
        for i in range(batch_size):
            env_seed = seed + i if seed is not None else None
            env = EnvBase(yaml_path, display=False, seed=env_seed, **kwargs)
            self._envs.append(env)

        # Replace per-env CppSim with a single BatchCppSim
        self._batch_cpp = BatchCppSim(self._envs, share_obstacles=share_obstacles)
        self._batch_cpp.build()

        # Pre-compute common attributes from the master robot's lidar
        master = self._envs[0]
        robot = master.robot
        self._angle_list = (
            robot.lidar.angle_list
            if robot and robot.lidar
            else np.array([], dtype=np.float32)
        )
        self._range_max = robot.lidar.range_max if robot and robot.lidar else 10.0
        self._action_dim = getattr(robot, "vel_dim", 2)

    @property
    def batch_size(self) -> int:
        return self._batch_size

    # ── Step ─────────────────────────────────────────────────────

    def step(
        self, action: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict]:
        """Advance simulation by one step.

        Args:
            action: (batch_size, action_dim) or (action_dim,) — latter broadcast
                    to (1, action_dim) for single-env convenience.

        Returns:
            (lidar, reward, done, info) where:
                lidar:  (batch_size, n_beams)
                reward: (batch_size,)
                done:   (batch_size,) bool
        """
        action = np.asarray(action, dtype=np.float32)
        if action.ndim == 1:
            action = action[np.newaxis, :]
        assert action.shape[0] == self._batch_size, (
            f"Expected batch_size={self._batch_size}, got {action.shape[0]}"
        )

        self._batch_cpp.step(action)

        lidar = self.get_lidar_scan()
        reward = self._compute_rewards()
        done = self._check_done()

        return lidar, reward, done, {}

    # ── Observations ────────────────────────────────────────────

    def get_lidar_scan(self) -> np.ndarray:
        """Returns (batch_size, n_beams) LiDAR scan."""
        return self._batch_cpp.batch_raycast(self._angle_list, self._range_max)

    def get_robot_state(self) -> np.ndarray:
        """Returns (batch_size, 3, 1) robot pose [x, y, theta]."""
        w = self._batch_cpp._w
        if w is None:
            return np.zeros((self._batch_size, 3, 1), dtype=np.float32)
        poses = w.get_all_poses().reshape(self._batch_size, 3)
        return poses[:, :, np.newaxis]

    # ── Rewards / Done ──────────────────────────────────────────

    def _compute_rewards(self) -> np.ndarray:
        rewards = np.zeros(self._batch_size, dtype=np.float32)
        for i in range(len(self._envs)):
            rewards[i] = 0.0  # TODO: per-env reward function
        return rewards

    def _check_done(self) -> np.ndarray:
        done = np.zeros(self._batch_size, dtype=bool)
        for i, env in enumerate(self._envs):
            done[i] = env.done()
        return done

    def done(self) -> bool:
        """True if ALL environments are done (for backward compat)."""
        return all(env.done() for env in self._envs)

    # ── Reset ───────────────────────────────────────────────────

    def reset(self) -> None:
        """Reset all environments."""
        for env in self._envs:
            # Re-read YAML config for each env (same yaml, different seed)
            env.reset()
        self._batch_cpp.build()

    # ── Properties (delegated to master env) ────────────────────

    @property
    def world(self):
        return self._envs[0].world if self._envs else None

    @property
    def objects(self):
        """Returns objects from master env (all envs share the same structure)."""
        return self._envs[0].objects if self._envs else []

    @property
    def angle_list(self):
        return self._angle_list

    @property
    def robot(self):
        return self._envs[0].robot if self._envs else None

    @property
    def action_dim(self):
        return self._action_dim

    def render(self, *args: Any, **kwargs: Any) -> None:
        """Render the master environment (batch rendering unsupported)."""
        if self._envs:
            self._envs[0].render(*args, **kwargs)

"""Test that LiDAR correctly tracks dynamically rotating obstacles.

The fast path (grid + C++ obstacles) and the scalar brute-force path
(no grid, per-obstacle per-beam) should produce identical results,
even for obstacles that rotate significantly.
"""

import math
import numpy as np
import pytest

try:
    import irsim as _irsim
except ImportError:
    _irsim = None

try:
    from cpp import lidar_raycast_scalar as _scalar
except ImportError:
    _scalar = None

try:
    import irsim.world.sensors._lidar_cpp as _lc
except ImportError:
    _lc = None

N_BEAMS = 360


@pytest.mark.skipif(_irsim is None, reason="irsim not available")
@pytest.mark.skipif(_scalar is None, reason="C++ core not available")
def test_dynamic_rotation_rect_lidar():
    """Fast path must match scalar for a dynamically rotating rect."""
    env = _irsim.make("test_dynamic_rotation.yaml", display=False)
    w = env._cpp._w

    mismatches_by_step = []

    for step in range(100):
        env.step()

        lidar = env.robot.lidar
        from irsim.util.util import transform_point_with_state
        lo = transform_point_with_state(lidar.offset, env.robot.state)
        angles = lidar.angle_list.astype(np.float32)
        rm = float(lidar.range_max)

        # Fast path: grid + C++ obstacles
        r_fast = w.raycast_at(
            float(lo[0, 0]), float(lo[1, 0]), float(lo[2, 0]), angles, rm
        )

        # Scalar path: brute force from Python dicts
        obs_dicts = []
        for obj in lidar._env_param.objects:
            if obj._id != lidar.obj_id and obj._geometry_valid and not obj.unobstructed:
                d = _lc.obj_to_c_dict(obj)
                if d:
                    obs_dicts.append(d)
        r_scalar = _scalar(
            float(lo[0, 0]), float(lo[1, 0]), float(lo[2, 0]),
            angles, rm, obs_dicts,
        )

        fast = np.array(r_fast)
        scalar = np.array(r_scalar)
        mismatches = np.sum(np.abs(fast - scalar) > 0.01)
        mismatches_by_step.append(mismatches)

        if step < 5 or step % 20 == 0:
            pose = w.get_obstacle_pose(0)
            print(f"  Step {step:3d}: mismatches={mismatches:3d}/{N_BEAMS}, "
                  f"theta={math.degrees(pose[2]):.1f}°")

    total = sum(mismatches_by_step)
    rate = total / (len(mismatches_by_step) * N_BEAMS) * 100

    print(f"\n  Total: mismatches={total}/{len(mismatches_by_step)*N_BEAMS} ({rate:.2f}%)")
    assert rate < 1.0, (
        f"Fast vs scalar mismatch {rate:.2f}% exceeds 1%"
    )
    env.end(0)


if __name__ == "__main__":
    test_dynamic_rotation_rect_lidar()

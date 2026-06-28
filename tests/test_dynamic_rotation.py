"""Test that LiDAR correctly tracks dynamically rotating obstacles.

Creates a scene with a rectangular obstacle that rotates during simulation.
The LiDAR data is compared against Shapely-calculated ground truth from
the obstacle's actual rotated vertices, verifying alignment over many steps.
"""

import math
import numpy as np
import pytest

try:
    import irsim
except ImportError:
    irsim = None

try:
    import shapely
    from shapely.geometry import Polygon, LineString, Point
except ImportError:
    shapely = None

RANGE_MAX = 12.0
N_BEAMS = 360


def _rotation_matrix(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def _rect_vertices(cx, cy, hw, hh, theta):
    """Return 4 vertices of a rotated rect in CCW order."""
    R = _rotation_matrix(theta)
    corners = np.array([[hw, hh], [-hw, hh], [-hw, -hh], [hw, -hh]])
    return (R @ corners.T).T + np.array([cx, cy])


@pytest.mark.skipif(irsim is None, reason="irsim not available")
@pytest.mark.skipif(shapely is None, reason="shapely not available")
def test_dynamic_rotation_rect_lidar():
    """LiDAR ray hitting a dynamically rotating rect must match Shapely."""
    import irsim as _irsim

    env = _irsim.make("test_dynamic_rotation.yaml", display=False)
    w = env._cpp._w

    # Run for many steps so the obstacle rotates significantly
    mismatches_by_step = []

    for step in range(100):
        env.step()

        lidar = env.robot.lidar
        ox = float(lidar.lidar_origin[0, 0])
        oy = float(lidar.lidar_origin[1, 0])
        heading = float(lidar.lidar_origin[2, 0])

        # Get obstacle state from C++
        # There's 1 dynamic obstacle (rect)
        if w.num_dynamic_obstacles() < 1:
            continue
        pose = w.get_obstacle_pose(0)

        # Build Shapely rect from C++ obstacle state
        hw, hh = 0.75, 0.4  # length/2, width/2 from YAML
        verts = _rect_vertices(pose[0], pose[1], hw, hh, pose[2])
        poly = Polygon(verts)

        # Check LiDAR data
        ranges = lidar.range_data
        mismatches = 0
        for i in range(N_BEAMS):
            angle = lidar.angle_list[i]
            r = ranges[i]

            # Ray direction in world frame
            world_angle = angle + heading
            dx, dy = math.cos(world_angle), math.sin(world_angle)

            ray = LineString([(ox, oy), (ox + RANGE_MAX * dx, oy + RANGE_MAX * dy)])
            inter = ray.intersection(poly)

            if inter.is_empty:
                expected = RANGE_MAX
            elif inter.geom_type == "Point":
                expected = Point(ox, oy).distance(inter)
            elif inter.geom_type == "MultiPoint":
                expected = min(Point(ox, oy).distance(p) for p in inter.geoms)
            else:
                expected = RANGE_MAX

            diff = abs(r - expected)
            if diff > 0.05:
                mismatches += 1

        mismatches_by_step.append(mismatches)
        if step < 5 or step % 20 == 0:
            print(f"  Step {step:3d}: mismatches={mismatches:3d}/{N_BEAMS}, "
                  f"theta={math.degrees(pose[2]):.1f}°")

    total_mismatches = sum(mismatches_by_step)
    total_beams = len(mismatches_by_step) * N_BEAMS
    rate = total_mismatches / total_beams * 100

    print(f"\n  Total over {len(mismatches_by_step)} steps: "
          f"mismatches={total_mismatches}/{total_beams} ({rate:.2f}%)")

    assert rate < 5.0, (
        f"LiDAR vs Shapely mismatch rate {rate:.2f}% exceeds 5%"
    )
    env.end(0)


if __name__ == "__main__":
    test_dynamic_rotation_rect_lidar()

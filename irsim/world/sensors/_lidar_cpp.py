"""C++ LiDAR bridge — standalone functions with no Shapely dependency."""

import numpy as np

from irsim.util.random import rng

try:
    from cpp import lidar_raycast as _c_lidar_raycast

    HAS_C_CORE = True
except ImportError:
    HAS_C_CORE = False
    _c_lidar_raycast = None


def c_step(lidar, state) -> bool:
    """Try C++ accelerated LiDAR step. Returns True on success."""
    if not HAS_C_CORE:
        return False

    if lidar.has_velocity:
        return False

    ep = lidar._env_param
    objects = ep.objects if ep is not None else []

    obs_dicts = []
    heading = float(lidar.lidar_origin[2, 0]) if lidar.lidar_origin.shape[0] > 2 else 0.0

    for obj in objects:
        if obj._id == lidar.obj_id or not obj._geometry_valid or obj.unobstructed:
            continue
        shape = getattr(obj, "shape", None)
        if shape == "map":
            rects = map_to_c_dicts(obj, downsample_m=0.5)
            obs_dicts.extend(rects)
            continue
        if shape == "polygon":
            pass
        d = obj_to_c_dict(obj)
        if d:
            obs_dicts.append(d)

    if not obs_dicts:
        lidar.range_data[:] = lidar.range_max
        return True

    origin = lidar.lidar_origin
    ox = float(origin[0, 0]) if origin.shape[0] >= 1 else 0.0
    oy = float(origin[1, 0]) if origin.shape[1] >= 1 else 0.0

    try:
        result = _c_lidar_raycast(
            ox, oy, heading,
            lidar.angle_list.astype(np.float32),
            float(lidar.range_max),
            obs_dicts,
        )
        if result is not None and len(result) == lidar.number:
            lidar.range_data[:] = result
            if lidar.noise:
                for i in range(lidar.number):
                    if lidar.range_data[i] < lidar.range_max:
                        lidar.range_data[i] += rng.normal(0, lidar.std)
            return True
    except Exception:
        pass
    return False


def obj_to_c_dict(obj) -> dict | None:
    """Convert a world object to a C++ obstacle dict."""
    shape = getattr(obj, "shape", None)
    pos = getattr(obj, "position", None)
    if shape is None or pos is None or pos.size < 2:
        return None
    gf = getattr(obj, "gf", None)

    d = None
    if shape == "circle":
        d = {
            "type": "circle", "x": float(pos[0, 0]), "y": float(pos[1, 0]),
            "radius": float(getattr(gf, "radius", 0.5)),
        }
    elif shape == "rectangle":
        verts = getattr(gf, "vertices", None)
        if verts is not None and verts.shape[1] == 4:
            d = {
                "type": "polygon", "x": float(pos[0, 0]), "y": float(pos[1, 0]),
                "vertices": [
                    [float(verts[0, i]), float(verts[1, i])]
                    for i in range(verts.shape[1])
                ],
            }
        else:
            length = float(getattr(gf, "length", 1.0))
            width = float(getattr(gf, "width", 1.0))
            d = {
                "type": "rect", "x": float(pos[0, 0]), "y": float(pos[1, 0]),
                "half_w": length / 2, "half_h": width / 2,
            }
    elif shape == "polygon":
        verts = getattr(obj, "vertices", None)
        if verts is not None and verts.shape[1] >= 3:
            d = {
                "type": "polygon", "x": 0.0, "y": 0.0,
                "vertices": [
                    [float(verts[0, i]), float(verts[1, i])]
                    for i in range(verts.shape[1])
                ],
            }
    elif shape == "linestring":
        verts = getattr(obj, "vertices", None)
        if verts is not None and verts.shape[1] >= 2:
            d = {
                "type": "linestring", "x": 0.0, "y": 0.0,
                "vertices": [
                    [float(verts[0, i]), float(verts[1, i])]
                    for i in range(verts.shape[1])
                ],
            }

    if d is None:
        return None

    vel_xy = getattr(obj, "velocity_xy", None)
    if vel_xy is not None:
        vel = np.asarray(vel_xy, dtype=float).reshape(-1)[:2]
        d["vx"] = float(vel[0])
        d["vy"] = float(vel[1])
    else:
        d["vx"] = 0.0
        d["vy"] = 0.0
    return d


def map_to_c_dicts(obj, downsample_m: float = 0.5) -> list[dict]:
    """Convert a map obstacle's grid to a list of C++ rect dicts.

    Merges adjacent occupied cells into larger rectangles to avoid
    ghost-wall gaps, while keeping the rect count manageable.
    The merge itself serves as implicit downsampling; the
    ``downsample_m`` parameter is reserved for future use.
    """
    grid = getattr(obj, "grid_map", None)
    if grid is None or grid.size == 0:
        return []
    reso = getattr(obj, "grid_reso", None)
    if reso is None or reso.size < 2:
        return []
    rx, ry = float(reso[0, 0]), float(reso[1, 0])
    if rx <= 0 or ry <= 0:
        return []
    offset = getattr(obj, "world_offset", [0.0, 0.0])
    ox, oy = float(offset[0]), float(offset[1])
    occ = []
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
                all_occupied = True
                for j in range(gj, gje + 1):
                    if grid[gie + 1, j] <= 50:
                        all_occupied = False
                        break
                if not all_occupied:
                    break
                gie += 1
            for i in range(gi, gie + 1):
                for j in range(gj, gje + 1):
                    visited[i][j] = True
            cx = ox + (gi + gie + 1) * rx * 0.5
            cy = oy + (gj + gje + 1) * ry * 0.5
            half_w = (gie - gi + 1) * rx * 0.5
            half_h = (gje - gj + 1) * ry * 0.5
            occ.append({"type": "rect", "x": cx, "y": cy,
                        "half_w": half_w, "half_h": half_h,
                        "vx": 0.0, "vy": 0.0})
    return occ

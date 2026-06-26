from __future__ import annotations

from typing import Any, Protocol, runtime_checkable

import numpy as np
from shapely.geometry import Point

from .grid_map_generator_base import GridMapGenerator
from .image_map_generator import ImageGridGenerator
from .obstacle_map import (
    CELL_CENTER_OFFSET,
    COLLISION_RADIUS_FACTOR,
    OCCUPANCY_THRESHOLD,
)

# ---------------------------------------------------------------------------
# Typed protocol - structural contract expected by all path planners
# ---------------------------------------------------------------------------


@runtime_checkable
class EnvGridMap(Protocol):
    """Structural type accepted by all path planners.

    Any object that exposes the attributes below (including :class:`Map`)
    is a valid ``EnvGridMap``.  Planners should annotate their *env_map*
    parameter with this protocol instead of the concrete :class:`Map`
    class to support duck-typed map objects.

    Collision precedence (adopted by every planner):
      1. Grid lookup (O(1) per cell) when *grid* is not ``None``; if the grid
         reports occupied, the point is in collision.
      2. When the grid reports free or is unavailable, Shapely geometry
         intersection with *obstacle_list* is used.  Planners therefore
         combine grid and obstacle_list when both are present.
    """

    width: float
    height: float
    resolution: float
    obstacle_list: list
    grid: np.ndarray | None
    world_offset: tuple[float, float]

    @property
    def grid_resolution(self) -> tuple[float, float] | None:
        """Actual cell size ``(x_reso, y_reso)`` derived from *grid* shape and world size."""
        ...

    def grid_occupied(
        self,
        x: float,
        y: float,
        margin_x: float = 0.0,
        margin_y: float = 0.0,
        threshold: float = 50.0,
    ) -> bool | None:
        """Check if any grid cell within the bounding box is occupied."""
        ...

    def is_collision(self, geometry) -> bool:
        """Check collision for a Shapely geometry against the map."""
        ...


def _downsample_occupancy_grid(
    grid: np.ndarray,
    width: float,
    height: float,
    target_resolution: float,
) -> np.ndarray:
    """Downsample an occupancy grid to a coarser resolution (conservative: block max).

    Each coarse cell is the max of the fine cells it covers, so any obstacle
    in the block keeps the coarse cell occupied. Output dtype and 0-100 range
    are preserved.

    Args:
        grid: Fine-resolution occupancy grid (0-100), shape (fine_nx, fine_ny).
        width: World width in metres (x).
        height: World height in metres (y).
        target_resolution: Desired cell size in metres (same for x and y).

    Returns:
        Downsampled grid, shape (coarse_nx, coarse_ny), same dtype as grid.
    """
    fine_nx, fine_ny = grid.shape[0], grid.shape[1]
    coarse_nx = max(1, round(width / target_resolution))
    coarse_ny = max(1, round(height / target_resolution))
    out = np.zeros((coarse_nx, coarse_ny), dtype=grid.dtype)
    for ic in range(coarse_nx):
        i_lo = int(ic * fine_nx / coarse_nx)
        i_hi = int((ic + 1) * fine_nx / coarse_nx)
        if i_hi <= i_lo:
            i_hi = i_lo + 1
        i_hi = min(i_hi, fine_nx)
        for jc in range(coarse_ny):
            j_lo = int(jc * fine_ny / coarse_ny)
            j_hi = int((jc + 1) * fine_ny / coarse_ny)
            if j_hi <= j_lo:
                j_hi = j_lo + 1
            j_hi = min(j_hi, fine_ny)
            out[ic, jc] = np.max(grid[i_lo:i_hi, j_lo:j_hi])
    return out


def _grid_collision_geometry(
    grid: np.ndarray,
    grid_reso: tuple[float, float],
    geometry,
    world_offset: tuple[float, float] = (0.0, 0.0),
) -> bool:
    """Check collision of a Shapely geometry against an occupancy grid.

    Uses the same logic as ObstacleMap.check_grid_collision.
    """
    if grid is None:
        return False

    minx, miny, maxx, maxy = geometry.bounds
    x_reso, y_reso = grid_reso
    offset_x, offset_y = world_offset

    i_min = max(0, int((minx - offset_x) / x_reso))
    i_max = min(grid.shape[0] - 1, int((maxx - offset_x) / x_reso))
    j_min = max(0, int((miny - offset_y) / y_reso))
    j_max = min(grid.shape[1] - 1, int((maxy - offset_y) / y_reso))

    if i_min > i_max or j_min > j_max:
        return False

    collision_radius = max(x_reso, y_reso) * COLLISION_RADIUS_FACTOR

    for i in range(i_min, i_max + 1):
        for j in range(j_min, j_max + 1):
            if grid[i, j] > OCCUPANCY_THRESHOLD:
                cell_x = offset_x + (i + CELL_CENTER_OFFSET) * x_reso
                cell_y = offset_y + (j + CELL_CENTER_OFFSET) * y_reso
                cell_center = Point(cell_x, cell_y)
                if geometry.distance(cell_center) <= collision_radius:
                    return True

    return False


def resolve_obstacle_map(
    obstacle_map: str | np.ndarray | dict[str, Any] | None = None,
    world_width: float | None = None,
    world_height: float | None = None,
) -> np.ndarray | None:
    """Resolve obstacle_map to None or a float64 occupancy grid ndarray."""
    if obstacle_map is None:
        return None
    if isinstance(obstacle_map, np.ndarray):
        return np.asarray(obstacle_map, dtype=np.float64)
    if isinstance(obstacle_map, str):
        obstacle_map = {"name": "image", "path": obstacle_map}
    if isinstance(obstacle_map, dict) and obstacle_map.get("name"):
        name = obstacle_map.get("name")
        if name == "image":
            path = obstacle_map.get("path")
            if not path:
                raise ValueError("obstacle_map image generator requires 'path'.")
            gen = ImageGridGenerator(path=path).generate()
            return np.asarray(gen.grid, dtype=np.float64)
        if world_width is None or world_height is None:
            raise ValueError(
                "obstacle_map generator spec (non-image) requires world_width and "
                "world_height (passed by World.gen_grid_map)."
            )
        return build_grid_from_generator(
            obstacle_map,
            world_width=world_width,
            world_height=world_height,
        )
    raise TypeError(
        "obstacle_map must be None, an ndarray, or a generator spec dict with 'name'."
    )


def build_grid_from_generator(
    spec: dict[str, Any],
    world_width: float,
    world_height: float,
) -> np.ndarray:
    """Build a grid map from a YAML grid_generator spec."""
    name = spec.get("name")
    if not name or name not in GridMapGenerator.registry:
        known = ", ".join(GridMapGenerator.registry)
        raise ValueError(
            f"Unknown or missing grid_generator name: {name!r}. Known: {known}"
        )
    resolution = spec.get("resolution")
    if resolution is None:
        raise ValueError(
            "obstacle_map generator spec must include 'resolution' (meters per cell)."
        )
    grid_width = max(1, round(float(world_width) / float(resolution)))
    grid_height = max(1, round(float(world_height) / float(resolution)))

    cls = GridMapGenerator.registry[name]
    params = {
        k: v
        for k, v in spec.items()
        if k not in ("name", "resolution") and k in cls.yaml_param_names
    }
    params["width"] = grid_width
    params["height"] = grid_height

    return np.asarray(cls(**params).generate().grid, dtype=np.float64)

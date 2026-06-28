"""
Utility functions for IR-SIM simulation.

This package contains helper functions for:
- Mathematical operations
- Coordinate transformations
- File operations
- Geometry utilities
"""

from .util import (
    WrapToPi,
    WrapToRegion,
    cross_product,
    dist_hypot,
    file_check,
    find_file,
    gen_inequal_from_vertex,
    geometry_transform,
    is_2d_list,
    is_convex_and_ordered,
    omni_to_diff,
    random_point_range,
    relative_position,
    transform_point_with_state,
    vertices_transform,
)

__all__ = [
    "WrapToPi",
    "WrapToRegion",
    "cross_product",
    "dist_hypot",
    "file_check",
    "find_file",
    "gen_inequal_from_vertex",
    "geometry_transform",
    "is_2d_list",
    "is_convex_and_ordered",
    "omni_to_diff",
    "random_point_range",
    "relative_position",
    "transform_point_with_state",
    "vertices_transform",
]

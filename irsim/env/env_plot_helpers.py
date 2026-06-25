"""Shared matplotlib drawing helpers for env_plot."""

from typing import Any

import matplotlib.transforms as mtransforms
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Arrow, Circle, Ellipse, Rectangle, Wedge
from matplotlib.patches import Polygon as MplPolygon
from mpl_toolkits.mplot3d import Axes3D, art3d


def linewidth_from_data_units(
    linewidth: float, axis: Any, reference: str = "y"
) -> float:
    """
    Convert a linewidth in data units to linewidth in points.

    Parameters
    ----------
    linewidth: float
        Linewidth in data units of the respective reference-axis
    axis: matplotlib axis
        The axis which is used to extract the relevant transformation
        data (data limits and size must not change afterwards)
    reference: string
        The axis that is taken as a reference for the data width.
        Possible values: 'x' and 'y'. Defaults to 'y'.

    Returns
    -------
    linewidth: float
        Linewidth in points
    """
    fig = axis.get_figure()
    if reference == "x":
        length = fig.bbox_inches.width * axis.get_position().width
        value_range = np.diff(axis.get_xlim()).item()
    elif reference == "y":
        length = fig.bbox_inches.height * axis.get_position().height
        value_range = np.diff(axis.get_ylim()).item()
    # Convert length to points
    length *= 72
    # Scale linewidth to value range
    return linewidth * (length / value_range)


def draw_patch(
    ax: Any,
    shape: str,
    state: np.ndarray | None = None,
    radius: float | None = None,
    vertices: np.ndarray | None = None,
    color: str | None = None,
    zorder: int | None = None,
    linestyle: str | None = None,
    **kwargs: Any,
) -> Any:
    """
    Draw a geometric element (patch or line) on the given axes.

    Supported shapes and expected inputs:
    - circle: use ``state`` (x,y,theta) and ``radius``
    - rectangle: prefer ``vertices`` (2xN) else use ``width``/``height``
    - polygon: use ``vertices`` (2xN)
    - ellipse: use ``width``/``height`` with ``state`` transform
    - wedge: use ``radius`` and either ``theta1``/``theta2`` (deg) or ``fov`` (rad)
    - arrow: use ``state`` for position/orientation
    - line|linestring: use ``vertices`` (2xN)

    Returns the created matplotlib artist.
    """

    created_element = None
    facecolor = kwargs.pop("facecolor", None)
    edgecolor = kwargs.pop("edgecolor", None)
    alpha = kwargs.pop("alpha", None)
    fill = kwargs.pop("fill", None)
    state = state if state is not None else np.zeros((3, 1))

    if shape == "circle":
        if radius is None:
            raise ValueError("circle requires radius")
        patch = Circle((0.0, 0.0), radius)
        created_element = ax.add_patch(patch)
        set_patch_property(
            created_element, ax, state=state, color=color,
            facecolor=facecolor, edgecolor=edgecolor, alpha=alpha,
            zorder=zorder, linestyle=linestyle, fill=fill,
        )

    elif shape == "rectangle":
        if vertices is not None:
            patch = MplPolygon(vertices.T)
            created_element = ax.add_patch(patch)
            set_patch_property(
                created_element, ax, state=state,
                color=color, facecolor=facecolor, edgecolor=edgecolor,
                alpha=alpha, zorder=zorder, linestyle=linestyle, fill=fill,
            )
        else:
            width = kwargs.pop("width", None)
            height = kwargs.pop("height", None)
            if width is None or height is None:
                raise ValueError("rectangle requires either vertices or width/height")
            xy = (float(vertices[0, 0]), float(vertices[1, 0]))
            patch = Rectangle(xy, width, height)
            created_element = ax.add_patch(patch)
            set_patch_property(
                created_element, ax, state=state,
                color=color, facecolor=facecolor, edgecolor=edgecolor,
                alpha=alpha, zorder=zorder, linestyle=linestyle, fill=fill,
            )

    elif shape == "polygon":
        if vertices is None:
            raise ValueError("polygon requires vertices (2xN)")
        patch = MplPolygon(vertices.T)
        created_element = ax.add_patch(patch)
        set_patch_property(
            created_element, ax, state=state,
            color=color, facecolor=facecolor, edgecolor=edgecolor,
            alpha=alpha, zorder=zorder, linestyle=linestyle, fill=fill,
        )

    elif shape == "ellipse":
        width = kwargs.pop("width", None)
        height = kwargs.pop("height", None)
        if width is None or height is None:
            raise ValueError("ellipse requires width and height")
        patch = Ellipse((0.0, 0.0), width, height, angle=0.0)
        created_element = ax.add_patch(patch)
        set_patch_property(
            created_element, ax, state=state,
            color=color, facecolor=facecolor, edgecolor=edgecolor,
            alpha=alpha, zorder=zorder, linestyle=linestyle, fill=fill,
        )

    elif shape == "wedge":
        use_radius = radius if radius is not None else kwargs.pop("radius", None)
        if use_radius is None:
            raise ValueError("wedge requires radius")
        if "theta1" in kwargs and "theta2" in kwargs:
            theta1 = kwargs.pop("theta1")
            theta2 = kwargs.pop("theta2")
        else:
            fov = kwargs.pop("fov", np.pi)
            theta1 = -180 * fov / (2 * np.pi)
            theta2 = 180 * fov / (2 * np.pi)
        patch = Wedge((0.0, 0.0), use_radius, theta1, theta2)
        created_element = ax.add_patch(patch)
        set_patch_property(
            created_element, ax, state=state,
            color=color, facecolor=facecolor, edgecolor=edgecolor,
            alpha=alpha, zorder=zorder, linestyle=linestyle, fill=fill,
        )

    elif shape == "arrow":
        arrow_length = kwargs.pop("arrow_length", 0.4)
        arrow_width = kwargs.pop("arrow_width", 0.6)
        theta = kwargs.pop("theta", float(state[2, 0]) if state.shape[0] >= 3 else 0.0)
        x = float(state[0, 0])
        y = float(state[1, 0])
        dx = float(arrow_length * np.cos(theta))
        dy = float(arrow_length * np.sin(theta))
        patch = Arrow(x, y, dx, dy, width=arrow_width)
        created_element = ax.add_patch(patch)
        set_patch_property(
            created_element, ax, state=None,
            color=color if color is not None else kwargs.pop("arrow_color", None),
            alpha=alpha,
            zorder=zorder if zorder is not None else kwargs.pop("arrow_zorder", None),
            fill=fill,
        )

    elif shape in ("line", "linestring"):
        if vertices is None:
            raise ValueError("line/linestring requires vertices (2xN)")
        if isinstance(ax, Axes3D):
            line3d = art3d.Line3D(
                vertices[0, :], vertices[1, :], zs=kwargs.pop("z", np.zeros((3,)))
            )
            if color is not None:
                line3d.set_color(color)
            if alpha is not None:
                line3d.set_alpha(alpha)
            if zorder is not None:
                line3d.set_zorder(zorder)
            if fill is not None:
                line3d.set_fill(fill)
            ax.add_line(line3d)
            created_element = line3d
        else:
            line2d = Line2D(vertices[0, :], vertices[1, :])
            if linestyle is not None:
                line2d.set_linestyle(linestyle)
            if color is not None:
                line2d.set_color(color)
            if alpha is not None:
                line2d.set_alpha(alpha)
            if zorder is not None:
                line2d.set_zorder(zorder)
            ax.add_line(line2d)
            created_element = line2d

    else:
        raise ValueError(f"Unsupported shape type: {shape}")

    if (
        isinstance(ax, Axes3D)
        and created_element is not None
        and shape not in ("line", "linestring")
    ):
        art3d.patch_2d_to_3d(created_element, z=kwargs.pop("z", 0), zdir="z")

    return created_element


def set_patch_property(
    element: Any,
    ax: Any,
    state: np.ndarray | None = None,
    **kwargs: Any,
) -> None:
    """
    Apply transform and style properties to a patch/artist.

    - If ``state`` provided with at least 3 rows, apply rotation + translation.
    - Apply color/facecolor/edgecolor, alpha, zorder, linestyle when supported.
    """
    if state is not None and state.shape[0] >= 3:
        x = float(state[0, 0])
        y = float(state[1, 0])
        theta = float(state[2, 0])
        transform = mtransforms.Affine2D().rotate(theta).translate(x, y) + ax.transData
        if hasattr(element, "set_transform"):
            element.set_transform(transform)

    color = kwargs.get("color")
    facecolor = kwargs.get("facecolor")
    edgecolor = kwargs.get("edgecolor")
    alpha = kwargs.get("alpha")
    zorder = kwargs.get("zorder")
    linestyle = kwargs.get("linestyle")
    linewidth = kwargs.get("linewidth")
    fill = kwargs.get("fill")

    if color is not None and hasattr(element, "set_color"):
        element.set_color(color)
    if facecolor is not None and hasattr(element, "set_facecolor"):
        element.set_facecolor(facecolor)
    if edgecolor is not None and hasattr(element, "set_edgecolor"):
        element.set_edgecolor(edgecolor)
    if alpha is not None and hasattr(element, "set_alpha"):
        element.set_alpha(alpha)
    if zorder is not None and hasattr(element, "set_zorder"):
        element.set_zorder(zorder)
    if linestyle is not None and hasattr(element, "set_linestyle"):
        element.set_linestyle(linestyle)
    if linewidth is not None and hasattr(element, "set_linewidth"):
        element.set_linewidth(linewidth)
    if fill is not None and hasattr(element, "set_fill"):
        element.set_fill(fill)

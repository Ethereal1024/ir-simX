"""Plotting mixin for ObjectBase — all rendering methods."""

from math import cos, pi, sin

import matplotlib.transforms as mtransforms
import numpy as np
from matplotlib import image
from matplotlib.patches import Arrow, Circle, Wedge
from mpl_toolkits.mplot3d import Axes3D

from irsim.config.path_param import path_manager
from irsim.env.env_plot import draw_patch, linewidth_from_data_units, set_patch_property
from irsim.util.util import file_check


class ObjectBasePlotMixin:
    def plot(
        self,
        ax,
        state: np.ndarray | None = None,
        vertices: np.ndarray | None = None,
        **kwargs,
    ):
        """
        Plot the object on the given axis.

        Args:
            ax: Matplotlib axis object for plotting.
            state: State vector [x, y, theta, ...] defining object position and orientation.
            vertices: Vertices array defining object shape for polygon/rectangle objects.
            **kwargs: Plotting configuration options.
        """

        if state is None:
            state = self.state
        if vertices is None:
            vertices = self.vertices

        self._plot(ax, state, vertices, **kwargs)

    def _init_plot(self, ax, **kwargs):
        """
        Initialize plotting elements using zero state and initial vertices.

        Returns:
            list: Names of plot attributes created (e.g., 'object_patch', 'goal_patch').
        """
        # Apply handler-derived show_arrow default when not explicitly set
        if (
            self.kf is not None
            and "show_arrow" not in self.plot_kwargs
            and "show_arrow" not in kwargs
        ):
            kwargs.setdefault("show_arrow", self.kf.show_arrow)
        return self._plot(
            ax, self.original_state, self.original_vertices, initial=True, **kwargs
        )

    def _plot(self, ax, state, vertices, initial: bool = False, **kwargs):
        """
        Plot the object with the specified state and vertices.

        Args:
            ax: Matplotlib axis object for plotting.
            state: State vector [x, y, theta, ...] defining object position and orientation.
            vertices: Vertices array defining object shape for polygon/rectangle objects.
            initial: Whether the plot is for the initial state. Defaults to False.
            **kwargs: Plotting configuration options:

            Object visualization properties:
                - obj_linestyle (str): Line style for object outline (e.g., '-', '--', ':', '-.').
                - obj_zorder (int): Z-order (drawing layer) for object elements; defaults to 3 for robots, 1 for obstacles.
                - obj_color (str): Color of the object.
                - obj_alpha (float): Transparency of the object (0.0 to 1.0).

                - show_goal (bool): Whether to show the goal position. Defaults to False.
                    - goal_color (str): Color of the goal marker. Defaults to object color.
                    - goal_zorder (int): Z-order of the goal marker. Defaults to 1.
                    - goal_alpha (float): Transparency of the goal marker. Defaults to 0.5.
                - show_text (bool): Whether to show text information. Defaults to False.
                    - text_color (str): Color of the text. Defaults to 'k'.
                    - text_size (int): Font size of the text. Defaults to 10.
                    - text_zorder (int): Z-order of the text. Defaults to 2.
                - show_arrow (bool): Whether to show the velocity arrow. Defaults to False.
                    - arrow_color (str): Color of the arrow. Defaults to "gold".
                    - arrow_length (float): Length of the arrow. Defaults to 0.4.
                    - arrow_width (float): Width of the arrow. Defaults to 0.6.
                    - arrow_zorder (int): Z-order of the arrow. Defaults to 4.
                - show_trajectory (bool): Whether to show the trajectory line. Defaults to False.
                    - traj_color (str): Color of the trajectory. Defaults to object color.
                    - traj_style (str): Line style of the trajectory. Defaults to "-".
                    - traj_width (float): Width of the trajectory line. Defaults to object width.
                    - traj_alpha (float): Transparency of the trajectory. Defaults to 0.5.
                - show_trail (bool): Whether to show object trails. Defaults to False.
                    - trail_freq (int): Frequency of trail display (every N steps). Defaults to 2.
                    - trail_edgecolor (str): Edge color of the trail. Defaults to object color.
                    - trail_linewidth (float): Width of the trail outline. Defaults to 0.8.
                    - trail_alpha (float): Transparency of the trail. Defaults to 0.7.
                    - trail_fill (bool): Whether to fill the trail shape. Defaults to False.
                    - trail_color (str): Fill color of the trail. Defaults to object color.
                - show_sensor (bool): Whether to show sensor visualizations. Defaults to True.
                - show_fov (bool): Whether to show field of view visualization. Defaults to False.
                    - fov_color (str): Fill color of the field of view. Defaults to "lightblue".
                    - fov_edge_color (str): Edge color of the field of view. Defaults to "blue".
                    - fov_zorder (int): Z-order of the field of view. Defaults to 1.
                    - fov_alpha (float): Transparency of the field of view. Defaults to 0.5.

        Creates and stores the following plot elements:
            - object_patch: Main object visualization (patch)
            - object_line: Object outline for linestring shapes (line)
            - object_img: Object image for description-based visualization
            - goal_patch: Goal position marker (patch)
            - _text: Object text label
            - _goal_text: Goal text label
            - arrow_patch: Velocity direction arrow (patch)
            - trajectory_line: Trajectory path visualization (line)
            - fov_patch: Field of view visualization (patch)

        Returns:
            list: List of plot attribute names that were created.
        """

        self.plot_attr_list = [
            "object_patch",
            "object_line",
            "object_img",
            "goal_patch",
            "_text",
            "_goal_text",
            "arrow_patch",
            "trajectory_line",
            "fov_patch",
        ]

        self.plot_kwargs.update(kwargs)
        self.ax = ax

        self.show_goal = self.plot_kwargs.get("show_goal", False)
        self.show_goal_text = self.plot_kwargs.get("show_goal_text", False)
        self.show_goals = self.plot_kwargs.get("show_goals", False)
        show_text = self.plot_kwargs.get("show_text", False)
        show_arrow = self.plot_kwargs.get("show_arrow", False)
        show_trajectory = self.plot_kwargs.get("show_trajectory", False)
        self.show_trail = self.plot_kwargs.get("show_trail", False)
        self.show_sensor = self.plot_kwargs.get("show_sensor", True)
        show_fov = self.plot_kwargs.get("show_fov", False)

        self.trail_freq = self.plot_kwargs.get("trail_freq", 2)

        if self.shape != "map":
            self.plot_object(ax, state, vertices, **self.plot_kwargs)

        if self.show_goal:
            goal_state = state if initial else self.goal
            goal_vertices = vertices if initial else self.goal_vertices
            self.plot_goal(ax, goal_state, goal_vertices, **self.plot_kwargs)

        if show_text:
            self.plot_text(ax, state, **self.plot_kwargs)

        if show_arrow:
            current_velocity = self.velocity_xy if np.any(state) else np.zeros((2, 1))
            arrow_theta = 0.0 if initial else self.heading
            self.plot_arrow(
                ax, state, current_velocity, arrow_theta, **self.plot_kwargs
            )

        if show_trajectory:
            trajectory_data = self.trajectory if np.any(state) else []
            self.plot_trajectory(ax, trajectory_data, **self.plot_kwargs)

        if (
            self.show_trail
            and self._world_param.count % self.trail_freq == 0
            and self._world_param.count > 0
        ):
            self.plot_trail(ax, state, self.vertices, **self.plot_kwargs)

        if self.show_sensor:
            [sensor.plot(ax, state, **self.plot_kwargs) for sensor in self.sensors]

        if show_fov:
            self.plot_fov(ax, **self.plot_kwargs)

        return self.plot_attr_list

    def _step_plot(self, **kwargs):
        """
        Update the positions and properties of all plot elements created in _init_plot.
        Elements are updated using transforms for patches and set_data/set_data_3d for lines,
        based on the object's current state, in both 2D and 3D.

        Update methods by element type:
        - Patches (object_patch, goal_patch, arrow_patch, fov_patch): Updated using matplotlib transforms
        - Lines (object_line, trajectory_line): Updated using set_data method
        - Text (_text): Updated using set_position method
        - Images (object_img): Updated using extent and transform methods

        Args:
            **kwargs: Dynamic plotting properties to update during rendering:

                Object visualization properties:
                - obj_linestyle (str): Line style for object outline (e.g., '-', '--', ':', '-.').
                - obj_zorder (int): Z-order (drawing layer) for object elements.
                - obj_color (str): Color of the object.
                - obj_alpha (float): Transparency of the object (0.0 to 1.0).

                Goal visualization properties:
                - goal_color (str): Color of the goal marker.
                - goal_alpha (float): Transparency of the goal marker.
                - goal_zorder (int): Z-order for goal elements.

                Arrow visualization properties:
                - arrow_color (str): Color of the velocity arrow.
                - arrow_alpha (float): Transparency of the arrow.
                - arrow_zorder (int): Z-order for arrow elements.

                Trajectory visualization properties:
                - traj_color (str): Color of the trajectory line.
                - traj_style (str): Line style of the trajectory (e.g., '-', '--', ':', '-.').
                - traj_width (float): Width of the trajectory line.
                - traj_alpha (float): Transparency of the trajectory line.
                - traj_zorder (int): Z-order for trajectory elements.

                Text visualization properties:
                - text_color (str): Color of the text label.
                - text_size (int): Font size of the text.
                - text_alpha (float): Transparency of the text.
                - text_zorder (int): Z-order for text elements.

                Field of view properties:
                - fov_color (str): Fill color of the field of view.
                - fov_alpha (float): Transparency of the field of view.
                - fov_edge_color (str): Edge color of the field of view.
                - fov_zorder (int): Z-order of the field of view.

                Trail visualization properties (for new trail elements):
                - trail_edgecolor (str): Edge color of the trail.
                - trail_linewidth (float): Width of the trail outline.
                - trail_alpha (float): Transparency of the trail.
                - trail_fill (bool): Whether to fill the trail shape.
                - trail_color (str): Fill color of the trail.
                - trail_zorder (int): Z-order for trail elements.

        Note:
            This method updates existing plot elements in place. Trail elements are created
            new each time based on trail_freq setting and are not updated but recreated.
        """

        x = self.state[0, 0]
        y = self.state[1, 0]
        r_phi = self.state[2, 0]
        r_phi_ang = 180 * r_phi / pi

        for attr in self.plot_attr_list:
            if hasattr(self, attr):
                element = getattr(self, attr)

                if attr == "object_patch":
                    # For patches created at origin in _init_plot, use transform to position them
                    obj_kwargs = {
                        "color": kwargs.get("obj_color", self.color),
                        "alpha": kwargs.get("obj_alpha"),
                        "zorder": kwargs.get("obj_zorder"),
                        "linestyle": kwargs.get("obj_linestyle"),
                    }

                    set_patch_property(element, self.ax, state=self.state, **obj_kwargs)

                elif attr == "object_line":
                    # For lines, use set_data to update coordinates (works for both 2D and 3D)
                    vertices = self.vertices
                    cos_phi = np.cos(r_phi)
                    sin_phi = np.sin(r_phi)
                    rotation_matrix = np.array(
                        [[cos_phi, -sin_phi], [sin_phi, cos_phi]]
                    )
                    rotated_vertices = np.dot(vertices.T, rotation_matrix.T).T
                    translated_vertices = rotated_vertices + np.array([[x], [y]])

                    element.set_data(
                        translated_vertices[0, :], translated_vertices[1, :]
                    )

                    # Update object line properties
                    if "obj_linestyle" in kwargs:
                        element.set_linestyle(kwargs["obj_linestyle"])

                    if "obj_color" in kwargs:
                        element.set_color(kwargs["obj_color"])

                    if "obj_alpha" in kwargs:
                        element.set_alpha(kwargs["obj_alpha"])

                    if "obj_zorder" in kwargs:
                        element.set_zorder(kwargs["obj_zorder"])

                elif attr == "object_img":
                    # Update image position and rotation using transform
                    start_x = self.vertices[0, 0]
                    start_y = self.vertices[1, 0]

                    if not isinstance(self.ax, Axes3D):
                        # Update image extent based on current vertices
                        element.set_extent(
                            [
                                start_x,
                                start_x + self.length,
                                start_y,
                                start_y + self.width,
                            ]
                        )

                        # Create new transform for image
                        trans_data = (
                            mtransforms.Affine2D().rotate_deg_around(
                                start_x, start_y, r_phi_ang
                            )
                            + self.ax.transData
                        )
                        element.set_transform(trans_data)

                elif attr == "goal_patch":
                    goal_kwargs = {
                        "color": kwargs.get("goal_color"),
                        "alpha": kwargs.get("goal_alpha"),
                        "zorder": kwargs.get("goal_zorder"),
                    }

                    if self.goal is None:
                        element.set_visible(False)
                        continue
                    if not element.get_visible():
                        element.set_visible(True)

                    if self.goal.shape[0] > 2:
                        goal_state = self.goal
                    else:
                        goal_state = np.pad(
                            self.goal, (0, 1), "constant", constant_values=0
                        )

                    set_patch_property(
                        element, self.ax, state=goal_state, **goal_kwargs
                    )

                elif attr == "arrow_patch":
                    # Update arrow patch using set_element_property
                    if isinstance(element, Arrow):
                        # Calculate orientation for arrow direction
                        theta = self.heading

                        arrow_state = np.array([[x], [y], [theta]])

                        arrow_kwargs = {
                            "color": kwargs.get("arrow_color"),
                            "alpha": kwargs.get("arrow_alpha"),
                            "zorder": kwargs.get("arrow_zorder"),
                        }

                        set_patch_property(
                            element, self.ax, state=arrow_state, **arrow_kwargs
                        )

                elif attr == "trajectory_line":
                    # Update trajectory line using set_data (works for both 2D and 3D)
                    if isinstance(element, list) and len(element) > 0:
                        line = element[0]
                        x_list = [
                            t[0, 0] for t in self.trajectory[-self.keep_traj_length :]
                        ]
                        y_list = [
                            t[1, 0] for t in self.trajectory[-self.keep_traj_length :]
                        ]

                        if isinstance(self.ax, Axes3D):
                            # For 3D, add z-coordinate (set to 0)
                            z_list = [0] * len(x_list)
                            line.set_data_3d(x_list, y_list, z_list)
                        else:
                            line.set_data(x_list, y_list)

                        ax = line.axes
                        if ax is not None:
                            linewidth = kwargs.get("traj_width", self.width)
                            linewidth_data = linewidth_from_data_units(
                                linewidth, ax, "y"
                            )
                            line.set_linewidth(linewidth_data)

                        if "traj_color" in kwargs:
                            line.set_color(kwargs["traj_color"])

                        if "traj_style" in kwargs:
                            line.set_linestyle(kwargs["traj_style"])

                        if "traj_alpha" in kwargs:
                            line.set_alpha(kwargs["traj_alpha"])

                        if "traj_zorder" in kwargs:
                            line.set_zorder(kwargs["traj_zorder"])

                elif attr == "fov_patch":
                    # Update FOV patch using set_element_property
                    if isinstance(element, Wedge | Circle):
                        direction = r_phi if self.state_dim >= 3 else 0
                        fov_state = np.array([[x], [y], [direction]])

                        fov_kwargs = {
                            "alpha": kwargs.get("fov_alpha"),
                            "zorder": kwargs.get("fov_zorder"),
                        }

                        set_patch_property(
                            element, self.ax, state=fov_state, **fov_kwargs
                        )

        # Update text position using set_position (works for both 2D and 3D)
        if hasattr(self, "_text"):
            text = self._text
            # Prefer runtime kwargs, then initial plot kwargs, fallback to default
            default_text_pos = [-self.radius - 0.1, self.radius + 0.1]
            text_position = kwargs.get(
                "text_position",
                self.plot_kwargs.get("text_position", default_text_pos),
            )

            text.set_position((x + text_position[0], y + text_position[1]))

            # Sync display text (may have been changed via set_text)
            text.set_text(self._get_text())

            # Update text properties
            if "text_color" in kwargs:
                text.set_color(kwargs["text_color"])

            if "text_size" in kwargs:
                text.set_fontsize(kwargs["text_size"])

            if "text_alpha" in kwargs:
                text.set_alpha(kwargs["text_alpha"])

            if "text_zorder" in kwargs:
                text.set_zorder(kwargs["text_zorder"])

        # Update goal text position using set_position (works for both 2D and 3D)
        if self.goal is not None:
            goal_x = self.goal[0, 0]
            goal_y = self.goal[1, 0]
            if hasattr(self, "_goal_text"):
                goal_text = self._goal_text
                # Prefer runtime kwargs, then initial plot kwargs, fallback to default
                default_text_pos = [-self.radius - 0.1, self.radius + 0.1]
                text_position = kwargs.get(
                    "text_position",
                    self.plot_kwargs.get("text_position", default_text_pos),
                )

                goal_text.set_position(
                    (goal_x + text_position[0], goal_y + text_position[1])
                )

                # Sync goal display text (may have been changed via set_goal_text)
                goal_text.set_text(self._get_goal_text())

                # Update text properties
                if "text_color" in kwargs:
                    goal_text.set_color(kwargs["text_color"])

                if "text_size" in kwargs:
                    goal_text.set_fontsize(kwargs["text_size"])

                if "text_alpha" in kwargs:
                    goal_text.set_alpha(kwargs["text_alpha"])

                if "text_zorder" in kwargs:
                    goal_text.set_zorder(kwargs["text_zorder"])

        # Handle trail plotting (creates new elements each time)
        if self.show_trail and self._world_param.count % self.trail_freq == 0:
            self.plot_trail(
                self.ax, self.state, self.original_vertices, **self.plot_kwargs
            )

        # Update sensors
        if self.show_sensor:
            [sensor.step_plot() for sensor in self.sensors]

    def plot_object(
        self,
        ax,
        state: np.ndarray | None = None,
        vertices: np.ndarray | None = None,
        **kwargs,
    ):
        """
        Plot the object itself in the specified coordinate system.

        Args:
            ax: Matplotlib axis object
            state: State of the object (x, y, r_phi) defining position and orientation.
                   If None, uses the object's current state. Defaults to None.
            vertices: Vertices of the object [[x1, y1], [x2, y2], ...] for polygon and rectangle shapes.
                     If None, uses the object's current vertices. Defaults to None.
            **kwargs: Additional plotting options
                - obj_linestyle (str): Line style for object outline, defaults to '-'
                - obj_zorder (int): Drawing layer order, defaults to 3 if object is robot, 1 if object is the obstacle.
                - obj_color (str): Color of the object, defaults to 'k' (black).
                - obj_alpha (float): Transparency of the object, defaults to 1.0.

        Returns:
            None

        Raises:
            Exception: If the underlying patch creation fails (e.g., unsupported shape or backend issues).
        """
        obj_linestyle = kwargs.get("obj_linestyle", "-")
        obj_zorder = kwargs.get("obj_zorder", 3) if self.role == "robot" else 1

        state = self.state if state is None else state
        vertices = self.vertices if vertices is None else vertices

        # Handle 3D plot or no description case
        if self.description is None or isinstance(ax, Axes3D):
            try:
                if self.shape != "map":
                    self.object_patch = draw_patch(
                        ax,
                        shape=self.shape,
                        state=state,
                        radius=self.radius,
                        vertices=vertices,
                        color=self.color,
                        linestyle=obj_linestyle,
                        zorder=obj_zorder,
                    )

                    self.plot_patch_list.append(self.object_patch)

            except Exception as e:
                self.logger.error(f"Error occurred while plotting object: {e!s}")
                raise

        else:
            self.plot_object_image(ax, state, vertices, self.description, **kwargs)

    def plot_object_image(
        self,
        ax,
        state: np.ndarray | None = None,
        vertices: np.ndarray | None = None,
        description: str | None = None,
        **kwargs,
    ):
        """
        Plot the object using an image file based on the description.

        Args:
            ax: Matplotlib axis object for plotting.
            state (Optional[np.ndarray]): State of the object (x, y, r_phi) defining position and orientation.
                                        If None, uses the object's current state. Defaults to None.
            vertices (Optional[np.ndarray]): Vertices of the object for positioning the image.
                                           If None, uses the object's current vertices. Defaults to None.
            description (str): Path or name of the image file to display. Defaults to None.
            **kwargs: Additional plotting options (currently unused).

        Note:
            The image file is searched in the world/description/ directory relative to the project root.
            The image is rotated and positioned according to the object's state and vertices.
        """
        if vertices is None or state is None:
            return
        robot_image_path = file_check(
            description, root_path=path_manager.root_path + "/world/description/"
        )
        if robot_image_path is None:
            return
        start_x = float(vertices[0, 0])
        start_y = float(vertices[1, 0])
        r_phi = float(state[2, 0])
        r_phi_ang = 180 * r_phi / pi
        obj_zorder = kwargs.get("obj_zorder", 2)

        robot_img_read = image.imread(robot_image_path)

        robot_img = ax.imshow(
            robot_img_read,
            extent=[
                float(start_x),
                float(start_x + self.length),
                float(start_y),
                float(start_y + self.width),
            ],
            zorder=obj_zorder,
        )
        trans_data = (
            mtransforms.Affine2D().rotate_deg_around(start_x, start_y, r_phi_ang)
            + ax.transData
        )
        robot_img.set_transform(trans_data)

        self.plot_patch_list.append(robot_img)
        self.object_img = robot_img

    def plot_trajectory(
        self, ax, trajectory: list | None = None, keep_traj_length: int = 0, **kwargs
    ):
        """
        Plot the trajectory path of the object using the specified trajectory data.

        Args:
            ax: Matplotlib axis.
            trajectory: List of trajectory points to plot, where each point is a numpy array [x, y, theta, ...].
                       If None, uses self.trajectory. Defaults to None.
            keep_traj_length (int): Number of steps to keep from the end of trajectory.
                              If 0, plots entire trajectory. Defaults to 0.
            **kwargs: Additional plotting options:
                traj_color (str): Color of the trajectory line.
                traj_style (str): Line style of the trajectory.
                traj_width (float): Width of the trajectory line.
                traj_alpha (float): Transparency of the trajectory line.
                traj_zorder (int): Zorder of the trajectory.
        """

        if trajectory is None:
            trajectory = self.trajectory

        self.keep_traj_length = keep_traj_length

        traj_color = kwargs.get("traj_color", self.color)
        traj_style = kwargs.get("traj_style", "-")
        traj_width = kwargs.get("traj_width", self.width)
        traj_alpha = kwargs.get("traj_alpha", 0.5)
        traj_zorder = kwargs.get("traj_zorder", 0)

        x_list = [t[0, 0] for t in trajectory[-self.keep_traj_length :]]
        y_list = [t[1, 0] for t in trajectory[-self.keep_traj_length :]]

        linewidth = linewidth_from_data_units(traj_width, ax, "y")

        if isinstance(ax, Axes3D):
            linewidth = traj_width * 10

        solid_capstyle = "round" if self.shape == "circle" else "butt"

        self.trajectory_line = ax.plot(
            x_list,
            y_list,
            color=traj_color,
            linestyle=traj_style,
            linewidth=linewidth,
            solid_joinstyle="round",
            solid_capstyle=solid_capstyle,
            alpha=traj_alpha,
            zorder=traj_zorder,
        )

        self.plot_line_list.append(self.trajectory_line)

    def plot_goal(
        self,
        ax,
        goal_state: np.ndarray | None = None,
        vertices: np.ndarray | None = None,
        goal_color: str | None = None,
        goal_zorder: int | None = 1,
        goal_alpha: float | None = 0.5,
        **kwargs,
    ):
        """
        Plot the goal position of the object in the specified coordinate system.

        Args:
            ax: Matplotlib axis.
            goal_state: State of the goal (x, y, r_phi) defining goal position and orientation.
                       If None, nothing is plotted. Defaults to None.
            vertices: Vertices for polygon/rectangle goal shapes.
                     If None, uses original_vertices. Defaults to None.
            goal_color (str): Color of the goal marker. Defaults to be the color of the object.
            goal_zorder (int): Zorder of the goal marker. Defaults to 1.
            goal_alpha (float): Transparency of the goal marker. Defaults to 0.5.
        """

        goal_color = self.color if goal_color is None else goal_color

        if goal_state is None:
            return

        self.goal_patch = draw_patch(
            ax,
            shape=self.shape,
            state=goal_state,
            radius=self.radius,
            vertices=vertices,
            color=goal_color,
            alpha=goal_alpha,
            zorder=goal_zorder,
        )

        self.plot_patch_list.append(self.goal_patch)

    def plot_text(self, ax, state: np.ndarray | None = None, **kwargs):
        """
        Plot the text label of the object at the specified position.

        Args:
            ax: Matplotlib axis.
            state: State of the object (x, y, r_phi) to determine text position.
                   If None, uses the object's current state. Defaults to None.
            **kwargs: Additional plotting options.

                - text_color (str): Color of the text, default is 'k'.
                - text_size (int): Font size of the text, default is 10.
                - text_position (list): Position offset from object center [dx, dy],
                  default is [-radius-0.1, radius+0.1].
                - text_zorder (int): Zorder of the text. Defaults to 2.
                - text_alpha (float): Transparency of the text. Defaults to 1.

        Note:
            Subsequent updates via _step_plot honor the configured text_position if provided.
        """

        if state is None:
            state = self.state

        text_color = kwargs.get("text_color", "k")
        text_size = kwargs.get("text_size", 10)
        text_position = kwargs.get(
            "text_position", [-self.radius - 0.1, self.radius + 0.1]
        )
        text_zorder = kwargs.get("text_zorder", 2)
        text_alpha = kwargs.get("text_alpha", 1)

        x, y = state[0, 0], state[1, 0]

        if isinstance(ax, Axes3D):
            self._text = ax.text(
                x + text_position[0],
                y + text_position[1],
                self.z,
                self._get_text(),
                fontsize=text_size,
                color=text_color,
                zorder=text_zorder,
                alpha=text_alpha,
            )
        else:
            self._text = ax.text(
                x + text_position[0],
                y + text_position[1],
                self._get_text(),
                fontsize=text_size,
                color=text_color,
                zorder=text_zorder,
                alpha=text_alpha,
            )
        self.plot_text_list.append(self._text)

        if self.show_goal and self.show_goal_text:
            goal_x, goal_y = self.goal[0, 0], self.goal[1, 0]
            if isinstance(ax, Axes3D):
                self._goal_text = ax.text(
                    goal_x + text_position[0],
                    goal_y + text_position[1],
                    self.z,
                    self._get_goal_text(),
                    fontsize=text_size,
                    color=text_color,
                    zorder=text_zorder,
                    alpha=text_alpha,
                )
            else:
                self._goal_text = ax.text(
                    goal_x + text_position[0],
                    goal_y + text_position[1],
                    self._get_goal_text(),
                    fontsize=text_size,
                    color=text_color,
                    zorder=text_zorder,
                    alpha=text_alpha,
                )
            self.plot_text_list.append(self._goal_text)

    def plot_arrow(
        self,
        ax,
        state: np.ndarray | None = None,
        velocity: np.ndarray | None = None,
        arrow_theta: float | None = 0.0,
        arrow_length: float = 0.4,
        arrow_width: float = 0.6,
        arrow_color: str | None = None,
        arrow_zorder: int = 3,
        **kwargs,
    ):
        """
        Plot an arrow indicating the velocity orientation of the object at the specified position.

        Args:
            ax: Matplotlib axis.
            state: State of the object (x, y, r_phi) to determine arrow position.
                   If None, uses the object's current state. Defaults to None.
            velocity: Velocity of the object to determine arrow direction.
                     If None, uses the object's current velocity_xy. Defaults to None.
            arrow_length (float): Length of the arrow. Defaults to 0.4.
            arrow_width (float): Width of the arrow. Defaults to 0.6.
            arrow_color (str): Color of the arrow. Defaults to "gold".
            arrow_zorder (int): Z-order for drawing layer. Defaults to 4.
        """

        if state is None:
            state = self.state
        if velocity is None:
            velocity = self.velocity_xy
        if arrow_color is None:
            arrow_color = "gold"

        self.arrow_patch = draw_patch(
            ax,
            shape="arrow",
            state=state,
            color=arrow_color,
            zorder=arrow_zorder,
            arrow_length=arrow_length,
            arrow_width=arrow_width,
            theta=arrow_theta,
        )

        self.plot_patch_list.append(self.arrow_patch)

    def plot_trail(
        self,
        ax,
        state: np.ndarray | None = None,
        vertices: np.ndarray | None = None,
        keep_trail_length: int = 0,
        **kwargs,
    ):
        """
        Plot the trail/outline of the object at the specified position for visualization purposes.

        Args:
            ax: Matplotlib axis.
            state: State of the object (x, y, r_phi) to determine trail position and orientation.
                   If None, uses the object's current state. Defaults to None.
            vertices: Original vertices of the object for polygon and rectangle trail shapes.
                     If None, uses the object's current vertices. Defaults to None.
            keep_trail_length (int): Number of steps to keep from the recent trajectory of trail.
            **kwargs: Additional plotting options:
                trail_type (str): Type of trail shape, defaults to object's shape.
                trail_edgecolor (str): Edge color of the trail.
                trail_linewidth (float): Line width of the trail edge.
                trail_alpha (float): Transparency of the trail.
                trail_fill (bool): Whether to fill the trail shape.
                trail_color (str): Fill color of the trail.
                trail_zorder (int): Z-order of the trail.
        """

        if vertices is None:
            vertices = self.original_vertices

        trail_type = kwargs.get("trail_type", self.shape)
        trail_edgecolor = kwargs.get("trail_edgecolor", self.color)
        trail_linewidth = kwargs.get("trail_linewidth", 0.8)
        trail_alpha = kwargs.get("trail_alpha", 0.7)
        trail_fill = kwargs.get("trail_fill", False)
        trail_color = kwargs.get("trail_color", self.color)
        trail_zorder = kwargs.get("trail_zorder", 0)

        # angle in degrees, no longer needed due to generic draw_patch usage

        trail = draw_patch(
            ax,
            shape=trail_type,
            state=state,
            vertices=vertices,
            radius=self.radius,
            width=self.length,
            height=self.width,
            edgecolor=trail_edgecolor,
            facecolor=trail_color,
            fill=trail_fill,
            alpha=trail_alpha,
            linewidth=trail_linewidth,
            zorder=trail_zorder,
        )

        self.plot_trail_list.append(trail)

        if len(self.plot_trail_list) > keep_trail_length and keep_trail_length > 0:
            self.plot_trail_list.pop(0).remove()

    def plot_fov(self, ax, **kwargs):
        """
        Plot the field of view of the object.
        Creates FOV wedge at origin, will be positioned using transforms in step_plot.

        if fov is 2*pi, plot a circle, otherwise plot a wedge.

        Args:
            ax: Matplotlib axis.
            **kwargs: Additional plotting options.
                fov_color (str): Color of the field of view. Default is 'lightblue'.
                fov_edge_color (str): Edge color of the field of view. Default is 'blue'.
                fov_zorder (int): Z-order of the field of view. Default is 1.
                fov_alpha (float): Transparency of the field of view. Default is 0.5.

        Note:
            No-op when FOV is not configured (fov or fov_radius is None).
        """

        if self.fov is None or self.fov_radius is None:
            return

        fov_color = kwargs.get("fov_color", "lightblue")
        fov_edge_color = kwargs.get("fov_edge_color", "blue")
        fov_zorder = kwargs.get("fov_zorder", 1)
        fov_alpha = kwargs.get("fov_alpha", 0.5)

        # Create FOV wedge at origin with no rotation (-fov/2 to +fov/2 around 0 degrees)
        start_degree = -180 * self.fov / (2 * pi)
        end_degree = 180 * self.fov / (2 * pi)

        state = np.array([[0.0], [0.0], [0.0]])

        shape = "circle" if abs(self.fov - 2 * pi) < 0.01 else "wedge"

        self.fov_patch = draw_patch(
            ax,
            shape=shape,
            state=state,
            radius=self.fov_radius,
            theta1=start_degree,
            theta2=end_degree,
            facecolor=fov_color,
            edgecolor=fov_edge_color,
            alpha=fov_alpha,
            zorder=fov_zorder,
        )

        self.plot_patch_list.append(self.fov_patch)

    def plot_uncertainty(self, ax, **kwargs):
        """
        To be completed.
        """
        pass

    def plot_clear(self, all: bool = False):
        """
        Clear all plotted elements from the axis.

        Args:
            all (bool): If True, also clears trail elements. If False, keeps trail elements. Defaults to False.
        """
        [patch.remove() for patch in self.plot_patch_list]
        [line.pop(0).remove() for line in self.plot_line_list]
        [text.remove() for text in self.plot_text_list]

        if all:
            [trail.remove() for trail in self.plot_trail_list]
            self.plot_trail_list = []

        self.plot_patch_list = []
        self.plot_line_list = []
        self.plot_text_list = []

        [sensor.plot_clear() for sensor in self.sensors]

"""
This file is the implementation of the Reciprocal Velocity Obstacle (RVO) algorithm for multi-robot collision avoidance.

Author: Ruihua Han

reference: https://github.com/MengGuo/RVO_Py_MAS
"""

from math import asin, atan2, cos, pi, sin, sqrt

import numpy as np

from irsim.util.util import dist_hypot, log_error


class reciprocal_vel_obs:
    """
    A class to implement the Reciprocal Velocity Obstacle (RVO) algorithm for multi-robot collision avoidance.

    Args:

        state (list): The rvo state of the agent [x, y, vx, vy, radius, vx_des, vy_des].
        obs_state_list (list) : List of states of static obstacles [[x, y, vx, vy, radius]].
        vxmax (float): Maximum velocity in the x direction.
        vymax (float): Maximum velocity in the y direction.
        acce (float): Acceleration limit.
        factor (float): Penalty weighting factor for velocity selection.
        line_obs_list (list): List of line segments [[x1, y1, x2, y2], ...].
    """

    def __init__(
        self,
        state: list,
        obs_state_list=None,
        vxmax=1.5,
        vymax=1.5,
        acce=0.5,
        factor=1.0,
        line_obs_list=None,
    ):
        if obs_state_list is None:
            obs_state_list = []
        if line_obs_list is None:
            line_obs_list = []
        self.state = state
        self.obs_state_list = obs_state_list
        self.line_obs_list = line_obs_list
        self.vxmax = vxmax
        self.vymax = vymax
        self.acce = acce
        self.factor = factor

    def update(self, state, obs_state_list, line_obs_list=None):
        self.state = state
        self.obs_state_list = obs_state_list
        self.line_obs_list = line_obs_list if line_obs_list is not None else []

    def cal_vel(self, mode="rvo"):
        """
        Calculate the velocity of the agent based on the Reciprocal Velocity Obstacle (RVO) algorithm.

        Args:
            mode (str): The vo configure to calculate the velocity. It can be "rvo", "hrvo", or "vo".
                - rvo: Reciprocal Velocity Obstacle (RVO) algorithm, for multi-robot collision avoidance.
                - hrvo: Hybrid Reciprocal Velocity Obstacle (HRVO) algorithm, for multi-robot collision avoidance.
                - vo: Velocity Obstacle (VO) algorithm, for obstacle-robot collision avoidance.

        Returns:
            list[float]: Selected velocity [vx, vy].
        """

        if mode == "rvo":
            rvo_list = self.config_rvo()
        elif mode == "hrvo":
            rvo_list = self.config_hrvo()
        elif mode == "vo":
            rvo_list = self.config_vo()

        else:  # pragma: no cover - defensive guard; callers pass a valid mode
            log_error("wrong method mode, please input vo, rvo or hrvo")

        vo_outside, vo_inside = self.vel_candidate(rvo_list)
        return self.vel_select(vo_outside, vo_inside)

    def config_rvo(self):
        rvo_list = []

        for obstacle in self.obs_state_list:
            rvo = self.config_rvo_mode(obstacle)
            rvo_list.append(rvo)

        rvo_list.extend(self.config_vo_lines())

        return rvo_list

    def config_rvo_mode(self, obstacle):
        x = self.state[0]
        y = self.state[1]
        vx = self.state[2]
        vy = self.state[3]
        r = self.state[4]

        mode = "sta_circular" if vx == 0 and vy == 0 else "moving"

        if mode == "moving":
            mx = obstacle[0]
            my = obstacle[1]
            mvx = obstacle[2]
            mvy = obstacle[3]
            mr = obstacle[4]

            rvo_apex = [(vx + mvx) / 2, (vy + mvy) / 2]

        elif mode == "sta_circular":
            mx = obstacle[0]
            my = obstacle[1]
            mvx = 0
            mvy = 0
            mr = obstacle[2] + 0.2

            vo_apex = [mvx, mvy]
            rvo_apex = vo_apex  # vo
        else:  # pragma: no cover - unreachable; mode is "moving" or "sta_circular"
            log_error("wrong rvo mode")

        dis_mr = np.sqrt((my - y) ** 2 + (mx - x) ** 2)
        angle_mr = atan2(my - y, mx - x)

        if dis_mr < r + mr:
            dis_mr = r + mr

        ratio = (r + mr) / dis_mr

        if ratio > 1:
            ratio = 1
        if ratio < -1:
            ratio = -1

        half_angle = asin(ratio)
        line_left_ori = angle_mr + half_angle
        line_right_ori = angle_mr - half_angle

        line_left_vector = [cos(line_left_ori), sin(line_left_ori)]
        line_right_vector = [cos(line_right_ori), sin(line_right_ori)]

        # return [rvo_apex, line_left_ori, line_right_ori]

        return [rvo_apex, line_left_vector, line_right_vector]

    def config_hrvo(self):
        hrvo_list = []

        for obstacle in self.obs_state_list:
            # for circular: [x, y, radius]
            hrvo = self.config_hrvo_mode(obstacle)
            hrvo_list.append(hrvo)

        hrvo_list.extend(self.config_vo_lines())

        return hrvo_list

    def config_hrvo_mode(self, obstacle):
        x = self.state[0]
        y = self.state[1]
        vx = self.state[2]
        vy = self.state[3]
        r = self.state[4]

        mode = "sta_circular" if vx == 0 and vy == 0 else "moving"

        if mode == "moving":
            mx = obstacle[0]
            my = obstacle[1]
            mvx = obstacle[2]
            mvy = obstacle[3]
            mr = obstacle[4]

        elif mode == "sta_circular":
            mx = obstacle[0]
            my = obstacle[1]
            mvx = 0
            mvy = 0
            mr = obstacle[2] + 0.2

        else:  # pragma: no cover - unreachable; mode is "moving" or "sta_circular"
            log_error("wrong hrvo mode")

        rvo_apex = [(vx + mvx) / 2, (vy + mvy) / 2]
        vo_apex = [mvx, mvy]

        dis_mr = np.sqrt((my - y) ** 2 + (mx - x) ** 2)
        angle_mr = atan2(my - y, mx - x)

        if dis_mr < r + mr:
            dis_mr = r + mr

        ratio = (r + mr) / dis_mr

        half_angle = asin(ratio)
        line_left_ori = angle_mr + half_angle
        line_right_ori = angle_mr - half_angle

        line_left_vector = [cos(line_left_ori), sin(line_left_ori)]
        line_right_vector = [cos(line_right_ori), sin(line_right_ori)]

        if mode == "moving":
            cl_vector = [mx - x, my - y]

            cur_v = [vx - rvo_apex[0], vy - rvo_apex[1]]

            dis_rv = dist_hypot(rvo_apex[0], rvo_apex[1], vo_apex[0], vo_apex[1])
            radians_rv = atan2(rvo_apex[1] - vo_apex[1], rvo_apex[0] - vo_apex[0])

            diff = line_left_ori - radians_rv

            temp = pi - 2 * half_angle

            if temp == 0:
                temp = temp + 0.01

            dis_diff = dis_rv * sin(diff) / sin(temp)

            if reciprocal_vel_obs.cross_product(cl_vector, cur_v) <= 0:
                hrvo_apex = [
                    rvo_apex[0] - dis_diff * cos(line_right_ori),
                    rvo_apex[1] - dis_diff * sin(line_right_ori),
                ]
            else:
                hrvo_apex = [
                    vo_apex[0] + dis_diff * cos(line_right_ori),
                    vo_apex[1] + dis_diff * sin(line_right_ori),
                ]

            return [hrvo_apex, line_left_vector, line_right_vector]

        if mode == "sta_circular":
            return [vo_apex, line_left_vector, line_right_vector]
        return None

    def config_vo(self):
        vo_list = []

        for obstacle in self.obs_state_list:
            vo = self.config_vo_mode(obstacle)
            vo_list.append(vo)

        vo_list.extend(self.config_vo_lines())

        return vo_list

    def config_vo_mode(self, obstacle):
        x = self.state[0]
        y = self.state[1]
        vx = self.state[2]
        vy = self.state[3]
        r = self.state[4]

        mode = "sta_circular" if vx == 0 and vy == 0 else "moving"

        if mode == "moving":
            mx = obstacle[0]
            my = obstacle[1]
            mvx = obstacle[2]
            mvy = obstacle[3]
            mr = obstacle[4]

        elif mode == "sta_circular":
            mx = obstacle[0]
            my = obstacle[1]
            mvx = 0
            mvy = 0
            mr = obstacle[2] + 0.2

        else:  # pragma: no cover - unreachable; mode is "moving" or "sta_circular"
            log_error("wrong obstacle mode")

        vo_apex = [mvx, mvy]
        dis_mr = np.sqrt((my - y) ** 2 + (mx - x) ** 2)
        angle_mr = atan2(my - y, mx - x)

        if dis_mr < r + mr:
            dis_mr = r + mr

        ratio = (r + mr) / dis_mr

        if ratio > 1:
            ratio = 1
        if ratio < -1:
            ratio = -1

        half_angle = asin(ratio)
        line_left_ori = angle_mr + half_angle
        line_right_ori = angle_mr - half_angle

        line_left_vector = [cos(line_left_ori), sin(line_left_ori)]
        line_right_vector = [cos(line_right_ori), sin(line_right_ori)]

        return [vo_apex, line_left_vector, line_right_vector]

    def config_vo_lines(self):
        """Compute VO cones for line segment obstacles.

        For each segment, compute the angular span as seen from the agent,
        expanded by asin(r / dist) on each side to account for the agent radius.
        The apex is [0, 0] since line obstacles are static.
        """
        vo_list = []
        x = self.state[0]
        y = self.state[1]
        r = self.state[4]

        for seg in self.line_obs_list:
            x1, y1, x2, y2 = seg

            # Closest point on segment to agent
            dx_seg = x2 - x1
            dy_seg = y2 - y1
            seg_len_sq = dx_seg * dx_seg + dy_seg * dy_seg

            if seg_len_sq < 1e-12:
                continue

            t = ((x - x1) * dx_seg + (y - y1) * dy_seg) / seg_len_sq
            t = max(0.0, min(1.0, t))
            cx = x1 + t * dx_seg
            cy = y1 + t * dy_seg
            dist_closest = sqrt((x - cx) ** 2 + (y - cy) ** 2)

            if dist_closest < 1e-6:
                dist_closest = 1e-6

            # Angles from agent to each endpoint
            angle1 = atan2(y1 - y, x1 - x)
            angle2 = atan2(y2 - y, x2 - x)

            # Expand by agent radius at each endpoint
            dist1 = sqrt((x1 - x) ** 2 + (y1 - y) ** 2)
            dist2 = sqrt((x2 - x) ** 2 + (y2 - y) ** 2)

            expand1 = asin(min(1.0, r / max(dist1, r))) if dist1 > 1e-6 else pi / 2
            expand2 = asin(min(1.0, r / max(dist2, r))) if dist2 > 1e-6 else pi / 2

            # Determine which endpoint is "left" and which is "right"
            # by checking the angular difference
            diff = atan2(sin(angle2 - angle1), cos(angle2 - angle1))

            if diff >= 0:
                # angle2 is to the left of angle1
                left_angle = angle2 + expand2
                right_angle = angle1 - expand1
            else:
                # angle1 is to the left of angle2
                left_angle = angle1 + expand1
                right_angle = angle2 - expand2

            line_left_vector = [cos(left_angle), sin(left_angle)]
            line_right_vector = [cos(right_angle), sin(right_angle)]

            vo_list.append([[0, 0], line_left_vector, line_right_vector])

        return vo_list

    def vel_candidate(self, rvo_list):
        cur_vx = self.state[2]
        cur_vy = self.state[3]

        cur_vx_min = max((cur_vx - self.acce), -self.vxmax)
        cur_vx_max = min((cur_vx + self.acce), self.vxmax)
        cur_vy_min = max((cur_vy - self.acce), -self.vymax)
        cur_vy_max = min((cur_vy + self.acce), self.vymax)

        vx_grid = np.arange(cur_vx_min, cur_vx_max + 1e-9, 0.05)
        vy_grid = np.arange(cur_vy_min, cur_vy_max + 1e-9, 0.05)
        vx_2d, vy_2d = np.meshgrid(vx_grid, vy_grid, indexing="ij")
        vx_flat = vx_2d.ravel()
        vy_flat = vy_2d.ravel()
        n_cand = len(vx_flat)

        if n_cand == 0 or not rvo_list:
            return [], []

        # for each neighbor, compute cross products for all candidates at once
        in_any_vo = np.zeros(n_cand, dtype=bool)

        for rvo in rvo_list:
            apex_x, apex_y = rvo[0]
            lx, ly = rvo[1]
            rx, ry = rvo[2]

            rel_vx = vx_flat - apex_x
            rel_vy = vy_flat - apex_y

            cross_left = lx * rel_vy - ly * rel_vx
            cross_right = rx * rel_vy - ry * rel_vx

            in_this_vo = (cross_left <= 0) & (cross_right >= 0)
            in_any_vo |= in_this_vo

        vo_outside_cand = np.stack([vx_flat[~in_any_vo], vy_flat[~in_any_vo]], axis=1)
        vo_inside_cand = np.stack([vx_flat[in_any_vo], vy_flat[in_any_vo]], axis=1)

        return vo_outside_cand.tolist(), vo_inside_cand.tolist()

    def vel_select(self, vo_outside, vo_inside):
        vel_des = [self.state[5], self.state[6]]

        if len(vo_outside) != 0:
            arr = np.asarray(vo_outside)
            dx = arr[:, 0] - vel_des[0]
            dy = arr[:, 1] - vel_des[1]
            idx = (dx * dx + dy * dy).argmin()
            return vo_outside[idx]

        if len(vo_inside) != 0:
            arr = np.asarray(vo_inside)
            scores = np.array([self.penalty(v, vel_des, self.factor) for v in arr])
            idx = scores.argmin()
            return vo_inside[idx]

        return [0.0, 0.0]

    def penalty(self, vel, vel_des, factor):
        s = self.state
        x, y = s[0], s[1]
        tc_list = []

        if self.obs_state_list:
            obs_arr = np.asarray(self.obs_state_list)
            dx = obs_arr[:, 0] - x
            dy = obs_arr[:, 1] - y
            dist_sq = dx * dx + dy * dy
            r_sum = s[4] + obs_arr[:, 4]
            diff = np.maximum(dist_sq - r_sum * r_sum, 0.0)
            dis_vel = np.sqrt(diff)
            vel_trans_x = 2 * vel[0] - s[2] - obs_arr[:, 2]
            vel_trans_y = 2 * vel[1] - s[3] - obs_arr[:, 3]
            vel_trans_speed = np.sqrt(vel_trans_x * vel_trans_x + vel_trans_y * vel_trans_y) + 1e-7
            tcs = dis_vel / vel_trans_speed
            tc_list.append(tcs.min())

        for seg in self.line_obs_list:
            tc = self._tc_line_segment(x, y, s[4], vel, seg)
            tc_list.append(tc)

        if not tc_list:
            return dist_hypot(vel_des[0], vel_des[1], vel[0], vel[1])

        tc_min = min(tc_list)

        if tc_min == 0:
            tc_min = 0.0001

        return factor * (1 / tc_min) + dist_hypot(
            vel_des[0], vel_des[1], vel[0], vel[1]
        )

    @staticmethod
    def _tc_line_segment(x, y, r, vel, seg):
        """Compute time-to-collision between agent and a line segment.

        Uses ray-segment intersection with the segment expanded by agent radius.
        """
        x1, y1, x2, y2 = seg
        # Segment direction and normal
        dx_seg = x2 - x1
        dy_seg = y2 - y1
        seg_len = sqrt(dx_seg * dx_seg + dy_seg * dy_seg)

        if seg_len < 1e-12:
            return 1e6

        # Outward normal (toward agent side)
        nx = -dy_seg / seg_len
        ny = dx_seg / seg_len

        # Make sure normal points toward agent
        if nx * (x - x1) + ny * (y - y1) < 0:
            nx, ny = -nx, -ny

        # Perpendicular distance from agent to segment line
        perp_dist = nx * (x - x1) + ny * (y - y1)

        # Velocity component toward the segment
        vel_toward = -(nx * vel[0] + ny * vel[1])

        if vel_toward <= 1e-7:
            # Moving away or parallel — no collision
            return 1e6

        # Time to reach the expanded segment (offset by radius)
        tc = (perp_dist - r) / vel_toward

        if tc < 0:
            tc = 0

        # Check if the collision point is within the segment bounds
        # Project collision point onto segment axis
        col_x = x + vel[0] * tc
        col_y = y + vel[1] * tc
        t_proj = ((col_x - x1) * dx_seg + (col_y - y1) * dy_seg) / (seg_len * seg_len)

        # Allow some margin beyond endpoints for the agent radius
        margin = r / seg_len
        if t_proj < -margin or t_proj > 1 + margin:
            return 1e6

        return tc

    # judge the direction by vector
    @staticmethod
    def between_vector(line_left_vector, line_right_vector, line_vector):
        return bool(
            reciprocal_vel_obs.cross_product(line_left_vector, line_vector) <= 0
            and reciprocal_vel_obs.cross_product(line_right_vector, line_vector) >= 0
        )

    @staticmethod
    def cross_product(vector1, vector2):
        return float(vector1[0] * vector2[1] - vector2[0] * vector1[1])

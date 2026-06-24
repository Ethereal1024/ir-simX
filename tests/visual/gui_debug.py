"""gui.py with persistent LiDAR hit point accumulation.
All scan endpoints stay on screen permanently — drive around with WASD
and check for phantom points.

Usage:  MPLBACKEND=TkAgg conda run -n irsim_test python .../gui_debug.py"""
import os, sys; sys.path.insert(0, '/home/fanshu/Workplace/ir-simX')
import numpy as np
import matplotlib
import irsim

env = irsim.make('/home/fanshu/Workplace/ir-sim/ir-sim/usage/17gui_world/gui.yaml', save_ani=False, full=False)
robot = env.robot_list[0]

# Accumulate raw LiDAR endpoints (persistent)
all_x, all_y = [], []
scat = None

for _i in range(10000):
    env.step()
    env.render(0.05, show_goal=False)

    # Accumulate this frame's LiDAR hit points
    r = robot.lidar.range_data
    ang = robot.lidar.angle_list
    ox, oy, hd = robot.state[0,0], robot.state[1,0], robot.state[2,0]
    ex = ox + r * np.cos(ang + hd)
    ey = oy + r * np.sin(ang + hd)
    hit = r < robot.lidar.range_max * 0.9

    all_x.extend(ex[hit].tolist())
    all_y.extend(ey[hit].tolist())

    # Update scatter plot (persistent overlay on irsim's axes)
    ax = env._env_plot.ax
    if scat is None:
        scat = ax.scatter(all_x, all_y, s=2, c='red', alpha=0.3, zorder=5)
    else:
        scat.set_offsets(np.c_[all_x, all_y])
    ax.set_title(f'LiDAR hits accumulated: {len(all_x)}')

    if env.mouse_left_pos is not None:
        env.robot.set_goal(env.mouse_left_pos)
    if env.done():
        break

env.end(3)

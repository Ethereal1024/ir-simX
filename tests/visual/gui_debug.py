"""gui.py with persistent LiDAR accumulation + logging to file.
Drive around with WASD, exit with ESC, then send me /tmp/lidar_log.jsonl
Usage:  MPLBACKEND=TkAgg conda run -n irsim_test python .../gui_debug.py"""
import os, sys; sys.path.insert(0, '/home/fanshu/Workplace/ir-simX')
import numpy as np, json, matplotlib
import irsim

env = irsim.make('/home/fanshu/Workplace/ir-sim/ir-sim/usage/17gui_world/gui.yaml', save_ani=False, full=False)
robot = env.robot_list[0]

frame = 0
for _i in range(10000):
    env.step()
    env.render(0.05, show_goal=False)

    if env.mouse_left_pos is not None:
        env.robot.set_goal(env.mouse_left_pos)
    if env.done():
        break

env.end(3)

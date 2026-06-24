"""run_exp with persistent LiDAR + NeuPAN scan points (never cleared).
Run from NeuPAN/example/ directory:
cd /home/fanshu/Workplace/NeuPAN/example
MPLBACKEND=TkAgg conda run -n irsim_test python /home/fanshu/Workplace/ir-simX/tests/visual/run_exp_persist.py -e corridor -d diff"""
import sys; sys.path.insert(0, '/home/fanshu/Workplace/ir-simX')
sys.path.insert(0, '/home/fanshu/Workplace/NeuPAN')
import numpy as np
from neupan import neupan
import irsim, matplotlib; matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import argparse

def main(env_file, planner_file, max_steps=1000, **kw):
    env = irsim.make(env_file, display=True, **kw)
    plt.close('all')
    ax = env._env_plot.ax
    planner = neupan.init_from_yaml(planner_file)

    # store all raw LiDAR endpoints and NeuPAN points
    all_raw_x, all_raw_y = [], []
    all_np_x, all_np_y = [], []
    raw_plot = np_plot = None

    for i in range(max_steps):
        rs = env.get_robot_state()
        scan = env.get_lidar_scan()
        points = planner.scan_to_point(rs, scan)
        action, info = planner(rs, points, None)
        if info.get("stop"): print("NeuPAN stops")
        if info.get("arrive"): print("NeuPAN arrives"); break

        # accumulate raw LiDAR endpoints
        ox, oy, hd = rs[0,0], rs[1,0], rs[2,0]
        ex = ox + scan['ranges'] * np.cos(scan['angles'] + hd)
        ey = oy + scan['ranges'] * np.sin(scan['angles'] + hd)
        hit = scan['ranges'] < scan['range_max'] * 0.9
        all_raw_x.extend(ex[hit]); all_raw_y.extend(ey[hit])
        if raw_plot is None:
            raw_plot = ax.plot(all_raw_x, all_raw_y, 'r.', ms=2, alpha=0.3, zorder=3)[0]
        else:
            raw_plot.set_data(all_raw_x, all_raw_y)

        # accumulate NeuPAN points
        if len(points) > 0:
            all_np_x.extend(points[:,0]); all_np_y.extend(points[:,1])
            if np_plot is None:
                np_plot = ax.plot(all_np_x, all_np_y, 'g.', ms=3, alpha=0.5, zorder=4)[0]
            else:
                np_plot.set_data(all_np_x, all_np_y)

        # normal decorations
        env.draw_points(planner.dune_points, s=25, c="g", refresh=True)
        env.draw_points(planner.nrmp_points, s=13, c="r", refresh=True)
        env.draw_trajectory(planner.opt_trajectory, "r", refresh=True)
        env.draw_trajectory(planner.ref_trajectory, "b", refresh=True)

        env.step(action)
        env.render()
        if env.done(): break
        if i == 0:
            env.draw_trajectory(planner.initial_path, traj_type="-k", show_direction=False)
            env.render()

        if i % 50 == 0:
            print(f'Frame {i}: raw={len(all_raw_x)} neupan={len(all_np_x)}')

    env.end(3)

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("-e","--example", default="corridor")
    p.add_argument("-d","--kinematics", default="diff")
    p.add_argument("-m","--max_steps", type=int, default=1000)
    a = p.parse_args()
    main(a.example+"/"+a.kinematics+"/env.yaml", a.example+"/"+a.kinematics+"/planner.yaml", max_steps=a.max_steps)

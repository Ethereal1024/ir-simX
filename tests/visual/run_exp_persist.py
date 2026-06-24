"""Headless: accumulate raw LiDAR & NeuPAN points, save overlay PNGs every 50 frames.
Run from NeuPAN/example/: conda run -n irsim_test python ..."""
import os, sys; sys.path.insert(0, '/home/fanshu/Workplace/ir-simX'); sys.path.insert(0, '/home/fanshu/Workplace/NeuPAN')
import numpy as np; import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from neupan import neupan; import irsim

def main(env_file, planner_file, max_steps=500):
    base = '/home/fanshu/Workplace/NeuPAN/example'
    env = irsim.make(f'{base}/{env_file}', display=False)
    planner = neupan.init_from_yaml(f'{base}/{planner_file}')
    ax = env._env_plot.ax
    fig = ax.figure

    all_raw_x, all_raw_y = [], []
    all_np_x, all_np_y = [], []
    raw_scat = np_scat = None
    outdir = '/tmp/persist_frames'; os.makedirs(outdir, exist_ok=True)

    for i in range(max_steps):
        rs = env.get_robot_state()
        scan = env.get_lidar_scan()
        points = planner.scan_to_point(rs, scan)
        action, info = planner(rs, points, None)
        if info.get("stop") or info.get("arrive"): break

        ox, oy, hd = rs[0,0], rs[1,0], rs[2,0]
        n = len(scan['ranges'])
        scan_angles = np.linspace(scan['angle_min'], scan['angle_max'], n)
        ex = ox + scan['ranges'] * np.cos(scan_angles + hd)
        ey = oy + scan['ranges'] * np.sin(scan_angles + hd)
        hit = scan['ranges'] < scan['range_max'] * 0.9
        all_raw_x.extend(ex[hit]); all_raw_y.extend(ey[hit])
        if points is not None and points.size > 0:
            p = points.reshape(2, -1)
            all_np_x.extend(p[0]); all_np_y.extend(p[1])

        env.step(action)

        if i % 50 == 0 or i == max_steps - 1:
            print(f'Frame {i}: raw={len(all_raw_x)} neupan={len(all_np_x)}')
            # save frame with persistent overlay
            ax.cla()
            env._env_plot._init_plot(env._world, env._objects)
            if len(all_raw_x) > 0:
                ax.scatter(all_raw_x, all_raw_y, s=2, c='red', alpha=0.3, zorder=5, label=f'raw LiDAR ({len(all_raw_x)})')
            if len(all_np_x) > 0:
                ax.scatter(all_np_x, all_np_y, s=3, c='lime', alpha=0.5, zorder=6, label=f'NeuPAN ({len(all_np_x)})')
            ax.legend(fontsize=7, loc='upper right')
            ax.set_title(f'Frame {i} — raw={len(all_raw_x)}  neupan={len(all_np_x)}')
            fig.savefig(f'{outdir}/frame_{i:04d}.png', dpi=120, bbox_inches='tight')
            print(f'  saved {outdir}/frame_{i:04d}.png')

    # final analysis: are there NeuPAN points far from any raw point?
    from scipy.spatial import KDTree
    if len(all_np_x) > 0 and len(all_raw_x) > 0:
        raw_pts = np.c_[all_raw_x, all_raw_y]
        np_pts = np.c_[all_np_x, all_np_y]
        # for each NeuPAN point, find nearest raw point
        tree = KDTree(raw_pts)
        dists, _ = tree.query(np_pts)
        far = dists > 0.5
        print(f'\nNeuPAN points >0.5m from any raw LiDAR point: {far.sum()}/{len(np_pts)} ({100*far.sum()/len(np_pts):.1f}%)')
        if far.any():
            print(f'  Max distance: {dists.max():.3f}m')
            print(f'  Mean distance: {dists[far].mean():.3f}m')
            # save a figure highlighting the phantoms
            ax.cla(); env._env_plot._init_plot(env._world, env._objects)
            ax.scatter(raw_pts[:,0], raw_pts[:,1], s=2, c='red', alpha=0.3, zorder=5, label=f'raw ({len(raw_pts)})')
            ax.scatter(np_pts[~far,0], np_pts[~far,1], s=3, c='lime', alpha=0.5, zorder=6, label=f'neupan matched ({len(np_pts)-far.sum()})')
            ax.scatter(np_pts[far,0], np_pts[far,1], s=10, c='yellow', alpha=0.8, zorder=7, label=f'PHANTOM ({far.sum()})')
            ax.legend(fontsize=7, loc='upper right')
            ax.set_title(f'PHANTOM: {far.sum()} NeuPAN points >0.5m from raw LiDAR')
            fig.savefig(f'{outdir}/phantoms.png', dpi=120, bbox_inches='tight')
            print(f'  saved {outdir}/phantoms.png')
    else:
        print('Not enough data for comparison')
    print(f'\nAll frames saved to {outdir}/')

if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("-e","--example", default="corridor"); p.add_argument("-d","--kinematics", default="diff")
    p.add_argument("-m","--max_steps", type=int, default=500)
    a = p.parse_args()
    main(a.example+"/"+a.kinematics+"/env.yaml", a.example+"/"+a.kinematics+"/planner.yaml", max_steps=a.max_steps)

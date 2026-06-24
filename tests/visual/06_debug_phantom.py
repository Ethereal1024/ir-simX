"""Check LiDAR on irsim's own plot. Close window or Ctrl+C to exit.
Run: MPLBACKEND=TkAgg conda run -n irsim_test python ..."""
import sys; sys.path.insert(0, '/home/fanshu/Workplace/ir-simX')
import irsim, matplotlib.pyplot as plt, numpy as np

env = irsim.make('/home/fanshu/Workplace/NeuPAN/example/corridor/diff/env.yaml', display=True)
env.step()  # initialize LiDAR
robot = env.robot_list[0]

# use irsim's own plot axes
ax = env._env_plot.ax

step = 0
while True:
    # clear only our overlays (keep irsim's obstacle/robot rendering)
    # remove previous lidar/phantom artists
    for art in ax.lines + ax.collections + ax.patches:
        if hasattr(art, '_is_lidar'):
            art.remove()
    for text in ax.texts:
        text.remove()

    # LiDAR data
    r = robot.lidar.range_data.copy()
    ang = robot.lidar.angle_list
    ox, oy, hd = robot.state[0,0], robot.state[1,0], robot.state[2,0]
    ex = ox + r * np.cos(ang + hd)
    ey = oy + r * np.sin(ang + hd)
    hits = r < 9

    # scan lines (every 5th)
    for i in range(0, len(r), 5):
        c = 'r' if hits[i] else 'k'
        line, = ax.plot([ox, ex[i]], [oy, ey[i]], color=c, lw=0.2, alpha=0.3)
        line._is_lidar = True

    # hit dots
    pts = ax.plot(ex[hits], ey[hits], 'r.', ms=4)[0]
    pts._is_lidar = True

    # phantom check
    if hits.any():
        near_max = r > 8.5
        min_dists = np.full(len(r), 999.0)
        for j in np.where(hits & ~near_max)[0]:
            for obj in env._objects:
                if obj.role == 'obstacle' and obj.shape in ('polygon','rectangle'):
                    v = obj.gf.vertices
                    if v is None or v.shape[1] < 3: continue
                    for k in range(v.shape[1]):
                        x1,y1 = v[0,k], v[1,k]
                        x2,y2 = v[0,(k+1)%v.shape[1]], v[1,(k+1)%v.shape[1]]
                        el = np.hypot(x2-x1,y2-y1)
                        if el < 1e-6: continue
                        t = max(0,min(1,((ex[j]-x1)*(x2-x1)+(ey[j]-y1)*(y2-y1))/(el*el)))
                        d = np.hypot(ex[j]-(x1+t*(x2-x1)), ey[j]-(y1+t*(y2-y1)))
                        if d < min_dists[j]: min_dists[j] = d
        ph = (min_dists > 0.5) & hits & ~near_max
        if ph.any():
            pts2 = ax.plot(ex[ph], ey[ph], 'yD', ms=10)[0]
            pts2._is_lidar = True
            for j in np.where(ph)[0]:
                l2, = ax.plot([ox, ex[j]], [oy, ey[j]], 'y-', lw=1, alpha=0.8)
                l2._is_lidar = True
                print(f'Frame {step} beam {j}: ang={ang[j]:.3f} range={r[j]:.3f} '
                      f'end=({ex[j]:.2f},{ey[j]:.2f}) dist_to_edge={min_dists[j]:.3f}')

    ax.set_title(f'Frame {step} — LiDAR hits: {hits.sum()}/{len(r)}')
    plt.draw(); plt.pause(0.02)

    env.step(); step += 1
    if env.done() or step >= 200: break

print('Done')

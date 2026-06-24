"""Clean debug: raw C++ LiDAR + scan lines + phantom markers.
Close window or Ctrl+C to exit."""
import sys, numpy as np; sys.path.insert(0, '/home/fanshu/Workplace/ir-simX')
import matplotlib; matplotlib.use('TkAgg')
import irsim, matplotlib.pyplot as plt

env = irsim.make('/home/fanshu/Workplace/NeuPAN/example/corridor/diff/env.yaml', display=True)
robot = None
for obj in env._objects:
    if obj.role == 'robot': robot = obj; break

fig, ax = plt.subplots(figsize=(14,7))
plt.ion()
step = 0
while True:
    ax.cla()
    # obstacles
    for obj in env._objects:
        if obj.role == 'obstacle' and obj.shape in ('polygon','rectangle'):
            v = obj.gf.vertices
            if v is not None and v.shape[1] >= 3:
                ax.fill(v[0], v[1], alpha=0.15, fc='blue', ec='blue', lw=1)
        elif obj.role == 'obstacle' and obj.shape == 'circle':
            ax.add_patch(plt.Circle((obj.position[0,0],obj.position[1,0]),
                         getattr(obj.gf,'radius',0.5), alpha=0.15, fc='green', ec='green'))
    # robot
    rb = robot.gf.vertices
    if rb is not None:
        ax.fill(rb[0], rb[1], alpha=0.3, fc='gray', ec='black', lw=1)
    ox, oy, hd = robot.state[0,0], robot.state[1,0], robot.state[2,0]
    ax.plot(ox, oy, 'ro', ms=8)
    ax.plot([ox, ox+2*np.cos(hd)], [oy, oy+2*np.sin(hd)], 'r-', lw=2)

    # LiDAR
    r = robot.lidar.range_data.copy()
    ang = robot.lidar.angle_list
    ex = ox + r * np.cos(ang + hd)
    ey = oy + r * np.sin(ang + hd)
    hits = r < 9
    # scan lines (every 5th)
    for i in range(0, len(r), 5):
        c = 'r' if hits[i] else 'k'
        ax.plot([ox, ex[i]], [oy, ey[i]], color=c, lw=0.2, alpha=0.3)
    ax.plot(ex[hits], ey[hits], 'r.', ms=4, label=f'LiDAR ({hits.sum()}/{len(r)})')

    # phantom check (exclude near-range_max points)
    if hits.any():
        near_max = r > 8.5  # ignore points near range limit
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
            ax.plot(ex[ph], ey[ph], 'yD', ms=10, label=f'PHANTOM ({ph.sum()})')
            for j in np.where(ph)[0]:
                ax.plot([ox, ex[j]], [oy, ey[j]], 'y-', lw=1, alpha=0.8)
                print(f'Frame {step} beam {j}: ang={ang[j]:.3f} range={r[j]:.3f} '
                      f'end=({ex[j]:.2f},{ey[j]:.2f}) dist_to_edge={min_dists[j]:.3f}')

    ax.set_aspect('equal'); ax.grid(True, alpha=0.2)
    ax.set_xlim(-8, 25); ax.set_ylim(10, 30)
    ax.set_title(f'Frame {step}'); ax.legend(fontsize=8, loc='upper right')
    plt.tight_layout(); plt.draw(); plt.pause(0.02)

    env.step(); step += 1
    if env.done() or step >= 200: break

plt.ioff(); plt.show(block=True)
print('Done')

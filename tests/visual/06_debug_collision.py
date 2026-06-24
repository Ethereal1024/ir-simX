"""Debug: show C++ collision volumes vs rendered geometry + LiDAR scan.
Press Enter to step, Ctrl+C to exit."""
import sys, numpy as np; sys.path.insert(0, '/home/fanshu/Workplace/ir-simX')
import irsim, matplotlib.pyplot as plt, matplotlib.patches as mpatches
from matplotlib.collections import PatchCollection

env = irsim.make('/home/fanshu/Workplace/NeuPAN/example/corridor/diff/env.yaml', display=False)

robot = None
for obj in env._objects:
    if obj.role == 'robot':
        robot = obj
        break

# build C++ world to see what obstacles it has
import irsim_core._core as _cc
w = _cc.SimWorld()
w.set_step_time(env._world.step_time)

for obj in env._objects:
    if obj.role == 'obstacle' and obj.static and not obj.unobstructed:
        verts = getattr(obj.gf, 'vertices', None)
        if verts is not None and verts.shape[1] >= 3:
            vlist = [[float(verts[0,i]), float(verts[1,i])] for i in range(verts.shape[1])]
            w.add_obstacle({'type':'polygon','x':0,'y':0,'vertices':vlist})
        elif obj.shape == 'circle':
            r = float(getattr(obj.gf,'radius',0.5))
            w.add_obstacle({'type':'circle','x':float(obj.position[0,0]),
                           'y':float(obj.position[1,0]),'radius':r})

print(f'C++ obstacles: {w.num_obstacles()}')

fig, ax = plt.subplots(figsize=(14,7))
for i in range(200):
    ax.cla()
    # 1. Python rendered obstacles (filled polygons)
    for obj in env._objects:
        if obj.role == 'obstacle' and obj.shape in ('polygon','rectangle'):
            v = obj.gf.vertices
            if v is not None and v.shape[1] >= 3:
                ax.fill(v[0], v[1], alpha=0.15, fc='blue', ec='none')
        elif obj.role == 'obstacle' and obj.shape == 'circle':
            c = plt.Circle((obj.position[0,0],obj.position[1,0]),
                          getattr(obj.gf,'radius',0.5), alpha=0.15, fc='green', ec='none')
            ax.add_patch(c)

    # 2. C++ collision volumes (red outlines)
    n_obs = w.num_obstacles()
    for j in range(n_obs):
        # get obstacle data via raycast trick: send one ray to get debug info
        pass  # C++ doesn't expose obstacle AABB/polygon data directly

    # 3. LiDAR scan
    r = robot.lidar.range_data
    ang = robot.lidar.angle_list
    ox, oy, hd = robot.state[0,0], robot.state[1,0], robot.state[2,0]
    ex = ox + r * np.cos(ang + hd)
    ey = oy + r * np.sin(ang + hd)
    hits = r < 9

    # mark phantoms: hits with endpoint not near any obstacle
    ax.plot(ex[hits], ey[hits], 'r.', ms=4, label=f'scan ({hits.sum()})')
    ax.plot(ox, oy, 'ro', ms=8)

    # 4. Robot body
    rb = robot.gf.vertices
    if rb is not None:
        ax.fill(rb[0], rb[1], alpha=0.3, fc='gray', ec='black', lw=1)

    # 5. Obstacle edges (black outlines for reference)
    for obj in env._objects:
        if obj.role == 'obstacle' and obj.shape in ('polygon','rectangle'):
            v = obj.gf.vertices
            if v is not None and v.shape[1] >= 3:
                x = list(v[0]) + [v[0,0]]
                y = list(v[1]) + [v[1,0]]
                ax.plot(x, y, 'k-', lw=1, alpha=0.5)

    ax.set_aspect('equal'); ax.grid(True, alpha=0.2)
    ax.set_xlim(-8, 20); ax.set_ylim(10, 30)
    ax.set_title(f'Frame {i} — LiDAR hits: {hits.sum()}/{len(r)}')
    plt.tight_layout()
    plt.draw()
    plt.pause(0.01)

    # step
    env.step()
    if env.done():
        break

plt.ioff()
plt.show()

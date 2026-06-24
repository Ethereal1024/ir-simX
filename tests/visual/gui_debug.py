"""gui.py with persistent LiDAR accumulation + logging to file.
Drive around with WASD, exit with ESC, then send me /tmp/lidar_log.jsonl
Usage:  MPLBACKEND=TkAgg conda run -n irsim_test python .../gui_debug.py"""
import os, sys; sys.path.insert(0, '/home/fanshu/Workplace/ir-simX')
import numpy as np, json, matplotlib
import irsim

env = irsim.make('/home/fanshu/Workplace/ir-sim/ir-sim/usage/17gui_world/gui.yaml', save_ani=False, full=False)
robot = env.robot_list[0]

# ── Log file ──
log_path = '/tmp/lidar_log.jsonl'
log = open(log_path, 'w')

# Write obstacle geometry once
obs_data = []
for obj in env._objects:
    if obj.role == 'obstacle' and obj.shape == 'polygon':
        v = obj.gf.vertices
        obs_data.append({
            'id': obj._id,
            'verts': [[float(v[0,i]), float(v[1,i])] for i in range(v.shape[1])]
        })
log.write(json.dumps({'type': 'obstacles', 'data': obs_data}) + '\n')

# ── Persistent scatter overlay ──
ax = env._env_plot.ax
all_x, all_y = [], []
scat = None

frame = 0
for _i in range(10000):
    env.step()
    env.render(0.05, show_goal=False)

    r = robot.lidar.range_data
    ang = robot.lidar.angle_list
    ox, oy, hd = robot.state[0,0], robot.state[1,0], robot.state[2,0]
    ex = ox + r * np.cos(ang + hd)
    ey = oy + r * np.sin(ang + hd)
    hit = r < robot.lidar.range_max * 0.9

    # Accumulate for display
    all_x.extend(ex[hit].tolist())
    all_y.extend(ey[hit].tolist())
    if scat is None:
        scat = ax.scatter(all_x, all_y, s=2, c='red', alpha=0.3, zorder=5)
    else:
        scat.set_offsets(np.c_[all_x, all_y])

    # Log this frame's hits
    frame_data = {
        'type': 'frame',
        'frame': frame,
        'robot': [float(ox), float(oy), float(hd)],
        'hits': [[float(ex[i]), float(ey[i]), float(r[i])] for i in np.where(hit)[0]]
    }
    log.write(json.dumps(frame_data) + '\n')
    frame += 1

    if env.mouse_left_pos is not None:
        env.robot.set_goal(env.mouse_left_pos)
    if env.done():
        break

log.close()
env.end(3)
print(f'\nLogged {frame} frames to {log_path}')
print(f'Hit points total: {len(all_x)}')

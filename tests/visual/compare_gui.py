"""Compare C++ vs Python LiDAR on gui.yaml scene, frame by frame.
Saves PNGs to /tmp/gui_compare/ showing differences."""
import os, sys; sys.path.insert(0, '/home/fanshu/Workplace/ir-simX')
import numpy as np; import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt; import irsim

env = irsim.make('/home/fanshu/Workplace/ir-sim/ir-sim/usage/17gui_world/gui.yaml', display=False, log_level='ERROR')
env._world_param.control_mode = 'auto'  # auto mode for headless

robot = env.robot_list[0]
ax = env._env_plot.ax; fig = ax.figure

outdir = '/tmp/gui_compare'; os.makedirs(outdir, exist_ok=True)

for frame in range(5):
    env.step()
    rs = robot.state; r = robot.lidar.range_data.copy()
    ang = robot.lidar.angle_list
    ox, oy, hd = rs[0,0], rs[1,0], rs[2,0]
    
    # ====== C++ path ======
    import irsim_core as cc
    from irsim.world.sensors.lidar2d import Lidar2D
    ep = robot.lidar._env_param
    obs_dicts = []
    for obj in ep.objects:
        if obj._id == robot.lidar.obj_id or not obj._geometry_valid or obj.unobstructed:
            continue
        d = Lidar2D._obj_to_c_dict(obj)
        if d: obs_dicts.append(d)
    cpp_r = cc.lidar_raycast(ox, oy, hd, ang.astype(np.float32), 20.0, obs_dicts)
    
    # ====== Python Shapely path ======
    # Force Shapely by copying lidar and running its step with a fallback
    import shapely
    from irsim.util.util import geometry_transform
    lidar_copy = robot.lidar
    # Rebuild geometry
    new_geo = geometry_transform(lidar_copy._original_geometry, lidar_copy._state)
    shapely.prepare(new_geo)
    new_geo2, indices = lidar_copy.laser_geometry_process(new_geo)
    if len(indices) == 0:
        lidar_copy._geometry = lidar_copy._ensure_multi_linestring(new_geo2)
        lidar_copy.calculate_range()
    else:
        origin_pt = shapely.points(lidar_copy.lidar_origin[0,0], lidar_copy.lidar_origin[1,0])
        parts = shapely.get_parts(new_geo2)
        lidar_copy._geometry = shapely.MultiLineString(list(parts[shapely.intersects(parts, origin_pt)]))
        lidar_copy.calculate_range_vel(indices)
    py_r = lidar_copy.range_data.copy()
    
    # ====== Compare ======
    diff = np.abs(cpp_r - py_r)
    bad = diff > 0.05
    hit_diff = (cpp_r < 18) != (py_r < 18)
    
    print(f'Frame {frame}: range_diff>0.05={bad.sum()}/{len(cpp_r)}  hit_mismatch={hit_diff.sum()}')
    
    if bad.any():
        for i in np.where(bad)[0][:10]:
            print(f'  beam {i}: C++={cpp_r[i]:.3f} Py={py_r[i]:.3f} diff={diff[i]:.3f}')
    
    # ====== Save frame ======
    ax.cla()
    env._env_plot._init_plot(env._world, env._objects)
    
    # Robot
    ax.plot(ox, oy, 'ko', ms=6)
    ox2 = ox + 2*np.cos(hd); oy2 = oy + 2*np.sin(hd)
    ax.plot([ox, ox2], [oy, oy2], 'k-', lw=2)
    
    # C++ hits (red)
    cpp_ex = ox + cpp_r * np.cos(ang + hd)
    cpp_ey = oy + cpp_r * np.sin(ang + hd)
    cpp_hit = cpp_r < 18
    ax.plot(cpp_ex[cpp_hit], cpp_ey[cpp_hit], 'r.', ms=3, alpha=0.7, label=f'C++ ({cpp_hit.sum()})')
    
    # Python hits (blue) - slightly offset to see overlap
    py_ex = ox + py_r * np.cos(ang + hd)
    py_ey = oy + py_r * np.sin(ang + hd)
    py_hit = py_r < 18
    ax.plot(py_ex[py_hit], py_ey[py_hit], 'b.', ms=2, alpha=0.5, label=f'Py ({py_hit.sum()})')
    
    # Mismatches (yellow)
    if hit_diff.any():
        ax.plot(cpp_ex[hit_diff], cpp_ey[hit_diff], 'yo', ms=6, label=f'MISMATCH ({hit_diff.sum()})')
    
    ax.set_aspect('equal'); ax.legend(fontsize=7)
    ax.set_xlim(ox-15, ox+15); ax.set_ylim(oy-15, oy+15)
    ax.set_title(f'Frame {frame} — C++ diff >0.05m: {bad.sum()}')
    fig.savefig(f'{outdir}/frame{frame}.png', dpi=120, bbox_inches='tight')
    print(f'  saved {outdir}/frame{frame}.png')

print('\nDone. Check', outdir)

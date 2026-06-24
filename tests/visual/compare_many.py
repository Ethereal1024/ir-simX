"""Many random polygons: compare C++ vs Python LiDAR.
Generates 30 random polygons scattered around robot."""
import os, sys; sys.path.insert(0, '/home/fanshu/Workplace/ir-simX')
import numpy as np; import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
import shapely, shapely.geometry
from shapely import affinity
from irsim.lib.algorithm.generation import random_generate_polygon
from irsim.util.random import set_seed
import irsim_core as cc

set_seed(42)

# Generate 30 random polygons scattered in a ring around origin
polygons = []
for i in range(30):
    angle = i * 2 * np.pi / 30
    radius = 8 + np.random.uniform(-3, 3)  # 5-11m from origin
    cx, cy = radius * np.cos(angle), radius * np.sin(angle)
    
    verts = random_generate_polygon(number=1,
        center_range=[0, 0, 0, 0],  # generated at origin, we translate
        avg_radius_range=[0.5, 1.5],
        irregularity_range=[0.1, 0.8],
        spikeyness_range=[0.1, 0.7],
        num_vertices_range=[4, 10])
    
    # Translate and rotate
    shapely_poly = shapely.geometry.Polygon(verts)
    shapely_poly = affinity.translate(shapely_poly, cx, cy)
    shapely_poly = affinity.rotate(shapely_poly, np.random.uniform(0, 2*np.pi), origin='centroid')
    
    world_verts = np.array(shapely_poly.exterior.coords)[:-1]  # remove closing point
    polygons.append(world_verts)

# Build obstacle dicts for C++
obs_dicts = []
for v in polygons:
    obs_dicts.append({
        'type': 'polygon', 'x': 0, 'y': 0,
        'vertices': [[float(v[i,0]), float(v[i,1])] for i in range(len(v))]
    })

# Full 360° scan
angles = np.linspace(-np.pi, np.pi, 200, dtype=np.float32)
robot = (0, 0, 0.0)

# C++
cpp_r = cc.lidar_raycast(*robot, angles, 25.0, obs_dicts)

# Python Shapely
py_r = np.full_like(cpp_r, 25.0)
origin_pt = shapely.Point(0, 0)
ray_max = 25.0

for bi, ang in enumerate(angles):
    dir_vec = np.array([np.cos(ang), np.sin(ang)])
    min_t = ray_max
    for v in polygons:
        poly = shapely.geometry.Polygon(v)
        # Ray-polygon intersection via Shapely
        ray_end = shapely.Point(ray_max * np.cos(ang), ray_max * np.sin(ang))
        ray_line = shapely.LineString([(0, 0), (ray_max * np.cos(ang), ray_max * np.sin(ang))])
        inter = poly.intersection(ray_line)
        if inter.is_empty: continue
        if inter.geom_type == 'Point':
            d = np.hypot(inter.x, inter.y)
            if 0 < d < min_t: min_t = d
        elif inter.geom_type == 'MultiPoint':
            for pt in inter.geoms:
                d = np.hypot(pt.x, pt.y)
                if 0 < d < min_t: min_t = d
        elif inter.geom_type == 'LineString':
            # ray along edge - intersection starts at first point
            pt = inter.coords[0]
            d = np.hypot(pt[0], pt[1])
            if 0 < d < min_t: min_t = d
    py_r[bi] = min_t

# Compare
diff = np.abs(cpp_r - py_r)
bad = diff > 0.05
hit_diff = (cpp_r < 24) != (py_r < 24)

print(f'=== 30 random polygons, 200 beams ===')
print(f'Range diff >0.05m: {bad.sum()}/{len(cpp_r)} ({100*bad.sum()/len(cpp_r):.1f}%)')
print(f'Hit/miss mismatch: {hit_diff.sum()}/{len(cpp_r)}')
if bad.any():
    print(f'Max diff: {diff.max():.3f}m')
    worst = np.argmax(diff)
    print(f'Worst beam {worst}: C++={cpp_r[worst]:.3f} Py={py_r[worst]:.3f}')

# Plot
fig, axes = plt.subplots(1, 3, figsize=(18, 7))
# Left: scene + hits
ax = axes[0]
for v in polygons:
    ax.fill(v[:,0], v[:,1], alpha=0.2, fc='blue', ec='blue', lw=0.5)
ax.plot(0, 0, 'ro', ms=8)
cpp_ex = cpp_r * np.cos(angles)
cpp_ey = cpp_r * np.sin(angles)
ax.plot(cpp_ex, cpp_ey, 'r.', ms=2, alpha=0.5)
ax.set_aspect('equal'); ax.set_xlim(-15, 15); ax.set_ylim(-15, 15)
ax.set_title('Scene + C++ LiDAR')

# Middle: C++ hits in red, Python in blue
ax = axes[1]
ax.plot(0, 0, 'ko', ms=6)
cpp_hit = cpp_r < 24
py_hit = py_r < 24
ax.plot(cpp_ex[cpp_hit], cpp_ey[cpp_hit], 'r.', ms=3, alpha=0.6, label=f'C++ ({cpp_hit.sum()})')
py_ex = py_r * np.cos(angles)
py_ey = py_r * np.sin(angles)
ax.plot(py_ex[py_hit], py_ey[py_hit], 'b.', ms=2, alpha=0.4, label=f'Py ({py_hit.sum()})')
if hit_diff.any():
    ax.plot(cpp_ex[hit_diff], cpp_ey[hit_diff], 'yo', ms=6, label=f'MISMATCH ({hit_diff.sum()})')
ax.set_aspect('equal'); ax.set_xlim(-15, 15); ax.set_ylim(-15, 15)
ax.legend(fontsize=7); ax.set_title('C++ (red) vs Python (blue)')

# Right: zoomed
ax = axes[2]
ax.plot(0, 0, 'ko', ms=6)
for v in polygons:
    ax.fill(v[:,0], v[:,1], alpha=0.15, fc='blue', ec='blue', lw=0.5)
ax.plot(cpp_ex[cpp_hit], cpp_ey[cpp_hit], 'r.', ms=3, alpha=0.6)
ax.plot(py_ex[py_hit], py_ey[py_hit], 'b.', ms=2, alpha=0.4)
if hit_diff.any():
    ax.plot(cpp_ex[hit_diff], cpp_ey[hit_diff], 'yo', ms=6)
ax.set_aspect('equal'); ax.set_xlim(-8, 8); ax.set_ylim(-8, 8)
ax.set_title('Zoomed')

fig.tight_layout()
fig.savefig('/tmp/many_polygons.png', dpi=120)
print(f'\nSaved /tmp/many_polygons.png')

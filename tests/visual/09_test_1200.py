"""测试 1200 束激光雷达在多边形障碍物下的表现。
直接运行：conda run -n irsim_test python 09_test_1200.py"""
import sys; sys.path.insert(0, '/home/fanshu/Workplace/ir-simX')
import numpy as np, time, os; os.environ['MPLBACKEND'] = 'TkAgg'
import irsim_core as cc, matplotlib.pyplot as plt

# ── 生成 10 个随机多边形 ──
from irsim.lib.algorithm.generation import random_generate_polygon
from irsim.util.random import set_seed
set_seed(42)
polygons = []
for i in range(10):
    a = i * 2 * np.pi / 10
    r = 6 + np.random.uniform(-2, 2)
    cx, cy = r * np.cos(a), r * np.sin(a)
    v = random_generate_polygon(number=1,
        avg_radius_range=[0.5, 1.5], irregularity_range=[0.3, 0.7],
        spikeyness_range=[0.2, 0.6], num_vertices_range=[5, 9])
    # 平移到 (cx, cy) 并旋转
    import shapely.geometry, shapely.affinity
    sp = shapely.affinity.translate(shapely.geometry.Polygon(v), cx, cy)
    sp = shapely.affinity.rotate(sp, np.random.uniform(0, 2*np.pi))
    wv = np.array(sp.exterior.coords)[:-1]
    polygons.append(wv)

# ── 构造 C++ 障碍物 ──
obs_dicts = []
for v in polygons:
    obs_dicts.append({'type': 'polygon', 'x': 0, 'y': 0,
                      'vertices': v.tolist()})

# ── 1200 束 360° 扫描 ──
angles = np.linspace(-np.pi, np.pi, 1200, dtype=np.float32)

# 预热
for _ in range(10):
    cc.lidar_raycast(0, 0, 0, angles, 15.0, obs_dicts)

# 基准测试
N = 200
t0 = time.perf_counter()
for _ in range(N):
    cc.lidar_raycast(0, 0, 0, angles, 15.0, obs_dicts)
t = (time.perf_counter() - t0) / N

# 结果
r = cc.lidar_raycast(0, 0, 0, angles, 15.0, obs_dicts)
hits = (r < 14).sum()
ex = r * np.cos(angles); ey = r * np.sin(angles)

print(f'12 个多边形障碍物')
print(f'激光束数: {len(angles)}')
print(f'命中: {hits}/{len(r)}')
print(f'平均耗时: {t*1e6:.1f} us')
print(f'帧率: {1/t:,.0f} FPS')
print()

# ── 可视化 ──
fig, ax = plt.subplots(figsize=(8, 8))
for v in polygons:
    xs = list(v[:, 0]) + [v[0, 0]]
    ys = list(v[:, 1]) + [v[0, 1]]
    ax.fill(xs, ys, alpha=0.2, fc='blue', ec='blue', lw=1)
    ax.plot(xs, ys, 'b-', lw=1)
ax.plot(0, 0, 'ro', ms=8, label='机器人')
ax.plot(ex[hits], ey[hits], 'r.', ms=2, alpha=0.5, label=f'命中 ({hits})')
ax.plot(ex[~hits], ey[~hits], 'k.', ms=1, alpha=0.2, label=f'未命中 ({len(r)-hits})')
ax.set_aspect('equal')
ax.set_xlim(-10, 10); ax.set_ylim(-10, 10)
ax.grid(True, alpha=0.3)
ax.legend(fontsize=8)
ax.set_title(f'1200 束激光雷达 — {hits}/{len(r)} 命中  {t*1e6:.0f} µs')
plt.tight_layout()
plt.show(block=True)

"""Analyze lidar_log.jsonl — check each hit point against obstacle geometry.
Usage: conda run -n irsim_test python analyze_log.py /tmp/lidar_log.jsonl"""
import sys, json, numpy as np

log_path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/lidar_log.jsonl'
range_max = 20.0

with open(log_path) as f:
    lines = [json.loads(l) for l in f if l.strip()]

# Parse obstacles
obs_line = [l for l in lines if l['type'] == 'obstacles']
if not obs_line:
    print('No obstacle data found')
    sys.exit(1)
obstacles = obs_line[0]['data']

# Build shapely polygons for distance checks
import shapely.geometry
obs_polys = []
for o in obstacles:
    verts = np.array(o['verts'])
    poly = shapely.geometry.Polygon(verts)
    obs_polys.append(poly)

# Parse frames
frames = [l for l in lines if l['type'] == 'frame']
print(f'Loaded {len(frames)} frames, {len(obstacles)} obstacles')

total_hits = 0
phantoms = 0
phantom_detail = []

for f in frames:
    ox, oy, hd = f['robot']
    for hx, hy, hr in f['hits']:
        total_hits += 1
        pt = shapely.geometry.Point(hx, hy)
        # Distance to nearest obstacle edge
        min_dist = min(p.distance(pt) for p in obs_polys)
        if min_dist > 0.3:
            phantoms += 1
            phantom_detail.append((f['frame'], hx, hy, hr, min_dist))

print(f'Total hit points: {total_hits}')
print(f'Phantoms (>0.3m from any obstacle): {phantoms} ({100*phantoms/total_hits:.1f}%)' if total_hits else '')
if phantom_detail:
    print(f'\nFirst 20 phantoms:')
    for fr, hx, hy, hr, d in phantom_detail[:20]:
        print(f'  Frame {fr}: ({hx:.2f},{hy:.2f}) range={hr:.2f} dist_to_obs={d:.3f}')

# Also check: are there obstacle edges with NO hits?
print(f'\nObstacle edges with few or no hits within 20m of robot path:')
robot_path = np.array([f['robot'][:2] for f in frames])
for oi, poly in enumerate(obs_polys):
    coords = np.array(poly.exterior.coords)
    for ei in range(len(coords)-1):
        x1, y1 = coords[ei]
        x2, y2 = coords[ei+1]
        # Midpoint of edge
        mx, my = (x1+x2)/2, (y1+y2)/2
        # Was robot ever within 25m?
        dists = np.hypot(robot_path[:,0]-mx, robot_path[:,1]-my)
        if dists.min() < 25:
            # How many hit points are within 0.5m of this edge?
            edge_pts = shapely.geometry.LineString([(x1,y1),(x2,y2)])
            near_count = 0
            for f in frames:
                ox, oy = f['robot'][:2]
                if np.hypot(ox-mx, oy-my) < 25:
                    for hx, hy, hr in f['hits']:
                        d = edge_pts.distance(shapely.geometry.Point(hx, hy))
                        if d < 0.5:
                            near_count += 1
            if near_count < 3:
                print(f'  Obstacle {oi} edge {ei}: ({x1:.1f},{y1:.1f})→({x2:.1f},{y2:.1f}) only {near_count} nearby hits')

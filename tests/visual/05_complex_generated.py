"""Complex polygons using IR-SIM's random generator + handcrafted shapes."""
import sys, numpy as np; sys.path.insert(0, '/home/fanshu/Workplace/ir-simX')
import irsim_core as cc, matplotlib.pyplot as plt
from irsim.lib.algorithm.generation import random_generate_polygon
from irsim.util.random import set_seed

rb = np.array([[-0.8,-1],[-0.8,1],[0.8,1],[0.8,-1]])

def clean(verts):
    # remove duplicate consecutive vertices
    out = [verts[0]]
    for v in verts[1:]:
        if np.linalg.norm(np.array(v)-np.array(out[-1])) > 0.001:
            out.append(v)
    if np.linalg.norm(np.array(out[0])-np.array(out[-1])) < 0.001:
        out = out[:-1]
    return np.array(out) if len(out) >= 3 else verts

def run(verts_list, colors, title):
    angles = np.linspace(-1.3, 1.3, 120, dtype=np.float32)
    obs_list = [{'type':'polygon','x':0,'y':0,'vertices':v.tolist() if hasattr(v,'tolist') else v} for v in verts_list]
    ranges = cc.lidar_raycast(0,0,0, angles, 7.0, obs_list)
    ex = ranges*np.cos(angles); ey = ranges*np.sin(angles)
    hits = ranges < 6
    fig, ax = plt.subplots(figsize=(8,7))
    for v,c in zip(verts_list, colors):
        a = np.array(v)
        ax.fill(a[:,0], a[:,1], alpha=0.2, fc=c, ec=c, lw=1.5)
    ax.fill(rb[:,0],rb[:,1], alpha=0.35, fc='gray', ec='black', lw=1, label='robot')
    ax.plot(0,0,'ro',ms=6)
    for i in range(len(angles)):
        ax.plot([0,ex[i]],[0,ey[i]], color='r' if hits[i] else 'k', lw=0.3, alpha=0.3)
    ax.plot(ex[hits],ey[hits],'r.',ms=3,label=f'hits ({hits.sum()})')
    ax.plot(ex[~hits],ey[~hits],'kx',ms=2)
    ax.set_aspect('equal'); ax.grid(True,alpha=0.3); ax.legend(fontsize=8)
    ax.set_xlim(-2,6); ax.set_ylim(-3,3); ax.set_title(f'{title} — {hits.sum()}/{len(ranges)} hits')
    plt.tight_layout(); plt.show(block=True); input()

# set_seed(42)

# ── 1. IR-SIM random concave polygon (high spikeyness) ──
verts = clean(random_generate_polygon(number=1, center_range=[2,0,3,0],
    avg_radius_range=[1.2,1.5], irregularity_range=[0.3,0.8],
    spikeyness_range=[0.3,0.8], num_vertices_range=[6,10]))
print(f'Random concave ({len(verts)} verts):', verts)
run([verts], ['dodgerblue'], 'IR-SIM random concave')

# ── 2. IR-SIM regular-ish polygon (low irregularity) ──
verts = clean(random_generate_polygon(number=1, center_range=[2.5,0,3,0],
    avg_radius_range=[1.0,1.2], irregularity_range=[0,0.2],
    spikeyness_range=[0,0.3], num_vertices_range=[5,7]))
print(f'Near-regular ({len(verts)} verts):', verts)
run([verts], ['green'], 'IR-SIM near-regular')

# ── 3. Two separate polygons, robot between them ──
left = clean(random_generate_polygon(number=1, center_range=[0.8,-0.2,1.2,0.2],
    avg_radius_range=[0.4,0.6], irregularity_range=[0.3,0.6],
    spikeyness_range=[0.2,0.5], num_vertices_range=[4,6]))
right = clean(random_generate_polygon(number=1, center_range=[2.5,0,3,0],
    avg_radius_range=[0.6,1.0], irregularity_range=[0.2,0.5],
    spikeyness_range=[0.2,0.5], num_vertices_range=[5,8]))
run([left, right], ['orange', 'purple'], 'Two random polygons')

# ── 4. Many-vertex polygon (12 verts) ──
verts = clean(random_generate_polygon(number=1, center_range=[2.5,-0.2,3,0.2],
    avg_radius_range=[1.0,1.3], irregularity_range=[0.2,0.5],
    spikeyness_range=[0.3,0.7], num_vertices_range=[12,14]))
run([verts], ['darkred'], '12-vertex random polygon')

# ── 5. Mixed: polygon + circle ──
verts = clean(random_generate_polygon(number=1, center_range=[2.5,0,3,0],
    avg_radius_range=[0.8,1.2], irregularity_range=[0.3,0.7],
    spikeyness_range=[0.2,0.6], num_vertices_range=[5,8]))
circle = {'type':'circle','x':4.5,'y':-0.5,'radius':0.7}
obs_list = [{'type':'polygon','x':0,'y':0,'vertices':verts.tolist()}, circle]
angles = np.linspace(-1.3,1.3,120,dtype=np.float32)
ranges = cc.lidar_raycast(0,0,0,angles,7.0,obs_list)
ex=ranges*np.cos(angles); ey=ranges*np.sin(angles); hits=ranges<6
fig,ax=plt.subplots(figsize=(8,7))
a=np.array(verts); ax.fill(a[:,0],a[:,1],alpha=0.2,fc='blue',ec='blue',lw=1.5)
ax.add_patch(plt.Circle((circle['x'],circle['y']),circle['radius'],alpha=0.2,fc='green',ec='green',lw=1.5))
ax.fill(rb[:,0],rb[:,1],alpha=0.35,fc='gray',ec='black',lw=1,label='robot')
ax.plot(0,0,'ro',ms=6)
for i in range(len(angles)):
    ax.plot([0,ex[i]],[0,ey[i]],color='r' if hits[i] else 'k',lw=0.3,alpha=0.3)
ax.plot(ex[hits],ey[hits],'r.',ms=3,label=f'hits ({hits.sum()})')
ax.plot(ex[~hits],ey[~hits],'kx',ms=2)
ax.set_aspect('equal');ax.grid(True,alpha=0.3);ax.legend(fontsize=8)
ax.set_xlim(-2,6);ax.set_ylim(-3,3)
ax.set_title(f'Random polygon + circle — {hits.sum()}/{len(ranges)} hits')
plt.tight_layout();plt.show(block=True)

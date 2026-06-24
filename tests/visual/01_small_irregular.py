"""Small irregular obstacles ~1-2m, robot at origin with body shown.
Press Enter after each plot to advance."""
import sys, numpy as np; sys.path.insert(0, '/home/fanshu/Workplace/ir-simX')
import irsim_core as cc, matplotlib.pyplot as plt

rb = np.array([[-0.8,-1],[-0.8,1],[0.8,1],[0.8,-1]])
scenes = [
    ("Tiny triangle 1.2m", 'orange', [[2,0],[3,0.8],[1.8,0.6]]),
    ("Irregular ~1.5m", 'blue', [[1.5,-0.3],[2.8,0.5],[2.2,1.3],[1.0,1.0],[1.8,0.2]]),
    ("Thin L ~1.5m concave", 'purple', [[0.5,-0.5],[2,0.5],[2,0.9],[0.9,0.9],[0.9,1.5],[0.5,1.5]]),
    ("C-shape ~2m concave", 'brown', [[1.5,-0.8],[3,-0.8],[3,1.2],[1.5,1.2],[1.5,0.8],[2.6,0.8],[2.6,-0.4],[1.5,-0.4]]),
    ("Narrow corridor ~1.5m", 'darkgreen', [[2.5,-0.8],[2.5,-0.1],[2,-0.1],[2,0.1],[2.5,0.1],[2.5,0.8],[3.5,0.8],[3.5,-0.8]]),
]
for name, color, verts in scenes:
    obs = {'type':'polygon','x':0,'y':0,'vertices':verts}
    angles = np.linspace(-1.2, 1.2, 80, dtype=np.float32)
    ranges = cc.lidar_raycast(0,0,0, angles, 6.0, [obs])
    ex = ranges*np.cos(angles); ey = ranges*np.sin(angles)
    hits = ranges < 5
    fig, ax = plt.subplots(figsize=(7,7))
    v = np.array(verts); ax.fill(v[:,0],v[:,1], alpha=0.25, fc=color, ec=color, lw=2)
    ax.fill(rb[:,0],rb[:,1], alpha=0.3, fc='gray', ec='black', lw=1, label='robot')
    ax.plot(0,0,'ro',ms=6)
    for i in range(len(angles)):
        ax.plot([0,ex[i]],[0,ey[i]], color='r' if hits[i] else 'k', lw=0.3, alpha=0.3)
    ax.plot(ex[hits],ey[hits],'r.',ms=4,label=f'hits ({hits.sum()})')
    ax.plot(ex[~hits],ey[~hits],'kx',ms=3)
    ax.set_aspect('equal'); ax.grid(True,alpha=0.3); ax.legend(fontsize=8)
    ax.set_xlim(-2.5,5); ax.set_ylim(-3,3)
    ax.set_title(f'{name} — {hits.sum()}/{len(ranges)} hits',fontsize=10)
    plt.tight_layout(); plt.show(block=True); input()

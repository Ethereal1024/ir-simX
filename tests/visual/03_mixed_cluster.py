"""Mixed small polygon/circle cluster."""
import sys, numpy as np; sys.path.insert(0, '/home/fanshu/Workplace/ir-simX')
import irsim_core as cc, matplotlib.pyplot as plt

rb = np.array([[-0.8,-1],[-0.8,1],[0.8,1],[0.8,-1]])
obs = [
    {'type':'polygon','x':0,'y':0,'vertices':[[2.5,-0.3],[3.2,0.2],[2.8,0.8],[2.0,0.5]]},
    {'type':'circle','x':4.0,'y':-0.5,'radius':0.6},
    {'type':'polygon','x':0,'y':0,'vertices':[[4.5,0.8],[5.2,0.6],[5.5,1.4],[4.8,1.6]]},
    {'type':'polygon','x':0,'y':0,'vertices':[[3.0,-1.2],[3.8,-1.5],[4.2,-0.8]]},
]
angles = np.linspace(-1.3, 1.3, 100, dtype=np.float32)
ranges = cc.lidar_raycast(0,0,0, angles, 6.0, obs)
ex = ranges*np.cos(angles); ey = ranges*np.sin(angles)
hits = ranges < 5

fig, ax = plt.subplots(figsize=(8,7))
colors = ['dodgerblue','green','orange','red']
for i, o in enumerate(obs):
    c = colors[i%4]
    if o['type']=='polygon':
        v=np.array(o['vertices']); ax.fill(v[:,0],v[:,1],alpha=0.25,fc=c,ec=c,lw=2)
    else:
        ax.add_patch(plt.Circle((o['x'],o['y']),o['radius'],alpha=0.25,fc=c,ec=c,lw=2))
ax.fill(rb[:,0],rb[:,1],alpha=0.3,fc='gray',ec='black',lw=1,label='robot')
ax.plot(0,0,'ro',ms=6)
for i in range(len(angles)):
    c = 'r' if hits[i] else 'k'
    ax.plot([0,ex[i]],[0,ey[i]], color=c, lw=0.3, alpha=0.3)
ax.plot(ex[hits],ey[hits],'r.',ms=4,label=f'hits ({hits.sum()})')
ax.plot(ex[~hits],ey[~hits],'kx',ms=3)
ax.set_aspect('equal'); ax.grid(True,alpha=0.3); ax.legend(fontsize=8)
ax.set_xlim(-2,7); ax.set_ylim(-3,3)
ax.set_title(f'Mixed cluster — {hits.sum()}/{len(ranges)} hits')
plt.tight_layout(); plt.show(block=True)

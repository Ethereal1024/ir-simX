"""Benchmark: time small obstacles at various beam counts."""
import sys, time, numpy as np; sys.path.insert(0, '/home/fanshu/Workplace/ir-simX')
import irsim_core as cc

irregular = {'type':'polygon','x':0,'y':0,'vertices':[[1.5,-0.3],[2.8,0.5],[2.2,1.3],[1.0,1.0],[1.8,0.2]]}
L = {'type':'polygon','x':0,'y':0,'vertices':[[0,0],[1.5,0],[1.5,0.4],[0.4,0.4],[0.4,1.5],[0,1.5]]}
circle = {'type':'circle','x':3,'y':0,'radius':0.8}
rect = {'type':'polygon','x':0,'y':0,'vertices':[[2.5,-0.6],[3.5,-0.6],[3.5,0.6],[2.5,0.6]]}

print(f'{"name":14s} {"n":>5s}  {"time":>8s}  {"FPS":>8s}')
for label, obs_list in [('irregular 5-gon',[irregular]),('concave L',[L]),
                         ('circle r=0.8',[circle]),('rect 1x1.2',[rect]),
                         ('all 4',[irregular,L,circle,rect])]:
    for n in [60, 200, 1200]:
        angles = np.linspace(-1.2,1.2,n,dtype=np.float32)
        for _ in range(10): cc.lidar_raycast(0,0,0,angles,6.0,obs_list)
        t0=time.perf_counter(); N=200 if n<=200 else 50
        for _ in range(N): cc.lidar_raycast(0,0,0,angles,6.0,obs_list)
        t=(time.perf_counter()-t0)/N*1e6
        print(f'{label:14s}  n={n:4d}  {t:8.1f} us  {1e6/t:8.0f} FPS')

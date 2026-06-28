<div align="center">

# IR-SIM-X: C++ Accelerated IR-SIM

*A C++-accelerated fork of [IR-SIM](https://github.com/hanruihua/ir-sim) — faster LiDAR, collision detection, and kinematics via AVX2 SIMD and pybind11*

<a href="https://github.com/Ethereal1024/ir-simX"><img src="https://img.shields.io/badge/python-3.10%20%7C%203.11%20%7C%203.12%20%7C%203.13%20%7C%203.14-blue?style=for-the-badge" alt="Python Version"></a>
<a href="LICENSE"><img src="https://img.shields.io/badge/License-MIT-blue?style=for-the-badge" alt="License"></a>

</div>

## Overview

**irsim-x** is a C++-accelerated fork of [IR-SIM](https://github.com/hanruihua/ir-sim), an open-source, Python-based lightweight robot simulator for navigation, control, and reinforcement learning. The core simulation loop (LiDAR raycasting, collision detection, kinematics stepping, and A\* planning) has been ported to C++ via pybind11, with **AVX2 SIMD** acceleration for LiDAR and batch multi-environment simulation.

The original Python-only paths have been removed — **C++ is the only execution path**. Compared to the original IR-SIM, IR-SIM-X achieves **5–50× faster LiDAR raycasting**, **8× faster end-to-end step** on sparse environments, and supports **batch SIMD simulation** across up to 1024 parallel environments.

## Key Features

- **C++ LiDAR** — AVX2 SIMD circle/rect raycast + spatial hash grid for polygon/linestring, 5–50× faster
- **C++ Collision Detection** — SAT (Separating Axis Theorem) with ear-clip triangulation for concave polygons
- **C++ Kinematics** — diff, omni, omni_angular, ackermann steering
- **C++ SimWorld** — unified step, collision, dynamic obstacle (circle/rect/polygon/linestring) management
- **Batch SIMD** — up to 1024 environments with cross-environment SIMD LiDAR and kinematics
- **FMCW LiDAR** — C++ accelerated per-beam radial velocity computation
- **YAML-driven** — same simple configuration as IR-SIM, no code changes needed for existing scenarios

## Installation

> **Requires Python >= 3.10 and a C++17 compiler** (for building the extension)

```bash
pip install irsim-x

# With keyboard control and extras
pip install irsim-x[all]
```

### From source

```bash
git clone https://github.com/Ethereal1024/ir-simX.git
cd ir-simX
pip install -e .
```

## Quick Start

A minimal example — a differential-drive robot navigates toward a goal:

```python
import irsim

env = irsim.make('robot_world.yaml')

for i in range(300):
    env.step()
    env.render()
    if env.done():
        break

env.end()
```

YAML Configuration (`robot_world.yaml`):

```yaml
world:
  height: 10
  width: 10
  step_time: 0.1

robot:
  kinematics: {name: 'diff'}
  shape: {name: 'circle', radius: 0.2}
  state: [1, 1, 0]
  goal: [9, 9, 0]
  behavior: {name: 'dash'}
```

For more examples, see the [usage directory](https://github.com/Ethereal1024/ir-simX/tree/main/usage).

## Support

| **Category**     | **Features**                                                                                                                                                                            |
| ---------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Kinematics**   | Differential Drive · Omnidirectional · Omnidirectional with Angular · Ackermann Steering                                                                                                |
| **Sensors**      | 2D LiDAR · 2D FMCW LiDAR · FOV Detector                                                                                                                                                 |
| **Geometries**   | Circle · Rectangle · Polygon · LineString · Binary Grid Map                                                                                                                             |
| **Behaviors**    | dash · RVO (Reciprocal Velocity Obstacle) · ORCA (Optimal Reciprocal Collision Avoidance) · SFM (Social Force Model)                                                                    |

## Performance

| Scenario | Beams | Time | FPS | Path |
|----------|-------|------|-----|------|
| NeuPAN corridor (mixed) | 100 | 0.866 ms | 1,154 | Grid DDA |
| 10 polygons | 1200 | 299 µs | 3,345 | Grid DDA |
| Circle (AVX2) | 1200 | 31.5 µs | 31,709 | AVX2 SIMD |
| Maze 251 segs (Grid) | 1200 | 189 µs | 5,291 | Grid DDA |

## Projects Using IR-SIM

### Academic Publications

- **[RAL & ICRA 2023]** [rl-rvo-nav](https://github.com/hanruihua/rl_rvo_nav) — Reinforcement learning-based RVO behavior for multi-robot navigation.
- **[RAL & IROS 2023]** [RDA_planner](https://github.com/hanruihua/RDA_planner) — Accelerated collision-free motion planner for cluttered environments.
- **[T-RO 2025]** [NeuPAN](https://github.com/hanruihua/NeuPAN) — Direct point robot navigation with end-to-end model-based learning.

### Community Projects

- [DRL-robot-navigation-IR-SIM](https://github.com/reiniscimurs/DRL-robot-navigation-IR-SIM) — Deep reinforcement learning for robot navigation.
- [AutoNavRL](https://github.com/harshmahesheka/AutoNavRL) — Autonomous navigation using reinforcement learning.
- [IRSIM-3DGS-Bridge](https://github.com/Wayneyujie/IRSIM-3DGS-Bridge) — A closed-loop bridge from 3D Gaussian Splatting scenes to IR-SIM planning/following.

## Citation

**IR-SIM-X** is a C++-accelerated fork of **IR-SIM**. If you find it useful, please consider citing the original IR-SIM paper:

```bibtex
@article{han2026ir,
  title={IR-SIM: A Lightweight Skill-Native Simulator for Navigation, Learning, and Benchmarking},
  author={Han, Ruihua and Wang, Shuai and Li, Chengyang and Gao, Rui and Wang, Xinyi and Liu, Zhe and Li, Guoliang and Lu, Yupu and Hao, Qi and Pan, Jia and Zhao, Hengshuang},
  journal={arXiv preprint arXiv:2606.08729},
  year={2026}
}
```

## License

irsim-x is released under the [MIT License](LICENSE). The original IR-SIM is also MIT-licensed — see the [original repository](https://github.com/hanruihua/ir-sim) for details.

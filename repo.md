# ir-simX — C++ Accelerated IR-SIM Fork

## 概述

ir-simX 是 [hanruihua/ir-sim](https://github.com/hanruihua/ir-sim) 的 C++ 加速分支，通过 pybind11 将 LiDAR 射线求交、碰撞检测、运动学与 A* 规划移植到 C++，在不修改原有 Python API 的前提下显著提升仿真性能。

## 架构

```
irsim/                      # Python 包（修改入口）
├── env/env_base.py         # C++ SimWorld 集成：_build_cpp_world, _cpp_step, _sync_cpp_to_python
├── world/sensors/lidar2d.py # C++ LiDAR 路径：_c_step, _obj_to_c_dict
└── world/object_base.py    # Robot/Obstacle 基类

irsim_core/                 # C++ pybind11 扩展
├── include/
│   └── geometry.h          # 几何原语：Vec2, AABB, 射线-圆/矩形/多边形求交, Cyrus-Beck, 凹多边形 edge-by-edge, 耳切三角剖分
├── src/
│   ├── lidar.cpp           # LiDAR 射线求交（标量 + AVX2）
│   ├── collision.cpp       # SAT 碰撞检测 + 凹多边形逐边检测
│   ├── kinematics.cpp      # diff/omni/acker/omni_angular 运动学
│   ├── world.cpp           # SimWorld：统一步进 + 碰撞 + 变换
│   └── astar.cpp           # A* 网格规划
├── bindings/
│   └── pybind_module.cpp   # Python↔C++ 绑定
└── __init__.py             # 从 _core 重新导出
```

## 已完成

### 核心算法

- [x] **凸多边形射线求交** — Cyrus-Beck 算法（inward/outward normal 正确处理）
- [x] **凹多边形射线求交** — 逐边最小 t 法（含边界命中：原点在边上返回 t=0）
- [x] **圆形射线求交** — 二次方程求最小正根
- [x] **矩形射线求交** — 转 AABB slab test（原矩形作为多边形传入支持旋转）
- [x] **AVX2 向量化圆形求交** — 8 束并行处理
- [x] **SAT 凸多边形碰撞检测**
- [x] **凹多边形碰撞检测** — 逐边内部点测试 + 中心距离近似
- [x] **运动学** — diff/omni/acker/omni_angular
- [x] **A* 网格规划**
- [x] **SimWorld** — 统一步进、碰撞、速度/加速度裁剪、局部→世界坐标变换

### 集成与修复

- [x] `_build_cpp_world` 矩形障碍物改用多边形顶点（修复 `half_w=0.5` 导致的 1×1 碰撞体）
- [x] `_obj_to_c_dict` 矩形改用多边形（修复 LiDAR 矩形尺寸）
- [x] `_sync_cpp_to_python` 补充 `gf.step()`、`mid_process()`、`trajectory.append()`
- [x] `_cpp_step` 补充 `pre_process()`、`gen_behavior_vel(None)`
- [x] 顶点 buffer 独立存储（修复 `vector<Vec2>` 共享覆盖 bug）
- [x] 移除 AVX2 dispatch 规避 pybind11 调用约定不匹配

### 测试工具

- [x] `tests/visual/01_small_irregular.py` — 小而不规则的多边形
- [x] `tests/visual/02_concave_notches.py` — 深凹槽多边形
- [x] `tests/visual/03_mixed_cluster.py` — 多边形 + 圆混合
- [x] `tests/visual/04_benchmark.py` — 性能基准测试
- [x] `tests/visual/05_complex_generated.py` — IR-SIM 随机生成器生成的多边形
- [x] `tests/visual/06_debug_phantom.py` — 虚影检测 + 扫描线可视化
- [x] `tests/visual/gui_debug.py` — gui.yaml 持久 LiDAR 累积 + JSON 日志
- [x] `tests/visual/run_exp_persist.py` — NeuPAN 场景持久点累积 + KDTree 分析
- [x] `tests/visual/compare_gui.py` — C++ vs Python Shapely 逐束对比
- [x] `tests/visual/compare_many.py` — 30 个随机多边形批量对比
- [x] `tests/visual/analyze_log.py` — JSON 日志离线分析
- [x] `tests/visual/09_test_1200.py` — 1200 束 + 10 个随机多边形

## 性能

| 场景 | 束数 | 耗时 | FPS |
|------|------|------|-----|
| NeuPAN corridor（纯 C++） | 100 | 0.866 ms | 1,154 |
| 10 个多边形 | 1200 | 299 µs | 3,345 |
| 圆形（AVX2） | 1200 | 31.5 µs | 31,709 |

## Repo 结构

```
ir-simX/
├── irsim/                  # Python 包（修改部分）
├── irsim_core/             # C++ pybind11 扩展
├── tests/visual/           # 可视化测试脚本
├── setup.py                # 构建脚本（含 stale .so 自动清理）
├── pyproject.toml
└── repo.md                 # 本文件
```

## 注意事项 / 经验教训

### 1. Editable install stale .so 陷阱

**现象**：`pip install -e .` 不会删除 site-packages 中旧的 `irsim_core.so`。Python 默认的 `PathFinder`（`sys.meta_path[2]`）优先级高于 editable 的 `_EditableFinder`（`sys.meta_path[4]`），导致旧 `.so` 优先加载，所有 C++ 修改失效。

**解决**：`setup.py` 中已加入自动清理：

```python
for sp in site.getsitepackages():
    for f in glob.glob(os.path.join(sp, 'irsim_core*.so*')):
        os.remove(f)
```

**重装/更新命令**（唯一需要的命令）：

```bash
pip install --force-reinstall --no-deps -e .
```

**验证**：

```bash
python -c "
import irsim_core, os, datetime
print(datetime.datetime.fromtimestamp(os.path.getmtime(irsim_core.__file__)))
"
```

显示今天的日期即为最新。

### 2. C++ 与 Python 的隐式约定

- `gf.vertices` 返回 `(2, N)` 世界坐标数组（已通过 `shapely.geometry.exterior.coords._coords.T[:, :-1]` 去除了末端的闭合重复点）
- `gf` 没有 `half_w` / `half_h` 属性；矩形尺寸需从 `gf.vertices` 或 `gf.length` / `gf.width` 推导
- `py_to_obstacle` 中多边形顶点需由调用方持久化；`Obstacle` 的 `verts` 指针在结构体复制后保持有效的前提是源 buffer 存活

### 3. SimWorld vs standalone LiDAR

| | SimWorld | 独立 `lidar_raycast` |
|---|---|---|
| 用途 | 物理步进 + 碰撞检测 | LiDAR 传感器 |
| 障碍物来源 | `_build_cpp_world` | 每次从 Python 对象构建 |
| 多边形存储 | `polygon_vertices_`（持久） | `verts_bufs`（lambda 局部） |

两者的障碍物数据相互独立，互不影响。

### 4. C++ 路径覆盖的 Python 功能

`_cpp_step` 目前完整覆盖了 Python `obj.step()` 的所有功能：

| 功能 | 状态 |
|---|---|
| `pre_process()`（wander/loop 目标更新） | ✅ |
| `gen_behavior_vel()`（dash/rvo/sfm 行为速度） | ✅ |
| 物理步进 + 碰撞检测 | ✅（C++ SimWorld） |
| `mid_process()`（角度包裹/维度填充） | ✅ |
| `_state` 更新 | ✅ |
| `_velocity` 更新 | ✅ |
| `_geometry` + `gf.step()` | ✅ |
| `trajectory.append()` | ✅ |
| `_invalidate_reactive_cache()` | ✅ |
| `sensor_step()`（LiDAR） | ✅ |
| `_status_step()`（碰撞停止） | ✅ |

## 开发计划

### 短期

- [ ] 补全单元测试（`tests/` 目录）
- [ ] 添加 CI（GitHub Actions + xvfb-run）
- [ ] 文档更新（Sphinx / docstrings）

### 中期

- [ ] 3D 支持（`EnvBase3D` + 3D LiDAR）
- [ ] 动态障碍物 C++ 加速（当前 fallback 回 Python）
- [ ] FMCW LiDAR C++ 加速

### 长期

- [ ] GPU 加速（CUDA LiDAR）
- [ ] 分布式多机器人仿真
- [ ] ROS 2 集成

## 运行示例

```bash
# 从走廊场景启动
cd /home/fanshu/Workplace/NeuPAN/example
MPLBACKEND=TkAgg conda run -n irsim_test python /home/fanshu/Workplace/ir-simX/tests/visual/run_exp_persist.py -e corridor -d diff

# GUI 场景（键盘控制 + 持久 LiDAR）
MPLBACKEND=TkAgg conda run -n irsim_test python /home/fanshu/Workplace/ir-simX/tests/visual/gui_debug.py

# 1200 束性能测试
conda run -n irsim_test python /home/fanshu/Workplace/ir-simX/tests/visual/09_test_1200.py
```

# ir-simX — C++ Accelerated IR-SIM Fork

## 概述

ir-simX 是 [hanruihua/ir-sim](https://github.com/hanruihua/ir-sim) 的 C++ 加速分支，通过 pybind11 将 LiDAR 射线求交、碰撞检测、运动学与 A\* 规划移植到 C++。**C++ 是唯一的执行路径**，不再保留 Python 兜底。

## 架构

```
irsim/                      # Python 包（修改入口）
├── env/env_base.py         # C++ SimWorld 集成；无 Python fallback
├── world/
│   ├── sensors/lidar2d.py  # C++ LiDAR 路径：_c_step, _obj_to_c_dict
│   └── object_base.py      # Robot/Obstacle 基类（check_status 简化版）
└── lib/handler/
    └── kinematics_handler.py

irsim_core/                 # C++ pybind11 扩展
├── include/
│   ├── geometry.h          # 几何原语 + 射线求交 + 耳切三角剖分
│   ├── collision.h         # SAT + 通用障碍物碰撞检测
│   ├── kinematics.h        # diff/omni/acker/omni_angular
│   ├── world.h             # SimWorld + RobotState + DynamicObstacle
│   ├── astar.h             # A* 规划器
│   └── lidar.h             # LiDAR raycasting（标量 + AVX2）
├── src/
│   ├── lidar.cpp
│   ├── collision.cpp
│   ├── kinematics.cpp
│   ├── world.cpp
│   └── astar.cpp
├── bindings/pybind_module.cpp
└── __init__.py
```

## 已完成

### 核心算法

- [x] **凸多边形射线求交** — Cyrus-Beck 算法（inward/outward normal 正确处理）
- [x] **凹多边形射线求交** — 逐边最小 t 法 + 耳切三角剖分
- [x] **圆形 / 矩形射线求交** — 二次方程 / AABB slab test
- [x] **AVX2 向量化圆形求交** — 8 束并行
- [x] **SAT 凸多边形碰撞检测**
- [x] **凹多边形碰撞检测** — 耳切三角剖分 + 每个三角形 SAT，替代已删除的错误 all_inside 检测和 center-to-edge fallback
- [x] **运动学** — diff/omni/acker/omni_angular（含 Ackermann 正确 wheelbase 和 steer_angle 管理）
- [x] **A\* 网格规划**
- [x] **SimWorld** — 统一步进、碰撞、速度/加速度/转向角裁剪

### 动态障碍物

- [x] **C++ 状态管理** — `DynamicObstacle` 结构体 + 步进 + 几何同步
- [x] **碰撞检测** — obstacle-vs-obstacle（圆-圆/圆-矩形/矩形-矩形/多边形-SAT）
- [x] **碰撞同步** — 回写 `collision_flag` 到 Python，`check_status()` 设 `stop_flag`
- [x] **三种形状** — circle / rectangle（含旋转检测）/ polygon 完整支持

### 架构重构

- [x] **移除 Python fallback 路径** — `_objects_step` / `_c_step_available` / Python 碰撞检测全部删除
- [x] **`step()` 直接调用 C++** — C++ 核心不可用时 `raise RuntimeError`
- [x] **`check_status()` 精简** — 只处理 arrival 和 `collision_flag`，不再依赖 STRtree
- [x] **`build_tree()` 仅初始化时调用一次** — 运行时不再重建

### 集成与修复

- [x] 机器人顶点传递格式修正 `.T.flatten()`（原 `.flatten()` 传错顺序）
- [x] `polygon_vertices_` 改用 `std::deque` 防止指针悬空（`std::vector` reallocation 问题）
- [x] `add_dynamic_*` 重复调用清理（`_build_cpp_world` 对动态障碍物只调一次）
- [x] `vel_acc` / `vel_min` / `vel_max` 未用维度默认值从 0 改为 inf
- [x] `ObjState` position → state 改用 `obj.state` 获取 theta
- [x] 碰撞检测时序修正（先 obstacle step 再 collision check）
- [x] SAT 跳过退化边（零长度边导致法线为零）
- [x] 网格地图 LiDAR `_map_to_c_dicts` 改用贪心矩形合并（替代步长采样，消除穿墙间隙）

### 测试

- [x] **66 个 C++ 核心单元测试**（`tests/test_cpp_core.py`）— 覆盖 SimWorld、运动学、A\*、LiDAR、碰撞
- [x] **8 个箭头默认值测试**（`tests/test_arrow_default.py`）
- [x] **3 个可视化 demo** — circle / polygon / rect 动态障碍物

## 经验教训与注意事项

### 1. Editable install stale .so 陷阱

**现象**：`pip install -e .` 不会删除 site-packages 中旧的 `irsim_core.so`。Python 默认的 `PathFinder`（`sys.meta_path[2]`）优先级高于 editable 的 `_EditableFinder`（`sys.meta_path[4]`），导致旧 `.so` 优先加载。

**解决**：`setup.py` 中已加入自动清理。

**重装命令**：
```bash
pip install --force-reinstall --no-deps -e .
```

### 2. C++ vector reallocation 导致指针悬空

`polygon_vertices_` 最初是 `std::vector<std::vector<Vec2>>`。每次 `push_back` 新多边形时，vector 可能整体 reallocate，**所有之前存储的 `data()` 指针全部变脏**。SAT 读出垃圾数据产生假阳性碰撞。

**解决**：改用 `std::deque<std::vector<Vec2>>`，deque 在 `push_back` 时不移动已有元素。

### 3. `_build_cpp_world` 的重复 add 陷阱

动态障碍物在 `_build_cpp_world` 中被调用了**两次** `add_obstacle`：一次显式的 `w.add_obstacle({"type":"circle",...})`，一次 `add_dynamic_obstacle` 内部。`obstacles_` 里有两个圆，`step_dynamic_obstacles` 只更新了第一个，第二个永远停在初始位置，形成"幽灵障碍物"。

**解决**：非静态障碍物跳过 `w.add_obstacle()`，只通过 `_add_dynamic_obstacle_to_cpp` 添加。

### 4. 凹多边形 `all_inside` 检测的假阳性

`all_inside` 用左法线判断"所有机器人顶点是否在多边形每条边的左侧"。这对 CCW 凸多边形成立，但对凹多边形——凹进去的边**内部在右侧，左侧反而是外部**——导致机器人距离多边形 20+ 单位时被误判为"在多边形内部"。

**解决**：删除 `all_inside`，改用耳切三角剖分 + 每个三角形跑 SAT。`ear_clip_triangulate` 也修复了一个丢失 winding sign 检查的 bug。

### 5. Ackermann 运动学硬编码

`step_acker` 中 `float L = 0.5f` 硬编码轴距，忽略了 YAML 配置的 `wheelbase: 3`。同时直接用**目标转向角**算转弯半径，而不是用当前转向角（Python 语义）。

**解决**：`RobotState` 增加 `steer_angle` 和 `wheelbase` 字段；`step_acker` 使用当前转向角计算 omega，再更新到目标值。

### 6. `obj.position` 不包含 theta

`_add_dynamic_obstacle_to_cpp` 用 `getattr(obj, "position", None)` 获取角度，但 `obj.position` 只返回 `state[:2]`（不含 theta），导致动态障碍物的初始角度始终为 0。

**解决**：改用 `getattr(obj, "state", None)`。

### 7. `vel_acc` / `vel_min` / `vel_max` 默认值

3 元素 padding 数组 `np.zeros(3)` 初始化，前 N 个赋有效值，剩下的默认 0。这导致 unused 维度的加速度限制为 0（永远无法加速），速度限制为 0（永远不能运动）。

**解决**：改为 `np.full(3, -inf / inf)`。

### 8. 机器人顶点传递格式

`original_vertices` 形状为 `(2, N)`（行主序：x 行、y 行），但 C++ `set_robot_vertices` 期望交错格式 `[x0, y0, x1, y1, ...]`。直接 `.flatten()` 产生 `[x0, x1, ..., y0, y1, ...]`。

**解决**：`.T.flatten()` 先转置再 flatten。

### 9. 网格地图 LiDAR 步长采样

`_map_to_c_dicts` 原先固定步长采样 grid cell，薄墙可能落在采样点之间被漏检。

**解决**：改用贪心矩形合并算法——扫描每个占据 cell，将相邻 cell 合并为尽量大的矩形，零遗漏、无间隙，同时矩形数量更少。

### 10. C++ 与 Python 的隐式约定

- `gf.vertices` 返回 `(2, N)` 世界坐标数组
- `gf` 没有 `half_w` / `half_h` 属性；矩形尺寸需从 `gf.vertices` 或 `gf.length` / `gf.width` 推导
- 多边形顶点传递时必须保证 `verts` 指针在生命周期内持久有效

### 11. SimWorld vs standalone LiDAR 的关系

| | SimWorld | 独立 `lidar_raycast` |
|---|---|---|
| 用途 | 物理步进 + 碰撞检测 | LiDAR 传感器 |
| 障碍物来源 | `_build_cpp_world` | 每次从 Python 对象构建 |
| 多边形存储 | `polygon_vertices_`（deque 持久） | `verts_bufs`（lambda 局部） |

两者的障碍物数据相互独立，互不影响。

### 12. 运行原始 ir-sim 对比

ir-simX 移除了 Python fallback，但可使用 `-P` 标志运行 `run_original.sh` 用 ML 环境中的原始 ir-sim 做对比测试：
```bash
./run_original.sh -m pytest tests/test_kinematics.py -v
```

## 性能

| 场景 | 束数 | 耗时 | FPS |
|------|------|------|-----|
| NeuPAN corridor（纯 C++） | 100 | 0.866 ms | 1,154 |
| 10 个多边形 | 1200 | 299 µs | 3,345 |
| 圆形（AVX2） | 1200 | 31.5 µs | 31,709 |

## 开发计划

### 短期

- [ ] **FMCW LiDAR C++ 加速** — 复用 `lidar.cpp` 的射线求交，额外计算径向速度
- [ ] **文档更新** — Sphinx docstrings 以反映 C++ 路径变更

### 中期

- [ ] **3D 支持** — `EnvBase3D` + 3D LiDAR 接入 C++
- [ ] **linestring 动态障碍物** — 当前 C++ 不支持，需扩展 DynamicObstacle

### 长期

- [ ] **GPU 加速（CUDA LiDAR）**
- [ ] **分布式多机器人仿真**
- [ ] **ROS 2 集成**

## 运行示例

```bash
# 圆形动态障碍物 demo
MPLBACKEND=TkAgg conda run -n irsim_test python usage/24_dynamic_cpp_demo/run_demo.py circle

# 多边形动态障碍物 demo
MPLBACKEND=TkAgg conda run -n irsim_test python usage/24_dynamic_cpp_demo/run_demo.py polygon

# 矩形动态障碍物 demo
MPLBACKEND=TkAgg conda run -n irsim_test python usage/24_dynamic_cpp_demo/run_demo.py rect

# GUI 键盘控制
MPLBACKEND=TkAgg conda run -n irsim_test python usage/17gui_world/gui.py

# HM3D 网格地图 + Ackermann 机器人
MPLBACKEND=TkAgg conda run -n irsim_test python usage/10grid_map/grid_map_hm3d.py

# 原始 ir-sim 对比（ML 环境）
./run_original.sh -m pytest tests/test_kinematics.py -v
```

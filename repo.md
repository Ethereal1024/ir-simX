# ir-simX — C++ Accelerated IR-SIM Fork

## 概述

ir-simX 是 [hanruihua/ir-sim](https://github.com/hanruihua/ir-sim) 的 C++ 加速分支，通过 pybind11 将 LiDAR 射线求交、碰撞检测、运动学与 A\* 规划移植到 C++。**C++ 是唯一的执行路径**，不再保留 Python 兜底。

## 架构

```
irsim/                      # Python 包
├── env/
│   ├── env_base.py         # EnvBase 主类（1502 行）
│   ├── _cpp_sim.py         # CppSim — C++ SimWorld 桥接（533 行）
│   ├── env_plot.py         # 2D 可视化（625 行）
│   ├── env_plot_helpers.py # 绘制辅助函数（263 行）
│   ├── env_base3d.py       # 3D 可视化投影层
│   ├── env_config.py       # YAML 配置解析
│   ├── env_logger.py       # 日志
│   └── env_plot3d.py       # 3D 可视化
├── world/
│   ├── object_base.py      # ObjectBase 主类（1871 行）
│   ├── object_plot.py      # ObjectBasePlotMixin 绘制方法（968 行）
│   ├── sensors/
│   │   ├── lidar2d.py      # 2D LiDAR（622 行）
│   │   ├── _lidar_cpp.py   # C++ LiDAR 桥接（187 行）
│   │   └── fmcw_lidar2d.py # FMCW LiDAR
│   └── map/
│       ├── map_core.py     # Map 类
│       └── map_utils.py    # EnvGridMap + 工具函数
└── lib/
    └── ...

cpp/                         # C++ pybind11 扩展
├── include/
│   ├── geometry.h          # 几何原语 + 射线求交 + 耳切三角剖分 + ray-linestring
│   ├── collision.h         # SAT + 通用障碍物碰撞检测（含 linestring）
│   ├── kinematics.h        # diff/omni/acker/omni_angular
│   ├── world.h             # SimWorld + RobotState + DynamicObstacle（含 linestring）
│   ├── astar.h             # A* 规划器
│   └── lidar.h             # LiDAR raycasting（标量 + AVX2）+ fmcw_lidar_raycast
├── src/
│   ├── lidar.cpp           # lidar_raycast + fmcw_lidar_raycast
│   ├── collision.cpp       # robot-obstacle + obstacle-obstacle（含 linestring SAT）
│   ├── kinematics.cpp
│   ├── world.cpp           # SimWorld + add_dynamic_linestring_obstacle
│   └── astar.cpp
├── bindings/pybind_module.cpp
└── __init__.py
```

## 已完成

### 核心算法

- [x] **凸多边形射线求交** — Cyrus-Beck 算法（inward/outward normal 正确处理）
- [x] **凹多边形射线求交** — 逐边最小 t 法 + 耳切三角剖分
- [x] **圆形 / 矩形射线求交** — 二次方程 / AABB slab test
- [x] **线段（linestring）射线求交** — 逐段最小 t 法
- [x] **AVX2 向量化圆形求交** — 8 束并行
- [x] **AVX2 向量化 AABB 矩形求交** — 8 束并行 slab test
- [x] **AVX2 分发启用** — `lidar_raycast()` 在 AVX2 可用时自动启用 SIMD（修复 blend mask 符号位 bug）
- [x] **SAT 凸多边形碰撞检测**
- [x] **凹多边形碰撞检测** — 耳切三角剖分 + 每个三角形 SAT，替代已删除的错误 all_inside 检测和 center-to-edge fallback
- [x] **Linestring 碰撞检测** — 逐段建模为 OBB（厚 0.05）+ SAT
- [x] **运动学** — diff/omni/acker/omni_angular（含 Ackermann 正确 wheelbase 和 steer_angle 管理）
- [x] **A\* 网格规划**
- [x] **SimWorld** — 统一步进、碰撞、速度/加速度/转向角裁剪

### 动态障碍物

- [x] **C++ 状态管理** — `DynamicObstacle` 结构体 + 步进 + 几何同步
- [x] **碰撞检测** — obstacle-vs-obstacle（圆-圆/圆-矩形/矩形-矩形/多边形-SAT/linestring-SAT）
- [x] **碰撞同步** — 回写 `collision_flag` 到 Python，`check_status()` 设 `stop_flag`
- [x] **四种形状** — circle / rectangle（含旋转检测）/ polygon / linestring 完整支持

### 传感器

- [x] **2D LiDAR C++ 加速** — 独立 `lidar_raycast` 函数 + `SimWorld.raycast()`
- [x] **FMCW LiDAR C++ 加速** — `fmcw_lidar_raycast` 复用射线引擎 + 逐束径向速度计算
- [x] **Grid map 矩形合并** — 贪心合并算法，消除步长采样的穿墙间隙

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
- [x] **`static: True` 机器人在 C++ 路径中被正确冻结**（`_cpp_step` 加 `or obj.static` 检查）
- [x] **障碍物到达目标后不再振荡** — `_cpp_step` G1 在步进前检测 `check_arrive(obj.goal)` 并直接归零速度
- [x] **障碍物步进前预写 C++ 速度** — 使 C++ 内部 acc 裁剪退化为 no-op（`cur_vel == act`）
- [x] **LiDAR heading 修正** — 使用 `self.lidar_origin[2,0]`（含传感器 offset 旋转）替代 `parent_state[2,0]`（仅机器人朝向）

### 测试

- [x] **66 个 C++ 核心单元测试**（`tests/test_cpp_core.py`）— 覆盖 SimWorld、运动学、A\*、LiDAR、碰撞
- [x] **20 个 FMCW LiDAR 测试**（`tests/test_fmcw_lidar2d.py`）
- [x] **8 个箭头默认值测试**（`tests/test_arrow_default.py`）
- [x] **3 个可视化 demo** — circle / polygon / rect 动态障碍物

### 代码结构整理

- [x] **`irsim_core/` → `cpp/`** — 目录名明确表示 C++ 扩展
- [x] **`env_plot.py` → `env_plot_helpers.py`** — 提取 3 个模块级绘制辅助函数（318 行）
- [x] **`lidar2d.py` → `_lidar_cpp.py`** — 提取 C++ LiDAR 桥接（164 行）
- [x] **`object_base.py` → `object_plot.py`** — 提取 14 个绘制方法为 mixin 类（952 行）
- [x] **`env_base.py` → `_cpp_sim.py`** — 提取 5 个 C++ 集成方法为 `CppSim` 类（508 行）
- [x] **`map/__init__.py` → `map_core.py` + `map_utils.py`** — 拆分 Map 类与工具函数
- [x] **删除死代码** — `geometry.cpp` 占位文件、`transform_vertices`、`get_edge_normals`
- [x] **清理杂物** — `tests/test.md` stray 文件、`.gitignore` 缓存目录

## 已修复问题摘要

22 个已知问题（严重 #1–#5、中等 #6–#12、低 #13–#22）已于 2026-06 全部修复，845 测试通过。主要修复项：

- **索引对齐**：Python 侧 `_cpp_id` 追踪 C++ 对象 ID；C++ 侧 `poly_verts_index` / `obs_index` 修复 static+dynamic 混合时的几何体错位
- **旋转矩形**：碰撞检测用 SAT（非退化 AABB），AABB 预过滤正确处理旋转角
- **Robot×Robot 碰撞**：`detect_collisions` 添加 SAT 循环
- **LiDAR**：C++ 路径添加 noise 支持；`has_velocity` 走 Python fallback；网格地图用贪心合并
- **行为同步**：G1 添加 `pre_process()`；`post_process()` 接入主循环；动态 linestring 激活
- **默认值**：pybind 限幅改为 `±FLT_MAX`；rect 尺寸从 `gf` 读取
- **AVX2**：圆形 + AABB 矩形 8 束 SIMD；FMCW LiDAR 扩展 AVX2 + 径向速度追踪

## 参考

- **编译**：`python3 setup.py build_ext --inplace`（`pip install -e .` 不触发 C++ 重编译）
- **SimWorld vs 独立 LiDAR**：SimWorld 用持久化 `polygon_vertices_`（deque）；独立 `lidar_raycast` 每步从 Python dict 构建，互不影响
- **顶点传递**：`gf.vertices` 返回 `(2,N)` 世界坐标；传给 C++ 需 `.T.flatten()` 交错格式
- **LiDAR heading**：必须用 `self.lidar_origin[2,0]`（含传感器 offset 旋转），非 `parent_state[2,0]`
- **网格地图阈值**：`> 50` 对齐 `OCCUPANCY_THRESHOLD = 50`（ImageGridGenerator 缩放到 0-100）

## 性能

| 场景 | 束数 | 耗时 | FPS | 路径 |
|------|------|------|-----|------|
| NeuPAN corridor（纯 C++） | 100 | 0.866 ms | 1,154 | 混合（CIRCLE/RECT SIMD + 标量） |
| 10 个多边形 | 1200 | 299 µs | 3,345 | 标量（多边形/LINESTRING） |
| 圆形（AVX2） | 1200 | 31.5 µs | 31,709 | AVX2 SIMD |
| AABB 矩形（AVX2） | 1200 | ~80 µs | ~12,500 | AVX2 SIMD slab test |

## 开发计划

### 已完成

- [x] **FMCW LiDAR C++ 加速**
- [x] **linestring 动态障碍物**
- [x] **已知问题修复（#1–#22）** — 全部 22 个问题修复，845 测试通过
- [x] **AVX2 LiDAR 启用** — 圆形 + AABB 矩形 8 束 SIMD；FMCW AVX2 扩展
- [x] **动态障碍物索引修复** — `poly_verts_index` + `obs_index`
- [x] **代码清理** — 删除死代码，拆分大文件，整理目录结构

### 待办

- [ ] **S3: `object_base.py` 进一步拆分** — 提取 `get_vel_range`、`input_state_check` 等独立工具函数（~150 行）
- [ ] **S4: `env_plot.py` 绘制方法提取** — `draw_trajectory`、`draw_points` 等为 mixin 或 helper（~144 行）
- [ ] **S5: `env_plot.py` 导出方法提取** — `save_figure` + `save_animate` → `env_plot_exporter.py`（~115 行）
- [ ] **S6: `object_base.py` 工具函数提取** — `get_desired_omni_vel`、`get_init_Gh` 等纯计算函数（~100 行）

### 远期

- [ ] **3D 支持** — `EnvBase3D` + 3D LiDAR 接入 C++
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

# FMCW LiDAR demo
MPLBACKEND=TkAgg conda run -n irsim_test python usage/22fmcw_lidar_world/fmcw_lidar_world.py

# 原始 ir-sim 对比（ML 环境）
./run_original.sh -m pytest tests/test_kinematics.py -v
```

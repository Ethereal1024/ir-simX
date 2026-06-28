# ir-simX — C++ Accelerated IR-SIM Fork

## 概述

ir-simX 是 [hanruihua/ir-sim](https://github.com/hanruihua/ir-sim) 的 C++ 加速分支，通过 pybind11 将 LiDAR 射线求交、碰撞检测、运动学与 A\* 规划移植到 C++。**C++ 是唯一的执行路径**，不再保留 Python 兜底。

## 架构

```
irsim/                      # Python 包
├── env/
│   ├── env_base.py         # EnvBase 主类（1502 行）
│   ├── _cpp_sim.py         # CppSim — C++ SimWorld 桥接（533 行）
│   ├── _batch_cpp_sim.py   # BatchCppSim — C++ BatchSimWorld 桥接（~275 行）
│   ├── batch_env_base.py   # BatchEnvBase 批处理环境类（~150 行）
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
│   ├── batch_world.h       # BatchSimWorld — SoA 布局 + SIMD 多环境仿真
│   ├── simd_config.h       # SIMD_WIDTH 编译期检测 + BatchConfig
│   ├── astar.h             # A* 规划器
│   └── lidar.h             # LiDAR raycasting（标量 + AVX2）+ fmcw_lidar_raycast
├── src/
│   ├── lidar.cpp           # lidar_raycast + fmcw_lidar_raycast
│   ├── collision.cpp       # robot-obstacle + obstacle-obstacle（含 linestring SAT）
│   ├── kinematics.cpp
│   ├── world.cpp           # SimWorld + add_dynamic_linestring_obstacle
│   ├── batch_world.cpp     # BatchSimWorld 构造 + 步进调度
│   ├── batch_kinematics.cpp # SIMD 运动学 kernel（diff/omni/omni_angular/acker）
│   ├── batch_lidar.cpp     # 跨环境 beam-major SIMD LiDAR（模式 A/B）
│   ├── batch_collision.cpp # AABB SIMD 过滤 + SAT 回退
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
| 迷宫 251 segs（SpatialHashGrid） | 1200 | 189 µs | 5,291 | 网格 DDA |
| 500 rects（SpatialHashGrid） | 1200 | 124 µs | 8,064 | 网格 DDA |

### 四种地图 benchmark（优化最终版）

| Map | Obs | Segs | env.step(ms) | FPS | 瓶颈 |
|-----|-----|------|-------------|-----|------|
| Sparse | 62 | 211 | **0.133** | **7,506** | Python 调用开销 |
| Maze | 2 | 98 | **0.290** | **3,448** | C++ LiDAR |
| Graph | 8 | 53 | **0.148** | **6,755** | C++ LiDAR |
| WFC_warehouse | 6 | 126 | **0.577** | **1,732** | C++ LiDAR |

优化历程：
1. **SpatialHashGrid 统一四种类型** — LiDAR 从 O(N) 变为 O(√N)，500 rects 从 ~15ms 降至 0.125ms
2. **绕过 Python dict 序列化** — `SimWorld::raycast_at` 直接使用 C++ obstacles_，Sparse 从 1.15ms 降至 0.14ms（8.2×）
3. **FMCW + 动态障碍物支持** — 完整的 fast path，per-robot 独立 fallback

## 开发计划

### 已完成

- [x] **FMCW LiDAR C++ 加速**
- [x] **linestring 动态障碍物**
- [x] **已知问题修复（#1–#22）** — 全部 22 个问题修复，845 测试通过
- [x] **AVX2 LiDAR 启用** — 圆形 + AABB 矩形 8 束 SIMD；FMCW AVX2 扩展
- [x] **动态障碍物索引修复** — `poly_verts_index` + `obs_index`
- [x] **代码清理** — 删除死代码，拆分大文件，整理目录结构

### 已完成（批处理 SIMD 仿真）

- [x] **`simd_config.h`** — 编译期 AVX-512/AVX2/标量回退检测，`SIMD_WIDTH` 常量
- [x] **`batch_world.h` + `batch_world.cpp`** — `BatchSimWorld` SoA 布局，管理 N 环境的 x/y/theta/vx/vy/omega 等核心数组
- [x] **`batch_kinematics.cpp`** — SIMD 运动学 kernel（diff/omni/omni_angular/acker）+ 速度/加速度裁剪
- [x] **`batch_lidar.cpp`** — 跨环境 beam-major SIMD LiDAR：
  - **模式 A（共享障碍物，默认）**：每个 beam 同时处理 SIMD_WIDTH 个环境的圆形/矩形求交
  - **模式 B（per-environment 障碍物）**：逐个环境标量回退
  - 多边形/linestring 降级标量回退
- [x] **`batch_collision.cpp`** — AABB 预过滤（SIMD 8 环境并行）+ SAT 标量回退
- [x] **BatchSimWorld pybind11 绑定** — `BatchConfig` + `BatchSimWorld` 类暴露给 Python
- [x] **`_batch_cpp_sim.py`** — `BatchCppSim` Python 桥接类（build/step/sync）
- [x] **`batch_env_base.py`** — `BatchEnvBase` 类，与 `EnvBase` 接口兼容，返回 `(batch_size, ...)` 形状
- [x] **`irsim.make(batch_size=4)`** — 支持 `batch_size` 参数，>1 时返回 `BatchEnvBase`；`share_obstacles` 参数选择模式 A/B
- [x] **`setup.py` / `CMakeLists.txt`** — 注册新的 .cpp/.h 文件

### 待办

- [ ] **S3: `object_base.py` 进一步拆分** — 提取 `get_vel_range`、`input_state_check` 等独立工具函数（~150 行）

### 性能优化 — 消除 Python dict 序列化瓶颈

当前 LiDAR 数据流中 Python 侧构造 obstacle dict 占 sensor_step 的 81%（Sparse 62 obs 场景）。
优化方案：让 LiDAR 直接使用 `SimWorld` 内部的 `obstacles_` 数组，跳过 Python → dict → C++ 的双重序列化。

#### 实施步骤

- [x] **P1: SimWorld 添加 `raycast_at` 方法** — 接受自定义 origin/heading（支持传感器 offset 旋转）
- [x] **P2: `_cpp_sim.py` 改用 `w.raycast_at()`** — 替代 `_objects_sensor_step()`，对 LiDAR 类型跳过 dict 构建
- [x] **P3: 动态障碍物位置同步** — `step_dynamic_obstacles` 中同步 `obs.vx/vy` 和到 `obstacles_`；运动后重建 `lidar_grid_`
- [x] **P4: FMCW LiDAR 接入** — `SimWorld::fmcw_raycast_at` 直接调用 `fmcw_lidar_raycast`，跳过 Python dict 序列化
- [x] **P5: 批量环境的对应优化** — BatchSimWorld 中 `batch_raycast` 已直接使用 `obstacles_`，已验证一致
- [ ] **S4: `env_plot.py` 绘制方法提取** — `draw_trajectory`、`draw_points` 等为 mixin 或 helper（~144 行）
- [ ] **S5: `env_plot.py` 导出方法提取** — `save_figure` + `save_animate` → `env_plot_exporter.py`（~115 行）
- [ ] **S6: `object_base.py` 工具函数提取** — `get_desired_omni_vel`、`get_init_Gh` 等纯计算函数（~100 行）

### 迭代改进（Batch SIMD）

- [ ] **多边形/linestring 障碍物支持** — 当前 batch 模式跳过 polygon/linestring，需加持久化 vertex 存储
- [ ] **动态障碍物批处理** — `BatchSimWorld` 暂不支持动态障碍物步进
- [ ] **OpenMP 多线程** — 当前 `step()` 中 vertex 变换和碰撞检测是单线程循环
- [ ] **Per-environment 奖励函数** — `BatchEnvBase._compute_rewards()` 目前返回全零
- [ ] **`reset()` 支持 per-env 种子** — 不同环境不同种子初始位置
- [ ] **Benchmark 测试** — 对比 batch_size=1/8/64/1024 与单环境 SimWorld 性能
- [ ] **SoA 数组 32 字节对齐** — 当前用 `_mm256_loadu_ps`（未对齐），改为 aligned allocator 后用 `_mm256_load_ps`

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

# ir-simX — C++ Accelerated IR-SIM Fork

## 概述

ir-simX 是 [hanruihua/ir-sim](https://github.com/hanruihua/ir-sim) 的 C++ 加速分支，通过 pybind11 将 LiDAR 射线求交、碰撞检测、运动学与 A\* 规划移植到 C++。**C++ 是唯一的执行路径**，不再保留 Python 兜底。

## 架构

```
irsim/                      # Python 包（修改入口）
├── env/env_base.py         # C++ SimWorld 集成；无 Python fallback
├── env/env_base3d.py       # 3D 可视化投影层（无独立物理/传感器）
├── world/
│   ├── sensors/lidar2d.py  # C++ LiDAR 路径：_c_step, _obj_to_c_dict
│   ├── sensors/fmcw_lidar2d.py  # FMCW LiDAR：_c_step C++ 路径 + Shapely fallback
│   └── object_base.py      # Robot/Obstacle 基类（check_status 简化版）
└── lib/handler/
    └── kinematics_handler.py

irsim_core/                 # C++ pybind11 扩展
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

## 已知问题

以下问题是在 C++ 路径与 Python 路径行为对比中发现的，按影响程度排列。

### 严重 — 可能导致数据损坏或行为异常

1. **动态 polygon 障碍物被重复注册**（`env_base.py:394-432`）
   `w.add_obstacle(vd)` 在 `if not obj.static:` 之外无条件调用，polygon 同时拥有静态副本（卡在初始位置）和动态副本。对比 circle/rect 的对应逻辑正确地在 `if obj.static:` 内部调用。

2. **C++→Python 索引映射假设不安全**（`env_base.py:666-667, 710-711`）
   `_sync_cpp_to_python` 假设 `py_obstacles[i]` 在 C++ 中的索引就是 `i`。但当 `_build_cpp_world` 中 `continue` 跳过某些对象时（缺少 position、geometry、verts），会导致索引错位——C++ 对象 N 的数据被写入 Python 对象 N+1。

3. **`polygon_vertices_[i]` 索引映射错误**（`world.cpp:257, 272`）
   `polygon_vertices_` deque 存储所有 polygon/linestring（静态+动态），但 `step_dynamic_obstacles` 用动态障碍物索引 `i` 直接访问。存在静态 polygon 时取到错误的顶点数据，导致碰撞检测使用错误的几何体。

4. **旋转矩形碰撞退化为 AABB**（`collision.cpp:187-189`）
   `check_rect_rect` 只用轴对称重叠判断。`needs_sat()` 未将 `theta ≠ 0` 的 RECT 路由到 SAT 路径，两个旋转矩形的碰撞产生假阴性。

5. **未检测 Robot×Robot 碰撞**（`world.cpp:341-382`）
   `detect_collisions` 只检查 robot×obstacle 和 obstacle×obstacle，多机器人场景中两个机器人相撞无法被检测。

### 中等 — 功能缺失或静默异常

6. **C++ LiDAR 忽略 `noise=True`**（`lidar2d.py:259-261`）
   C++ 路径直接赋原值，Python Shapely 路径会加高斯噪声。`noise=True` 时 C++ 返回噪声自由数据。

7. **C++ LiDAR 忽略 `has_velocity=True`**（`lidar2d.py`）
   速度数组永远为零，Python 路径会从命中障碍物获取速度信息。

8. **`_geometry_valid` 被无条件设为 True**（`env_base.py:704, 742`）
   Python 路径用 `shapely.is_valid()` 校验几何体有效性，C++ 路径跳过校验。如果 C++ 产生无效几何体（如自交 polygon），Python 侧无法感知。

9. **`post_process()` 从未被调用**（`env_base.py` vs `object_base.py:535`）
   Python 的 `step()` 在传感器步进后调用 `post_process()`，C++ 路径没有。当前是 no-op，但子类重写会静默失效。

10. **动态 linestring 障碍物是死代码**（`env_base.py:433-454` vs `623-627`）
    `_build_cpp_world` 注册了动态 linestring，但 `_cpp_step` 的 G2 白名单不含 `"linestring"`，永远被送入零速度。整个 `_add_dynamic_linestring_obstacle_to_cpp` 方法无效。

11. **障碍物 `pre_process()` 在 G1 时被跳过**（`env_base.py:617-621`）
    Python 路径无条件调用 `pre_process()`，C++ 的 G1（stop_flag 或 check_arrive）跳过它，导致 wander/loop 目标更新在已到达的障碍物上失效。

12. **RECT AABB 不处理旋转**（`geometry.h:78-80`）
    `compute_aabb()` 对 RECT 类型忽略 `theta`。`step_dynamic_obstacles` 后重算时产生过小的包围盒。

### 低 — 设计缺陷或潜在风险

13. **用户提供的速度绕过 Python acc 裁剪**（`env_base.py:582-591`）
    Python 路径即使用户提供速度也经过 `get_vel_range()` 裁剪。C++ 路径对 `action[obj._id]` 直接传给 C++，仅由 C++ 内部裁剪。

14. **Linestring 碰撞厚度硬编码 0.05**（`collision.cpp:147, 243`）
    两处独立硬编码，无自定义机制。Python 侧也是 0.05，一致但不可配置。

15. **`_map_to_c_dicts` 参数 `downsample_m` 是死代码**（`lidar2d.py:317`）
    函数签名接受参数但从未使用。

16. **`_map_to_c_dicts` 占用阈值硬编码 50**（`lidar2d.py:340`）
    假设 0-100 灰度范围，无文档记录。对于 0-255 地图会被截断。

17. **`point_in_polygon` 函数名误导**（`geometry.h:316-324`）
    名称暗示通用性但仅适用于 CCW 凸多边形。无前置检查，调错即返回错误结果。

18. **`alloca` 无限增长风险**（`collision.cpp:35`）
    SAT 不检查顶点数上限直接用 `alloca`。超大 polygon（如 10 万顶点）会栈溢出。

19. **pybind 绑定中 omega 默认限幅 ±1**（`pybind_module.cpp`）
    Python 未传 `vel_min/max` 时 C++ 默认 `[-1,1]`。acker/omni_angular 的 omega 通道被强加不必要的 ±1 限制，而 Python 侧 pad 为 ±inf。

20. **矩形 `_obj_to_c_dict` 回退尺寸硬编码**（`lidar2d.py:286-287`）
    `verts.shape[1] != 4` 时（罕见但可能），回退到 `half_w=0.5, half_h=0.5`，忽略实际 `length`/`width`。

21. **机器人顶点回退使用 world-frame**（`env_base.py:316-327`）
    `original_vertices` 缺失时回退到 `vertices`（可能是 world 坐标而非 local 坐标），注释承认是补丁。

22. **`_obj_to_c_dict` 同时传递 world 坐标 + position**（`lidar2d.py:280, 292, 298`）
    polygon/rectangle/ls 的 vertices 是 world 坐标，但同时传了 `x,y` 位置。如果 C++ 将两者叠加会双重平移。当前 C++ 对 POLYGON/LINESTRING 忽略 `x,y`，但对 RECT 使用 `x,y`。

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

### 12. LiDAR heading 必须含传感器 offset 旋转

`_c_step` 传入 C++ 的 heading 必须是 `self.lidar_origin[2,0]`（传感器世界朝向 = robot.theta + offset.theta），而非 `parent_state[2,0]`（仅 robot.theta）。当 YAML 中 `offset: [x, y, theta]` 的 theta ≠ 0 时，用错 heading 会导致所有激光束整体偏转。

### 13. `gen_behavior_vel` 裁剪导致障碍物到达后振荡

`OmniDash` 在 dist < goal_threshold 时返回 `[0,0]`，但 `gen_behavior_vel` 调用 `get_vel_range` 做加速度裁剪，`np.clip(0, cur-acc*dt, cur+acc*dt)` 当 `cur_vel` 非零时把 `0` 推进非零值，障碍物"滑行"过目标后回弹。修复方案是在 `_cpp_step` 的 G1 中用 `obj.check_arrive(obj.goal)` 提前拦截，跳过 `gen_behavior_vel`。

### 14. C++ 编译命令

`pip install -e .` **不会触发 C++ 重编译**。修改任何 `.cpp`/`.h` 文件后必须手动执行：

```bash
python3 setup.py build_ext --inplace
```

### 15. 运行原始 ir-sim 对比

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

### 短期（当前迭代）

- [x] **FMCW LiDAR C++ 加速** — 复用 `lidar.cpp` 的射线求交，额外计算径向速度
- [x] **文档更新** — Sphinx docstrings 以反映 C++ 路径变更
- [x] **linestring 动态障碍物** — C++ 支持 ray-linestring 求交、SAT 碰撞检测、动态步进
- [ ] **已知问题修复（严重）** — 详见 `## 已知问题` 中的 #1–#5
- [ ] **已知问题修复（中等）** — 详见 `## 已知问题` 中的 #6–#12

### 中期

- [ ] **3D 支持** — `EnvBase3D` + 3D LiDAR 接入 C++（调研完成：当前 3D 是纯可视化投影层）
- [ ] **Robot×Robot 碰撞检测**
- [ ] **动态 linestring 激活** — G2 白名单补充 `"linestring"`，使 `_add_dynamic_linestring_obstacle_to_cpp` 生效

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

# FMCW LiDAR demo
MPLBACKEND=TkAgg conda run -n irsim_test python usage/22fmcw_lidar_world/fmcw_lidar_world.py

# 原始 ir-sim 对比（ML 环境）
./run_original.sh -m pytest tests/test_kinematics.py -v
```

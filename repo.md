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

## 已知问题与修复方案

以下问题是在 C++ 路径与 Python 路径行为对比中发现的。每个问题附带可执行的修复方案。

---

### 严重 — 可能导致数据损坏或行为异常

#### #1 动态 polygon 障碍物被重复注册

**位置**：`env_base.py:394-432`

**根因**：`w.add_obstacle(vd)` 在 `if not obj.static:` 条件外无条件执行。对比 circle（`line 346: if obj.static:`）和 rect（`line 363: if obj.static:`）正确地加了门控。

**修复**：
1. 将 `line 411: w.add_obstacle(vd)` 移入 `if obj.static:` 分支内
2. 对动态 polygon 只调用 `add_dynamic_polygon_obstacle`，不放 `add_obstacle`
3. 验证：在 `_build_cpp_world` 结束后，`w.num_dynamic_obstacles()` 应等于 YAML 中非静态障碍物数量，`w.num_obstacles()` 应等于静态障碍物数 + 动态障碍物数（因为 `add_dynamic_polygon_obstacle` 内部已注册 `obstacles_`）

**预演代码**：
```python
elif shape == "polygon":
    verts = getattr(obj, "vertices", None)
    if verts is None or verts.shape[1] < 3:
        continue
    if obj.static:
        vlist = [{...}]
        for vd in vlist:
            w.add_obstacle(vd)
    if not obj.static:
        did = w.add_dynamic_polygon_obstacle(...)
        ...
```

**验证方式**：
```bash
PYTHONPATH= python3 -m pytest tests/test_cpp_core.py -q
# 专门写一个 test: 创建一个动态 polygon 障碍物，step 10 步，
# 检查 w.num_obstacles() 不随时间增长
```

---

#### #2 C++→Python 索引映射假设不安全

**位置**：`env_base.py:666-667`, `710-711`（sync）；`env_base.py:333-399`（build 中的 continue）

**根因**：`_sync_cpp_to_python` 遍历 `py_robots = [obj for obj ...]` / `py_obstacles = [...]` 并用 `for rid/did in range(...)` 做 1:1 索引访问。但当 `_build_cpp_world` 中因 `pos is None` / `gf is None` / `verts invalid` 而 `continue` 跳过某对象时，C++ 中存储的对象数 < Python 中的对象数，索引就错位了。

**修复方案 A（保守）**：
1. 在 `_build_cpp_world` 中不再用 `continue` 静默跳过——改为记录 `warned_count` 并 log error
2. 对于确实无法添加到 C++ 的对象，也调用某种占位注册（如零半径圆）以保持索引对齐
3. 在 `_sync_cpp_to_python` 开头断言 `w.num_robots() == len(py_robots)`，不等则 raise

**修复方案 B（彻底）**：
1. 为每个 Python 对象存储其 C++ 索引（`_cpp_id` 属性）
2. `_sync_cpp_to_python` 改用 `obj._cpp_id` 而非遍历索引来访问 C++ 数据
3. `_cpp_step` 的 action 数组同样用 `_cpp_id` 对齐

**推荐 B**（一劳永逸），但 A 可作为快速止损先行实施。

**验证方式**：
```python
# 构造场景：3 个 robot，其中 1 个没有 position
# 调用 env.step() 应不崩溃且状态正确
```

---

#### #3 `polygon_vertices_[i]` 索引映射错误

**位置**：`world.cpp:257`（POLYGON 同步）, `world.cpp:269`（LINESTRING 同步）

**根因**：`polygon_vertices_` deque 存储所有 polygon/linestring 的顶点（包括静态和动态）。但 `step_dynamic_obstacles` 用动态障碍物索引 `i` 直接作为 `polygon_vertices_` 索引。当静态 polygon 注册在动态 polygon 之前时，两者索引就不对齐。

**修复**：
1. 在 `DynamicObstacle` 结构体中添加 `size_t poly_verts_index` 字段，记录该动态障碍物在 `polygon_vertices_` 中的真实索引
2. `add_dynamic_polygon_obstacle` / `add_dynamic_linestring_obstacle` 在 `push_back(verts)` 后保存 `dob.poly_verts_index = polygon_vertices_.size() - 1`
3. `step_dynamic_obstacles` 改用 `polygon_vertices_[dob.poly_verts_index]` 而非 `polygon_vertices_[i]`
4. 同时修复 C++ 端的 POLYGON 和 LINESTRING 两个分支

**注意事项**：
- Circle/rect 动态障碍物不在 `polygon_vertices_` 中存储，它们的 `poly_verts_index` 应设为特殊值（如 `SIZE_MAX`）并添加防护
- `polygon_vertices_` deque 保证 push_back 后已有元素的指针有效，所以存储 size_t 索引而非指针

**验证方式**：
```bash
PYTHONPATH= python3 -c "
from irsim_core import SimWorld
w = SimWorld()
# 先添加静态 polygon，再添加动态 polygon，然后 step
w.add_obstacle({'type':'polygon','vertices':[[0,0],[1,0],[1,1]]})
w.add_dynamic_polygon_obstacle(0, 5,5,0, [[4,4],[6,4],[6,6],[4,6]])
w.step_dynamic_obstacles(np.array([0.5,0,0], dtype=np.float32), 3)
# 检查碰撞几何是否随动态障碍物移动（而非卡在初始位置）
"
```

---

#### #4 旋转矩形碰撞退化为 AABB

**位置**：`collision.cpp:187-189`（`check_rect_rect`），`collision.cpp:177-184`（`check_circle_rect`），`collision.cpp:212`（`needs_sat`）

**根因**：`check_rect_rect` 和 `check_circle_rect` 假设矩形为轴对齐。而 `needs_sat()` 只检查 `POLYGON || LINESTRING`，不检查旋转 RECT。

**修复**：
1. 在 `needs_sat()` 中增加条件：`|| (t == ShapeType::RECT && std::abs(obs.theta) > 1e-6f)` （注意 `needs_sat` 是单个参数的 lambda，需要分别对 a 和 b 判断）
2. 修改 `needs_sat` 为接受两个 Obstacle 的函数，或改为内联条件
3. 在 `to_convex_shapes` 中 RECT 已有分支（`obs_to_quad` → 4 点 quad），但未考虑旋转。需要增加旋转 RECT 的 4 点世界坐标计算：`rect_center ± rotate(half_w, half_h, theta)`

**预演代码**（`collision.cpp:212` 附近）：
```cpp
auto needs_sat = [](const Obstacle& a, const Obstacle& b) -> bool {
    auto check_one = [](ShapeType t, float theta) {
        return t == ShapeType::POLYGON || t == ShapeType::LINESTRING ||
               (t == ShapeType::RECT && std::abs(theta) > 1e-6f);
    };
    return check_one(a.type, a.theta) || check_one(b.type, b.theta);
};
// 调用处改为 needs_sat(a, b)
```

**验证方式**：
```bash
PYTHONPATH= python3 -c "
from irsim_core import SimWorld
import numpy as np
w = SimWorld()
w.add_obstacle({'type':'rect','x':0,'y':0,'half_w':1,'half_h':2})
# 添加一个 45° 旋转的矩形，两者应碰撞但 axis-aligned 检查会漏
w.add_dynamic_rect_obstacle(0, 0.5, 0, 0.785, 1, 2)  # rotated 45°
w.step(np.array([0,0,0], dtype=np.float32), 3)
# 它们重叠，collision 应为 True
"
```

---

#### #5 未检测 Robot×Robot 碰撞

**位置**：`world.cpp:341-382`

**根因**：`detect_collisions` 有 robot×obstacle 和 obstacle×obstacle 检查，但无 robot×robot。

**修复**：
1. 在 `detect_collisions` 开头添加 robot×robot 碰撞检测循环
2. 每个 robot 的 `world_vertices` 已在上一步 `step()` 中计算好
3. 用 SAT 检测两个 robot 的 `world_vertices` 是否相交
4. 如果碰撞，两个 robot 都设置 `collision = True`

**预演代码**（`world.cpp:detect_collisions` 最前面添加）：
```cpp
// Robot vs robot collision
for (size_t ri = 0; ri < robots_.size(); ri++) {
    for (size_t rj = ri + 1; rj < robots_.size(); rj++) {
        if (sat_intersect(robots_[ri].world_vertices.data(),
                          (int)robots_[ri].world_vertices.size(),
                          robots_[rj].world_vertices.data(),
                          (int)robots_[rj].world_vertices.size())) {
            robots_[ri].collision = true;
            robots_[rj].collision = true;
        }
    }
}
```

**注意事项**：
- 需 `#include "collision.h"`（world.cpp 已包含）以访问 `sat_intersect`
- 此检查应放在 robot 碰撞复位（`r.collision = false`）之后、obstacle 检查之前

**验证方式**：
```bash
PYTHONPATH= python3 -c "
from irsim_core import SimWorld
import numpy as np
w = SimWorld()
r0 = w.add_robot(0, 0,0,0)  # origin
r1 = w.add_robot(0, 0.1,0,0)  # overlapping
w.set_robot_vertices(r0, np.array([-0.3,-0.3,0.3,-0.3,0.3,0.3,-0.3,0.3], dtype=np.float32))
w.set_robot_vertices(r1, np.array([-0.3,-0.3,0.3,-0.3,0.3,0.3,-0.3,0.3], dtype=np.float32))
w.step(np.array([0,0,0, 0,0,0], dtype=np.float32), 3)
assert w.check_robot_collision(0) == True
assert w.check_robot_collision(1) == True
"
```

---

### 中等 — 功能缺失或静默异常

#### #6 C++ LiDAR 忽略 noise=True

**位置**：`lidar2d.py:259-261`

**根因**：`_c_step` 直接 `self.range_data[:] = result`，未应用噪声。Python fallback 在 `calculate_range:468` 中 `rng.normal(0, self.std, ...)`。

**修复**：在 `_c_step` 的 result 赋值后添加噪声处理（参照 FMCW 的 `_c_step:208-220`）：
```python
if ranges is not None and len(ranges) == self.number:
    self.range_data[:] = ranges
    if self.noise:
        for i in range(self.number):
            if self.range_data[i] < self.range_max:  # only noise hits
                self.range_data[i] += rng.normal(0, self.std)
    return True
```

**注意事项**：
- 仅对被命中的光束加噪声（未命中保持 `range_max`），与 Python 路径一致
- FMCW LiDAR 已有正确的噪声处理作为参考

**验证方式**：
```bash
PYTHONPATH= python3 -m pytest tests/test_fmcw_lidar2d.py::test_range_and_velocity_noise -v
# 再写一个针对 base Lidar2D 的 noise test
```

---

#### #7 C++ LiDAR 忽略 has_velocity=True

**位置**：`lidar2d.py:259-261`

**根因**：C++ 路径未调用 `calculate_range_vel`，速度数组保持全零。

**修复**：该功能设计与 FMCW LiDAR 的 `radial_velocity` 重叠。推荐在 C++ 路径中**不实现 `has_velocity`**，而是引导用户用 `fmcw_lidar2d` 替代。做法：
1. 在 `_c_step` 开头检查 `self.has_velocity`，若为 True 则 return False（强制走 Python fallback）
2. 文档中注明：需要速度信息请用 `fmcw_lidar2d`

**替代方案**（如果要实现）：在 `_c_step` 内调用 `calculate_range_vel(intersect_index)`，但需要障碍物命中信息——当前 C++ 只返回 ranges，不返回命中对象。要么扩展 C++ 接口，要么不做。

**验证方式**：确认 `fmcw_lidar2d` 的 `radial_velocity` 能覆盖此需求即可。

---

#### #8 `_geometry_valid` 无条件设为 True

**位置**：`env_base.py:704, 742`

**根因**：sync 中两处 `py_obj._geometry_valid = True`，跳过了 `shapely.is_valid()`。

**修复**：
1. 在 `_sync_cpp_to_python` 的几何更新后，调用 `py_obj._geometry_valid = shapely.is_valid(py_obj._geometry)` 替代硬编码 True
2. 确保 `import shapely` 已在文件顶部

**预演**：
```python
py_obj._geometry = py_obj.gf.step(py_obj.state)
py_obj._geometry_valid = shapely.is_valid(py_obj._geometry)  # was: True
```

**注意事项**：Shapely 的 `is_valid` 对于简单几何体（圆/矩形的 polygon 近似）永远是 True，性能开销可忽略。对用户自定义的复杂 polygon 是必要的校验。

---

#### #9 `post_process()` 从未被调用

**位置**：`env_base.py:561-659` vs `object_base.py:535`

**根因**：`_cpp_step` 在 sync 和 sensor step 后直接调用 `_status_step`，没有 `post_process` 的调用点。

**修复**：在 `_sync_cpp_to_python` 结束后、`_status_step` 之前，遍历所有 robots 和 dynamic obstacles 调用 `post_process()`：
```python
self._sync_cpp_to_python()
self._objects_sensor_step()
# Add post_process (mirrors obj.step())
for obj in self.objects:
    if not obj.static and hasattr(obj, 'post_process'):
        obj.post_process()
self._status_step()
self._world.step()
```

**注意事项**：当前 `post_process` 是 no-op，所以这个修复不会改变行为，但预防了未来的静默回归。

---

#### #10 动态 linestring 障碍物是死代码

**位置**：`env_base.py:623-627`

**根因**：`_cpp_step` 的 G2 白名单 `("circle", "rectangle", "polygon")` 不含 `"linestring"`。

**修复**：
1. 在白名单中添加 `"linestring"`
2. 确认 `_add_dynamic_linestring_obstacle_to_cpp` 在 `_build_cpp_world` 中被正确调用（line 450-451，已存在）
3. 确认 C++ 的 `step_dynamic_obstacles` 中 LINESTRING 顶点同步已实现（`world.cpp:269-279`，已存在）

**预演**：
```python
if shape not in ("circle", "rectangle", "polygon", "linestring"):
```

**注意事项**：
- 需配合修复 #3（`polygon_vertices_` 索引），否则 linestring 的碰撞几何会引用错误数据
- Linestring 仅支持平移，不支持旋转（local_linestring_verts 的同步逻辑只加了 dx/dy 没有旋转）——这在当前范围内可接受，因为 linestring 的典型用例是平移运动

**验证方式**：
```bash
PYTHONPATH= python3 -c "
from irsim_core import SimWorld
import numpy as np
w = SimWorld()
w.add_dynamic_linestring_obstacle(1, 5,0,0, [[4,-1],[6,1]], [-1,-1],[1,1],[1,1])
w.step_dynamic_obstacles(np.array([-0.5, 0.2, 0], dtype=np.float32), 3)
px,py,pt = w.get_obstacle_pose(0)
assert px < 5.0  # moved left
"
```

---

#### #11 障碍物 pre_process() 在 G1 时被跳过

**位置**：`env_base.py:617-621`

**根因**：G1（stop_flag 或 check_arrive）直接 `continue`，不调用 `pre_process()`。Python 路径的无条件调用允许 wander/loop 在到达后更新目标。

**修复**：在 G1 的零速度赋值之前，仍然调用 `obj.pre_process()`：
```python
if getattr(obj, "stop_flag", False) or obj.check_arrive(obj.goal):
    obj.pre_process()  # allow wander/loop to renew goal even when stopped
    w.set_obstacle_velocity(did, 0.0, 0.0)
    obj._velocity = np.zeros(obj.vel_shape)
    obs_act_list.extend([0.0, 0.0, 0.0])
    continue
```

**注意事项**：
- `pre_process()` 在 `arrive_flag=True` 时会更换目标并清除 `arrive_flag`，使下次 `check_arrive` 返回 False，障碍物恢复运动
- 如果 `stop_flag=True`（碰撞停止），`pre_process()` 可能会更换 wander 目标——但障碍物应该保持在停止状态直到碰撞解除。碰撞解除由 `reset()` 处理，在此场景下 wander 更换目标后障碍物被 G1 拦截是合理行为

---

#### #12 RECT AABB 不处理旋转

**位置**：`geometry.h:78-80`

**根因**：`compute_aabb()` 对 RECT 只做 axis-aligned 扩展，未考虑 `theta`。

**修复**：对 `theta ≠ 0` 的 RECT，计算旋转后的四个角点的 AABB：
```cpp
} else if (type == ShapeType::RECT) {
    if (std::abs(theta) < 1e-6f) {
        aabb = AABB{center - Vec2{half_w, half_h}, center + Vec2{half_w, half_h}};
    } else {
        float c = std::cos(theta), s = std::sin(theta);
        Vec2 corners[4] = {
            center + Vec2{ half_w*c - half_h*s,  half_w*s + half_h*c},
            center + Vec2{-half_w*c - half_h*s, -half_w*s + half_h*c},
            center + Vec2{ half_w*c + half_h*s,  half_w*s - half_h*c},
            center + Vec2{-half_w*c + half_h*s, -half_w*s - half_h*c}
        };
        aabb = AABB();
        for (auto& p : corners) aabb.expand(p);
    }
}
```

**注意事项**：这个修复与 #4 互补——#4 确保碰撞检测路由到 SAT，#12 确保 AABB 预过滤正确。

---

### 低 — 设计缺陷或潜在风险

#### #13 用户速度绕过 Python acc 裁剪

**位置**：`env_base.py:582-591`

**修复**：在 `obj.gen_behavior_vel` 分支外，对用户直接传入的 `action[obj._id]` 也应用 `get_vel_range()` 裁剪：
```python
a = action[obj._id] if obj._id < len(action) else None
if a is not None:
    a = np.asarray(a).ravel()
    # Clip user action same as gen_behavior_vel does
    min_vel, max_vel = obj.get_vel_range()
    a = np.clip(a.reshape(-1, 1), min_vel, max_vel).ravel()
```

---

#### #14 Linestring 碰撞厚度硬编码 0.05

**位置**：`collision.cpp:147, 243`

**修复**：在 `Obstacle` 结构体中添加 `float linestring_half_thickness = 0.05f` 字段。在 pybind 的 `py_to_obstacle` 中从 dict 读取可选字段 `"thickness"`。Python 侧 `_obj_to_c_dict` 中传入。

---

#### #15 `_map_to_c_dicts` 参数 `downsample_m` 死代码

**位置**：`lidar2d.py:317`

**修复**：删除未使用的参数，或实现下采样逻辑。推荐保留为未来接口但暂时标记 `# TODO`。

---

#### #16 `_map_to_c_dicts` 占用阈值硬编码 50

**位置**：`lidar2d.py:340`

**修复**：将 `> 50` 改为 `> 0`（与 Python 路径中 `ObstacleMap` 的 `grid_map > 0` 检查对齐）。Python 路径用 `grid_map > 0` 判断占据，C++ 用 `> 50` 是不必要的中间值。

---

#### #17 `point_in_polygon` 函数名误导

**位置**：`geometry.h:316-324`

**修复**：重命名为 `point_in_convex_polygon`，并在函数内部加 `assert(is_convex_polygon(...))`（仅 debug build）。检查调用点是否仅用于凸多边形。

---

#### #18 `alloca` 无限增长风险

**位置**：`collision.cpp:35`

**修复**：在 `sat_intersect` 入口处添加顶点数上限检查：
```cpp
if (n_a > 256 || n_b > 256) return false;  // degenerate input, skip SAT
```
256 顶点远大于正常 polygon（通常 3-8 顶点）。

---

#### #19 pybind omega 默认限幅 ±1

**位置**：`pybind_module.cpp`（多处 `add_dynamic_*` 和 `add_robot` lambdas）

**修复**：将默认值改为 `{-inf, -inf, -inf}` / `{inf, inf, inf}`，与 Python 侧 padding 对齐：
```cpp
float vmin[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
float vmax[3] = { FLT_MAX,  FLT_MAX,  FLT_MAX};
float vacc[3] = { FLT_MAX,  FLT_MAX,  FLT_MAX};
```

**注意事项**：`FLT_MAX` 替代 `inf`（C++ 中 `std::numeric_limits<float>::max()` 或直接 `3.402823466e+38F`）。同时需确保 `vel_min[j]` 和 `vel_max[j]` 的裁剪逻辑能正确处理 `FLT_MAX` 极值。

---

#### #20 矩形 `_obj_to_c_dict` 回退尺寸硬编码

**位置**：`lidar2d.py:286-287`

**修复**：从 `gf.length` / `gf.width` 读取真实尺寸：
```python
else:
    length = float(getattr(gf, 'length', 1.0))
    width = float(getattr(gf, 'width', 1.0))
    d = {"type": "rect", "x": ..., "half_w": length/2, "half_h": width/2}
```

---

#### #21 机器人顶点回退使用 world-frame

**位置**：`env_base.py:316-327`

**修复**：当 `original_vertices` 不存在时，不 fallback 到 `vertices`（可能是 world-frame），而是保持 C++ 默认的 0.32×0.24 矩形。并在日志中 warning。

---

#### #22 `_obj_to_c_dict` 同时传 world 坐标 + position

**位置**：`lidar2d.py:280, 292, 298`

**分析**：当前 C++ 对 POLYGON/LINESTRING 类型忽略 `x, y`，只用 vertices。对 RECT 类型使用 `x, y`。这不是 bug，但有混淆风险。

**修复**：在 `_obj_to_c_dict` 中对 polygon/linestring 传入 `x=0, y=0`（明确表示不使用），并在 C++ 侧 `py_to_obstacle` 的 POLYGON/LINESTRING 分支中不读取 `x, y`。对 RECT 保持现状。

---

## 修复优先级建议

**第一轮**（每次修改 C++ 后执行 `python3 setup.py build_ext --inplace`）：
1. #1 动态 polygon 重复注册 — 1 行修复
2. #12 RECT AABB 旋转 — 10 行修复
3. #4 旋转矩碰撞退化为 AABB — 15 行修复
4. #5 Robot×Robot 碰撞 — 10 行修复

**第二轮**：
5. #3 `polygon_vertices_` 索引 — 需改动 C++ struct + 多处引用
6. #2 索引映射假设 — 需改动 Python 侧多处
7. #6 C++ LiDAR noise — 5 行修复
8. #7 has_velocity — 2 行修复（引导用 fmcw）

**第三轮**：
9-22 剩余低优先级项目，可穿插进行

每轮结束后运行：
```bash
PYTHONPATH= python3 -m pytest tests/ -q \
  --ignore=tests/test_all_objects_3d.py --ignore=tests/test_all_objects.py
```
确认 845 tests 仍通过。

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

# SIMD Multi-Environment Batch Simulation — Implementation Plan

## 1. Motivation

Current irsim-x runs **one environment per SimWorld instance**. To run N environments,
Python code forks N processes (SubprocVecEnv), each holding its own C++ SimWorld.
This limits parallelism due to:

- Fork overhead (process count, memory duplication)
- Pipe/IPC serialization cost
- No cross-environment instruction-level parallelism (SIMD)

The paper achieves 4096 environments via **single-process, multi-threaded, SIMD-batched
simulation**: all environments share one C++ state array in SoA (Structure-of-Arrays)
layout, processed with AVX2 and OpenMP.

## 2. API Design

```python
# Single environment (backward compatible)
env = irsim.make("config.yaml")
lidar = env.get_lidar_scan()    # shape: (1200,)
state = env.get_robot_state()   # shape: (3, 1)

# Batch environment — same methods, extra batch dim
env = irsim.make("config.yaml", batch_size=16)
lidar = env.get_lidar_scan()    # shape: (16, 1200)
state = env.get_robot_state()   # shape: (16, 3, 1)
actions = policy(lidar, state)  # (16, 3)
lidar_next, reward, done, info = env.step(actions)
```

**Principle**: Every public method keeps its name and keyword argument order.
The only difference is the leading batch dimension in tensor shapes.

## 3. Internal Dispatch

```
irsim.make(yaml, batch_size=1)
    │
    ├── batch_size == 1
    │   └── EnvBase → CppSim → SimWorld  (existing path, 8-beam AVX2 LiDAR)
    │       No regression; zero changes to existing code.
    │
    └── batch_size > 1
        └── BatchEnvBase → BatchCppSim → BatchSimWorld
            SoA layout, OpenMP + SIMD for all operations.
            Same Python API shapes: (batch_size, ...) vs (...).
```

Rationale for **unified BatchSimWorld** for all batch_size > 1:

| Argument | Explanation |
|----------|-------------|
| Kinematics SIMD | Cross-environment SIMD benefits kinematics even at small N |
| Cache efficiency | SoA layout is more cache-friendly than AoS `vector<RobotState>` |
| Code simplicity | One code path vs special-casing N < SIMD_WIDTH |
| No IPC overhead | Single-process OpenMP threading, zero serialization |

The cost: when batch_size < SIMD_WIDTH, some SIMD lanes are idle. This is
acceptable because:
- The cost is bound by `ceil(N / SIMD_WIDTH)` iterations
- Kinematics acceleration + cache efficiency offset the idle lanes
- For a clean API, code simplicity outweighs micro-optimizing small N

## 4. SIMD Width Constants

Defined at compile time based on detected instruction set:

```cpp
// cpp/include/simd_config.h  (new file)
#if defined(__AVX512F__)
    constexpr int SIMD_WIDTH = 16;   // AVX-512: 512-bit / 32-bit float
#elif defined(__AVX2__)
    constexpr int SIMD_WIDTH = 8;    // AVX2:   256-bit / 32-bit float
#else
    constexpr int SIMD_WIDTH = 1;    // Scalar fallback
#endif
```

At runtime, `BatchSimWorld` allocates its SoA arrays in chunks of `SIMD_WIDTH`,
padding the last chunk if `batch_size % SIMD_WIDTH != 0`.

## 5. Memory Layout: SoA Transformation

### Current (AoS — Array of Structs)

```cpp
struct RobotState {
    float x, y, theta;      // pose
    float vx, vy, omega;    // velocity
    // ... vertices, limits, flags
};  // ~132 bytes per robot

std::vector<RobotState> robots_;
```

Layout: `[R0_x | R0_y | R0_θ | R0_vx | ... | R132 | R1_x | R1_y | ...]`
— `x` values of different robots are 132 bytes apart. No SIMD-friendly access.

### Target (SoA — Structure of Arrays)

```cpp
class BatchSimWorld {
    int batch_size_;
    int alloc_size_;  // padded to SIMD_WIDTH boundary

    // SoA core — each vector is SIMD_WIDTH-aligned, length = alloc_size_
    alignas(32) std::vector<float> x_, y_, theta_;         // pose
    alignas(32) std::vector<float> vx_, vy_, omega_;       // velocity
    alignas(32) std::vector<float> sin_theta_, cos_theta_; // precomputed
    alignas(32) std::vector<float> vel_min_[3], vel_max_[3], vel_acc_[3];

    // Per-robot (not vectorizable — different shapes/counts)
    std::vector<KinematicsType> kin_types_;
    std::vector<std::vector<Vec2>> local_vertices_;
    std::vector<std::vector<Vec2>> world_vertices_;

    // Obstacles — shared or per-environment
    std::vector<std::vector<Obstacle>> per_env_obstacles_;
};
```

Layout: `[x0 | x1 | x2 | x3 | x4 | x5 | x6 | x7 | y0 | y1 | ...]`
— one `_mm256_load_ps` loads 8 environments' `x` values from contiguous memory.

Memory for 4096 environments: ~30–50 MB total (state ~540 KB, obstacles
~6–10 MB, LiDAR output buffer ~19.6 MB).

## 6. SIMD Kernels

### 6.1 Clipping (velocity + acceleration bounds)

```cpp
void batch_clip_actions(float* actions, int env_id_start, int n) {
    for (int i = 0; i < n; i += SIMD_WIDTH) {
        __m256 a = _mm256_load_ps(actions + i);
        __m256 vmin = _mm256_load_ps(vel_min_[dim] + i);
        __m256 vmax = _mm256_load_ps(vel_max_[dim] + i);
        __m256 acc_lo = _mm256_load_ps(cur_vel + i) - _mm256_load_ps(acc + i) * dt_;
        __m256 acc_hi = _mm256_load_ps(cur_vel + i) + _mm256_load_ps(acc + i) * dt_;
        vmin = _mm256_max_ps(vmin, acc_lo);
        vmax = _mm256_min_ps(vmax, acc_hi);
        a = _mm256_min_ps(_mm256_max_ps(a, vmin), vmax);
        _mm256_store_ps(actions + i, a);
    }
}
```

### 6.2 Kinematics (omni_angular — holonomic with yaw)

```cpp
void batch_step_omni_angular(int env_start, int n, float dt) {
    for (int i = 0; i < n; i += SIMD_WIDTH) {
        __m256 _x  = _mm256_load_ps(x_ + env_start + i);
        __m256 _y  = _mm256_load_ps(y_ + env_start + i);
        __m256 _t  = _mm256_load_ps(theta_ + env_start + i);
        __m256 _vx = _mm256_load_ps(vx_ + env_start + i);
        __m256 _vy = _mm256_load_ps(vy_ + env_start + i);
        __m256 _om = _mm256_load_ps(omega_ + env_start + i);
        __m256 _dt = _mm256_set1_ps(dt);

        // Rotate velocity to world frame
        __m256 ct, st;
        sincos_ps(_t, &st, &ct);  // SVML intrinsic or polynomial approx

        __m256 wx = _mm256_sub_ps(_mm256_mul_ps(_vx, ct),
                                  _mm256_mul_ps(_vy, st));
        __m256 wy = _mm256_add_ps(_mm256_mul_ps(_vx, st),
                                  _mm256_mul_ps(_vy, ct));

        _x = _mm256_add_ps(_x, _mm256_mul_ps(wx, _dt));
        _y = _mm256_add_ps(_y, _mm256_mul_ps(wy, _dt));
        _t = _mm256_add_ps(_t, _mm256_mul_ps(_om, _dt));

        _mm256_store_ps(x_ + env_start + i, _x);
        _mm256_store_ps(y_ + env_start + i, _y);
        _mm256_store_ps(theta_ + env_start + i, _t);
    }
}
```

**sincos_ps**: Use Intel SVML (`-lsvml`), or a 6th-order polynomial approximation
for platforms without SVML.

### 6.3 LiDAR Raycasting — Cross-Environment SIMD

**Core idea**: For each beam angle, process that beam across `SIMD_WIDTH`
environments simultaneously, instead of processing `SIMD_WIDTH` beams for one
environment.

```cpp
void batch_lidar_raycast_avx2(
    const float* cos_angle, const float* sin_angle,  // beam direction (same for all envs)
    int batch_n, float range_max, float* ranges_out)
{
    float r_max[8];
    _mm256_store_ps(r_max, _mm256_set1_ps(range_max));
    __m256 best = _mm256_set1_ps(range_max);

    // For each obstacle shared across this chunk of environments...
    for (const auto& obs : per_env_obstacles_[0]) {
        if (obs.type == CIRCLE) {
            // Load 8 environments' robot positions
            __m256 ox = _mm256_load_ps(x_ + start);
            __m256 oy = _mm256_load_ps(y_ + start);

            // Ray-circle intersection for all 8 simultaneously
            __m256 dx = _mm256_sub_ps(ox, _mm256_set1_ps(obs.cx));
            __m256 dy = _mm256_sub_ps(oy, _mm256_set1_ps(obs.cy));
            __m256 b  = _mm256_mul_ps(dx, cos_ang) + _mm256_mul_ps(dy, sin_ang);
            __m256 c  = _mm256_mul_ps(dx, dx) + _mm256_mul_ps(dy, dy)
                      - _mm256_set1_ps(obs.r * obs.r);
            __m256 disc = _mm256_sub_ps(_mm256_mul_ps(b, b), c);
            // ... select valid hits, update best
        }
        // Similar for rect, polygon (fallback to scalar for polygons)
    }
    _mm256_store_ps(ranges_out + start, best);
}
```

Outer loops:

```
for beam_idx in 0..n_beams:
    cos_angle = cos(angle_min + beam_idx * angle_increment)
    sin_angle = sin(angle_min + beam_idx * angle_increment)
    for env_start in 0..batch_size step SIMD_WIDTH:
        batch_lidar_raycast_avx2(cos, sin, env_start,
                                 min(SIMD_WIDTH, batch_size - env_start),
                                 range_max, ranges + beam_idx * batch_size)
```

### 6.4 Collision Detection

SAT-based collision detection is harder to vectorize across environments because
vertex counts vary. Strategy:

1. **AABB pre-filter** (SIMD-friendly): Check aligned bounding boxes for all
   robots against all obstacles — simple min/max comparisons, vectorizable.
2. **SAT fallback** (scalar): Run full SAT only for robot-obstacle pairs that
   pass the AABB filter. This is a minority of pairs in practice.

```cpp
void batch_aabb_filter(int env_start, int n, const Obstacle& obs, bool* results) {
    alignas(32) float min_x[SIMD_WIDTH], max_x[SIMD_WIDTH];
    // Compute AABB from world_vertices_ for this batch
    // Compare with obs's AABB — 8-wide min/max comparisons
    for (int i = 0; i < n; i += SIMD_WIDTH) {
        __m256 o_min_x = _mm256_set1_ps(obs.aabb.min_x);
        __m256 o_max_x = _mm256_set1_ps(obs.aabb.max_x);
        __m256 r_min_x = _mm256_load_ps(robot_min_x + i);
        __m256 r_max_x = _mm256_load_ps(robot_max_x + i);
        __m256 overlap = _mm256_and_ps(
            _mm256_cmp_ps(r_max_x, o_min_x, _CMP_GE_OQ),
            _mm256_cmp_ps(o_max_x, r_min_x, _CMP_GE_OQ));
        // store results, fall back to scalar SAT for overlapping pairs
    }
}
```

## 7. Per-Environment Obstacles

Each environment in the batch has **different** procedurally generated obstacles.
`BatchSimWorld` stores one obstacle list per environment:

```cpp
class BatchSimWorld {
    // ...
    // Each environment has its own obstacle set (different map layout).
    // Length = batch_size (some may share the same obstacle set for efficiency).
    std::vector<ObstacleSet> env_obstacles_;

    // During OpenMP stepping, each thread operates on its assigned envs'
    // obstacle sets independently — no lock contention.
};
```

Where `ObstacleSet` is a lightweight view:

```cpp
struct ObstacleSet {
    const Obstacle* data;    // pointer into a pooled array
    int count;
};
```

Allocation strategy:
- Total obstacles across all environments: `sum(env_obstacles_.size())`
- Packed into one contiguous array with offset indices
- Thread-local iteration during collision detection

---

## 8. Action / Observation Memory Layout

### Actions Input (Interleaved — matches current C++ SimWorld convention)

```
Memory:  [env0_vx | env0_vy | env0_ω | env1_vx | env1_vy | env1_ω | ... ]
Index:   [0            1        2         3         4         5        ... ]
```

Action `i` for environment `e` is at `actions[e * action_dim + i]`.
This layout is what pybind11's `numpy → float*` already provides.

### LiDAR Output (SoA — cross-environment friendly)

```
Memory:  [beam0_env0 | beam0_env1 | ... | beam0_envN  | beam1_env0 | ... ]
Purpose: contiguous across envs for the same beam → SIMD load
```

This allows the cross-environment raycast to process 8 environments per beam
with a single `_mm256_load_ps`.

### Poses / Velocities Output (Interleaved — efficient Python slicing)

```
Poses:   [env0_x | env0_y | env0_θ | env1_x | env1_y | env1_θ | ... ]
         → Python reshapes to (batch_size, 3) then indexes envs individually.
```

---

## 9. Bulk State Sync (C++ → Python)

Instead of calling Python functions per-environment (which would be slow for
4096 envs), `BatchSimWorld` provides bulk getters:

```cpp
// C++ API
void get_all_poses(float* out) const;     // out length = batch_size × 3
void get_all_velocities(float* out) const; // out length = batch_size × 3
void get_all_collisions(bool* out) const;  // out length = batch_size
```

Python side (`_batch_cpp_sim.py`):

```python
self._w.step(act_array, 3)
poses = self._w.get_all_poses().reshape(-1, 3)   # (batch, 3)
collisions = self._w.get_all_collisions()          # (batch,)

for i in range(self._batch):
    # Direct numpy index into the bulk arrays — no per-env C++ round-trip
    env_list[i]._state[:2, 0] = poses[i, :2]
    env_list[i]._state[2, 0] = poses[i, 2]
    env_list[i].collision_flag = bool(collisions[i])
```

This avoids calling C++ 4096 times per step — instead, one bulk copy.

---

## 10. Thread Safety

Each OpenMP thread needs thread-local scratch buffers for operations that
cannot use the SoA arrays directly (e.g., vertex transformation output for
arbitrary-polygon collision shapes).

```cpp
void BatchSimWorld::step(...) {
    #pragma omp parallel
    {
        // Thread-local buffer for world-space vertices
        // Size = max vertices per robot in this chunk
        auto local_verts = std::vector<Vec2>(max_verts_per_robot_);

        #pragma omp for
        for (int chunk = 0; chunk < batch_size; chunk += SIMD_WIDTH) {
            // Use local_verts safely — each thread has its own copy
            batch_find_world_vertices(chunk, SIMD_WIDTH, local_verts);
            ...
        }
    }
}
```

**No mutex/lock needed** — all SoA array accesses are either read-only or
write to disjoint index ranges determined by the chunk boundaries.

---

## 11. Seed Handling

Each environment in the batch needs an independent random sequence for
procedural map generation:

```python
# Python side
def make(yaml_path, batch_size=1, seeds=None):
    if batch_size > 1:
        if seeds is None:
            seeds = range(batch_size)  # default: 0, 1, 2, ...
        # Pass seed to each env's generator during BatchSimWorld.build()
```

The seeds are used by the Python generators (Sparse/Maze/Graph/WFC) to produce
different maps per environment. The C++ BatchSimWorld receives obstacle data
from Python (built during `build()`) and does not need to generate maps itself.

---

## 12. BatchEnvBase Interface

```python
class BatchEnvBase:
    def __init__(self, yaml_path, batch_size, seed=None):
        self._envs = [EnvBase(yaml_path, display=False) for _ in range(batch_size)]
        # Later replaced with BatchCppSim for the C++ bridge
        self._batch_cpp = BatchCppSim(self._envs)
        self._batch_size = batch_size

    def step(self, action):
        # action shape: (batch_size, action_dim) or (action_dim,) for batch_size=1
        action = np.asarray(action, dtype=np.float32)
        if action.ndim == 1:
            action = action[np.newaxis, :]  # (3,) → (1, 3)
        assert action.shape[0] == self._batch_size

        self._batch_cpp.step(action)  # C++ bulk step
        self._batch_cpp._sync()       # C++ → Python state copy

        # Return per-env results as batched numpy arrays
        lidar = self.get_lidar_scan()     # (batch, 1200)
        reward = self._compute_rewards()  # (batch,)
        done = self._check_done()         # (batch,) bool
        return lidar, reward, done, {}

    def get_lidar_scan(self):
        # Returns (batch, 1200) — each row is one env's LiDAR
        return self._batch_cpp.batch_raycast(...)

    def get_robot_state(self):
        # Returns (batch, 3, 1) — each row is [x, y, theta]
        poses = self._batch_cpp.get_all_poses()
        return poses.reshape(self._batch_size, 3, 1)
```

All methods mirror `EnvBase` but with an extra leading batch dimension.
When `batch_size == 1`, any caller expecting the old shape `(1200,)` can
still index `lidar[0]` to get single-env data.

---

## 7. Multi-Threading with OpenMP

```
BatchSimWorld::step(actions):
    │
    #pragma omp parallel for num_threads(omp_get_max_threads())
    for (int chunk = 0; chunk < batch_size; chunk += CHUNK_SIZE) {
        int n = min(CHUNK_SIZE, batch_size - chunk);
        // Each thread processes its chunk:
        batch_clip_actions(actions, chunk, n);
        batch_step_kinematics(chunk, n);
        batch_find_world_vertices(chunk, n);  // transform local→world
        batch_aabb_filter(chunk, n);
        batch_collision_scalar(chunk, n);      // SAT fallback for filtered
    }

    #pragma omp parallel for
    for (int beam = 0; beam < n_beams; beam += BEAM_BATCH) {
        // Each thread processes a batch of beams across all environments
        for (int env = 0; env < batch_size; env += SIMD_WIDTH) {
            batch_lidar_raycast_avx2(beam, env, ...);
        }
    }
```

**CHUNK_SIZE**: `batch_size / omp_get_max_threads()`, rounded up to SIMD_WIDTH.
For 4096 envs on 14 cores: 4096/14 ≈ 293 → next SIMD_WIDTH multiple = 296
environments per thread.

**BEAM_BATCH**: 4–8 beams per thread, balancing load across LiDAR iterations.

## 13. File Changes — Zero Changes to Existing Code

| File | Action | Lines |
|------|--------|-------|
| `cpp/include/simd_config.h` | **New** | ~20 |
| `cpp/include/batch_world.h` | **New** | ~150 |
| `cpp/src/batch_world.cpp` | **New** | ~400 |
| `cpp/src/batch_kinematics.cpp` | **New** | ~150 |
| `cpp/src/batch_lidar.cpp` | **New** | ~200 |
| `cpp/src/batch_collision.cpp` | **New** | ~200 |
| `cpp/bindings/pybind_module.cpp` | **Append** ~50 lines at end | +50 |
| `irsim/env/_batch_cpp_sim.py` | **New** | ~200 |
| `irsim/env/batch_env_base.py` | **New** | ~100 |
| `irsim/__init__.py` | **Append** `batch_size` parameter to `make()` | +15 |
| `setup.py` | **Append** new .cpp files to Extension sources | +5 |

**Unchanged files** (zero lines modified):
- `cpp/include/world.h`
- `cpp/src/world.cpp`
- `cpp/src/kinematics.cpp`
- `cpp/src/lidar.cpp`
- `cpp/src/collision.cpp`
- `irsim/env/_cpp_sim.py`
- `irsim/env/env_base.py`

## 14. Throughput Estimate

Test: 14-core Xeon, AVX2, 4096 environments, Sparse generator, omni_angular robot,
1200-beam LiDAR, step_time=0.1s.

| Component | Current (1 env) | Batch 4096 (projected) | Speedup |
|-----------|----------------|----------------------|---------|
| Kinematics | ~0.08ms | ~0.01ms/env | ~8x |
| LiDAR | ~0.70ms | ~0.09ms/env | ~8x |
| Collision | ~0.40ms | ~0.20ms/env | ~2x |
| **Total step** | **~1.30ms** | **~0.30ms/env** | **~4x** |

**Batch throughput**: 4096 / 0.30ms per batch step ≈ **13.6M env-steps/sec**
**with 14 threads**.

Wall-clock: 100M steps → 100M / 13.6M ≈ **7.4 seconds** per 100M steps.
In practice (shared obstacles differ per env), this might be 30–60 seconds.

Compare to current fork approach: 14 envs at 900 steps/sec = 15.4 hours for 100M
steps. The batch approach is **800–1800x faster**.

## 15. Implementation Order

| Step | What | Depends on |
|------|------|------------|
| 1 | `simd_config.h` — compile-time SIMD width detection | Nothing |
| 2 | `batch_world.h` + `batch_world.cpp` — SoA layout, memory management | Step 1 |
| 3 | `batch_kinematics.cpp` — SIMD kinematics step functions | Step 2 |
| 4 | `batch_lidar.cpp` — cross-environment SIMD LiDAR | Step 2 |
| 5 | `batch_collision.cpp` — AABB filter + scalar SAT | Step 2 |
| 6 | `pybind_module.cpp` — BatchSimWorld bindings | Steps 2–5 |
| 7 | `_batch_cpp_sim.py` — Python bridge | Step 6 |
| 8 | `batch_env_base.py` — BatchEnvBase class | Step 7 |
| 9 | `irsim/__init__.py` — `make()` batch_size parameter | Step 8 |
| 10 | `setup.py` — add new .cpp files | All C++ steps |

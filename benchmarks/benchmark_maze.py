#!/usr/bin/env python3
"""Maze benchmark — measure step time vs maze complexity."""

import sys
import os
import time
import tempfile

import numpy as np

import irsim

sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../reproduction/paper_reproduction"))
from generators.maze_generator import MazeGenerator  # noqa: E402

HERE = os.path.dirname(__file__)
YAML = os.path.join(HERE, "maze_benchmark.yaml")


def build_env(grid_size, wall_thickness=0.1):
    gen = MazeGenerator(
        world_size=(10, 10),
        spacing=(0.6, 1.0),
        grid_size=grid_size,
        wall_thickness=wall_thickness,
        seed=42,
    )
    env = irsim.make(YAML, display=False)
    obstacles = gen.generate(env)
    return env, obstacles


def benchmark(grid_size, steps=300, wall_thickness=0.1):
    env, obs = build_env(grid_size, wall_thickness)

    total_segments = 0
    for o in env.objects:
        if hasattr(o, "vertices") and o.vertices is not None:
            total_segments += o.vertices.shape[1] - 1

    print(f"  grid_size={grid_size}, obstacles={len(obs)}, "
          f"linestring_segments={total_segments}")

    action = np.array([[0.3, 0.0, 0.0]], dtype=np.float32)
    for _ in range(30):
        env.step(action)

    # Full env.step() timing
    t_full = []
    for _ in range(steps):
        t0 = time.perf_counter()
        env.step(action)
        t_full.append((time.perf_counter() - t0) * 1000)
    env.end(0)

    return {
        "grid_size": grid_size,
        "obstacles": len(obs),
        "segments": total_segments,
        "full_ms": np.median(t_full),
        "fps": 1000.0 / np.median(t_full),
    }


def main():
    print("=" * 70)
    print("Maze benchmark (env.step, SpatialHashGrid enabled)")
    print("=" * 70)

    all_results = []
    for gs in [6, 8, 10, 12]:
        result = benchmark(gs, steps=300)
        all_results.append(result)
        print(f"  full={result['full_ms']:.3f}ms  FPS={result['fps']:.0f}")
        print()

    # Standalone LiDAR benchmark (C++ only)
    print("-" * 70)
    print("Standalone LiDAR (C++ lidar_raycast, 1200 beams)")
    print("-" * 70)
    from cpp import lidar_raycast
    for gs in [6, 8, 10, 12]:
        env, obs = build_env(gs)
        segs = 0
        for o in env.objects:
            if hasattr(o, "vertices") and o.vertices is not None:
                segs += o.vertices.shape[1] - 1
        obs_dicts = []
        for obj in env.objects:
            verts = getattr(obj, "vertices", None)
            if verts is not None and verts.shape[1] >= 2:
                vlist = [[float(verts[0, i]), float(verts[1, i])]
                         for i in range(verts.shape[1])]
                obs_dicts.append({"type": "linestring", "x": 0, "y": 0,
                                  "vertices": vlist})
        angles = np.linspace(-2.356, 2.356, 1200, dtype=np.float32)
        for _ in range(10):
            lidar_raycast(0, 0, 0, angles, 15.0, obs_dicts)
        t = []
        for _ in range(500):
            t0 = time.perf_counter()
            lidar_raycast(0, 0, 0, angles, 15.0, obs_dicts)
            t.append((time.perf_counter() - t0) * 1000)
        print(f"  {gs:>4}  segs={segs:<4}  LiDAR={np.median(t):.4f}ms")
        env.end(0)

    print()
    print("-" * 70)
    print(f"{'grid':>6}  {'obs':>4}  {'segments':>9}  {'full_step':>9}  {'FPS':>6}")
    print("-" * 70)
    for r in all_results:
        print(f"{r['grid_size']:>6}  {r['obstacles']:>4}  "
              f"{r['segments']:>9}  {r['full_ms']:>9.3f}  {r['fps']:>6.0f}")


if __name__ == "__main__":
    main()

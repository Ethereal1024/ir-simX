#!/usr/bin/env python3
"""Benchmark the 4 map types from the paper reproduction project."""

import sys
import os
import time

import numpy as np
import irsim

sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "../../reproduction/paper_reproduction"))
from generators.sparse_generator import SparseGenerator
from generators.maze_generator import MazeGenerator
from generators.graph_generator import GraphGenerator
from generators.wfc_generator import WFCGenerator

HERE = os.path.dirname(__file__)
YAML = os.path.join(HERE, "..", "..", "reproduction", "paper_reproduction",
                    "configs", "base_robot.yaml")


GENERATORS = {
    "Sparse": SparseGenerator(
        world_size=(20, 20),
        spacing=(0.5, 1.25),
        node_count=5,
        node_radius=0.25,
        seed=42,
        density=0.12,
    ),
    "Maze": MazeGenerator(
        world_size=(20, 20),
        spacing=(0.6, 1.25),
        node_count=5,
        node_radius=0.25,
        seed=42,
        wall_removal_rate=0.15,
        wall_thickness=0.08,
        grid_size=7,
    ),
    "Graph": GraphGenerator(
        world_size=(20, 20),
        spacing=(0.7, 1.25),
        node_count=5,
        node_radius=0.25,
        seed=42,
        point_removal_rate=0.2,
        edge_removal_rate=0.2,
        seed_point_count=12,
    ),
    "WFC_warehouse": WFCGenerator(
        world_size=(20, 20),
        spacing=(0.55, 1.0),
        node_count=5,
        node_radius=0.25,
        seed=42,
        preset="warehouse",
    ),
}


def count_obstacles(env):
    n_obs = 0
    n_segs = 0
    n_rects = 0
    n_circles = 0
    shapes = []
    for obj in env.objects:
        shape = getattr(obj, "shape", None)
        verts = getattr(obj, "vertices", None)
        if shape == "linestring" and verts is not None and verts.shape[1] >= 2:
            n_segs += verts.shape[1] - 1
            n_obs += 1
            shapes.append("L")
        elif shape == "polygon" and verts is not None and verts.shape[1] >= 3:
            n_segs += verts.shape[1]
            n_obs += 1
            shapes.append("P")
        elif shape == "rectangle":
            n_rects += 1
            n_obs += 1
            shapes.append("R")
        elif shape == "circle":
            n_circles += 1
            n_obs += 1
            shapes.append("C")
    return n_obs, n_segs, n_rects, n_circles, "".join(shapes)


def benchmark(gen, name, steps=300):
    print(f"  Generating {name}...", end=" ", flush=True)
    env = irsim.make(YAML, display=False)
    obstacles = gen.generate(env)
    n_obs, n_segs, n_rects, n_circles, shapes = count_obstacles(env)
    print(f"obs={n_obs} (C={n_circles} R={n_rects} S={n_segs} segs)")
    print(f"  shapes: {shapes[:80]}{'...' if len(shapes) > 80 else ''}")

    action = np.array([[0.3, 0.0, 0.0]], dtype=np.float32)
    for _ in range(30):
        env.step(action)

    t_full = []
    for _ in range(steps):
        t0 = time.perf_counter()
        env.step(action)
        t_full.append((time.perf_counter() - t0) * 1000)

    w = env._cpp._w
    for _ in range(10):
        w.step(np.array([0.3, 0.0, 0.0], dtype=np.float32), 3)
        env._objects_sensor_step()
    t_cpp = []
    for _ in range(steps):
        t0 = time.perf_counter()
        w.step(np.array([0.3, 0.0, 0.0], dtype=np.float32), 3)
        env._objects_sensor_step()
        t_cpp.append((time.perf_counter() - t0) * 1000)

    env.end(0)

    return {
        "name": name,
        "n_obs": n_obs,
        "n_segs": n_segs,
        "n_rects": n_rects,
        "n_circles": n_circles,
        "full_ms": np.median(t_full),
        "cpp_ms": np.median(t_cpp),
        "fps": 1000.0 / np.median(t_full),
    }


def main():
    print("=" * 80)
    print("Paper reproduction — 4 map types benchmark")
    print("=" * 80)

    results = []
    for name, gen in GENERATORS.items():
        result = benchmark(gen, name, steps=300)
        results.append(result)
        print(f"  env.step = {result['full_ms']:.3f}ms  "
              f"C++ = {result['cpp_ms']:.3f}ms  "
              f"FPS = {result['fps']:.0f}")
        print()

    print("-" * 80)
    print(f"{'Map':<16} {'Obs':>5} {'C':>4} {'R':>4} {'Segs':>5}  "
          f"{'env.step':>9} {'C++':>8} {'FPS':>6}")
    print("-" * 80)
    for r in results:
        print(f"{r['name']:<16} {r['n_obs']:>5} {r['n_circles']:>4} "
              f"{r['n_rects']:>4} {r['n_segs']:>5}  "
              f"{r['full_ms']:>9.3f} {r['cpp_ms']:>8.3f} {r['fps']:>6.0f}")


if __name__ == "__main__":
    main()

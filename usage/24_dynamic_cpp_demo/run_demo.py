"""Run C++-accelerated dynamic obstacle demos.

Usage:
    MPLBACKEND=TkAgg python usage/24_dynamic_cpp_demo/run_demo.py [circle|polygon|rect|all]

Each demo runs for 200 steps in a window, then prints C++ stats.
"""

import sys

import matplotlib

matplotlib.use("TkAgg")

DEMOS = {
    "circle": "circle_demo.yaml",
    "polygon": "polygon_demo.yaml",
    "rect": "rect_demo.yaml",
}


def run_demo(name: str, yaml_file: str):
    import irsim

    print(f"\n{'='*60}")
    print(f"  Dynamic obstacle demo: {name}")
    print(f"{'='*60}")

    env = irsim.make(yaml_file, save_ani=False)

    for i in range(200):
        env.step()
        env.render(0.01)
        if env.done():
            break

    # Print C++ acceleration stats
    w = getattr(env, "_cpp_world", None)
    if w is not None:
        print("  C++ SimWorld active:     YES")
        print(f"  Robots:                  {w.num_robots()}")
        print(f"  Dynamic obstacles:       {w.num_dynamic_obstacles()}")
        print(f"  Total obstacles:         {w.num_obstacles()}")
        for did in range(w.num_dynamic_obstacles()):
            px, py, pt = w.get_obstacle_pose(did)
            vx, vy, vo = w.get_obstacle_velocity(did)
            print(f"  Obstacle {did}: pos=({px:.2f}, {py:.2f}) vel=({vx:.2f}, {vy:.2f})")
    else:
        print("  C++ SimWorld:            NOT AVAILABLE")

    env.end(2)


if __name__ == "__main__":
    target = sys.argv[1] if len(sys.argv) > 1 else "all"

    if target == "all":
        for name, yaml_file in DEMOS.items():
            run_demo(name, f"usage/24_dynamic_cpp_demo/{yaml_file}")
    elif target in DEMOS:
        run_demo(target, f"usage/24_dynamic_cpp_demo/{DEMOS[target]}")
    else:
        print(f"Unknown demo: {target}. Choose from: {list(DEMOS.keys()) + ['all']}")

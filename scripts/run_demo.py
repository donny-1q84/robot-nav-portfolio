from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import List, Tuple

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from navsim.control import PurePursuitParams
from navsim.map import GridMap, demo_grid
from navsim.planner import astar
from navsim.sim import SimParams, simulate_path
from navsim.viz import plot_scene, render_gif


def parse_point(text: str) -> Tuple[int, int]:
    parts = text.split(",")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("Point must be in x,y format.")
    return int(parts[0]), int(parts[1])


def grid_to_path(plan: List[Tuple[int, int]]) -> List[Tuple[float, float]]:
    # Center path points in each grid cell.
    return [(x, y) for x, y in plan]


def run_demo(
    grid: GridMap,
    start: Tuple[int, int],
    goal: Tuple[int, int],
    out_png: Path,
    out_gif: Path | None,
    lookahead: float,
    speed: float,
) -> None:
    plan = astar(grid, start, goal)
    if plan is None:
        raise SystemExit("No path found for the given start/goal.")

    path = grid_to_path(plan.path)
    start_pose = (float(start[0]), float(start[1]), 0.0)
    poses = simulate_path(
        path,
        start_pose,
        SimParams(),
        PurePursuitParams(lookahead=lookahead, speed=speed),
    )

    plot_scene(grid, path, poses, start, goal, str(out_png))
    if out_gif is not None:
        render_gif(grid, path, poses, start, goal, str(out_gif))


def main() -> None:
    parser = argparse.ArgumentParser(description="2D robot navigation demo.")
    parser.add_argument("--start", type=parse_point, default="0,0")
    parser.add_argument("--goal", type=parse_point, default="9,9")
    parser.add_argument("--png", type=Path, default=Path("output.png"))
    parser.add_argument("--gif", type=Path, default=None)
    parser.add_argument("--lookahead", type=float, default=0.8)
    parser.add_argument("--speed", type=float, default=0.8)
    args = parser.parse_args()

    grid = demo_grid()
    run_demo(
        grid,
        start=args.start,
        goal=args.goal,
        out_png=args.png,
        out_gif=args.gif,
        lookahead=args.lookahead,
        speed=args.speed,
    )


if __name__ == "__main__":
    main()

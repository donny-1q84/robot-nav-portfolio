from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

import yaml

from navsim.control import PurePursuitParams
from navsim.costmap import CostMap
from navsim.collision import path_in_collision, trajectory_in_collision
from navsim.map import GridMap, demo_grid
from navsim.planner import astar
from navsim.sim import SimParams, simulate_path
from navsim.viz import plot_scene, render_gif


@dataclass
class DemoConfig:
    start: Tuple[int, int]
    goal: Tuple[int, int]
    output_png: Path
    output_gif: Path | None
    inflation_radius: float
    lookahead: float
    speed: float


def _parse_point(value: str) -> Tuple[int, int]:
    parts = value.split(",")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("Point must be in x,y format.")
    return int(parts[0]), int(parts[1])


def _load_config(path: Path) -> DemoConfig:
    data = yaml.safe_load(path.read_text()) or {}
    return DemoConfig(
        start=tuple(data.get("start", [0, 0])),
        goal=tuple(data.get("goal", [9, 9])),
        output_png=Path(data.get("output_png", "output.png")),
        output_gif=Path(data["output_gif"]) if data.get("output_gif") else None,
        inflation_radius=float(data.get("inflation_radius", 0.0)),
        lookahead=float(data.get("lookahead", 0.8)),
        speed=float(data.get("speed", 0.8)),
    )


def _grid_to_path(plan: list[Tuple[int, int]]) -> list[Tuple[float, float]]:
    return [(x, y) for x, y in plan]


def run_demo(
    grid: GridMap,
    cfg: DemoConfig,
    out_png: Path | None = None,
    out_gif: Path | None = None,
) -> None:
    costmap = CostMap.from_grid(grid, cfg.inflation_radius)
    plan_map = costmap.inflated_map()
    plan = astar(plan_map, cfg.start, cfg.goal)
    if plan is None:
        raise SystemExit("No path found for the given start/goal.")

    path = _grid_to_path(plan.path)
    start_pose = (float(cfg.start[0]), float(cfg.start[1]), 0.0)
    poses = simulate_path(
        path,
        start_pose,
        SimParams(),
        PurePursuitParams(lookahead=cfg.lookahead, speed=cfg.speed),
    )

    png_path = out_png if out_png is not None else cfg.output_png
    if path_in_collision(costmap, path):
        print("Warning: planned path intersects inflated obstacles.")
    if trajectory_in_collision(costmap, poses):
        print("Warning: trajectory intersects inflated obstacles.")

    plot_scene(
        grid,
        path,
        poses,
        cfg.start,
        cfg.goal,
        str(png_path),
        display_grid=costmap.inflated,
    )
    if out_gif is not None:
        render_gif(
            grid,
            path,
            poses,
            cfg.start,
            cfg.goal,
            str(out_gif),
            display_grid=costmap.inflated,
        )
    elif cfg.output_gif is not None:
        render_gif(
            grid,
            path,
            poses,
            cfg.start,
            cfg.goal,
            str(cfg.output_gif),
            display_grid=costmap.inflated,
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="2D robot navigation demo.")
    parser.add_argument("--config", type=Path, default=Path("configs/default.yaml"))
    parser.add_argument("--start", type=_parse_point, default=None)
    parser.add_argument("--goal", type=_parse_point, default=None)
    parser.add_argument("--png", type=Path, default=None)
    parser.add_argument("--gif", type=Path, default=None)
    parser.add_argument("--inflation-radius", type=float, default=None)
    parser.add_argument("--lookahead", type=float, default=None)
    parser.add_argument("--speed", type=float, default=None)
    args = parser.parse_args()

    cfg = _load_config(args.config)
    if args.start is not None:
        cfg.start = args.start
    if args.goal is not None:
        cfg.goal = args.goal
    if args.png is not None:
        cfg.output_png = args.png
    if args.gif is not None:
        cfg.output_gif = args.gif
    if args.inflation_radius is not None:
        cfg.inflation_radius = args.inflation_radius
    if args.lookahead is not None:
        cfg.lookahead = args.lookahead
    if args.speed is not None:
        cfg.speed = args.speed

    grid = demo_grid()
    run_demo(grid, cfg, out_png=args.png, out_gif=args.gif)


if __name__ == "__main__":
    main()

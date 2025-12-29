from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple

import yaml

from navsim.collision import path_in_collision, trajectory_in_collision
from navsim.control import PurePursuitParams
from navsim.costmap import CostMap
from navsim.dynamic import DynamicObstacle, DynamicObstacleField
from navsim.local_planner import DWAParams
from navsim.localization import LocalizationParams
from navsim.map import GridMap, demo_grid
from navsim.planner import astar
from navsim.sensors import SensorNoise
from navsim.sim import (
    SimParams,
    simulate_dwa,
    simulate_dwa_dynamic,
    simulate_dwa_dynamic_localized,
    simulate_dwa_localized,
    simulate_path,
    simulate_path_localized,
)
from navsim.viz import plot_scene, render_gif


@dataclass
class DemoConfig:
    start: Tuple[int, int]
    goal: Tuple[int, int]
    output_png: Path
    output_gif: Path | None
    inflation_radius: float
    local_planner: str
    dwa: DWAParams
    dynamic_enabled: bool
    dynamic_replan_interval: int
    dynamic_max_replans: int
    dynamic_obstacles: List[DynamicObstacle]
    localization_enabled: bool
    localization: LocalizationParams
    lookahead: float
    speed: float


def _parse_point(value: str) -> Tuple[int, int]:
    parts = value.split(",")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("Point must be in x,y format.")
    return int(parts[0]), int(parts[1])


def _load_config(path: Path) -> DemoConfig:
    data = yaml.safe_load(path.read_text()) or {}
    dwa_cfg = data.get("dwa", {}) or {}
    dyn_cfg = data.get("dynamic_obstacles", {}) or {}
    loc_cfg = data.get("localization", {}) or {}
    noise_cfg = loc_cfg.get("noise", {}) or {}
    obstacles: List[DynamicObstacle] = []
    for obstacle in dyn_cfg.get("obstacles", []) or []:
        position = obstacle.get("position", [0.0, 0.0])
        velocity = obstacle.get("velocity", [0.0, 0.0])
        obstacles.append(
            DynamicObstacle(
                x=float(position[0]),
                y=float(position[1]),
                vx=float(velocity[0]),
                vy=float(velocity[1]),
            )
        )
    return DemoConfig(
        start=tuple(data.get("start", [0, 0])),
        goal=tuple(data.get("goal", [9, 9])),
        output_png=Path(data.get("output_png", "output.png")),
        output_gif=Path(data["output_gif"]) if data.get("output_gif") else None,
        inflation_radius=float(data.get("inflation_radius", 0.0)),
        local_planner=str(data.get("local_planner", "dwa")),
        dwa=DWAParams(
            v_min=float(dwa_cfg.get("v_min", 0.0)),
            v_max=float(dwa_cfg.get("v_max", 1.0)),
            omega_max=float(dwa_cfg.get("omega_max", 2.0)),
            v_samples=int(dwa_cfg.get("v_samples", 5)),
            omega_samples=int(dwa_cfg.get("omega_samples", 11)),
            horizon=float(dwa_cfg.get("horizon", 1.5)),
            goal_weight=float(dwa_cfg.get("goal_weight", 1.0)),
            path_weight=float(dwa_cfg.get("path_weight", 0.4)),
            clearance_weight=float(dwa_cfg.get("clearance_weight", 0.2)),
        ),
        dynamic_enabled=bool(dyn_cfg.get("enabled", False)),
        dynamic_replan_interval=int(dyn_cfg.get("replan_interval", 10)),
        dynamic_max_replans=int(dyn_cfg.get("max_replans", 50)),
        dynamic_obstacles=obstacles,
        localization_enabled=bool(loc_cfg.get("enabled", False)),
        localization=LocalizationParams(
            noise=SensorNoise(
                odom_std_v=float(noise_cfg.get("odom_std_v", 0.05)),
                odom_std_omega=float(noise_cfg.get("odom_std_omega", 0.05)),
                meas_std_x=float(noise_cfg.get("meas_std_x", 0.2)),
                meas_std_y=float(noise_cfg.get("meas_std_y", 0.2)),
            ),
            init_cov=float(loc_cfg.get("init_cov", 0.5)),
            seed=int(loc_cfg.get("seed", 0)),
        ),
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
    dynamic_field = None
    dynamic_cells = None
    if cfg.dynamic_enabled and cfg.dynamic_obstacles:
        dynamic_field = DynamicObstacleField(cfg.dynamic_obstacles)
        dynamic_cells = dynamic_field.cells(grid)

    costmap = CostMap.from_grid(
        grid, cfg.inflation_radius, occupied=dynamic_cells
    )
    plan_map = costmap.inflated_map()
    plan = astar(plan_map, cfg.start, cfg.goal)
    if plan is None:
        raise SystemExit("No path found for the given start/goal.")

    path = _grid_to_path(plan.path)
    start_pose = (float(cfg.start[0]), float(cfg.start[1]), 0.0)
    poses: list[Tuple[float, float, float]] = []
    est_poses: list[Tuple[float, float, float]] | None = None

    if cfg.localization_enabled:
        if cfg.dynamic_enabled and dynamic_field is not None:
            if cfg.local_planner != "dwa":
                print("Warning: dynamic obstacles require DWA; switching to DWA.")
            poses, est_poses, path = simulate_dwa_dynamic_localized(
                path,
                start_pose,
                SimParams(),
                grid,
                cfg.inflation_radius,
                dynamic_field,
                cfg.dwa,
                cfg.goal,
                cfg.dynamic_replan_interval,
                cfg.dynamic_max_replans,
                cfg.localization,
            )
            costmap = CostMap.from_grid(
                grid,
                cfg.inflation_radius,
                occupied=dynamic_field.cells(grid),
            )
        elif cfg.local_planner == "dwa":
            poses, est_poses = simulate_dwa_localized(
                path,
                start_pose,
                SimParams(),
                costmap,
                cfg.dwa,
                cfg.localization,
            )
        else:
            poses, est_poses = simulate_path_localized(
                path,
                start_pose,
                SimParams(),
                PurePursuitParams(lookahead=cfg.lookahead, speed=cfg.speed),
                cfg.localization,
            )
    else:
        if cfg.dynamic_enabled and dynamic_field is not None:
            if cfg.local_planner != "dwa":
                print("Warning: dynamic obstacles require DWA; switching to DWA.")
            poses, path = simulate_dwa_dynamic(
                path,
                start_pose,
                SimParams(),
                grid,
                cfg.inflation_radius,
                dynamic_field,
                cfg.dwa,
                cfg.goal,
                cfg.dynamic_replan_interval,
                cfg.dynamic_max_replans,
            )
            costmap = CostMap.from_grid(
                grid,
                cfg.inflation_radius,
                occupied=dynamic_field.cells(grid),
            )
        elif cfg.local_planner == "dwa":
            poses = simulate_dwa(path, start_pose, SimParams(), costmap, cfg.dwa)
        else:
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
        est_poses=est_poses,
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
            est_poses=est_poses,
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
            est_poses=est_poses,
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="2D robot navigation demo.")
    parser.add_argument("--config", type=Path, default=Path("configs/default.yaml"))
    parser.add_argument("--start", type=_parse_point, default=None)
    parser.add_argument("--goal", type=_parse_point, default=None)
    parser.add_argument("--png", type=Path, default=None)
    parser.add_argument("--gif", type=Path, default=None)
    parser.add_argument("--inflation-radius", type=float, default=None)
    parser.add_argument(
        "--local-planner",
        choices=["pure_pursuit", "dwa"],
        default=None,
    )
    parser.add_argument(
        "--dynamic",
        dest="dynamic_enabled",
        action="store_true",
        default=None,
    )
    parser.add_argument(
        "--no-dynamic",
        dest="dynamic_enabled",
        action="store_false",
        default=None,
    )
    parser.add_argument("--replan-interval", type=int, default=None)
    parser.add_argument("--max-replans", type=int, default=None)
    parser.add_argument(
        "--localization",
        dest="localization_enabled",
        action="store_true",
        default=None,
    )
    parser.add_argument(
        "--no-localization",
        dest="localization_enabled",
        action="store_false",
        default=None,
    )
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
    if args.local_planner is not None:
        cfg.local_planner = args.local_planner
    if args.dynamic_enabled is not None:
        cfg.dynamic_enabled = args.dynamic_enabled
    if args.replan_interval is not None:
        cfg.dynamic_replan_interval = args.replan_interval
    if args.max_replans is not None:
        cfg.dynamic_max_replans = args.max_replans
    if args.localization_enabled is not None:
        cfg.localization_enabled = args.localization_enabled
    if args.lookahead is not None:
        cfg.lookahead = args.lookahead
    if args.speed is not None:
        cfg.speed = args.speed

    grid = demo_grid()
    run_demo(grid, cfg, out_png=args.png, out_gif=args.gif)


if __name__ == "__main__":
    main()

from __future__ import annotations

import argparse
import csv
import random
import statistics
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple

import yaml

from navsim.collision import trajectory_in_collision
from navsim.control import PurePursuitParams
from navsim.costmap import CostMap
from navsim.local_planner import DWAParams
from navsim.map import GridMap, demo_grid
from navsim.metrics import final_distance, goal_reached, path_length, trajectory_length
from navsim.planner import astar
from navsim.sim import SimParams, simulate_dwa, simulate_path

Node = Tuple[int, int]
Point = Tuple[float, float]


@dataclass
class BenchmarkConfig:
    inflation_radius: float
    local_planner: str
    lookahead: float
    speed: float
    dwa: DWAParams


def _load_config(path: Path) -> BenchmarkConfig:
    data = yaml.safe_load(path.read_text()) or {}
    dwa_cfg = data.get("dwa", {}) or {}
    return BenchmarkConfig(
        inflation_radius=float(data.get("inflation_radius", 0.0)),
        local_planner=str(data.get("local_planner", "dwa")),
        lookahead=float(data.get("lookahead", 0.8)),
        speed=float(data.get("speed", 0.8)),
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
    )


def _free_cells(costmap: CostMap) -> List[Node]:
    cells: List[Node] = []
    for y in range(costmap.height):
        for x in range(costmap.width):
            if not costmap.is_occupied((x, y)):
                cells.append((x, y))
    return cells


def _sample_start_goal(rng: random.Random, cells: List[Node]) -> Tuple[Node, Node]:
    if len(cells) < 2:
        raise ValueError("Not enough free cells to sample start/goal.")
    start = rng.choice(cells)
    goal = rng.choice(cells)
    while goal == start:
        goal = rng.choice(cells)
    return start, goal


def _grid_to_path(plan: List[Node]) -> List[Point]:
    return [(float(x), float(y)) for x, y in plan]


def run_trial(
    grid: GridMap,
    costmap: CostMap,
    cfg: BenchmarkConfig,
    rng: random.Random,
    sim_params: SimParams,
    free_cells: List[Node],
) -> dict:
    start, goal = _sample_start_goal(rng, free_cells)
    t0 = time.perf_counter()
    plan = astar(costmap.inflated_map(), start, goal)
    if plan is None:
        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        return {
            "start_x": start[0],
            "start_y": start[1],
            "goal_x": goal[0],
            "goal_y": goal[1],
            "plan_found": 0,
            "success": 0,
            "steps": 0,
            "path_length": 0.0,
            "traj_length": 0.0,
            "final_distance": float("inf"),
            "collision": 0,
            "elapsed_ms": elapsed_ms,
        }

    path = _grid_to_path(plan.path)
    start_pose = (float(start[0]), float(start[1]), 0.0)
    if cfg.local_planner == "dwa":
        poses = simulate_dwa(path, start_pose, sim_params, costmap, cfg.dwa)
    else:
        poses = simulate_path(
            path,
            start_pose,
            sim_params,
            PurePursuitParams(lookahead=cfg.lookahead, speed=cfg.speed),
        )
    elapsed_ms = (time.perf_counter() - t0) * 1000.0

    success = goal_reached(poses, (float(goal[0]), float(goal[1])), sim_params.goal_tolerance)
    collision = trajectory_in_collision(costmap, poses)
    return {
        "start_x": start[0],
        "start_y": start[1],
        "goal_x": goal[0],
        "goal_y": goal[1],
        "plan_found": 1,
        "success": int(success),
        "steps": max(0, len(poses) - 1),
        "path_length": path_length(path),
        "traj_length": trajectory_length(poses),
        "final_distance": final_distance(poses, (float(goal[0]), float(goal[1]))),
        "collision": int(collision),
        "elapsed_ms": elapsed_ms,
    }


def _summarize(rows: List[dict]) -> dict:
    trials = len(rows)
    plan_found = [row for row in rows if row["plan_found"] == 1]
    successes = [row for row in rows if row["success"] == 1]
    collisions = [row for row in rows if row["collision"] == 1]

    def mean(values: List[float]) -> float:
        return statistics.mean(values) if values else 0.0

    return {
        "trials": trials,
        "plan_success_rate": len(plan_found) / trials if trials else 0.0,
        "success_rate": len(successes) / trials if trials else 0.0,
        "collision_rate": len(collisions) / trials if trials else 0.0,
        "avg_steps": mean([row["steps"] for row in successes]),
        "avg_path_length": mean([row["path_length"] for row in successes]),
        "avg_traj_length": mean([row["traj_length"] for row in successes]),
        "avg_final_distance": mean([row["final_distance"] for row in rows]),
        "avg_elapsed_ms": mean([row["elapsed_ms"] for row in rows]),
    }


def _print_summary(summary: dict) -> None:
    print("Benchmark summary")
    print(f"Trials: {summary['trials']}")
    print(f"Plan success rate: {summary['plan_success_rate']:.2%}")
    print(f"Navigation success rate: {summary['success_rate']:.2%}")
    print(f"Collision rate: {summary['collision_rate']:.2%}")
    print(f"Avg steps (success): {summary['avg_steps']:.1f}")
    print(f"Avg path length (success): {summary['avg_path_length']:.2f}")
    print(f"Avg trajectory length (success): {summary['avg_traj_length']:.2f}")
    print(f"Avg final distance: {summary['avg_final_distance']:.2f}")
    print(f"Avg elapsed ms: {summary['avg_elapsed_ms']:.2f}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Run navsim benchmarks.")
    parser.add_argument("--config", type=Path, default=Path("configs/default.yaml"))
    parser.add_argument("--trials", type=int, default=30)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--csv", type=Path, default=Path("reports/benchmark.csv"))
    parser.add_argument(
        "--local-planner",
        choices=["pure_pursuit", "dwa"],
        default=None,
    )
    args = parser.parse_args()

    cfg = _load_config(args.config)
    if args.local_planner is not None:
        cfg.local_planner = args.local_planner

    grid = demo_grid()
    costmap = CostMap.from_grid(grid, cfg.inflation_radius)
    free_cells = _free_cells(costmap)
    rng = random.Random(args.seed)
    sim_params = SimParams()

    rows = []
    for _ in range(args.trials):
        rows.append(run_trial(grid, costmap, cfg, rng, sim_params, free_cells))

    if not rows:
        print("No trials to run.")
        return

    args.csv.parent.mkdir(parents=True, exist_ok=True)
    with args.csv.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    summary = _summarize(rows)
    _print_summary(summary)


if __name__ == "__main__":
    main()

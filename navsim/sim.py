from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple

from .collision import path_in_collision
from .control import PurePursuitParams, pure_pursuit_control
from .costmap import CostMap
from .dynamic import DynamicObstacleField
from .local_planner import DWAParams, dwa_control
from .map import GridMap
from .planner import astar


Pose = Tuple[float, float, float]
Point = Tuple[float, float]


@dataclass
class SimParams:
    dt: float = 0.1
    max_steps: int = 500
    goal_tolerance: float = 0.3


def simulate_path(
    path: List[Point],
    start_pose: Pose,
    params: SimParams,
    ctrl_params: PurePursuitParams,
) -> List[Pose]:
    poses: List[Pose] = [start_pose]
    target_idx = 0

    for _ in range(params.max_steps):
        x, y, yaw = poses[-1]
        gx, gy = path[-1]
        if math.hypot(gx - x, gy - y) <= params.goal_tolerance:
            break

        v, omega, target_idx = pure_pursuit_control(
            (x, y, yaw), path, ctrl_params, target_idx
        )

        x += v * math.cos(yaw) * params.dt
        y += v * math.sin(yaw) * params.dt
        yaw += omega * params.dt
        poses.append((x, y, yaw))

    return poses


def simulate_dwa(
    path: List[Point],
    start_pose: Pose,
    params: SimParams,
    costmap: CostMap,
    dwa_params: DWAParams,
) -> List[Pose]:
    poses: List[Pose] = [start_pose]
    stuck_steps = 0

    for _ in range(params.max_steps):
        x, y, yaw = poses[-1]
        gx, gy = path[-1]
        if math.hypot(gx - x, gy - y) <= params.goal_tolerance:
            break

        v, omega, _ = dwa_control((x, y, yaw), path, costmap, dwa_params, params.dt)
        if abs(v) < 1e-3 and abs(omega) < 1e-3:
            stuck_steps += 1
            if stuck_steps >= 10:
                break
        else:
            stuck_steps = 0

        x += v * math.cos(yaw) * params.dt
        y += v * math.sin(yaw) * params.dt
        yaw += omega * params.dt
        poses.append((x, y, yaw))

    return poses


def _grid_to_path(plan: List[Tuple[int, int]]) -> List[Point]:
    return [(float(x), float(y)) for x, y in plan]


def _pose_to_cell(pose: Pose) -> Tuple[int, int]:
    x, y, _ = pose
    return int(round(x)), int(round(y))


def simulate_dwa_dynamic(
    path: List[Point],
    start_pose: Pose,
    params: SimParams,
    base_grid: GridMap,
    inflation_radius: float,
    dynamic_field: DynamicObstacleField,
    dwa_params: DWAParams,
    goal: Tuple[int, int],
    replan_interval: int,
    max_replans: int,
) -> Tuple[List[Pose], List[Point]]:
    poses: List[Pose] = [start_pose]
    current_path = path
    stuck_steps = 0
    steps_since_replan = 0
    replans = 0

    for _ in range(params.max_steps):
        x, y, yaw = poses[-1]
        gx, gy = goal
        if math.hypot(gx - x, gy - y) <= params.goal_tolerance:
            break

        dynamic_field.step(params.dt, base_grid)
        costmap = CostMap.from_grid(
            base_grid,
            inflation_radius,
            occupied=dynamic_field.cells(base_grid),
        )

        needs_replan = False
        if replan_interval > 0 and steps_since_replan >= replan_interval:
            needs_replan = True
        if path_in_collision(costmap, current_path):
            needs_replan = True

        if needs_replan:
            start_cell = _pose_to_cell(poses[-1])
            plan = astar(costmap.inflated_map(), start_cell, goal)
            if plan is None:
                break
            current_path = _grid_to_path(plan.path)
            steps_since_replan = 0
            replans += 1
            if replans >= max_replans:
                break

        v, omega, _ = dwa_control(
            (x, y, yaw),
            current_path,
            costmap,
            dwa_params,
            params.dt,
        )
        if abs(v) < 1e-3 and abs(omega) < 1e-3:
            stuck_steps += 1
            if stuck_steps >= 10:
                steps_since_replan = replan_interval
        else:
            stuck_steps = 0

        x += v * math.cos(yaw) * params.dt
        y += v * math.sin(yaw) * params.dt
        yaw += omega * params.dt
        poses.append((x, y, yaw))
        steps_since_replan += 1

    return poses, current_path

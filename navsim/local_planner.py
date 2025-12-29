from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Tuple

from .collision import trajectory_in_collision
from .costmap import CostMap


Point = Tuple[float, float]
Pose = Tuple[float, float, float]


@dataclass
class DWAParams:
    v_min: float = 0.0
    v_max: float = 1.0
    omega_max: float = 2.0
    v_samples: int = 5
    omega_samples: int = 11
    horizon: float = 1.5
    goal_weight: float = 1.0
    path_weight: float = 0.4
    clearance_weight: float = 0.2


def _linspace(start: float, stop: float, num: int) -> List[float]:
    if num <= 1:
        return [start]
    step = (stop - start) / float(num - 1)
    return [start + i * step for i in range(num)]


def _simulate_trajectory(
    pose: Pose, v: float, omega: float, dt: float, horizon: float
) -> List[Pose]:
    steps = max(1, int(horizon / max(dt, 1e-3)))
    x, y, yaw = pose
    poses: List[Pose] = [(x, y, yaw)]
    for _ in range(steps):
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += omega * dt
        poses.append((x, y, yaw))
    return poses


def _min_distance(point: Point, obstacles: Iterable[Point]) -> float:
    if not obstacles:
        return float("inf")
    x, y = point
    return min(math.hypot(x - ox, y - oy) for ox, oy in obstacles)


def _trajectory_clearance(poses: Iterable[Pose], obstacles: Iterable[Point]) -> float:
    if not obstacles:
        return float("inf")
    min_dist = float("inf")
    for x, y, _ in poses:
        dist = _min_distance((x, y), obstacles)
        if dist < min_dist:
            min_dist = dist
    return min_dist


def _distance_to_path(point: Point, path: Iterable[Point]) -> float:
    x, y = point
    return min(math.hypot(x - px, y - py) for px, py in path)


def _obstacle_points(costmap: CostMap) -> List[Point]:
    obstacles: List[Point] = []
    for y, row in enumerate(costmap.inflated):
        for x, cell in enumerate(row):
            if cell == 1:
                obstacles.append((float(x), float(y)))
    return obstacles


def dwa_control(
    pose: Pose,
    path: List[Point],
    costmap: CostMap,
    params: DWAParams,
    dt: float,
) -> Tuple[float, float, List[Pose]]:
    goal = path[-1]
    obstacles = _obstacle_points(costmap)

    best_cost = float("inf")
    best_v = 0.0
    best_omega = 0.0
    best_traj: List[Pose] = [pose]

    v_samples = _linspace(params.v_min, params.v_max, params.v_samples)
    omega_samples = _linspace(-params.omega_max, params.omega_max, params.omega_samples)

    for v in v_samples:
        for omega in omega_samples:
            traj = _simulate_trajectory(pose, v, omega, dt, params.horizon)
            if trajectory_in_collision(costmap, traj):
                continue

            end_x, end_y, _ = traj[-1]
            goal_dist = math.hypot(goal[0] - end_x, goal[1] - end_y)
            path_dist = _distance_to_path((end_x, end_y), path)
            clearance = _trajectory_clearance(traj, obstacles)
            clearance_cost = 1.0 / max(clearance, 1e-3)

            cost = (
                params.goal_weight * goal_dist
                + params.path_weight * path_dist
                + params.clearance_weight * clearance_cost
            )

            if cost < best_cost:
                best_cost = cost
                best_v = v
                best_omega = omega
                best_traj = traj

    return best_v, best_omega, best_traj

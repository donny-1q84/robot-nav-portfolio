from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import List, Tuple

from .collision import path_in_collision
from .control import PurePursuitParams, pure_pursuit_control
from .costmap import CostMap, LocalCostmapParams
from .dynamic import DynamicObstacleField
from .local_planner import DWAParams, dwa_control
from .localization import EKF, LocalizationParams
from .map import GridMap
from .planner import plan_path
from .sensors import noisy_control, noisy_position

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


def _step_pose(pose: Pose, v: float, omega: float, dt: float) -> Pose:
    x, y, yaw = pose
    x += v * math.cos(yaw) * dt
    y += v * math.sin(yaw) * dt
    yaw += omega * dt
    return (x, y, yaw)


def simulate_path_localized(
    path: List[Point],
    start_pose: Pose,
    params: SimParams,
    ctrl_params: PurePursuitParams,
    loc_params: LocalizationParams,
) -> Tuple[List[Pose], List[Pose]]:
    rng = random.Random(loc_params.seed)
    ekf = EKF(start_pose, loc_params)
    true_poses: List[Pose] = [start_pose]
    est_poses: List[Pose] = [start_pose]
    target_idx = 0

    for _ in range(params.max_steps):
        est_pose = ekf.pose
        gx, gy = path[-1]
        if math.hypot(gx - est_pose[0], gy - est_pose[1]) <= params.goal_tolerance:
            break

        v, omega, target_idx = pure_pursuit_control(
            est_pose, path, ctrl_params, target_idx
        )

        true_pose = _step_pose(true_poses[-1], v, omega, params.dt)
        v_noisy, omega_noisy = noisy_control(v, omega, loc_params.noise, rng)
        ekf.predict(v_noisy, omega_noisy, params.dt)
        measurement = noisy_position(true_pose, loc_params.noise, rng)
        ekf.update(measurement)

        true_poses.append(true_pose)
        est_poses.append(ekf.pose)

    return true_poses, est_poses


def simulate_dwa(
    path: List[Point],
    start_pose: Pose,
    params: SimParams,
    costmap: CostMap,
    dwa_params: DWAParams,
    local_params: LocalCostmapParams | None = None,
) -> List[Pose]:
    poses: List[Pose] = [start_pose]
    stuck_steps = 0

    for _ in range(params.max_steps):
        x, y, yaw = poses[-1]
        gx, gy = path[-1]
        if math.hypot(gx - x, gy - y) <= params.goal_tolerance:
            break

        active_costmap = costmap
        if local_params and local_params.enabled:
            active_costmap = costmap.windowed(
                (x, y), local_params.radius, local_params.unknown_as_obstacle
            )
        v, omega, _ = dwa_control(
            (x, y, yaw), path, active_costmap, dwa_params, params.dt
        )
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


def simulate_dwa_localized(
    path: List[Point],
    start_pose: Pose,
    params: SimParams,
    costmap: CostMap,
    dwa_params: DWAParams,
    loc_params: LocalizationParams,
    local_params: LocalCostmapParams | None = None,
) -> Tuple[List[Pose], List[Pose]]:
    rng = random.Random(loc_params.seed)
    ekf = EKF(start_pose, loc_params)
    true_poses: List[Pose] = [start_pose]
    est_poses: List[Pose] = [start_pose]
    stuck_steps = 0

    for _ in range(params.max_steps):
        est_pose = ekf.pose
        gx, gy = path[-1]
        if math.hypot(gx - est_pose[0], gy - est_pose[1]) <= params.goal_tolerance:
            break

        active_costmap = costmap
        if local_params and local_params.enabled:
            active_costmap = costmap.windowed(
                (est_pose[0], est_pose[1]),
                local_params.radius,
                local_params.unknown_as_obstacle,
            )
        v, omega, _ = dwa_control(est_pose, path, active_costmap, dwa_params, params.dt)
        if abs(v) < 1e-3 and abs(omega) < 1e-3:
            stuck_steps += 1
            if stuck_steps >= 10:
                break
        else:
            stuck_steps = 0

        true_pose = _step_pose(true_poses[-1], v, omega, params.dt)
        v_noisy, omega_noisy = noisy_control(v, omega, loc_params.noise, rng)
        ekf.predict(v_noisy, omega_noisy, params.dt)
        measurement = noisy_position(true_pose, loc_params.noise, rng)
        ekf.update(measurement)

        true_poses.append(true_pose)
        est_poses.append(ekf.pose)

    return true_poses, est_poses


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
    global_planner: str,
    local_params: LocalCostmapParams | None = None,
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
        full_costmap = CostMap.from_grid(
            base_grid,
            inflation_radius,
            occupied=dynamic_field.cells(base_grid),
        )

        needs_replan = False
        if replan_interval > 0 and steps_since_replan >= replan_interval:
            needs_replan = True
        if path_in_collision(full_costmap, current_path):
            needs_replan = True

        if needs_replan:
            start_cell = _pose_to_cell(poses[-1])
            plan = plan_path(full_costmap.inflated_map(), start_cell, goal, global_planner)
            if plan is None:
                break
            current_path = _grid_to_path(plan.path)
            steps_since_replan = 0
            replans += 1
            if replans >= max_replans:
                break

        active_costmap = full_costmap
        if local_params and local_params.enabled:
            active_costmap = full_costmap.windowed(
                (x, y), local_params.radius, local_params.unknown_as_obstacle
            )
        v, omega, _ = dwa_control(
            (x, y, yaw),
            current_path,
            active_costmap,
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


def simulate_dwa_dynamic_localized(
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
    loc_params: LocalizationParams,
    global_planner: str,
    local_params: LocalCostmapParams | None = None,
) -> Tuple[List[Pose], List[Pose], List[Point]]:
    rng = random.Random(loc_params.seed)
    ekf = EKF(start_pose, loc_params)
    true_poses: List[Pose] = [start_pose]
    est_poses: List[Pose] = [start_pose]
    current_path = path
    stuck_steps = 0
    steps_since_replan = 0
    replans = 0

    for _ in range(params.max_steps):
        est_pose = ekf.pose
        gx, gy = goal
        if math.hypot(gx - est_pose[0], gy - est_pose[1]) <= params.goal_tolerance:
            break

        dynamic_field.step(params.dt, base_grid)
        full_costmap = CostMap.from_grid(
            base_grid,
            inflation_radius,
            occupied=dynamic_field.cells(base_grid),
        )

        needs_replan = False
        if replan_interval > 0 and steps_since_replan >= replan_interval:
            needs_replan = True
        if path_in_collision(full_costmap, current_path):
            needs_replan = True

        if needs_replan:
            start_cell = _pose_to_cell(est_pose)
            plan = plan_path(full_costmap.inflated_map(), start_cell, goal, global_planner)
            if plan is None:
                break
            current_path = _grid_to_path(plan.path)
            steps_since_replan = 0
            replans += 1
            if replans >= max_replans:
                break

        active_costmap = full_costmap
        if local_params and local_params.enabled:
            active_costmap = full_costmap.windowed(
                (est_pose[0], est_pose[1]),
                local_params.radius,
                local_params.unknown_as_obstacle,
            )
        v, omega, _ = dwa_control(
            est_pose, current_path, active_costmap, dwa_params, params.dt
        )
        if abs(v) < 1e-3 and abs(omega) < 1e-3:
            stuck_steps += 1
            if stuck_steps >= 10:
                steps_since_replan = replan_interval
        else:
            stuck_steps = 0

        true_pose = _step_pose(true_poses[-1], v, omega, params.dt)
        v_noisy, omega_noisy = noisy_control(v, omega, loc_params.noise, rng)
        ekf.predict(v_noisy, omega_noisy, params.dt)
        measurement = noisy_position(true_pose, loc_params.noise, rng)
        ekf.update(measurement)

        true_poses.append(true_pose)
        est_poses.append(ekf.pose)
        steps_since_replan += 1

    return true_poses, est_poses, current_path

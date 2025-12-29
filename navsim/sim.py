from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple

from .control import PurePursuitParams, pure_pursuit_control


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

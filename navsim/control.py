from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple

Point = Tuple[float, float]


@dataclass
class PurePursuitParams:
    lookahead: float = 0.8
    speed: float = 0.8
    max_omega: float = 2.0


def _wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def pure_pursuit_control(
    pose: Tuple[float, float, float],
    path: List[Point],
    params: PurePursuitParams,
    last_target_idx: int,
) -> Tuple[float, float, int]:
    x, y, yaw = pose

    # Find a target point ahead of the robot.
    target_idx = last_target_idx
    for i in range(last_target_idx, len(path)):
        px, py = path[i]
        if math.hypot(px - x, py - y) >= params.lookahead:
            target_idx = i
            break
    else:
        target_idx = len(path) - 1

    tx, ty = path[target_idx]
    angle_to_target = math.atan2(ty - y, tx - x)
    alpha = _wrap_angle(angle_to_target - yaw)

    # Unicycle model: curvature k = 2*sin(alpha)/L, use L=1
    curvature = 2.0 * math.sin(alpha) / max(params.lookahead, 1e-3)
    omega = max(-params.max_omega, min(params.max_omega, curvature * params.speed))
    return params.speed, omega, target_idx

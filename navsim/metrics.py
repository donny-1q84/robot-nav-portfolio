from __future__ import annotations

import math
from typing import Iterable, Tuple


Point = Tuple[float, float]
Pose = Tuple[float, float, float]


def _segment_length(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def path_length(path: Iterable[Point]) -> float:
    total = 0.0
    prev = None
    for point in path:
        if prev is not None:
            total += _segment_length(prev, point)
        prev = point
    return total


def trajectory_length(poses: Iterable[Pose]) -> float:
    total = 0.0
    prev = None
    for x, y, _ in poses:
        if prev is not None:
            total += _segment_length(prev, (x, y))
        prev = (x, y)
    return total


def goal_reached(poses: Iterable[Pose], goal: Point, tolerance: float) -> bool:
    last_pose = None
    for last_pose in poses:
        pass
    if last_pose is None:
        return False
    return _segment_length((last_pose[0], last_pose[1]), goal) <= tolerance


def final_distance(poses: Iterable[Pose], goal: Point) -> float:
    last_pose = None
    for last_pose in poses:
        pass
    if last_pose is None:
        return float("inf")
    return _segment_length((last_pose[0], last_pose[1]), goal)

from __future__ import annotations

from typing import Iterable, Tuple

from .costmap import CostMap

Point = Tuple[float, float]
Pose = Tuple[float, float, float]


def _point_to_cell(point: Point) -> Tuple[int, int]:
    x, y = point
    return int(round(x)), int(round(y))


def point_in_collision(costmap: CostMap, point: Point) -> bool:
    cell = _point_to_cell(point)
    if not costmap.in_bounds(cell):
        return True
    return costmap.is_occupied(cell)


def path_in_collision(costmap: CostMap, path: Iterable[Point]) -> bool:
    return any(point_in_collision(costmap, point) for point in path)


def trajectory_in_collision(costmap: CostMap, poses: Iterable[Pose]) -> bool:
    return any(point_in_collision(costmap, (x, y)) for x, y, _ in poses)

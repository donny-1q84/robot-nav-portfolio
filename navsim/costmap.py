from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Tuple

from .map import Grid, GridMap


Node = Tuple[int, int]


def _overlay_grid(grid: Grid, occupied: Iterable[Node]) -> Grid:
    height = len(grid)
    width = len(grid[0]) if height else 0
    overlaid = [row[:] for row in grid]
    for x, y in occupied:
        if 0 <= x < width and 0 <= y < height:
            overlaid[y][x] = 1
    return overlaid


def _inflate_grid(grid: Grid, radius: float) -> Grid:
    height = len(grid)
    width = len(grid[0]) if height else 0
    inflated = [row[:] for row in grid]
    if radius <= 0.0:
        return inflated

    rad = int(math.ceil(radius))
    radius_sq = radius * radius + 1e-9

    for y in range(height):
        for x in range(width):
            if grid[y][x] != 1:
                continue
            for dy in range(-rad, rad + 1):
                for dx in range(-rad, rad + 1):
                    if dx * dx + dy * dy > radius_sq:
                        continue
                    ny = y + dy
                    nx = x + dx
                    if 0 <= nx < width and 0 <= ny < height:
                        inflated[ny][nx] = 1
    return inflated


@dataclass(frozen=True)
class CostMap:
    base: GridMap
    inflated: Grid
    inflation_radius: float

    @classmethod
    def from_grid(
        cls,
        grid: GridMap,
        inflation_radius: float,
        occupied: Iterable[Node] | None = None,
    ) -> "CostMap":
        radius = max(0.0, float(inflation_radius))
        base_grid = _overlay_grid(grid.grid, occupied) if occupied else grid.grid
        inflated = _inflate_grid(base_grid, radius)
        return cls(base=grid, inflated=inflated, inflation_radius=radius)

    @property
    def height(self) -> int:
        return self.base.height

    @property
    def width(self) -> int:
        return self.base.width

    def in_bounds(self, node: Node) -> bool:
        return self.base.in_bounds(node)

    def is_occupied(self, node: Node) -> bool:
        x, y = node
        return self.inflated[y][x] == 1

    def inflated_map(self) -> GridMap:
        return GridMap(grid=self.inflated)

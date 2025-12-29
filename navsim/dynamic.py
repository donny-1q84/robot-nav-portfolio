from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

from .map import GridMap


Node = Tuple[int, int]


@dataclass
class DynamicObstacle:
    x: float
    y: float
    vx: float
    vy: float

    def step(self, dt: float, grid: GridMap) -> None:
        nx = self.x + self.vx * dt
        ny = self.y + self.vy * dt

        if nx < 0.0 or nx > grid.width - 1:
            self.vx *= -1.0
            nx = max(0.0, min(grid.width - 1, nx))
        if ny < 0.0 or ny > grid.height - 1:
            self.vy *= -1.0
            ny = max(0.0, min(grid.height - 1, ny))

        cell = (int(round(nx)), int(round(ny)))
        if grid.in_bounds(cell) and not grid.is_free(cell):
            self.vx *= -1.0
            self.vy *= -1.0
            nx = self.x
            ny = self.y

        self.x = nx
        self.y = ny

    def cell(self, grid: GridMap) -> Node | None:
        cell = (int(round(self.x)), int(round(self.y)))
        if grid.in_bounds(cell):
            return cell
        return None


@dataclass
class DynamicObstacleField:
    obstacles: List[DynamicObstacle]

    def step(self, dt: float, grid: GridMap) -> None:
        for obstacle in self.obstacles:
            obstacle.step(dt, grid)

    def cells(self, grid: GridMap) -> List[Node]:
        cells: List[Node] = []
        for obstacle in self.obstacles:
            cell = obstacle.cell(grid)
            if cell is not None:
                cells.append(cell)
        return cells

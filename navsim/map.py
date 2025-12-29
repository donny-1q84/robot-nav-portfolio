from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Tuple

Grid = list[list[int]]


@dataclass(frozen=True)
class GridMap:
    grid: Grid

    @property
    def height(self) -> int:
        return len(self.grid)

    @property
    def width(self) -> int:
        return len(self.grid[0]) if self.grid else 0

    def in_bounds(self, node: Tuple[int, int]) -> bool:
        x, y = node
        return 0 <= x < self.width and 0 <= y < self.height

    def is_free(self, node: Tuple[int, int]) -> bool:
        x, y = node
        return self.grid[y][x] == 0

    def neighbors(self, node: Tuple[int, int]) -> Iterable[Tuple[int, int]]:
        x, y = node
        candidates = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
        for nxt in candidates:
            if self.in_bounds(nxt) and self.is_free(nxt):
                yield nxt


def demo_grid() -> GridMap:
    # 0 free, 1 obstacle
    grid = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0, 0, 1, 1, 1, 0],
        [0, 0, 0, 1, 0, 0, 0, 0, 1, 0],
        [0, 1, 0, 1, 0, 1, 1, 0, 1, 0],
        [0, 1, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 1, 1, 1, 1, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 1, 1, 0],
        [0, 1, 1, 1, 0, 0, 0, 0, 1, 0],
        [0, 0, 0, 1, 0, 1, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
    ]
    return GridMap(grid=grid)

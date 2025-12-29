from __future__ import annotations

import heapq
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from .map import GridMap


Node = Tuple[int, int]


@dataclass
class PlanResult:
    path: List[Node]
    cost: float


def manhattan(a: Node, b: Node) -> float:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def reconstruct(came_from: Dict[Node, Node], start: Node, goal: Node) -> List[Node]:
    node = goal
    path = [node]
    while node != start:
        node = came_from[node]
        path.append(node)
    path.reverse()
    return path


def astar(grid: GridMap, start: Node, goal: Node) -> Optional[PlanResult]:
    if not grid.in_bounds(start) or not grid.in_bounds(goal):
        return None
    if not grid.is_free(start) or not grid.is_free(goal):
        return None

    open_heap: List[Tuple[float, Node]] = []
    heapq.heappush(open_heap, (0.0, start))
    came_from: Dict[Node, Node] = {}
    g_cost: Dict[Node, float] = {start: 0.0}

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current == goal:
            path = reconstruct(came_from, start, goal)
            return PlanResult(path=path, cost=g_cost[current])

        for nxt in grid.neighbors(current):
            tentative = g_cost[current] + 1.0
            if nxt not in g_cost or tentative < g_cost[nxt]:
                came_from[nxt] = current
                g_cost[nxt] = tentative
                f_cost = tentative + manhattan(nxt, goal)
                heapq.heappush(open_heap, (f_cost, nxt))

    return None

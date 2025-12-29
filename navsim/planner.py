from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

from .map import GridMap

Node = Tuple[int, int]


@dataclass
class PlanResult:
    path: List[Node]
    cost: float


def manhattan(a: Node, b: Node) -> float:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def euclidean(a: Node, b: Node) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _neighbors(grid: GridMap, node: Node, diagonal: bool = False) -> Iterable[Node]:
    x, y = node
    candidates = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
    if diagonal:
        candidates += [
            (x + 1, y + 1),
            (x + 1, y - 1),
            (x - 1, y + 1),
            (x - 1, y - 1),
        ]
    for nxt in candidates:
        if grid.in_bounds(nxt) and grid.is_free(nxt):
            yield nxt


def _bresenham(a: Node, b: Node) -> Iterable[Node]:
    x0, y0 = a
    x1, y1 = b
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    while True:
        yield (x0, y0)
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy


def line_of_sight(grid: GridMap, a: Node, b: Node) -> bool:
    for node in _bresenham(a, b):
        if not grid.in_bounds(node) or not grid.is_free(node):
            return False
    return True


def reconstruct(came_from: Dict[Node, Node], start: Node, goal: Node) -> List[Node]:
    node = goal
    path = [node]
    while node != start:
        node = came_from[node]
        path.append(node)
    path.reverse()
    return path


def astar(
    grid: GridMap,
    start: Node,
    goal: Node,
    heuristic=manhattan,
) -> Optional[PlanResult]:
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

        for nxt in _neighbors(grid, current):
            tentative = g_cost[current] + 1.0
            if nxt not in g_cost or tentative < g_cost[nxt]:
                came_from[nxt] = current
                g_cost[nxt] = tentative
                f_cost = tentative + heuristic(nxt, goal)
                heapq.heappush(open_heap, (f_cost, nxt))

    return None


def dijkstra(grid: GridMap, start: Node, goal: Node) -> Optional[PlanResult]:
    return astar(grid, start, goal, heuristic=lambda *_: 0.0)


def theta_star(grid: GridMap, start: Node, goal: Node) -> Optional[PlanResult]:
    if not grid.in_bounds(start) or not grid.in_bounds(goal):
        return None
    if not grid.is_free(start) or not grid.is_free(goal):
        return None

    open_heap: List[Tuple[float, Node]] = []
    heapq.heappush(open_heap, (0.0, start))
    parent: Dict[Node, Node] = {start: start}
    g_cost: Dict[Node, float] = {start: 0.0}

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current == goal:
            path = reconstruct(parent, start, goal)
            return PlanResult(path=path, cost=g_cost[current])

        for nxt in _neighbors(grid, current, diagonal=True):
            if nxt not in g_cost:
                g_cost[nxt] = float("inf")
                parent[nxt] = current

            if line_of_sight(grid, parent[current], nxt):
                tentative = g_cost[parent[current]] + euclidean(parent[current], nxt)
                if tentative < g_cost[nxt]:
                    parent[nxt] = parent[current]
                    g_cost[nxt] = tentative
                    f_cost = tentative + euclidean(nxt, goal)
                    heapq.heappush(open_heap, (f_cost, nxt))
            else:
                tentative = g_cost[current] + euclidean(current, nxt)
                if tentative < g_cost[nxt]:
                    parent[nxt] = current
                    g_cost[nxt] = tentative
                    f_cost = tentative + euclidean(nxt, goal)
                    heapq.heappush(open_heap, (f_cost, nxt))

    return None


def plan_path(
    grid: GridMap,
    start: Node,
    goal: Node,
    method: str = "astar",
) -> Optional[PlanResult]:
    method = method.lower()
    if method == "astar":
        return astar(grid, start, goal)
    if method == "dijkstra":
        return dijkstra(grid, start, goal)
    if method == "theta":
        return theta_star(grid, start, goal)
    raise ValueError(f"Unknown planner method: {method}")

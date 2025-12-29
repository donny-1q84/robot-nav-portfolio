from navsim.map import demo_grid
from navsim.planner import astar, dijkstra, plan_path, theta_star


def test_astar_finds_path():
    grid = demo_grid()
    start = (0, 0)
    goal = (9, 9)
    result = astar(grid, start, goal)
    assert result is not None
    assert result.path[0] == start
    assert result.path[-1] == goal


def test_astar_blocks_obstacles():
    grid = demo_grid()
    start = (1, 1)  # obstacle in demo grid
    goal = (9, 9)
    result = astar(grid, start, goal)
    assert result is None


def test_dijkstra_matches_astar_cost():
    grid = demo_grid()
    start = (0, 0)
    goal = (9, 9)
    a_star = astar(grid, start, goal)
    dijk = dijkstra(grid, start, goal)
    assert a_star is not None
    assert dijk is not None
    assert abs(a_star.cost - dijk.cost) < 1e-6


def test_theta_star_path_shorter_or_equal():
    grid = demo_grid()
    start = (0, 0)
    goal = (9, 9)
    a_star = astar(grid, start, goal)
    theta = theta_star(grid, start, goal)
    assert a_star is not None
    assert theta is not None
    assert theta.cost <= a_star.cost + 1e-6


def test_plan_path_dispatch():
    grid = demo_grid()
    start = (0, 0)
    goal = (9, 9)
    result = plan_path(grid, start, goal, method="dijkstra")
    assert result is not None

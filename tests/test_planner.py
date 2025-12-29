from navsim.map import demo_grid
from navsim.planner import astar


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

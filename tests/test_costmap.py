from navsim.costmap import CostMap
from navsim.map import GridMap


def test_inflation_marks_neighbors():
    grid = GridMap(
        [
            [0, 0, 0],
            [0, 1, 0],
            [0, 0, 0],
        ]
    )
    costmap = CostMap.from_grid(grid, 1.0)
    assert costmap.is_occupied((1, 1))
    assert costmap.is_occupied((1, 0))
    assert costmap.is_occupied((0, 1))
    assert not costmap.is_occupied((0, 0))


def test_zero_inflation_matches_grid():
    grid = GridMap(
        [
            [0, 0, 0],
            [0, 1, 0],
            [0, 0, 0],
        ]
    )
    costmap = CostMap.from_grid(grid, 0.0)
    assert costmap.is_occupied((1, 1))
    assert not costmap.is_occupied((1, 0))

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


def test_dynamic_overlay_marks_cells():
    grid = GridMap(
        [
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
        ]
    )
    costmap = CostMap.from_grid(grid, 0.0, occupied=[(0, 0), (2, 2)])
    assert costmap.is_occupied((0, 0))
    assert costmap.is_occupied((2, 2))


def test_windowed_masks_far_obstacles():
    grid = GridMap(
        [
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 1],
        ]
    )
    costmap = CostMap.from_grid(grid, 0.0)
    windowed = costmap.windowed((0.0, 0.0), radius=1.0)
    assert not windowed.is_occupied((2, 2))


def test_windowed_unknown_as_obstacle():
    grid = GridMap(
        [
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
        ]
    )
    costmap = CostMap.from_grid(grid, 0.0)
    windowed = costmap.windowed((1.0, 1.0), radius=0.5, unknown_as_obstacle=True)
    assert windowed.is_occupied((0, 0))

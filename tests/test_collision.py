from navsim.collision import path_in_collision, point_in_collision, trajectory_in_collision
from navsim.costmap import CostMap
from navsim.map import GridMap


def _make_costmap() -> CostMap:
    grid = GridMap(
        [
            [0, 0, 0],
            [0, 1, 0],
            [0, 0, 0],
        ]
    )
    return CostMap.from_grid(grid, 0.0)


def test_point_in_collision():
    costmap = _make_costmap()
    assert point_in_collision(costmap, (1.0, 1.0))
    assert not point_in_collision(costmap, (0.0, 0.0))
    assert point_in_collision(costmap, (-1.0, 0.0))


def test_path_in_collision():
    costmap = _make_costmap()
    path = [(0.0, 0.0), (1.0, 1.0)]
    assert path_in_collision(costmap, path)


def test_trajectory_in_collision():
    costmap = _make_costmap()
    poses = [(0.0, 0.0, 0.0), (1.0, 1.0, 0.0)]
    assert trajectory_in_collision(costmap, poses)

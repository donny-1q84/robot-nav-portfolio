from navsim.dynamic import DynamicObstacle, DynamicObstacleField
from navsim.map import GridMap


def test_dynamic_obstacle_bounces():
    grid = GridMap(
        [
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
        ]
    )
    obs = DynamicObstacle(x=2.0, y=1.0, vx=1.0, vy=0.0)
    obs.step(1.0, grid)
    assert 0.0 <= obs.x <= 2.0
    assert obs.vx < 0.0


def test_dynamic_field_cells_in_bounds():
    grid = GridMap(
        [
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
        ]
    )
    field = DynamicObstacleField(
        [DynamicObstacle(x=1.0, y=1.0, vx=0.0, vy=0.0)]
    )
    cells = field.cells(grid)
    assert cells == [(1, 1)]

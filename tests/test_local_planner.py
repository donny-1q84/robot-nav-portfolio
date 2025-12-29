from navsim.costmap import CostMap
from navsim.local_planner import DWAParams, dwa_control
from navsim.map import GridMap


def test_dwa_moves_toward_goal():
    grid = GridMap(
        [
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
        ]
    )
    costmap = CostMap.from_grid(grid, 0.0)
    path = [(0.0, 0.0), (2.0, 0.0)]
    pose = (0.0, 0.0, 0.0)
    params = DWAParams(
        v_min=0.0,
        v_max=1.0,
        omega_max=1.0,
        v_samples=3,
        omega_samples=3,
        horizon=1.0,
        goal_weight=1.0,
        path_weight=0.0,
        clearance_weight=0.0,
    )
    v, omega, traj = dwa_control(pose, path, costmap, params, dt=0.1)
    assert v > 0.0
    assert abs(omega) <= params.omega_max
    assert len(traj) > 1


def test_dwa_returns_stop_when_blocked():
    grid = GridMap(
        [
            [1, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
        ]
    )
    costmap = CostMap.from_grid(grid, 0.0)
    path = [(0.0, 0.0), (2.0, 0.0)]
    pose = (0.0, 0.0, 0.0)
    params = DWAParams(v_samples=2, omega_samples=3, horizon=0.5)
    v, omega, traj = dwa_control(pose, path, costmap, params, dt=0.1)
    assert v == 0.0
    assert omega == 0.0
    assert len(traj) == 1

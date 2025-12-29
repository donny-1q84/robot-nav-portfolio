from navsim.metrics import final_distance, goal_reached, path_length, trajectory_length


def test_path_length():
    path = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    assert abs(path_length(path) - 2.0) < 1e-6


def test_trajectory_length():
    poses = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)]
    assert abs(trajectory_length(poses) - 2.0) < 1e-6


def test_goal_reached():
    poses = [(0.0, 0.0, 0.0), (0.4, 0.0, 0.0)]
    assert goal_reached(poses, (0.5, 0.0), tolerance=0.2)


def test_final_distance_empty():
    assert final_distance([], (1.0, 1.0)) == float("inf")

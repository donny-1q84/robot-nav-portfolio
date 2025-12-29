from navsim.localization import EKF, LocalizationParams
from navsim.sensors import SensorNoise


def _step_pose(pose, v, omega, dt):
    x, y, yaw = pose
    return (x + v * dt, y, yaw + omega * dt)


def test_ekf_tracks_without_noise():
    params = LocalizationParams(
        noise=SensorNoise(
            odom_std_v=0.0,
            odom_std_omega=0.0,
            meas_std_x=0.0,
            meas_std_y=0.0,
        ),
        init_cov=0.1,
        seed=0,
    )
    ekf = EKF((0.0, 0.0, 0.0), params)
    true_pose = (0.0, 0.0, 0.0)
    for _ in range(5):
        v = 1.0
        omega = 0.0
        dt = 1.0
        true_pose = _step_pose(true_pose, v, omega, dt)
        ekf.predict(v, omega, dt)
        ekf.update((true_pose[0], true_pose[1]))

    est = ekf.pose
    assert abs(est[0] - true_pose[0]) < 1e-6
    assert abs(est[1] - true_pose[1]) < 1e-6

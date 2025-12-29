from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple

from .sensors import SensorNoise


Pose = Tuple[float, float, float]


@dataclass
class LocalizationParams:
    noise: SensorNoise
    init_cov: float = 0.5
    seed: int = 0


def _wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _mat_mul(a: List[List[float]], b: List[List[float]]) -> List[List[float]]:
    rows = len(a)
    cols = len(b[0])
    inner = len(b)
    out = [[0.0 for _ in range(cols)] for _ in range(rows)]
    for i in range(rows):
        for j in range(cols):
            out[i][j] = sum(a[i][k] * b[k][j] for k in range(inner))
    return out


def _mat_transpose(a: List[List[float]]) -> List[List[float]]:
    return [list(row) for row in zip(*a)]


def _mat_add(a: List[List[float]], b: List[List[float]]) -> List[List[float]]:
    return [[a[i][j] + b[i][j] for j in range(len(a[0]))] for i in range(len(a))]


def _mat_sub(a: List[List[float]], b: List[List[float]]) -> List[List[float]]:
    return [[a[i][j] - b[i][j] for j in range(len(a[0]))] for i in range(len(a))]


class EKF:
    def __init__(self, initial_pose: Pose, params: LocalizationParams) -> None:
        self.x = [float(initial_pose[0]), float(initial_pose[1]), float(initial_pose[2])]
        self.P = [
            [params.init_cov, 0.0, 0.0],
            [0.0, params.init_cov, 0.0],
            [0.0, 0.0, params.init_cov],
        ]
        self.params = params

    @property
    def pose(self) -> Pose:
        return (self.x[0], self.x[1], self.x[2])

    def predict(self, v: float, omega: float, dt: float) -> None:
        x, y, yaw = self.x
        sin_yaw = math.sin(yaw)
        cos_yaw = math.cos(yaw)
        x += v * cos_yaw * dt
        y += v * sin_yaw * dt
        yaw = _wrap_angle(yaw + omega * dt)
        self.x = [x, y, yaw]
        f = [
            [1.0, 0.0, -v * dt * sin_yaw],
            [0.0, 1.0, v * dt * cos_yaw],
            [0.0, 0.0, 1.0],
        ]

        q_xy = self.params.noise.odom_std_v * dt
        q_yaw = self.params.noise.odom_std_omega * dt
        q = [
            [q_xy * q_xy, 0.0, 0.0],
            [0.0, q_xy * q_xy, 0.0],
            [0.0, 0.0, q_yaw * q_yaw],
        ]

        fp = _mat_mul(f, self.P)
        fpf = _mat_mul(fp, _mat_transpose(f))
        self.P = _mat_add(fpf, q)

    def update(self, measurement: Tuple[float, float]) -> None:
        z_x, z_y = measurement
        x, y, yaw = self.x

        h = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ]
        r = [
            [self.params.noise.meas_std_x ** 2, 0.0],
            [0.0, self.params.noise.meas_std_y ** 2],
        ]

        y_res = [[z_x - x], [z_y - y]]
        hp = _mat_mul(h, self.P)
        s = _mat_add(_mat_mul(hp, _mat_transpose(h)), r)
        det = s[0][0] * s[1][1] - s[0][1] * s[1][0]
        if abs(det) < 1e-9:
            return
        s_inv = [
            [s[1][1] / det, -s[0][1] / det],
            [-s[1][0] / det, s[0][0] / det],
        ]

        pht = _mat_mul(self.P, _mat_transpose(h))
        k = _mat_mul(pht, s_inv)

        x_update = _mat_mul(k, y_res)
        self.x = [
            x + x_update[0][0],
            y + x_update[1][0],
            _wrap_angle(yaw + x_update[2][0]),
        ]

        kh = _mat_mul(k, h)
        i_kh = _mat_sub(
            [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            kh,
        )
        self.P = _mat_mul(i_kh, self.P)

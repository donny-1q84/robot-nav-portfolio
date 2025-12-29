from __future__ import annotations

import random
from dataclasses import dataclass
from typing import Tuple

Pose = Tuple[float, float, float]


@dataclass
class SensorNoise:
    odom_std_v: float = 0.05
    odom_std_omega: float = 0.05
    meas_std_x: float = 0.2
    meas_std_y: float = 0.2


def noisy_control(
    v: float, omega: float, noise: SensorNoise, rng: random.Random
) -> Tuple[float, float]:
    return (
        v + rng.gauss(0.0, noise.odom_std_v),
        omega + rng.gauss(0.0, noise.odom_std_omega),
    )


def noisy_position(pose: Pose, noise: SensorNoise, rng: random.Random) -> Tuple[float, float]:
    x, y, _ = pose
    return (
        x + rng.gauss(0.0, noise.meas_std_x),
        y + rng.gauss(0.0, noise.meas_std_y),
    )

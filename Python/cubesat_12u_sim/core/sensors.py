"""Sensors: 3-axis MEMS rate gyroscope model (bias + white noise + random walk)."""
from __future__ import annotations
import numpy as np
from cubesat_12u_sim.core import config as C


class RateGyro:
    def __init__(self, seed: int | None = 0):
        self.rng = np.random.default_rng(seed)
        self.bias = C.GYRO_BIAS0.copy()

    def measure(self, omega_true: np.ndarray, dt: float) -> np.ndarray:
        """Sample gyro at one step: w_meas = w_true + bias + white noise; evolve bias RW."""
        # bias random walk: db = w_bias * sqrt(dt), with correlation-time scaling approx
        self.bias += self.rng.normal(0, 1, 3) * C.GYRO_BIAS_WALK * np.sqrt(dt)
        # measurement white noise: sigma = ARW / sqrt(dt)
        sigma = C.GYRO_ARW / np.sqrt(dt)
        noise = self.rng.normal(0, 1, 3) * sigma
        return np.asarray(omega_true, float) + self.bias + noise

    def reset(self):
        self.bias = C.GYRO_BIAS0.copy()

"""
Two independent ways to fuse an accelerometer + gyroscope into one stable
angle estimate. Reimplemented, algorithm-for-algorithm identical, in
../../host/include/fusion/ (C++, the real host app) -- this Python copy
exists so every notebook and lab can plot and experiment with the exact
same math with zero hardware and zero compiler involved.

Why fuse two sensors instead of trusting one: the accelerometer measures
gravity's direction directly (so it never drifts over time), but it's
noisy on every single reading and gets actively wrong under linear
acceleration (it can't tell "tilted" from "accelerating sideways").
The gyroscope measures rotation RATE cleanly and instantly, but integrating
a rate into an angle accumulates any tiny bias in that rate forever --
a gyro-only angle estimate drifts steadily away from the truth even
sitting perfectly still. Neither sensor alone is trustworthy; the whole
point of fusion is combining a noisy-but-unbiased source with a
smooth-but-drifting one.
"""

import math


def accel_to_pitch(accel_y: float, accel_z: float, accel_x: float) -> float:
    """
    The angle gravity implies, straight from the accelerometer, in
    degrees. When the device is level, gravity reads almost entirely on
    the Z axis; tilt it and some of that 1g reading shifts onto Y. This
    has zero long-term drift (gravity's direction never changes) but is
    noisy sample-to-sample and wrong the instant the device is under real
    linear acceleration, not just gravity.
    """
    return math.degrees(math.atan2(accel_y, math.sqrt(accel_x ** 2 + accel_z ** 2)))


class ComplementaryFilter:
    """
    The simplest fusion that actually works: mostly trust the gyro's
    smooth, drifting estimate, and nudge it a small amount toward the
    accelerometer's noisy-but-unbiased estimate on every update. `alpha`
    close to 1.0 means "trust the gyro almost completely, correct slowly";
    the correction term is exactly what keeps the drift from accumulating
    forever.
    """

    def __init__(self, alpha: float = 0.98):
        self.alpha = alpha
        self.angle = 0.0

    def update(self, accel_angle: float, gyro_rate: float, dt: float) -> float:
        gyro_estimate = self.angle + gyro_rate * dt
        self.angle = self.alpha * gyro_estimate + (1.0 - self.alpha) * accel_angle
        return self.angle


class KalmanFilter1D:
    """
    A 2-state Kalman filter (angle, and the gyro's own bias) for one axis.
    More principled than the complementary filter's fixed `alpha`: instead
    of a hand-picked constant blend, it tracks its own uncertainty
    (`P`, a 2x2 covariance matrix) and computes the statistically optimal
    blend weight (the Kalman gain) on every single update -- and it
    explicitly estimates and removes the gyro's bias as a second state
    variable, rather than just tolerating the drift it causes.

    This is the same well-known formulation used by several widely-shared
    open-source "Kalman filter for IMU angle" implementations (see
    ../../../knowledge/READING_LIST.md) -- not a novel filter, a faithful
    port of the standard one.
    """

    def __init__(self, q_angle: float = 0.001, q_bias: float = 0.003, r_measure: float = 0.03):
        self.q_angle = q_angle    # process noise: how much we trust the model between updates
        self.q_bias = q_bias      # process noise: how fast we expect the bias itself to drift
        self.r_measure = r_measure  # measurement noise: how much we trust the accelerometer
        self.angle = 0.0
        self.bias = 0.0
        # 2x2 error covariance matrix, flattened as four scalars for clarity.
        self.p00 = self.p01 = self.p10 = self.p11 = 0.0

    def update(self, accel_angle: float, gyro_rate: float, dt: float) -> float:
        # --- predict step ---
        rate = gyro_rate - self.bias
        self.angle += dt * rate

        self.p00 += dt * (dt * self.p11 - self.p01 - self.p10 + self.q_angle)
        self.p01 -= dt * self.p11
        self.p10 -= dt * self.p11
        self.p11 += self.q_bias * dt

        # --- measurement update step ---
        innovation = accel_angle - self.angle
        s = self.p00 + self.r_measure  # innovation covariance
        k0 = self.p00 / s              # Kalman gain for the angle state
        k1 = self.p10 / s              # Kalman gain for the bias state

        self.angle += k0 * innovation
        self.bias += k1 * innovation

        p00_prev, p01_prev = self.p00, self.p01
        self.p00 -= k0 * p00_prev
        self.p01 -= k0 * p01_prev
        self.p10 -= k1 * p00_prev
        self.p11 -= k1 * p01_prev

        return self.angle

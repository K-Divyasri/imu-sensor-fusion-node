import sys
import math
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from imu_sim.fusion import ComplementaryFilter, KalmanFilter1D, accel_to_pitch
from imu_sim.synth import generate_motion


def test_accel_to_pitch_level_device_reads_zero():
    # Level device: all 1g on Z, nothing on X/Y -- should read ~0 degrees.
    assert abs(accel_to_pitch(accel_y=0.0, accel_z=1.0, accel_x=0.0)) < 1e-6


def test_accel_to_pitch_90_degrees_tilt():
    # Tipped 90 degrees onto its side: all 1g now on Y, nothing on Z.
    pitch = accel_to_pitch(accel_y=1.0, accel_z=0.0, accel_x=0.0)
    assert abs(pitch - 90.0) < 1e-6


def _rmse(a, b):
    n = len(a)
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)) / n)


def test_gyro_only_integration_drifts_away_from_truth():
    """The whole reason fusion exists: naive gyro-only integration drifts
    steadily because of the fixed bias baked into generate_motion's synthetic
    gyro readings -- this test proves the drift is real, not a strawman."""
    rows = generate_motion(duration_s=20.0, sample_rate_hz=200.0, seed=1)
    dt = 1.0 / 200.0

    gyro_only_angle = 0.0
    truth, gyro_only = [], []
    for row in rows:
        gyro_only_angle += row["gyro_y"] * dt
        truth.append(row["true_pitch_deg"])
        gyro_only.append(gyro_only_angle)

    # Error grows over time -- compare the first second's RMSE to the last
    # second's RMSE and confirm the drift is real and growing, not just noise.
    early_error = _rmse(truth[:200], gyro_only[:200])
    late_error = _rmse(truth[-200:], gyro_only[-200:])
    assert late_error > early_error * 3  # drift compounds; this is a conservative margin


def test_complementary_filter_beats_gyro_only_over_time():
    rows = generate_motion(duration_s=20.0, sample_rate_hz=200.0, seed=1)
    dt = 1.0 / 200.0

    comp = ComplementaryFilter(alpha=0.98)
    gyro_only_angle = 0.0
    truth, fused, gyro_only = [], [], []
    for row in rows:
        accel_angle = accel_to_pitch(row["accel_y"], row["accel_z"], row["accel_x"])
        fused.append(comp.update(accel_angle, row["gyro_y"], dt))
        gyro_only_angle += row["gyro_y"] * dt
        gyro_only.append(gyro_only_angle)
        truth.append(row["true_pitch_deg"])

    # Judge on the second half only -- the complementary filter needs a
    # short settling period before its correction term catches up from a
    # cold start at angle=0.
    half = len(truth) // 2
    assert _rmse(truth[half:], fused[half:]) < _rmse(truth[half:], gyro_only[half:])


def test_kalman_filter_beats_gyro_only_over_time():
    rows = generate_motion(duration_s=20.0, sample_rate_hz=200.0, seed=1)
    dt = 1.0 / 200.0

    kf = KalmanFilter1D()
    gyro_only_angle = 0.0
    truth, fused, gyro_only = [], [], []
    for row in rows:
        accel_angle = accel_to_pitch(row["accel_y"], row["accel_z"], row["accel_x"])
        fused.append(kf.update(accel_angle, row["gyro_y"], dt))
        gyro_only_angle += row["gyro_y"] * dt
        gyro_only.append(gyro_only_angle)
        truth.append(row["true_pitch_deg"])

    half = len(truth) // 2
    assert _rmse(truth[half:], fused[half:]) < _rmse(truth[half:], gyro_only[half:])


def test_kalman_estimates_the_gyro_bias():
    # generate_motion's default gyro_bias_deg_s=2.0 -- after enough samples
    # to settle, the Kalman filter's own bias state should land close to it.
    rows = generate_motion(duration_s=30.0, sample_rate_hz=200.0, seed=1)
    dt = 1.0 / 200.0
    kf = KalmanFilter1D()
    for row in rows:
        accel_angle = accel_to_pitch(row["accel_y"], row["accel_z"], row["accel_x"])
        kf.update(accel_angle, row["gyro_y"], dt)
    assert abs(kf.bias - 2.0) < 0.5

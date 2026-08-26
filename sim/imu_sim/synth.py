"""
Synthetic IMU data -- a fake device gently rocking back and forth in
pitch, sitting still otherwise (no real linear acceleration, so the
accelerometer's implied angle is a fair, if noisy, ground truth). Used
throughout notebooks/labs/tests so every idea in this project is provable
without a real MPU6050 plugged into anything.

Also includes a byte-stream corruptor, used to prove StreamParser's
resync logic actually recovers from dropped/mangled bytes rather than
just working on a clean stream by luck.
"""

import math
import random

from .frame import IMUFrame, pack


def generate_motion(
    duration_s: float = 10.0,
    sample_rate_hz: float = 200.0,
    seed: int = 42,
    accel_noise_std: float = 0.03,
    gyro_noise_std: float = 0.5,
    gyro_bias_deg_s: float = 2.0,
    amplitude_deg: float = 30.0,
    frequency_hz: float = 0.1,
):
    """
    Returns a list of row dicts: t_s, true_pitch_deg (ground truth),
    device_us, accel_x/y/z (g), gyro_x/y/z (deg/s) -- everything needed to
    both build IMUFrames and score a fusion filter's accuracy against the
    truth it's trying to recover.
    """
    rng = random.Random(seed)
    dt = 1.0 / sample_rate_hz
    n_samples = int(duration_s * sample_rate_hz)
    omega = 2 * math.pi * frequency_hz

    rows = []
    for i in range(n_samples):
        t = i * dt
        true_pitch_deg = amplitude_deg * math.sin(omega * t)
        true_rate_deg_s = amplitude_deg * omega * math.cos(omega * t)  # d/dt of the above

        pitch_rad = math.radians(true_pitch_deg)
        # Gravity (1g) rotated by the true pitch angle, plus sensor noise.
        # No real linear acceleration in this synthetic motion, so this is
        # exactly what a real accelerometer would read on a device tilting
        # at this angle while otherwise stationary.
        accel_x = 0.0 + rng.gauss(0, accel_noise_std)
        accel_y = math.sin(pitch_rad) + rng.gauss(0, accel_noise_std)
        accel_z = math.cos(pitch_rad) + rng.gauss(0, accel_noise_std)

        # Gyro reads the true rate PLUS a fixed bias PLUS noise -- the bias
        # is what makes naive integration drift steadily even though the
        # noise itself averages toward zero.
        gyro_x = rng.gauss(0, gyro_noise_std)
        gyro_y = true_rate_deg_s + gyro_bias_deg_s + rng.gauss(0, gyro_noise_std)
        gyro_z = rng.gauss(0, gyro_noise_std)

        rows.append({
            "t_s": t,
            "true_pitch_deg": true_pitch_deg,
            "device_us": int(round(t * 1_000_000)),
            "accel_x": accel_x, "accel_y": accel_y, "accel_z": accel_z,
            "gyro_x": gyro_x, "gyro_y": gyro_y, "gyro_z": gyro_z,
        })
    return rows


def rows_to_frames(rows):
    """Attach a sequence number to each row and turn it into an IMUFrame."""
    return [
        IMUFrame(
            seq=i,
            device_us=row["device_us"],
            accel_x=row["accel_x"], accel_y=row["accel_y"], accel_z=row["accel_z"],
            gyro_x=row["gyro_x"], gyro_y=row["gyro_y"], gyro_z=row["gyro_z"],
        )
        for i, row in enumerate(rows)
    ]


def frames_to_bytes(frames) -> bytes:
    """Pack a whole session as one concatenated byte stream -- exactly the
    format the C++ host app's --replay mode reads, and exactly what a real
    serial port would have delivered byte-by-byte over time."""
    return b"".join(pack(f) for f in frames)


def corrupt_stream(data: bytes, seed: int = 1, drop_probability: float = 0.001,
                    flip_probability: float = 0.001) -> bytes:
    """
    Simulates a noisy serial link: each byte has a small independent
    chance of being dropped entirely (shifting every later frame's
    alignment) or bit-flipped (breaking that one frame's CRC). Used to
    prove StreamParser's resync-by-one-byte algorithm actually recovers
    and keeps parsing every OTHER frame correctly, rather than just
    working when the whole stream happens to be clean.
    """
    rng = random.Random(seed)
    out = bytearray()
    for byte in data:
        if rng.random() < drop_probability:
            continue  # dropped -- never makes it into the output stream
        if rng.random() < flip_probability:
            byte ^= 1 << rng.randrange(8)  # flip one random bit
        out.append(byte)
    return bytes(out)

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from imu_sim.synth import generate_motion, rows_to_frames, frames_to_bytes, corrupt_stream
from imu_sim.frame import FRAME_SIZE, StreamParser


def test_generate_motion_is_deterministic():
    a = generate_motion(duration_s=1.0, sample_rate_hz=200.0, seed=7)
    b = generate_motion(duration_s=1.0, sample_rate_hz=200.0, seed=7)
    assert a == b


def test_different_seeds_give_different_noise():
    a = generate_motion(duration_s=1.0, seed=1)
    b = generate_motion(duration_s=1.0, seed=2)
    assert a[0]["accel_x"] != b[0]["accel_x"]


def test_frames_to_bytes_round_trips_through_stream_parser():
    rows = generate_motion(duration_s=2.0, sample_rate_hz=200.0, seed=3)
    frames = rows_to_frames(rows)
    raw = frames_to_bytes(frames)
    assert len(raw) == len(frames) * FRAME_SIZE

    parser = StreamParser()
    parsed = parser.feed(raw)
    assert parser.resync_byte_count == 0
    assert len(parsed) == len(frames)

    # Field-by-field with a tolerance, not exact equality: the wire format
    # packs each reading into a float32, so a Python float64 value picks up
    # a tiny (~1e-7 relative) truncation error on the round trip -- a real
    # property of the wire format, not a bug in the parser.
    for original, round_tripped in zip(frames, parsed):
        assert original.seq == round_tripped.seq
        assert original.device_us == round_tripped.device_us
        for field in ("accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z"):
            assert getattr(original, field) == pytest.approx(getattr(round_tripped, field), rel=1e-6)


def test_corrupt_stream_still_recovers_most_frames():
    rows = generate_motion(duration_s=5.0, sample_rate_hz=200.0, seed=4)  # 1000 frames
    frames = rows_to_frames(rows)
    raw = frames_to_bytes(frames)

    noisy = corrupt_stream(raw, seed=1, drop_probability=0.0005, flip_probability=0.0005)

    parser = StreamParser()
    parsed = parser.feed(noisy)
    # Some frames near a corruption event are expected to be lost, but the
    # overwhelming majority should still come through clean -- proving the
    # resync algorithm recovers instead of derailing the entire stream.
    assert len(parsed) > len(frames) * 0.8

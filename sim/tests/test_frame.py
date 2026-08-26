import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from imu_sim.frame import IMUFrame, MAGIC, FRAME_SIZE, pack, try_parse_one, StreamParser


# Values chosen as exact binary fractions (powers of two) so they round-trip
# through the wire format's float32 fields with zero precision loss --
# real accel/gyro readings won't be this tidy, but that's a property of
# float32 truncation (see test_synth.py's tolerant comparison for the
# realistic case), not something worth re-testing here.
SAMPLE = IMUFrame(
    seq=42, device_us=123_456,
    accel_x=0.25, accel_y=-0.25, accel_z=0.5,
    gyro_x=1.5, gyro_y=-0.375, gyro_z=0.125,
)


def test_frame_size_is_36_bytes():
    assert FRAME_SIZE == 36
    assert len(pack(SAMPLE)) == FRAME_SIZE


def test_pack_then_parse_round_trips_exactly():
    raw = pack(SAMPLE)
    frame, consumed = try_parse_one(raw, 0)
    assert consumed == FRAME_SIZE
    assert frame == SAMPLE


def test_parse_rejects_wrong_magic():
    raw = bytearray(pack(SAMPLE))
    raw[0] = 0x00  # corrupt the magic byte
    frame, consumed = try_parse_one(bytes(raw), 0)
    assert frame is None
    assert consumed == 1  # caller should slide forward by one byte and retry


def test_parse_rejects_bad_crc():
    raw = bytearray(pack(SAMPLE))
    raw[-1] ^= 0xFF  # corrupt the CRC byte, magic and body untouched
    frame, consumed = try_parse_one(bytes(raw), 0)
    assert frame is None
    assert consumed == 1


def test_parse_returns_zero_consumed_when_not_enough_bytes_yet():
    raw = pack(SAMPLE)[:10]  # a truncated, in-flight frame
    frame, consumed = try_parse_one(raw, 0)
    assert frame is None
    assert consumed == 0


def test_stream_parser_handles_two_back_to_back_frames():
    second = IMUFrame(seq=43, device_us=123_461, accel_x=0, accel_y=0, accel_z=1,
                       gyro_x=0, gyro_y=0, gyro_z=0)
    stream = pack(SAMPLE) + pack(second)

    parser = StreamParser()
    frames = parser.feed(stream)
    assert frames == [SAMPLE, second]
    assert parser.frames_parsed == 2
    assert parser.resync_byte_count == 0


def test_stream_parser_handles_a_frame_split_across_two_feeds():
    raw = pack(SAMPLE)
    parser = StreamParser()
    assert parser.feed(raw[:20]) == []          # incomplete -- nothing yet
    assert parser.feed(raw[20:]) == [SAMPLE]    # rest arrives -- now it parses


def test_stream_parser_resyncs_after_garbage_bytes():
    garbage = b"\x00\x11\x22"  # noise that isn't a valid frame
    stream = garbage + pack(SAMPLE)

    parser = StreamParser()
    frames = parser.feed(stream)
    assert frames == [SAMPLE]
    assert parser.resync_byte_count == len(garbage)


def test_stream_parser_survives_a_dropped_byte_mid_stream():
    # Simulates a real dropped serial byte: one frame arrives intact, the
    # NEXT frame is missing one byte from partway through it. The parser
    # should recover the first frame cleanly, fail the corrupted second
    # frame's CRC check (correctly refusing to trust a broken frame), and
    # resync in time to correctly parse a clean third frame afterward.
    second = IMUFrame(seq=1, device_us=5000, accel_x=0, accel_y=0, accel_z=1,
                       gyro_x=0, gyro_y=0, gyro_z=0)
    third = IMUFrame(seq=2, device_us=10000, accel_x=0, accel_y=0, accel_z=1,
                      gyro_x=0, gyro_y=0, gyro_z=0)

    second_raw = bytearray(pack(second))
    del second_raw[10]  # drop one byte from the middle of the second frame

    stream = pack(SAMPLE) + bytes(second_raw) + pack(third)

    parser = StreamParser()
    frames = parser.feed(stream)
    assert frames == [SAMPLE, third]  # the corrupted middle frame is correctly dropped
    assert parser.resync_byte_count > 0  # and it took some resync work to get there

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from imu_sim.crc16 import crc16_ccitt_false


def test_known_test_vector():
    # The standard CRC-16/CCITT-FALSE check value for the ASCII string
    # "123456789", published in every CRC catalog -- if this doesn't
    # match, the algorithm itself is wrong, not just this project's usage.
    assert crc16_ccitt_false(b"123456789") == 0x29B1


def test_different_data_gives_different_crc():
    assert crc16_ccitt_false(b"hello") != crc16_ccitt_false(b"world")


def test_one_bit_flip_changes_the_crc():
    original = b"\x01\x02\x03\x04"
    flipped = b"\x01\x02\x03\x05"  # last byte's low bit flipped
    assert crc16_ccitt_false(original) != crc16_ccitt_false(flipped)

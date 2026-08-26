"""
The wire format one IMUFrame travels in, over a byte-stream UART link.

UART is NOT message-atomic the way UDP is -- market-engine's UDPReceiver
can trust that one recv() call returns exactly one MarketUpdate, because
the kernel network stack preserves datagram boundaries. A serial port has
no such boundary: bytes just arrive, one after another, and a single byte
can be dropped, corrupted, or the receiver can start listening mid-frame.
So this format adds two things UDP didn't need: a MAGIC sync marker to
find the start of a frame in an arbitrary byte stream, and a CRC to prove
the bytes that followed weren't corrupted -- and StreamParser below knows
how to recover (resync) when either check fails, instead of just crashing
or silently misreading garbage as a valid frame.

Layout (little-endian, 36 bytes total):
    magic       2 bytes   0xAA 0x55 -- frame sync marker
    seq         uint32    monotonically increasing sample counter
    device_us   uint32    micros() on the ESP32 at the time of the I2C read
    accel_x/y/z float32   g (1g = 9.81 m/s^2)
    gyro_x/y/z  float32   degrees/second
    crc         uint16    CRC-16/CCITT-FALSE over seq..gyro_z (32 bytes)

Reimplemented, field-for-field identical, in
../../host/include/core/frame.hpp (C++, the real host app) and
../../firmware/imu_node/include/frame.h (C++, the ESP32 firmware) --
all three must agree byte-for-byte or nothing can talk to anything else.
"""

import struct
from dataclasses import dataclass

from .crc16 import crc16_ccitt_false

MAGIC = b"\xAA\x55"
_BODY_FORMAT = "<II6f"          # seq, device_us, ax, ay, az, gx, gy, gz
_BODY_SIZE = struct.calcsize(_BODY_FORMAT)   # 4 + 4 + 6*4 = 32
FRAME_SIZE = len(MAGIC) + _BODY_SIZE + 2      # + crc (uint16) = 36


@dataclass
class IMUFrame:
    seq: int
    device_us: int
    accel_x: float
    accel_y: float
    accel_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float


def pack(frame: IMUFrame) -> bytes:
    """Serialize one IMUFrame to its exact 36-byte wire format."""
    body = struct.pack(
        _BODY_FORMAT,
        frame.seq, frame.device_us,
        frame.accel_x, frame.accel_y, frame.accel_z,
        frame.gyro_x, frame.gyro_y, frame.gyro_z,
    )
    crc = crc16_ccitt_false(body)
    return MAGIC + body + struct.pack("<H", crc)


def try_parse_one(buf: bytes, start: int):
    """
    Attempt to parse exactly one frame starting at buf[start:].

    Returns (frame_or_None, bytes_consumed). bytes_consumed is always >= 1
    once there's enough data to make a decision, so the caller can always
    advance and keep scanning -- this is the resync algorithm in its
    smallest form: on ANY failure (no magic here, bad CRC), advance by
    exactly one byte and try again from there, rather than giving up on
    the whole stream over one bad byte.

    Returns (None, 0) if there simply aren't enough bytes yet to know
    either way -- the caller should wait for more data before retrying
    at this same position.
    """
    if start + FRAME_SIZE > len(buf):
        return None, 0  # not enough bytes buffered yet -- wait, don't advance

    if buf[start:start + 2] != MAGIC:
        return None, 1  # not a sync marker here -- slide forward one byte

    body = buf[start + 2:start + 2 + _BODY_SIZE]
    crc_received, = struct.unpack("<H", buf[start + 2 + _BODY_SIZE:start + FRAME_SIZE])
    crc_computed = crc16_ccitt_false(body)
    if crc_received != crc_computed:
        # Looked like a frame (right magic bytes) but failed the integrity
        # check -- could be a corrupted frame, or could be two stray 0xAA
        # 0x55 bytes inside an unrelated payload that happen to look like a
        # sync marker. Either way, don't trust it -- slide forward one byte
        # and keep scanning, exactly like a bad-magic miss.
        return None, 1

    seq, device_us, ax, ay, az, gx, gy, gz = struct.unpack(_BODY_FORMAT, body)
    return IMUFrame(seq, device_us, ax, ay, az, gx, gy, gz), FRAME_SIZE


class StreamParser:
    """
    Feed it bytes as they arrive (from a real serial port, or a replay
    file, or a unit test's synthetic corrupted stream) and it hands back
    every complete, valid frame it can find -- buffering partial data
    across calls exactly like a real serial reader has to, since a frame
    can arrive split across two separate reads.
    """

    def __init__(self):
        self._buffer = bytearray()
        self.frames_parsed = 0
        self.resync_byte_count = 0  # bytes discarded while searching for a valid frame

    def feed(self, data: bytes):
        self._buffer.extend(data)
        frames = []
        pos = 0
        while True:
            frame, consumed = try_parse_one(bytes(self._buffer), pos)
            if consumed == 0:
                break  # not enough data buffered for a decision -- stop here
            if frame is not None:
                frames.append(frame)
                self.frames_parsed += 1
            else:
                self.resync_byte_count += 1
            pos += consumed
        del self._buffer[:pos]  # drop everything we've made a decision about
        return frames

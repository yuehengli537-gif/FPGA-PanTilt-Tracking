"""Versioned UART protocol shared with servo_uart; angles are command estimates."""
from dataclasses import dataclass
import binascii
import math
import struct

HELLO, GET_STATUS = 0x01, 0x02
SET_TARGET, ENABLE, HOLD, HOME, SET_LIMITS, GET_LIMITS = range(0x10, 0x16)
SET_MODE = 0x20
HELLO_REPLY, STATUS, LIMITS, ERROR = 0x81, 0x90, 0x91, 0xE0
HEADER = b"\xa5\x5a"
VERSION = 1


@dataclass(frozen=True)
class Frame:
    kind: int
    seq: int
    flags: int = 0
    payload: bytes = bytes(8)


def encode(frame: Frame) -> bytes:
    if len(frame.payload) != 8:
        raise ValueError("Payload must be 8 bytes")
    body = bytes((VERSION, frame.kind, frame.seq, frame.flags)) + frame.payload
    return HEADER + body + struct.pack(">H", binascii.crc_hqx(body, 0xFFFF))


class StreamParser:
    """Retain partial frames and resynchronize after corruption or noise."""
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data: bytes) -> list[Frame]:
        self.buffer.extend(data)
        frames = []
        while len(self.buffer) >= 2:
            pos = self.buffer.find(HEADER)
            if pos < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1] == 0xA5 else b""
                break
            del self.buffer[:pos]
            if len(self.buffer) < 16:
                break
            packet = bytes(self.buffer[:16])
            if packet[2] != VERSION or binascii.crc_hqx(packet[2:14], 0xFFFF) != int.from_bytes(packet[14:16], "big"):
                del self.buffer[0]
                continue
            frames.append(Frame(packet[3], packet[4], packet[5], packet[6:14]))
            del self.buffer[:16]
        return frames


def pack_us(*values: int) -> bytes:
    if len(values) > 4:
        raise ValueError("At most four values")
    return struct.pack("<4H", *(list(values) + [0] * (4 - len(values))))


def unpack_us(payload: bytes) -> tuple[int, int, int, int]:
    return struct.unpack("<4H", payload)


def angle_to_us(angle: float) -> int:
    value = float(angle)
    if not math.isfinite(value) or not 0 <= value <= 180:
        raise ValueError("角度必须是 0～180° 之间的数字")
    return round(500 + value * 2000 / 180)


def us_to_angle(pulse: int) -> float:
    return round((pulse - 500) * 180 / 2000, 2)


def validate_limits(values):
    pmin, pmax, tmin, tmax = values
    if not (500 <= pmin <= 1500 <= pmax <= 2500 and 500 <= tmin <= 1500 <= tmax <= 2500):
        raise ValueError("两轴限位需在 0～180° 内，并包含 90° 中位")


ERROR_MESSAGES = {1: "固件不支持该指令", 2: "数值越界或当前状态不允许修改", 3: "请先启用舵机输出", 4: "当前固件尚未支持自动跟随"}

"""Real serial and an explicitly labelled, protocol-compatible simulation."""
import time
from .protocol import *


class SerialTransport:
    def __init__(self, port):
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("缺少串口库，请先运行 scripts/setup-servo-ui.ps1") from exc
        self.port = serial.Serial(port=None, baudrate=115200, timeout=0.03,
                                  write_timeout=0.3, rtscts=False, dsrdtr=False)
        self.port.dtr = False
        self.port.rts = False
        self.port.port = port
        self.port.open()
        self.port.reset_input_buffer()

    def write(self, data):
        if self.port.write(data) != len(data):
            raise OSError("串口未完整发送指令")

    def read(self):
        return self.port.read(max(1, min(self.port.in_waiting, 4096)))

    def close(self):
        self.port.close()


class DemoTransport:
    """No hardware access. Simulated position is only the PWM command."""
    def __init__(self, clock=time.monotonic, supports_auto=False):
        self.clock = clock
        self.last_frame = clock()
        self.last_request = clock()
        self.enabled = False
        self.watchdog = False
        self.supports_auto = supports_auto
        self.auto_mode = False
        self.current = [1500, 1500]
        self.target = [1500, 1500]
        self.limits = [1300, 1700, 1460, 1540]
        self.input = StreamParser()
        self.output = bytearray()
        self.closed = False

    def _advance(self):
        now = self.clock()
        # Advance only until the watchdog deadline, then hold that last command.
        end = min(now, self.last_request + 1.0)
        frames = max(0, int((end - self.last_frame) / 0.02))
        if frames:
            if self.enabled:
                for axis in range(2):
                    distance = self.target[axis] - self.current[axis]
                    self.current[axis] += min(abs(distance), frames * 2) * (1 if distance >= 0 else -1)
            self.last_frame += frames * 0.02
        if now - self.last_request >= 1.0:
            self.target = self.current.copy()
            self.watchdog = True
            self.auto_mode = False
            self.last_frame = now

    def _flags(self):
        return (int(self.enabled) | (int(self.current != self.target) << 1) |
                (int(self.watchdog) << 2) | (int(self.auto_mode) << 3))

    def write(self, data):
        if self.closed:
            raise OSError("演示连接已关闭")
        self._advance()
        for frame in self.input.feed(data):
            if frame.flags:
                continue
            self.last_request = self.clock()
            kind, payload, error = STATUS, None, 0
            values = unpack_us(frame.payload)
            zero_commands = (HELLO, GET_STATUS, HOLD, HOME, GET_LIMITS)
            invalid_reserved = ((frame.kind in zero_commands and any(frame.payload)) or
                                (frame.kind == SET_TARGET and any(frame.payload[4:])) or
                                (frame.kind in (ENABLE, SET_MODE) and any(frame.payload[1:])))
            if invalid_reserved:
                error = 2
            elif frame.kind == HELLO:
                kind, payload = HELLO_REPLY, b"PT20" + bytes((1, 0, 3 if self.supports_auto else 1, 0))
            elif frame.kind == GET_STATUS:
                pass
            elif frame.kind == GET_LIMITS:
                kind, payload = LIMITS, pack_us(*self.limits)
            elif frame.kind == SET_LIMITS:
                try:
                    validate_limits(values)
                    if self.enabled:
                        raise ValueError()
                except ValueError:
                    error = 2
                else:
                    self.limits = list(values)
                    kind, payload = LIMITS, pack_us(*values)
            elif frame.kind == ENABLE:
                if frame.payload[0] not in (0, 1):
                    error = 2
                else:
                    if frame.payload[0] and not self.enabled:
                        self.current, self.target = [1500, 1500], [1500, 1500]
                        self.last_frame = self.clock()
                    if not frame.payload[0]:
                        self.target = self.current.copy()
                    self.enabled = bool(frame.payload[0])
                    self.auto_mode = False
                    self.watchdog = False
            elif frame.kind in (SET_TARGET, HOLD, HOME):
                if not self.enabled:
                    error = 3
                elif frame.kind == SET_TARGET:
                    if not (self.limits[0] <= values[0] <= self.limits[1] and self.limits[2] <= values[1] <= self.limits[3]):
                        error = 2
                    else:
                        self.target = list(values[:2])
                        self.auto_mode = False
                        self.watchdog = False
                else:
                    self.target = self.current.copy() if frame.kind == HOLD else [1500, 1500]
                    self.auto_mode = False
                    self.watchdog = False
            elif frame.kind == SET_MODE:
                if frame.payload[0] not in (0, 1):
                    error = 2
                elif frame.payload[0] and not self.supports_auto:
                    error = 4
                elif frame.payload[0] and not self.enabled:
                    error = 3
                else:
                    # Simulate mode arbitration only; no synthetic camera target.
                    self.auto_mode = bool(frame.payload[0])
                    self.target = self.current.copy()
                    self.watchdog = False
            else:
                error = 1
            if error:
                reply = Frame(ERROR, frame.seq, 0, bytes((error, frame.kind)) + bytes(6))
            else:
                reply = Frame(kind, frame.seq, self._flags(), payload or pack_us(*self.current, *self.target))
            self.output.extend(encode(reply))

    def read(self):
        self._advance()
        chunk = bytes(self.output)
        self.output.clear()
        if not chunk:
            time.sleep(0.005)
        return chunk

    def close(self):
        self.closed = True

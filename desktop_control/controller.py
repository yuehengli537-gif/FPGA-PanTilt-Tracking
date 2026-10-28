"""GUI-independent command service; serial I/O runs on one background worker."""
import queue
import json
from pathlib import Path
import threading
import time
from .protocol import *
from .transport import DemoTransport, SerialTransport
from .coordinates import MountAxis
from .ports import describe_ports


class DeviceRejected(RuntimeError):
    pass


class PriorityPending(Exception):
    pass


class ResponseTimeout(TimeoutError):
    """No matching reply arrived; transport I/O errors keep their own type."""
    pass


class ServoController:
    def __init__(self, transport_factory=None, directions_path=None):
        self._factory = transport_factory
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._preempt = threading.Event()
        self._priority_revision = 0
        self._thread = None
        self._commands = queue.Queue(maxsize=8)
        self._events = queue.Queue(maxsize=200)
        self._seq = 0
        self._last_reply = None
        self._state = dict(connected=False, connecting=False, demo=False, enabled=False,
                           moving=False, watchdog=False, supports_auto=False, pending=False,
                           port="", firmware="", mode="manual", error="", pan_angle=90.0,
                           tilt_angle=0.0, target_pan=90.0, target_tilt=0.0,
                           last_reply_age=None, status_revision=0)
        self._limits = [1300, 1700, 1460, 1540]
        self._positions = [1500, 1500, 1500, 1500]
        self._directions_path = Path(directions_path) if directions_path else None
        self._axes = (MountAxis(90, 180), MountAxis(0, 45, minimum=-45))
        if self._directions_path and self._directions_path.exists():
            saved = json.loads(self._directions_path.read_text(encoding="utf-8"))
            if (saved.get("version") != 1 or
                    type(saved.get("pan_reversed")) is not bool or
                    type(saved.get("tilt_reversed")) is not bool):
                raise ValueError("云台方向设置文件无效，请修正设置文件后启动；未连接硬件")
            self._axes = (MountAxis(90, 180, saved["pan_reversed"]),
                          MountAxis(0, 45, saved["tilt_reversed"], minimum=-45))
        self._reproject()

    def _reproject(self):
        """Caller owns the state lock, except during single-threaded init."""
        for index, name in enumerate(("pan", "tilt")):
            axis = self._axes[index]
            low, high = axis.angle_limits(*self._limits[index * 2:index * 2 + 2])
            self._state.update({name + "_angle": axis.from_us(self._positions[index]),
                                "target_" + name: axis.from_us(self._positions[index + 2]),
                                name + "_pulse_us": self._positions[index],
                                "target_" + name + "_pulse_us": self._positions[index + 2],
                                name + "_min": low, name + "_max": high,
                                name + "_input_min": max(axis.minimum, low),
                                name + "_input_max": min(axis.maximum, high),
                                name + "_reversed": axis.reversed})
        expected = self._axes[0].pulse_limits(0, 180) + self._axes[1].pulse_limits(-45, 45)
        self._state["profile_ready"] = self._state["connected"] and self._limits == expected
        self._state["coordinate_note"] = "水平：正面左0° / 正前90° / 右180°；俯仰：向上−45° / 回正0° / 向下＋45°。本次装配两轴不反转；更改方向或装配后需重新核对。角度为指令值。"

    def set_directions(self, pan_reversed, tilt_reversed):
        """Change coordinates only. Never write a motion or limit command."""
        with self._lock:
            if self._state["connecting"] or self._state["pending"] or self._state["enabled"]:
                raise RuntimeError("请等待指令完成并关闭 PWM 输出后修改方向")
            if type(pan_reversed) is not bool or type(tilt_reversed) is not bool:
                raise ValueError("方向设置必须为布尔值")
            if self._directions_path:
                data = dict(version=1, pan_reversed=pan_reversed, tilt_reversed=tilt_reversed)
                self._directions_path.parent.mkdir(parents=True, exist_ok=True)
                temporary = self._directions_path.with_suffix(".tmp")
                temporary.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
                temporary.replace(self._directions_path)
            self._axes = (MountAxis(90, 180, pan_reversed), MountAxis(0, 45, tilt_reversed, minimum=-45))
            self._reproject()
            self._log("方向已保存，仅更新坐标；未发送运动指令。按新方向重新应用所需限位。")

    def apply_mount_range(self):
        self.set_limits(0, 180, -45, 45)

    @staticmethod
    def ports():
        try:
            from serial.tools import list_ports
            return describe_ports(list_ports.comports())
        except ImportError:
            return []

    def snapshot(self):
        with self._lock:
            state = self._state.copy()
            state["last_reply_age"] = None if self._last_reply is None else time.monotonic() - self._last_reply
            return state

    def _update(self, **values):
        with self._lock:
            self._state.update(values)
            self._reproject()

    def _log(self, message):
        if self._events.full():
            try:
                self._events.get_nowait()
            except queue.Empty:
                pass
        self._events.put_nowait(time.strftime("%H:%M:%S ") + message)

    def poll_events(self):
        messages = []
        while True:
            try:
                messages.append(self._events.get_nowait())
            except queue.Empty:
                return messages

    def connect(self, port="", demo=False):
        if self._thread and self._thread.is_alive():
            raise RuntimeError("连接尚未关闭，请稍候")
        if not demo and not port:
            raise ValueError("请先选择开发板串口")
        self._stop = threading.Event()
        self._preempt = threading.Event()
        self._priority_revision = 0
        self._commands = queue.Queue(maxsize=8)
        self._last_reply = None
        self._update(connected=False, connecting=True, pending=False, demo=demo,
                     port="演示设备" if demo else port, error="", enabled=False, firmware="",
                     supports_auto=False, mode="manual", watchdog=False, moving=False)
        self._thread = threading.Thread(target=self._run, args=(port, demo), daemon=True)
        self._thread.start()

    def disconnect(self):
        with self._lock:
            self._stop.set()
            self._update(connected=False, connecting=False, pending=False)

    def close(self):
        self.disconnect()
        if self._thread:
            self._thread.join(timeout=1.2)

    def _enqueue(self, kind, payload=bytes(8), priority=False):
        with self._lock:
            if not self._state["connected"]:
                raise RuntimeError("请先连接设备并等待握手完成")
            if self._state["pending"] and not priority:
                raise RuntimeError("上一条指令尚未确认，请稍候")
            if priority:
                self._priority_revision += 1
                while True:
                    try:
                        self._commands.get_nowait()
                    except queue.Empty:
                        break
                self._preempt.set()
            self._commands.put_nowait((kind, payload, self._priority_revision if priority else None))
            self._state["pending"] = True

    def set_angles(self, pan, tilt):
        with self._lock:
            if not self._state["enabled"]:
                raise RuntimeError("请先启用舵机输出")
            values = [self._axes[0].to_us(pan), self._axes[1].to_us(tilt)]
            pmin, pmax, tmin, tmax = self._limits
            if not (pmin <= values[0] <= pmax and tmin <= values[1] <= tmax):
                raise ValueError("输入角度超出当前限位，请查看两轴可用角度范围")
            self._enqueue(SET_TARGET, pack_us(*values))

    def enable(self, on):
        self._enqueue(ENABLE, bytes((int(bool(on)),)) + bytes(7), priority=not on)

    def hold(self):
        if not self.snapshot()["enabled"]:
            raise RuntimeError("当前舵机输出已关闭")
        self._enqueue(HOLD, priority=True)

    def home(self):
        if not self.snapshot()["enabled"]:
            raise RuntimeError("请先启用舵机输出")
        self._enqueue(HOME)

    def set_limits(self, pan_min, pan_max, tilt_min, tilt_max):
        with self._lock:
            if self._state["enabled"]:
                raise RuntimeError("请先关闭输出，再修改角度限位")
            values = (self._axes[0].pulse_limits(pan_min, pan_max) +
                      self._axes[1].pulse_limits(tilt_min, tilt_max))
            validate_limits(values)
            self._enqueue(SET_LIMITS, pack_us(*values))

    def set_mode(self, mode):
        if mode not in ("manual", "auto"):
            raise ValueError("未知控制模式")
        with self._lock:
            if mode == "auto":
                if not self._state["supports_auto"]:
                    raise RuntimeError("当前固件尚未接入摄像头自动跟随")
                if not self._state["enabled"]:
                    raise RuntimeError("请先启用舵机输出，再开始红球跟随")
            self._enqueue(SET_MODE, bytes((mode == "auto",)) + bytes(7))

    def _exchange(self, transport, parser, kind, payload=bytes(8), timeout=0.7, closing=False, urgent_revision=None):
        self._seq = (self._seq + 1) & 255
        seq = self._seq
        expected = {HELLO: HELLO_REPLY, SET_LIMITS: LIMITS, GET_LIMITS: LIMITS}.get(kind, STATUS)
        with self._lock:
            if self._stop.is_set() and not closing:
                raise InterruptedError()
            if not closing and ((urgent_revision is None and self._preempt.is_set()) or
                                (urgent_revision is not None and urgent_revision != self._priority_revision)):
                raise PriorityPending()
            transport.write(encode(Frame(kind, seq, 0, payload)))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._stop.is_set() and not closing:
                raise InterruptedError()
            if not closing and ((urgent_revision is None and self._preempt.is_set()) or
                                (urgent_revision is not None and urgent_revision != self._priority_revision)):
                raise PriorityPending()
            for reply in parser.feed(transport.read()):
                if reply.seq != seq:
                    continue
                if reply.kind == ERROR:
                    self._last_reply = time.monotonic()
                    raise DeviceRejected(ERROR_MESSAGES.get(reply.payload[0], "设备拒绝指令"))
                if reply.kind == expected:
                    self._last_reply = time.monotonic()
                    self._apply_reply(reply)
                    if kind == SET_MODE and bool(reply.flags & 8) != bool(payload[0]):
                        requested = "自动跟随" if payload[0] else "手动模式"
                        actual = "自动跟随" if reply.flags & 8 else "手动模式"
                        raise DeviceRejected(
                            f"控制模式未切换：请求{requested}，开发板仍报告{actual}"
                            f"（状态 0x{reply.flags:02X}）；请检查固件和通信。")
                    return reply
        raise ResponseTimeout("未收到开发板有效回复，请检查串口和板上固件")

    def _query(self, transport, parser, kind):
        """Retry one missing read-only reply, never an action or an I/O failure."""
        if kind not in (HELLO, GET_LIMITS, GET_STATUS):
            raise ValueError("Only read-only commands may be retried")
        try:
            return self._exchange(transport, parser, kind)
        except ResponseTimeout:
            self._log("只读查询未收到回复，重试一次；未重发运动指令")
            # _exchange allocates a fresh sequence and checks stop/priority again.
            return self._exchange(transport, parser, kind)

    def _apply_reply(self, reply):
        common = dict(enabled=bool(reply.flags & 1), moving=bool(reply.flags & 2),
                      watchdog=bool(reply.flags & 4), mode="auto" if reply.flags & 8 else "manual")
        if reply.kind == HELLO_REPLY:
            if reply.payload[:4] != b"PT20" or not reply.payload[6] & 1:
                raise RuntimeError("串口设备不是兼容的舵机控制固件")
            common.update(firmware=f"{reply.payload[4]}.{reply.payload[5]}", supports_auto=bool(reply.payload[6] & 2))
        elif reply.kind == LIMITS:
            values = unpack_us(reply.payload)
            validate_limits(values)
            with self._lock:
                self._limits = list(values)
        elif reply.kind == STATUS:
            values = unpack_us(reply.payload)
            if not all(500 <= value <= 2500 for value in values):
                raise RuntimeError("板端返回了无效脉宽")
            with self._lock:
                self._positions = list(values)
                common["status_revision"] = self._state["status_revision"] + 1
        self._update(**common)

    def _run(self, port, demo):
        transport, parser = None, StreamParser()
        identified = False
        try:
            transport = self._factory(port, demo) if self._factory else (DemoTransport(supports_auto=True) if demo else SerialTransport(port))
            self._query(transport, parser, HELLO)
            identified = True
            self._query(transport, parser, GET_LIMITS)
            self._query(transport, parser, GET_STATUS)
            if self._stop.is_set():
                return
            self._update(connected=True, connecting=False, error="")
            self._log("演示模式已连接（不控制硬件）" if demo else f"{port} 握手成功，未自动使能或移动")
            next_poll = time.monotonic() + 0.25
            while not self._stop.is_set():
                try:
                    command = self._commands.get(timeout=max(0.01, min(0.05, next_poll - time.monotonic())))
                except queue.Empty:
                    command = None
                if command:
                    kind, payload, revision = command
                    try:
                        urgent = kind == HOLD or (kind == ENABLE and payload[0] == 0)
                        if urgent:
                            self._preempt.clear()
                            # Allow a previous 16-byte reply (~1.4ms) to finish.
                            time.sleep(0.004)
                        self._exchange(transport, parser, kind, payload, urgent_revision=revision)
                        self._update(error="")
                        self._log({SET_TARGET: "角度指令已确认", ENABLE: "输出状态已确认", HOLD: "停止移动指令已确认，保持输出", HOME: "回中指令已确认", SET_LIMITS: "角度限位已更新", SET_MODE: "控制模式已确认"}.get(kind, "指令已确认"))
                    except DeviceRejected as exc:
                        self._update(error=str(exc))
                        self._log(str(exc))
                    except PriorityPending:
                        self._log("优先处理停止/关闭输出请求")
                    finally:
                        with self._lock:
                            self._state["pending"] = not self._commands.empty()
                if time.monotonic() >= next_poll:
                    try:
                        self._query(transport, parser, GET_STATUS)
                    except PriorityPending:
                        pass
                    next_poll = time.monotonic() + 0.25
        except InterruptedError:
            pass
        except Exception as exc:
            message = f"连接失败或中断：{exc}；自动跟随状态无法确认，请重新连接后检查并重新启动。"
            # Mark the connection unavailable before the best-effort closing HOLD.
            # Its reply may also time out, and the UI must not keep motion enabled.
            self._update(connected=False, connecting=False, pending=False, error=message)
            self._log(message)
        finally:
            if transport:
                if identified:
                    try:
                        self._exchange(transport, parser, HOLD, timeout=0.25, closing=True)
                    except Exception:
                        pass
                try:
                    transport.close()
                except Exception as exc:
                    self._log("串口关闭异常：" + str(exc))
            self._update(connected=False, connecting=False, pending=False)
            self._log("连接已关闭；保持位置不等于切断舵机电源")

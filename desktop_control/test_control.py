import binascii
import math
import time
import unittest
import tempfile
from pathlib import Path
from .coordinates import MountAxis
from .protocol import *
from .transport import DemoTransport
from .controller import ServoController, ResponseTimeout


def request(device, kind, payload=bytes(8), seq=1):
    device.write(encode(Frame(kind, seq, 0, payload)))
    return StreamParser().feed(device.read())[0]


def wait_until(predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("Condition timed out")


class ProtocolTests(unittest.TestCase):
    def test_crc_and_fragment_resynchronization(self):
        self.assertEqual(binascii.crc_hqx(b"123456789", 0xFFFF), 0x29B1)
        frame = Frame(SET_TARGET, 255, 0, pack_us(1600, 1510))
        packet = encode(frame)
        self.assertEqual(len(packet), 16)
        self.assertEqual(packet[6:10], b"\x40\x06\xe6\x05")
        parser = StreamParser()
        bad = bytearray(packet); bad[7] ^= 1
        self.assertEqual(parser.feed(b"junk\xa5" + bad + packet[:7]), [])
        self.assertEqual(parser.feed(packet[7:]), [frame])
        self.assertEqual(parser.feed(packet * 2), [frame, frame])

    def test_degrees_and_limits(self):
        for degrees, pulse in ((0, 500), (90, 1500), (180, 2500), (72, 1300), (86.4, 1460)):
            self.assertEqual(angle_to_us(degrees), pulse)
            self.assertAlmostEqual(us_to_angle(pulse), degrees)
        for invalid in (-1, 181, math.nan, math.inf, "bad"):
            with self.assertRaises(ValueError): angle_to_us(invalid)
        with self.assertRaises(ValueError): validate_limits((1600, 1700, 1460, 1540))

    def test_demo_rejections_and_watchdog(self):
        now = [0.0]
        device = DemoTransport(clock=lambda: now[0])
        self.assertEqual(request(device, SET_TARGET, pack_us(1500, 1500)).payload[0], 3)
        self.assertEqual(request(device, HELLO, b"x" + bytes(7)).payload[0], 2)
        self.assertEqual(request(device, SET_MODE, b"\1" + bytes(7)).payload[0], 4)
        request(device, SET_LIMITS, pack_us(500, 2500, 500, 2500))
        request(device, ENABLE, b"\1" + bytes(7))
        self.assertEqual(request(device, SET_LIMITS, pack_us(500, 2500, 500, 2500)).payload[0], 2)
        request(device, SET_TARGET, pack_us(2000, 2000))
        now[0] = 2.0
        reply = request(device, GET_STATUS)
        self.assertTrue(reply.flags & 4)
        pan, tilt, target_pan, target_tilt = unpack_us(reply.payload)
        self.assertEqual((pan, tilt), (target_pan, target_tilt))
        self.assertLessEqual(pan, 1600)
        self.assertGreaterEqual(pan, 1598)
        now[0] += 0.25
        self.assertEqual(unpack_us(request(device, GET_STATUS).payload)[:2], (pan, tilt))
        self.assertFalse(request(device, HOLD).flags & 4)


    def test_auto_demo_requires_enable_and_watchdog_never_resumes_tracking(self):
        now = [0.0]
        device = DemoTransport(clock=lambda: now[0], supports_auto=True)
        self.assertEqual(request(device, HELLO).payload[6], 3)
        self.assertEqual(request(device, SET_MODE, b"\1" + bytes(7)).payload[0], 3)
        request(device, ENABLE, b"\1" + bytes(7))
        request(device, SET_TARGET, pack_us(1600, 1520))
        now[0] = 0.1
        reply = request(device, SET_MODE, b"\1" + bytes(7))
        self.assertTrue(reply.flags & 8)
        self.assertEqual(device.target, device.current)  # Enter auto without a jump.
        now[0] = 1.2
        reply = request(device, GET_STATUS)
        self.assertTrue(reply.flags & 4)
        self.assertFalse(reply.flags & 8)
        self.assertTrue(reply.flags & 1)
        self.assertEqual(device.target, device.current)
        now[0] += 0.1
        self.assertFalse(request(device, GET_STATUS).flags & 8)
        self.assertFalse(request(device, HOLD).flags & 4)

    def test_auto_demo_manual_commands_exit_auto(self):
        device = DemoTransport(supports_auto=True)
        request(device, ENABLE, b"\1" + bytes(7))
        for kind, payload in ((SET_MODE, bytes(8)), (HOLD, bytes(8)),
                              (HOME, bytes(8)), (SET_TARGET, pack_us(1600, 1520)),
                              (ENABLE, bytes(8))):
            with self.subTest(kind=kind):
                self.assertTrue(request(device, SET_MODE, b"\1" + bytes(7)).flags & 8)
                reply = request(device, kind, payload)
                self.assertFalse(reply.flags & 8)
                if kind in (SET_MODE, HOLD, ENABLE):
                    self.assertEqual(device.current, device.target)


class SpyDemo(DemoTransport):
    def __init__(self, drop_target=False, supports_auto=False):
        super().__init__(supports_auto=supports_auto)
        self.kinds = []
        self.drop_target = drop_target

    def write(self, data):
        frames = StreamParser().feed(data)
        self.kinds.extend(frame.kind for frame in frames)
        super().write(data)
        if self.drop_target and any(frame.kind == SET_TARGET for frame in frames):
            self.output.clear()


class ControllerTests(unittest.TestCase):
    def test_connect_does_not_move_or_enable(self):
        device = SpyDemo()
        control = ServoController(transport_factory=lambda *_: device)
        try:
            control.connect(demo=True)
            wait_until(lambda: control.snapshot()["connected"])
            self.assertEqual(device.kinds[:3], [HELLO, GET_LIMITS, GET_STATUS])
            self.assertFalse(control.snapshot()["enabled"])
            self.assertFalse(control.snapshot()["supports_auto"])
            with self.assertRaises(RuntimeError): control.set_angles(90, 90)
            with self.assertRaises(RuntimeError): control.set_mode("auto")
            control.enable(True)
            wait_until(lambda: not control.snapshot()["pending"])
            self.assertTrue(control.snapshot()["enabled"])
            with self.assertRaises(ValueError): control.set_angles(180, 90)
            with self.assertRaises(RuntimeError): control.set_limits(0, 180, 0, 180)
            control.set_angles(100, 0)
            wait_until(lambda: not control.snapshot()["pending"])
            self.assertAlmostEqual(control.snapshot()["target_pan"], 99.99)
        finally:
            control.close()

    def test_auto_mode_requires_explicit_enable_and_firmware_ack(self):
        device = SpyDemo(supports_auto=True)
        control = ServoController(transport_factory=lambda *_: device)
        try:
            control.connect(demo=True)
            wait_until(lambda: control.snapshot()["connected"])
            self.assertTrue(control.snapshot()["supports_auto"])
            self.assertEqual(control.snapshot()["mode"], "manual")
            with self.assertRaises(RuntimeError): control.set_mode("auto")
            self.assertNotIn(SET_MODE, device.kinds)
            control.enable(True)
            wait_until(lambda: not control.snapshot()["pending"])
            control.set_mode("auto")
            wait_until(lambda: not control.snapshot()["pending"])
            self.assertEqual(control.snapshot()["mode"], "auto")
            control.hold()
            wait_until(lambda: not control.snapshot()["pending"])
            self.assertEqual(control.snapshot()["mode"], "manual")
            self.assertTrue(control.snapshot()["enabled"])
            control.set_mode("auto")
            wait_until(lambda: not control.snapshot()["pending"])
        finally:
            control.close()
        self.assertFalse(device.auto_mode)  # Disconnect sends HOLD, not auto-resume.

    def test_no_write_after_disconnect(self):
        control, device = ServoController(), SpyDemo()
        control.disconnect()
        with self.assertRaises(InterruptedError):
            control._exchange(device, StreamParser(), SET_TARGET, pack_us(1600, 1500))
        self.assertEqual(device.kinds, [])

    def test_mode_ack_mismatch_reports_actual_mode_without_success(self):
        class IgnoredMode(SpyDemo):
            def write(self, data):
                frame = StreamParser().feed(data)[0]
                if frame.kind == SET_MODE:
                    self.kinds.append(SET_MODE)
                    # Firmware accepts the frame but leaves its mode unchanged.
                    DemoTransport.write(self, encode(Frame(GET_STATUS, frame.seq)))
                else:
                    super().write(data)

        for requested in ("auto", "manual"):
            with self.subTest(requested=requested):
                device = IgnoredMode(supports_auto=True)
                device.enabled = True
                device.auto_mode = requested == "manual"
                actual = "auto" if device.auto_mode else "manual"
                control = ServoController(transport_factory=lambda *_: device)
                try:
                    control.connect(demo=True)
                    wait_until(lambda: control.snapshot()["connected"])
                    control.poll_events()
                    control.set_mode(requested)
                    wait_until(lambda: not control.snapshot()["pending"])
                    state = control.snapshot()
                    self.assertTrue(state["connected"])
                    self.assertEqual(state["mode"], actual)
                    self.assertIn("控制模式未切换", state["error"])
                    self.assertIn("请求自动跟随" if requested == "auto" else "请求手动模式", state["error"])
                    events = "\n".join(control.poll_events())
                    self.assertIn("控制模式未切换", events)
                    self.assertNotIn("控制模式已确认", events)
                    self.assertNotIn(ENABLE, device.kinds)
                finally:
                    control.close()

    def test_lost_auto_reply_marks_disconnected_before_closing_hold(self):
        class LostModeReply(SpyDemo):
            closing_snapshot = None

            def write(self, data):
                frame = StreamParser().feed(data)[0]
                if frame.kind == HOLD:
                    self.closing_snapshot = control.snapshot()
                super().write(data)
                if frame.kind == SET_MODE:
                    self.output.clear()

        device = LostModeReply(supports_auto=True)
        control = ServoController(transport_factory=lambda *_: device)
        try:
            control.connect(demo=True)
            wait_until(lambda: control.snapshot()["connected"])
            control.enable(True)
            wait_until(lambda: not control.snapshot()["pending"])
            control.poll_events()
            control.set_mode("auto")
            wait_until(lambda: device.closing_snapshot is not None)
            self.assertFalse(device.closing_snapshot["connected"])
            self.assertFalse(device.closing_snapshot["pending"])
            self.assertIn("未收到开发板有效回复", device.closing_snapshot["error"])
            self.assertIn("自动跟随状态无法确认", device.closing_snapshot["error"])
            self.assertNotIn("控制模式已确认", "\n".join(control.poll_events()))
            wait_until(lambda: not device.auto_mode)
        finally:
            control.close()

    def test_hold_preempts_lost_motion_reply(self):
        device = SpyDemo(drop_target=True)
        control = ServoController(transport_factory=lambda *_: device)
        try:
            control.connect(demo=True)
            wait_until(lambda: control.snapshot()["connected"])
            control.enable(True)
            wait_until(lambda: not control.snapshot()["pending"])
            control.set_angles(100, 0)
            wait_until(lambda: SET_TARGET in device.kinds)
            start = time.monotonic()
            control.hold()
            wait_until(lambda: HOLD in device.kinds, timeout=0.3)
            wait_until(lambda: not control.snapshot()["pending"])
            self.assertLess(time.monotonic() - start, 0.3)
            self.assertTrue(control.snapshot()["connected"])
            self.assertEqual(device.current, device.target)
        finally:
            control.close()

    def test_reconnect_readonly_and_disable_preempts(self):
        device = SpyDemo(drop_target=True)
        device.enabled = True
        device.current, device.target = [1550, 1500], [1550, 1500]
        control = ServoController(transport_factory=lambda *_: device)
        try:
            control.connect(demo=True)
            wait_until(lambda: control.snapshot()["connected"])
            self.assertEqual(device.current, [1550, 1500])
            self.assertNotIn(ENABLE, device.kinds)
            control.set_angles(100, 0)
            wait_until(lambda: SET_TARGET in device.kinds)
            control.enable(False)
            wait_until(lambda: not control.snapshot()["pending"])
            self.assertFalse(device.enabled)
            self.assertTrue(control.snapshot()["connected"])
        finally:
            control.close()

    def test_disable_preempts_lost_hold_reply(self):
        class DropHold(SpyDemo):
            def write(self, data):
                super().write(data)
                if StreamParser().feed(data)[0].kind == HOLD:
                    self.output.clear()
        device = DropHold()
        control = ServoController(transport_factory=lambda *_: device)
        try:
            control.connect(demo=True)
            wait_until(lambda: control.snapshot()["connected"])
            control.enable(True)
            wait_until(lambda: not control.snapshot()["pending"])
            control.hold()
            wait_until(lambda: HOLD in device.kinds)
            control.enable(False)
            wait_until(lambda: not device.enabled, timeout=0.3)
            wait_until(lambda: not control.snapshot()["pending"])
            self.assertTrue(control.snapshot()["connected"])
        finally:
            control.close()

    def test_wrong_device_never_connects_or_sends_motion(self):
        class WrongDevice(SpyDemo):
            def write(self, data):
                super().write(data)
                frame = StreamParser().feed(data)[0]
                if frame.kind == HELLO:
                    self.output[:] = encode(Frame(HELLO_REPLY, frame.seq, 0, b"NOPE\x01\x00\x01\x00"))
        device = WrongDevice()
        control = ServoController(transport_factory=lambda *_: device)
        control.connect(demo=True)
        wait_until(lambda: not control.snapshot()["connecting"])
        self.assertFalse(control.snapshot()["connected"])
        self.assertEqual(device.kinds, [HELLO])
        control.close()


class ReplyDropDemo(SpyDemo):
    """Record decoded requests and drop selected replies without real serial I/O."""
    def __init__(self, drop_kind=None, drop_remaining=0, supports_auto=False):
        super().__init__(supports_auto=supports_auto)
        self.drop_kind = drop_kind
        self.drop_remaining = drop_remaining  # -1 drops indefinitely.
        self.frames = []
        self.dropped = []

    def write(self, data):
        frame = StreamParser().feed(data)[0]
        self.frames.append(frame)
        super().write(data)
        if frame.kind == self.drop_kind and self.drop_remaining:
            if self.drop_remaining > 0:
                self.drop_remaining -= 1
            self.output.clear()
            self.dropped.append(frame)


class ReadOnlyRetryTests(unittest.TestCase):
    def control_for(self, device):
        control = ServoController(transport_factory=lambda *_: device)
        self.addCleanup(control.close)
        return control

    def connected_control(self, device):
        control = self.control_for(device)
        control.connect(demo=True)
        wait_until(lambda: control.snapshot()["connected"])
        return control

    def test_handshake_queries_retry_once_with_fresh_sequence(self):
        for kind in (HELLO, GET_LIMITS, GET_STATUS):
            with self.subTest(kind=kind):
                device = ReplyDropDemo(kind, 1)
                control = self.connected_control(device)
                try:
                    queries = [frame for frame in device.frames if frame.kind == kind]
                    self.assertEqual(len(queries), 2)
                    self.assertNotEqual(queries[0].seq, queries[1].seq)
                    self.assertEqual(len(device.dropped), 1)
                    self.assertFalse(control.snapshot()["enabled"])
                    self.assertEqual(control.snapshot()["error"], "")
                    self.assertTrue(all(frame.kind in (HELLO, GET_LIMITS, GET_STATUS)
                                        for frame in device.frames))
                finally:
                    control.close()

    def test_lost_heartbeat_reply_recovers_without_disconnect(self):
        device = ReplyDropDemo()
        control = self.connected_control(device)
        revision = control.snapshot()["status_revision"]
        device.drop_kind, device.drop_remaining = GET_STATUS, 1
        wait_until(lambda: device.dropped)
        self.assertTrue(control.snapshot()["connected"])
        wait_until(lambda: control.snapshot()["status_revision"] > revision)
        state = control.snapshot()
        self.assertTrue(state["connected"])
        self.assertFalse(state["pending"])
        self.assertEqual(state["error"], "")
        dropped = device.dropped[0]
        retry = device.frames[device.frames.index(dropped) + 1]
        self.assertEqual(retry.kind, GET_STATUS)
        self.assertNotEqual(retry.seq, dropped.seq)
        self.assertNotIn(ENABLE, device.kinds)
        self.assertNotIn(SET_MODE, device.kinds)

    def test_consecutive_heartbeat_drops_disconnect_after_one_retry(self):
        device = ReplyDropDemo()
        control = self.connected_control(device)
        status_count = device.kinds.count(GET_STATUS)
        device.drop_kind, device.drop_remaining = GET_STATUS, -1
        wait_until(lambda: not control.snapshot()["connected"])
        wait_until(lambda: device.closed)
        self.assertEqual(device.kinds.count(GET_STATUS), status_count + 2)
        self.assertEqual(len(device.dropped), 2)
        self.assertIn("未收到开发板有效回复", control.snapshot()["error"])
        self.assertEqual(device.kinds.count(HOLD), 1)

    def test_io_errors_are_not_retried(self):
        from serial import SerialException, SerialTimeoutException

        for error_type in (OSError, TimeoutError, SerialException, SerialTimeoutException):
            for operation in ("read", "write"):
                with self.subTest(error_type=error_type, operation=operation):
                    failure = error_type("transport failure")

                    class BrokenTransport(SpyDemo):
                        def write(self, data):
                            super().write(data)
                            if operation == "write":
                                raise failure

                        def read(self):
                            raise failure

                    device = BrokenTransport()
                    control = self.control_for(device)
                    with self.assertRaises(error_type) as caught:
                        control._query(device, StreamParser(), HELLO)
                    self.assertIs(caught.exception, failure)
                    self.assertEqual(device.kinds, [HELLO])

    def test_io_failure_during_heartbeat_disconnects_without_retry(self):
        class BrokenHeartbeat(ReplyDropDemo):
            fail = False

            def write(self, data):
                frame = StreamParser().feed(data)[0]
                if self.fail and frame.kind == GET_STATUS:
                    self.kinds.append(frame.kind)
                    raise OSError("test serial disconnected")
                super().write(data)

        device = BrokenHeartbeat()
        control = self.connected_control(device)
        status_count = device.kinds.count(GET_STATUS)
        device.fail = True
        wait_until(lambda: not control.snapshot()["connected"])
        wait_until(lambda: device.closed)
        self.assertEqual(device.kinds.count(GET_STATUS), status_count + 1)
        self.assertIn("test serial disconnected", control.snapshot()["error"])

    def test_watchdog_reply_after_retry_keeps_actual_manual_mode(self):
        class WatchdogAfterDrop(ReplyDropDemo):
            expired = False

            def write(self, data):
                if StreamParser().feed(data)[0].kind == GET_STATUS and self.dropped and not self.expired:
                    # Simulate the FPGA watchdog expiring during the missing reply.
                    self.last_request = self.clock() - 2
                    self.expired = True
                super().write(data)

        device = WatchdogAfterDrop(supports_auto=True)
        control = self.connected_control(device)
        control.enable(True)
        wait_until(lambda: not control.snapshot()["pending"])
        control.set_mode("auto")
        wait_until(lambda: not control.snapshot()["pending"])
        self.assertEqual(control.snapshot()["mode"], "auto")
        device.drop_kind, device.drop_remaining = GET_STATUS, 1
        wait_until(lambda: control.snapshot()["watchdog"])
        state = control.snapshot()
        self.assertTrue(state["connected"])
        self.assertEqual(state["mode"], "manual")
        self.assertTrue(state["watchdog"])
        self.assertFalse(device.auto_mode)
        self.assertEqual(device.kinds.count(SET_MODE), 1)
        self.assertEqual(device.kinds.count(ENABLE), 1)
        self.assertEqual(state["error"], "")

    def test_motion_missing_ack_is_not_replayed(self):
        device = ReplyDropDemo(SET_TARGET, -1)
        device.enabled = True
        control = self.connected_control(device)
        control.set_angles(100, 0)
        wait_until(lambda: not control.snapshot()["connected"])
        wait_until(lambda: device.closed)
        self.assertEqual(device.kinds.count(SET_TARGET), 1)
        self.assertEqual(device.kinds.count(HOLD), 1)
        self.assertIn("未收到开发板有效回复", control.snapshot()["error"])
        self.assertNotIn("角度指令已确认", "\n".join(control.poll_events()))

    def test_query_rejects_actions_before_writing(self):
        device = SpyDemo()
        control = self.control_for(device)
        for kind in (SET_TARGET, ENABLE, HOLD, HOME, SET_LIMITS, SET_MODE):
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                control._query(device, StreamParser(), kind)
        self.assertEqual(device.kinds, [])

    def test_hold_interrupts_heartbeat_retry(self):
        device = ReplyDropDemo()
        device.enabled = True
        control = self.connected_control(device)
        status_count = device.kinds.count(GET_STATUS)
        device.drop_kind, device.drop_remaining = GET_STATUS, -1
        wait_until(lambda: device.kinds.count(GET_STATUS) == status_count + 2)
        start = time.monotonic()
        control.hold()
        wait_until(lambda: HOLD in device.kinds, timeout=0.3)
        wait_until(lambda: not control.snapshot()["pending"])
        self.assertLess(time.monotonic() - start, 0.3)
        self.assertTrue(control.snapshot()["connected"])
        self.assertEqual(control.snapshot()["error"], "")
        self.assertEqual(device.kinds[:device.kinds.index(HOLD)].count(GET_STATUS), status_count + 2)
        device.drop_remaining = 0

    def test_disconnect_interrupts_heartbeat_retry(self):
        device = ReplyDropDemo()
        control = self.connected_control(device)
        status_count = device.kinds.count(GET_STATUS)
        device.drop_kind, device.drop_remaining = GET_STATUS, -1
        wait_until(lambda: device.kinds.count(GET_STATUS) == status_count + 2)
        start = time.monotonic()
        control.disconnect()
        wait_until(lambda: device.closed, timeout=0.3)
        self.assertLess(time.monotonic() - start, 0.3)
        self.assertFalse(control.snapshot()["connected"])
        self.assertEqual(device.kinds.count(GET_STATUS), status_count + 2)

    def test_exchange_missing_reply_has_distinct_exception(self):
        device = ReplyDropDemo(HELLO, -1)
        control = self.control_for(device)
        with self.assertRaises(ResponseTimeout):
            control._exchange(device, StreamParser(), HELLO, timeout=0.02)
        self.assertEqual(device.kinds, [HELLO])

class MountCoordinateTests(unittest.TestCase):
    def test_limits_reply_does_not_count_as_a_fresh_position_reply(self):
        control = ServoController()
        control._apply_reply(Frame(STATUS, 1, 0, pack_us(1500, 1540, 1500, 1540)))
        revision = control.snapshot()["status_revision"]
        control._apply_reply(Frame(LIMITS, 2, 0, pack_us(500, 2500, 1000, 2000)))
        self.assertEqual(control.snapshot()["status_revision"], revision)
        self.assertEqual(control.snapshot()["tilt_pulse_us"], 1540)
        control._apply_reply(Frame(STATUS, 3, 0, pack_us(1500, 1500, 1500, 1500)))
        self.assertGreater(control.snapshot()["status_revision"], revision)
        self.assertEqual(control.snapshot()["tilt_pulse_us"], 1500)

    def test_full_ranges_home_and_inverse_both_directions(self):
        for reversed_axis in (False, True):
            pan, tilt = MountAxis(90, 180, reversed_axis), MountAxis(0, 45, reversed_axis, minimum=-45)
            self.assertEqual(pan.to_us(90), 1500)
            self.assertEqual(tilt.to_us(0), 1500)
            self.assertEqual(pan.pulse_limits(0, 180), [500, 2500])
            self.assertEqual(tilt.pulse_limits(-45, 45), [1000, 2000])
            self.assertEqual(tilt.to_us(-45), 2000 if reversed_axis else 1000)
            self.assertEqual(tilt.to_us(45), 1000 if reversed_axis else 2000)
            self.assertEqual(tilt.pulse_limits(-30, 10), [1389, 1833] if reversed_axis else [1167, 1611])
            self.assertEqual(tilt.angle_limits(1460, 1540), [-3.6, 3.6])
            for axis in (pan, tilt):
                for angle in (axis.minimum, axis.minimum + 1.8, axis.neutral, axis.maximum):
                    self.assertAlmostEqual(axis.from_us(axis.to_us(angle)), angle, delta=0.05)
                for invalid in (axis.minimum - 0.1, axis.maximum + 0.1, math.nan, math.inf):
                    with self.assertRaises(ValueError): axis.to_us(invalid)
            with self.assertRaises(ValueError): tilt.pulse_limits(1, 45)
            with self.assertRaises(ValueError): pan.pulse_limits(100, 180)

    def test_apply_range_and_home_preserve_pulse_origin(self):
        device = SpyDemo()
        control = ServoController(transport_factory=lambda *_: device)
        try:
            control.connect(demo=True)
            wait_until(lambda: control.snapshot()["connected"])
            self.assertEqual(control.snapshot()["tilt_angle"], 0)
            self.assertEqual(control.snapshot()["tilt_min"], -3.6)
            self.assertEqual(control.snapshot()["tilt_input_min"], -3.6)
            control.apply_mount_range()
            wait_until(lambda: not control.snapshot()["pending"])
            self.assertTrue(control.snapshot()["profile_ready"])
            self.assertEqual(device.limits, [500, 2500, 1000, 2000])
            self.assertEqual(device.current, [1500, 1500])
            self.assertNotIn(ENABLE, device.kinds)
            self.assertNotIn(SET_TARGET, device.kinds)
            control.enable(True)
            wait_until(lambda: not control.snapshot()["pending"])
            control.set_angles(180, 45)
            wait_until(lambda: not control.snapshot()["pending"])
            self.assertEqual(device.target, [2500, 2000])
            control.set_angles(90, -45)
            wait_until(lambda: not control.snapshot()["pending"])
            self.assertEqual(device.target, [1500, 1000])
            self.assertEqual(control.snapshot()["target_tilt"], -45)
            with self.assertRaises(RuntimeError): control.set_directions(True, True)
            with self.assertRaises(ValueError): control.set_angles(90, -45.1)
            with self.assertRaises(ValueError): control.set_angles(90, 45.1)
            control.home()
            wait_until(lambda: not control.snapshot()["pending"])
            self.assertEqual(device.target, [1500, 1500])
            self.assertEqual((control.snapshot()["target_pan"], control.snapshot()["target_tilt"]), (90, 0))
        finally:
            control.close()

    def test_reverse_reprojects_raw_values_without_motion_and_keeps_symmetric_limits(self):
        device = SpyDemo()
        device.current, device.target = [1600, 1540], [1600, 1540]
        control = ServoController(transport_factory=lambda *_: device)
        try:
            control.connect(demo=True)
            wait_until(lambda: control.snapshot()["connected"])
            control.apply_mount_range()
            wait_until(lambda: not control.snapshot()["pending"])
            before = device.kinds.count(SET_LIMITS)
            control.set_directions(True, True)
            state = control.snapshot()
            self.assertEqual((state["pan_angle"], state["tilt_angle"]), (81, -3.6))
            self.assertEqual((state["tilt_min"], state["tilt_max"]), (-45, 45))
            self.assertTrue(state["profile_ready"])
            self.assertEqual(device.current, [1600, 1540])
            self.assertEqual(device.kinds.count(SET_LIMITS), before)
            self.assertNotIn(ENABLE, device.kinds)
            self.assertNotIn(SET_TARGET, device.kinds)
            control.apply_mount_range()
            wait_until(lambda: not control.snapshot()["pending"])
            self.assertEqual(device.limits, [500, 2500, 1000, 2000])
            self.assertTrue(control.snapshot()["profile_ready"])
            control.enable(True)
            wait_until(lambda: not control.snapshot()["pending"])
            control.set_angles(180, 45)
            wait_until(lambda: not control.snapshot()["pending"])
            self.assertEqual(device.target, [500, 1000])
        finally:
            control.close()

    def test_signed_custom_limits_survive_reverse_and_require_home(self):
        control = ServoController()
        control._apply_reply(Frame(LIMITS, 1, 0, pack_us(500, 2500, 1278, 2000)))
        state = control.snapshot()
        self.assertAlmostEqual(state["tilt_input_min"], -19.98)
        self.assertEqual(state["tilt_input_max"], 45)
        control.set_directions(False, True)
        state = control.snapshot()
        self.assertEqual(state["tilt_input_min"], -45)
        self.assertAlmostEqual(state["tilt_input_max"], 19.98)
        tilt = MountAxis(0, 45, minimum=-45)
        for bounds in ((-45, -1), (1, 45), (-46, 0), (0, 46), (math.nan, 45)):
            with self.subTest(bounds=bounds), self.assertRaises(ValueError):
                tilt.pulse_limits(*bounds)

    def test_direction_preferences_persist_without_hardware_or_motion(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "directions.json"
            control = ServoController(directions_path=path)
            control.set_directions(True, True)
            restored = ServoController(directions_path=path)
            self.assertTrue(restored.snapshot()["pan_reversed"])
            self.assertTrue(restored.snapshot()["tilt_reversed"])
            self.assertFalse(restored.snapshot()["connected"])
            path.write_text('{"version": 1, "pan_reversed": "false", "tilt_reversed": false}', encoding="utf-8")
            with self.assertRaises(ValueError): ServoController(directions_path=path)


if __name__ == "__main__":
    unittest.main()

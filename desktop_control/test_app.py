"""Tk control-chain tests; run with GUI permission, never with real serial.

Run: python -m unittest desktop_control.test_app -v
The windows are withdrawn. Tests invoke the actual widgets and pump Tk until
observable state changes, rather than relying on a short fixed sleep.
"""
import threading
import time
import tkinter as tk
import unittest
from unittest.mock import patch

from .app import ServoApp
from .ports import PortChoice
from .controller import ServoController
from .protocol import (ENABLE, GET_LIMITS, GET_STATUS, HELLO, HOLD, HOME,
                       SET_LIMITS, SET_MODE, SET_TARGET, Frame, StreamParser, angle_to_us, encode, unpack_us)
from .transport import DemoTransport


class RecordingDemo(DemoTransport):
    """Record requests and optionally withhold one command type's replies."""
    def __init__(self):
        super().__init__()
        self._record_lock = threading.Lock()
        self._requests = []
        self._withhold_kind = None
        self.ignore_mode = False
        self.fail_reads = False

    def withhold_replies(self, kind):
        with self._record_lock:
            self._withhold_kind = kind

    def requests(self, kind=None):
        with self._record_lock:
            return [frame for frame in self._requests if kind is None or frame.kind == kind]

    def write(self, data):
        frames = StreamParser().feed(data)
        if self.ignore_mode and len(frames) == 1 and frames[0].kind == SET_MODE:
            super().write(encode(Frame(GET_STATUS, frames[0].seq)))
        else:
            super().write(data)
        with self._record_lock:
            self._requests.extend(frames)
            if any(frame.kind == self._withhold_kind for frame in frames):
                self.output.clear()

    def read(self):
        if self.fail_reads:
            raise OSError("模拟串口连接中断")
        return super().read()


class RecordingController(ServoController):
    def __init__(self, device):
        self.connection_requests = []
        self.close_calls = 0

        def factory(port, demo):
            if not demo:
                raise AssertionError("UI tests must never connect to real hardware")
            self.connection_requests.append((port, demo))
            return device

        super().__init__(transport_factory=factory)

    @staticmethod
    def ports():
        return []  # Do not enumerate, select or open a real Windows device.

    def _exchange(self, *args, **kwargs):
        # Held replies keep the pending UI state deterministic on slow GUI hosts.
        # Preserve explicitly supplied closing timeouts and production semantics.
        kwargs.setdefault("timeout", 5.0)
        return super()._exchange(*args, **kwargs)

    def close(self):
        self.close_calls += 1
        super().close()


class ServoAppTests(unittest.TestCase):
    def setUp(self):
        self.serial_guard = patch("desktop_control.controller.SerialTransport",
                                  side_effect=AssertionError("Real serial is forbidden in UI tests"))
        self.serial_guard.start()
        self.addCleanup(self.serial_guard.stop)
        self.root = tk.Tk()
        self.root.withdraw()
        self.callback_errors = []
        self.root.report_callback_exception = lambda *args: self.callback_errors.append(args)
        self.device = RecordingDemo()
        self.controller = RecordingController(self.device)
        self.app = ServoApp(self.root, self.controller)
        # Drain ttk theme work before fast metadata-only cases destroy this root.
        self.root.update_idletasks()
        self._yield = threading.Event()

    def tearDown(self):
        if hasattr(self, "app") and not self.app.closed:
            self._cancel_timers()
            self.app.close()
        elif hasattr(self, "controller") and self.controller._thread:
            self.controller._thread.join(timeout=1.2)
        self.assertFalse(self.callback_errors, "Tk callback raised an exception")

    def _cancel_timers(self):
        # Prevent callbacks from a destroyed interpreter leaking into the next
        # test's Tk event loop; this does not replace the application's close.
        for timer in self.root.tk.call("after", "info"):
            self.root.after_cancel(timer)

    def wait_until(self, predicate, description, timeout=4.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.root.update()
            self.assertFalse(self.callback_errors, "Tk callback raised an exception")
            self.assertNotIn("界面状态更新失败", self.app.message_var.get())
            if predicate():
                return
            self._yield.wait(0.01)
        self.fail(f"Timed out: {description}; state={self.controller.snapshot()}; "
                  f"message={self.app.message_var.get()!r}")

    def connect_demo(self):
        self.app.demo_check.invoke()
        self.assertTrue(self.app.demo_var.get())
        self.app.connect_button.invoke()
        self.wait_until(lambda: self.app._snapshot.get("connected"), "demo handshake and UI poll")
        self.assertEqual(self.controller.connection_requests, [("", True)])
        self.assertIn("演示模式", self.app.badge.cget("text"))

    def enable_output(self):
        self.app.enable_button.invoke()
        self.wait_until(lambda: self.app._snapshot.get("enabled") and
                        not self.app._snapshot.get("pending"), "enable acknowledgement")
        self.assertEqual(self.device.requests(ENABLE)[-1].payload[0], 1)

    def edit(self, pan, tilt):
        # Exercise Entry -> StringVar -> trace -> dirty-state wiring.
        for axis, value in (("pan", pan), ("tilt", tilt)):
            entry = self.app.angle_entries[axis]
            entry.delete(0, tk.END)
            entry.insert(0, str(value))

    def wait_for_more_polls(self, count=2):
        before = len(self.device.requests(GET_STATUS))
        self.wait_until(lambda: len(self.device.requests(GET_STATUS)) >= before + count,
                        "background status polling")

    def test_connect_and_edit_are_readonly_and_edits_survive_polling(self):
        self.connect_demo()
        self.assertEqual([f.kind for f in self.device.requests()[:3]],
                         [HELLO, GET_LIMITS, GET_STATUS])
        self.assertFalse(self.controller.snapshot()["enabled"])
        self.assertTrue(self.app.send_button.instate(["disabled"]))
        self.edit("100", "1.8")
        self.wait_for_more_polls()
        self.assertEqual(self.app.angle_entries["pan"].get(), "100")
        self.assertEqual(self.app.angle_entries["tilt"].get(), "1.8")
        self.assertEqual(self.device.requests(SET_TARGET), [])
        self.assertEqual(self.device.requests(ENABLE), [])
        self.assertEqual(self.device.current, [1500, 1500])

    def test_enable_explicit_send_and_home_use_the_actual_buttons(self):
        self.connect_demo()
        self.enable_output()
        self.edit("100", "1.8")
        self.wait_for_more_polls()
        self.assertEqual(self.device.requests(SET_TARGET), [])
        self.app.send_button.invoke()
        self.wait_until(lambda: len(self.device.requests(SET_TARGET)) == 1 and
                        not self.app._snapshot.get("pending") and
                        self.app._snapshot.get("target_pan", 0) > 99,
                        "explicit target acknowledgement")
        values = unpack_us(self.device.requests(SET_TARGET)[0].payload)
        self.assertEqual(values, (angle_to_us(100), angle_to_us(91.8), 0, 0))
        self.assertEqual(self.app.angle_entries["pan"].get(), "100")
        self.app.home_button.invoke()
        self.wait_until(lambda: len(self.device.requests(HOME)) == 1 and
                        not self.app._snapshot.get("pending") and
                        self.app.angle_entries["pan"].get() == "90" and
                        self.app.angle_entries["tilt"].get() == "0", "home updates both input fields")
        self.wait_until(lambda: self.controller.snapshot()["pan_angle"] == 90 and
                        self.controller.snapshot()["tilt_angle"] == 0 and
                        not self.controller.snapshot()["moving"], "neutral command reached")

    def test_out_of_range_and_invalid_entries_do_not_send(self):
        self.connect_demo()
        self.enable_output()
        for pan, tilt in (("109", "0"), ("90", "4"), ("nan", "0"), ("abc", "0"), ("90", "-4")):
            with self.subTest(pan=pan, tilt=tilt):
                self.edit(pan, tilt)
                self.app.send_button.invoke()
                self.assertTrue(self.app._ui_error)
                self.assertEqual(self.device.requests(SET_TARGET), [])
        self.wait_for_more_polls()
        self.assertEqual(self.device.requests(SET_TARGET), [])
        self.assertEqual(self.device.target, [1500, 1500])

    def test_hold_and_disable_remain_usable_while_pending(self):
        self.connect_demo()
        self.enable_output()
        self.device.withhold_replies(SET_TARGET)
        self.edit("100", "0")
        self.app.send_button.invoke()
        self.wait_until(lambda: bool(self.device.requests(SET_TARGET)) and
                        self.app._snapshot.get("pending"), "pending target visible in UI")
        self.assertTrue(self.app.send_button.instate(["disabled"]))
        self.assertTrue(self.app.home_button.instate(["disabled"]))
        self.assertTrue(self.app.hold_button.instate(["!disabled"]))
        self.assertTrue(self.app.enable_button.instate(["!disabled"]))
        self.app.hold_button.invoke()
        self.wait_until(lambda: bool(self.device.requests(HOLD)) and
                        not self.app._snapshot.get("pending"), "HOLD button preempts pending target")
        self.assertEqual(self.device.current, self.device.target)

        self.device.withhold_replies(HOLD)
        before = len(self.device.requests(HOLD))
        self.app.hold_button.invoke()
        self.wait_until(lambda: len(self.device.requests(HOLD)) > before and
                        self.app._snapshot.get("pending"), "pending HOLD visible in UI")
        self.assertTrue(self.app.enable_button.instate(["!disabled"]))
        self.assertIn("关闭", self.app.enable_button.cget("text"))
        self.app.enable_button.invoke()
        self.wait_until(lambda: not self.app._snapshot.get("enabled") and
                        not self.app._snapshot.get("pending"), "disable button preempts pending HOLD")
        self.assertEqual(self.device.requests(ENABLE)[-1].payload[0], 0)
        self.assertFalse(self.device.enabled)
        self.assertTrue(self.controller.snapshot()["connected"])

    def test_apply_full_mount_range_is_explicit_and_does_not_enable_or_move(self):
        self.connect_demo()
        self.assertEqual(self.device.requests(SET_LIMITS), [])
        self.app.range_button.invoke()
        self.wait_until(lambda: self.app._snapshot.get("profile_ready") and
                        not self.app._snapshot.get("pending"), "full mount range reply")
        self.assertEqual(self.device.limits, [500, 2500, 1000, 2000])
        self.assertEqual(self.device.requests(ENABLE), [])
        self.assertEqual(self.device.requests(SET_TARGET), [])
        self.assertEqual(self.device.current, [1500, 1500])
        self.assertEqual(float(self.app.scale_widgets["tilt"].cget("from")), -45)
        self.assertEqual(float(self.app.scale_widgets["tilt"].cget("to")), 45)
        self.assertEqual(float(self.app.scale_widgets["pan"].cget("to")), 180)
        self.enable_output()
        self.edit("180", "45")
        self.app.send_button.invoke()
        self.wait_until(lambda: bool(self.device.requests(SET_TARGET)) and
                        not self.app._snapshot.get("pending"), "full range command in simulation")
        self.assertEqual(unpack_us(self.device.requests(SET_TARGET)[-1].payload), (2500, 2000, 0, 0))
        self.assertTrue(self.app.range_button.instate(["disabled"]))
        self.assertTrue(self.app.direction_save.instate(["disabled"]))

    def test_directions_are_explicit_and_old_negative_feedback_is_preserved(self):
        self.device.current = [1600, 1540]
        self.device.target = [1600, 1540]
        self.connect_demo()
        self.assertIn("-3.6", self.app._ranges["tilt"].get())
        self.assertEqual(self.app._axis_limits("tilt"), (-3.6, 3.6))
        self.app.direction_checks[0].invoke()
        self.app.direction_checks[1].invoke()
        self.assertFalse(self.controller.snapshot()["pan_reversed"])
        self.app.range_button.invoke()
        self.assertIn("先保存方向", self.app._ui_error)
        self.assertEqual(self.device.requests(SET_LIMITS), [])
        self.edit("100", "1")
        self.app.direction_save.invoke()
        self.wait_until(lambda: self.app.angle_entries["pan"].get() == "81" and
                        self.app.angle_entries["tilt"].get() == "-3.6", "reproject current targets after direction change")
        self.assertIn("-3.6", self.app._readbacks["tilt"].get())
        self.assertEqual(self.device.requests(SET_TARGET), [])
        self.assertEqual(self.device.requests(ENABLE), [])
        self.assertEqual(self.device.requests(SET_LIMITS), [])
        self.app.range_button.invoke()
        self.wait_until(lambda: self.app._snapshot.get("profile_ready") and
                        not self.app._snapshot.get("pending"), "reverse symmetric range applied")
        self.assertEqual(self.device.limits, [500, 2500, 1000, 2000])

    def test_reverse_full_range_preserves_symmetric_tilt_inputs(self):
        self.connect_demo()
        self.app.range_button.invoke()
        self.wait_until(lambda: self.app._snapshot.get("profile_ready") and
                        not self.app._snapshot.get("pending"), "normal range ready")
        self.app.direction_checks[1].invoke()
        self.app.direction_save.invoke()
        self.wait_until(lambda: self.app._snapshot.get("tilt_reversed") and
                        self.app._snapshot.get("profile_ready"), "reverse symmetric range")
        self.assertEqual(self.app._axis_limits("tilt"), (-45, 45))
        self.assertIn("-45", self.app._ranges["tilt"].get())
        self.assertEqual(float(self.app.scale_widgets["tilt"].cget("from")), -45)
        self.assertEqual(float(self.app.scale_widgets["tilt"].cget("to")), 45)
        self.assertEqual(self.device.requests(ENABLE), [])
        self.assertEqual(self.device.requests(SET_TARGET), [])
        self.enable_output()
        self.edit("90", "-45")
        self.app.send_button.invoke()
        self.wait_until(lambda: bool(self.device.requests(SET_TARGET)) and
                        not self.app._snapshot.get("pending") and
                        self.app._snapshot.get("target_tilt") == -45, "reverse negative tilt command")
        self.assertEqual(unpack_us(self.device.requests(SET_TARGET)[-1].payload), (1500, 2000, 0, 0))

    def test_negative_tilt_limits_and_command_use_actual_widgets(self):
        self.assertEqual(self.app._axis_limits("tilt"), (-45, 45))
        self.connect_demo()
        for key, value in (("pan_min", "0"), ("pan_max", "180"),
                           ("tilt_min", "-18"), ("tilt_max", "27")):
            self.app._limit_vars[key].set(value)
        self.app.limits_save.invoke()
        self.wait_until(lambda: bool(self.device.requests(SET_LIMITS)) and
                        not self.app._snapshot.get("pending") and
                        self.app._axis_limits("tilt") == (-18, 27), "negative tilt limit visible in UI")
        self.assertEqual(self.device.limits, [500, 2500, 1300, 1800])
        self.assertEqual(self.app._axis_limits("tilt"), (-18, 27))
        self.assertEqual(self.device.requests(ENABLE), [])
        self.enable_output()
        self.edit("90", "-9")
        self.app.send_button.invoke()
        self.wait_until(lambda: bool(self.device.requests(SET_TARGET)) and
                        not self.app._snapshot.get("pending") and
                        self.app._snapshot.get("target_tilt") == -9, "negative tilt target acknowledged")
        self.assertEqual(unpack_us(self.device.requests(SET_TARGET)[-1].payload), (1500, 1400, 0, 0))
        self.assertEqual(self.app.angle_entries["tilt"].get(), "-9")
        self.edit("90", "-19")
        self.app.send_button.invoke()
        self.assertEqual(len(self.device.requests(SET_TARGET)), 1)
        self.assertTrue(self.app._ui_error)

    def test_enable_home_replaces_dirty_target_coordinates(self):
        self.connect_demo()
        self.edit("100", "1.8")
        self.enable_output()
        self.wait_until(lambda: self.app.angle_entries["pan"].get() == "90" and
                        self.app.angle_entries["tilt"].get() == "0", "enable homes both input fields")
        self.assertEqual(self.device.target, [1500, 1500])
        self.assertEqual(self.device.requests(SET_TARGET), [])

    def test_auto_button_requires_capability_and_explicit_enable(self):
        self.device.supports_auto = True
        self.connect_demo()
        self.assertTrue(self.app.auto_button.instate(["disabled"]))
        self.assertTrue(self.app.tracking_hold_button.instate(["disabled"]))
        self.assertEqual(self.app.tracking_hold_button.cget("text"), "停止跟随 / 保持")
        self.app.tracking_hold_button.invoke()
        self.app.auto_button.invoke()
        self.assertEqual(self.device.requests(SET_MODE), [])
        self.assertEqual(self.device.requests(ENABLE), [])
        self.assertEqual(self.device.requests(HOLD), [])
        self.enable_output()
        self.assertTrue(self.app.auto_button.instate(["!disabled"]))
        self.assertTrue(self.app.tracking_hold_button.instate(["!disabled"]))
        self.edit("100", "1.8")
        self.app.auto_button.invoke()
        self.wait_until(lambda: self.app._snapshot.get("mode") == "auto" and
                        not self.app._snapshot.get("pending"), "auto acknowledged")
        self.assertTrue(self.app.send_button.instate(["disabled"]))
        self.assertTrue(self.app.home_button.instate(["disabled"]))
        self.assertTrue(self.app.angle_entries["pan"].instate(["disabled"]))
        self.assertEqual(self.app.angle_entries["pan"].get(), "90")
        self.assertEqual(self.device.requests(SET_TARGET), [])
        self.assertEqual(self.app.auto_button.cget("text"), "切回手动并保持")
        self.assertTrue(self.app.hold_button.instate(["!disabled"]))
        self.assertEqual(self.app.hold_button.cget("text"), "停止跟随 / 保持")
        self.assertEqual(self.app.tracking_hold_button.cget("text"), "停止跟随 / 保持")
        self.assertEqual(self.app.tracking_hold_button.master, self.app.auto_button.master)
        # Emulate new board PWM commands while auto is active, not camera detection.
        self.device.current, self.device.target = [1600, 1520], [1600, 1520]
        self.app.auto_button.invoke()
        self.wait_until(lambda: self.app._snapshot.get("mode") == "manual" and
                        not self.app._snapshot.get("pending") and
                        self.app.angle_entries["pan"].get() == "99", "manual retains latest board command")
        self.assertEqual(self.app.angle_entries["tilt"].get(), "1.8")
        self.assertTrue(self.app.send_button.instate(["!disabled"]))
        self.assertTrue(self.app.angle_entries["pan"].instate(["!disabled"]))
        self.assertEqual(self.device.current, [1600, 1520])
        self.assertEqual(self.device.requests(SET_TARGET), [])

    def test_unsupported_firmware_cannot_enter_auto(self):
        self.connect_demo()
        self.enable_output()
        self.assertTrue(self.app.auto_button.instate(["disabled"]))
        self.assertIn("仅支持手动", self.app.auto_button.cget("text"))
        self.app.auto_button.invoke()
        self.assertEqual(self.device.requests(SET_MODE), [])

    def test_hold_preempts_pending_auto_and_disable_exits_auto(self):
        self.device.supports_auto = True
        self.connect_demo()
        self.enable_output()
        self.device.withhold_replies(SET_MODE)
        self.app.auto_button.invoke()
        self.wait_until(lambda: bool(self.device.requests(SET_MODE)) and
                        self.app._snapshot.get("pending"), "auto reply withheld")
        self.assertEqual(self.app._snapshot.get("mode"), "manual")  # No optimistic success.
        self.assertTrue(self.app.auto_button.instate(["disabled"]))
        self.assertTrue(self.app.hold_button.instate(["!disabled"]))
        self.assertTrue(self.app.tracking_hold_button.instate(["!disabled"]))
        self.assertIn("确认前请勿视为跟随已启动", self.app.auto_hint_var.get())
        self.app.tracking_hold_button.invoke()
        self.wait_until(lambda: bool(self.device.requests(HOLD)) and
                        not self.app._snapshot.get("pending"), "HOLD interrupts pending auto")
        self.assertFalse(self.device.auto_mode)
        self.assertEqual(self.app._snapshot.get("mode"), "manual")
        self.device.withhold_replies(None)
        self.app.auto_button.invoke()
        self.wait_until(lambda: self.app._snapshot.get("mode") == "auto" and
                        not self.app._snapshot.get("pending"), "auto restart acknowledged")
        self.app.enable_button.invoke()
        self.wait_until(lambda: not self.app._snapshot.get("enabled") and
                        not self.app._snapshot.get("pending"), "disable exits auto")
        self.assertFalse(self.device.auto_mode)
        self.assertEqual(self.app._snapshot.get("mode"), "manual")
        self.assertTrue(self.app.auto_button.instate(["disabled"]))
        self.assertTrue(self.app.tracking_hold_button.instate(["disabled"]))

    def test_mode_ack_mismatch_stays_manual_and_displays_error(self):
        self.device.supports_auto = True
        self.connect_demo()
        self.enable_output()
        self.device.ignore_mode = True
        self.app.auto_button.invoke()
        self.wait_until(lambda: "控制模式未切换" in self.app.message_var.get() and
                        not self.app._snapshot.get("pending"), "mismatched mode acknowledgement")
        self.assertEqual(self.app._snapshot["mode"], "manual")
        self.assertEqual(self.app.auto_button.cget("text"), "开始红球跟随")
        self.assertTrue(self.app.send_button.instate(["!disabled"]))
        self.assertTrue(self.app.tracking_hold_button.instate(["!disabled"]))
        self.assertNotIn("控制模式已确认", self.app.log_widget.get("1.0", tk.END))

    def test_watchdog_exit_and_disconnect_clear_tracking_claims(self):
        self.device.supports_auto = True
        self.connect_demo()
        self.enable_output()
        self.app.auto_button.invoke()
        self.wait_until(lambda: self.app._snapshot.get("mode") == "auto" and
                        not self.app._snapshot.get("pending"), "auto acknowledged")
        # The next board status reports a watchdog exit; heartbeat cannot restart it.
        self.device.auto_mode = False
        self.device.watchdog = True
        self.wait_until(lambda: self.app._snapshot.get("watchdog"), "watchdog status visible")
        self.assertIn("自动跟随已退出", self.app.auto_hint_var.get())
        self.assertIn("跟随已退出", self.app.motion_var.get())
        self.assertEqual(self.app.auto_button.cget("text"), "开始红球跟随")
        self.wait_for_more_polls()
        self.assertEqual(len(self.device.requests(SET_MODE)), 1)
        self.assertFalse(self.device.auto_mode)

        self.app.auto_button.invoke()
        self.wait_until(lambda: self.app._snapshot.get("mode") == "auto" and
                        not self.app._snapshot.get("pending"), "explicit restart acknowledged")
        self.app.connect_button.invoke()
        self.wait_until(lambda: not self.app._snapshot.get("connected"), "disconnect visible")
        self.assertIn("自动跟随状态未知", self.app.auto_hint_var.get())
        self.assertNotIn("自动模式", self.app.motion_var.get())
        self.assertEqual(self.app.auto_button.cget("text"), "红球跟随 · 等待连接")
        self.assertEqual(self.app.hold_button.cget("text"), "停止移动 / 保持")
        self.assertTrue(self.app.tracking_hold_button.instate(["disabled"]))
        self.assertEqual(len(self.device.requests(SET_MODE)), 2)

    def test_connection_failure_clears_tracking_and_pending_controls(self):
        self.device.supports_auto = True
        self.connect_demo()
        self.enable_output()
        self.app.auto_button.invoke()
        self.wait_until(lambda: self.app._snapshot.get("mode") == "auto" and
                        not self.app._snapshot.get("pending"), "auto acknowledged")
        self.device.fail_reads = True
        self.wait_until(lambda: not self.app._snapshot.get("connected") and
                        "模拟串口连接中断" in self.app.message_var.get(), "connection failure visible")
        self.assertFalse(self.app._snapshot["pending"])
        self.assertIn("自动跟随状态无法确认", self.app.auto_hint_var.get())
        self.assertIn("自动跟随状态无法确认", self.app.message_var.get())
        self.assertNotIn("自动模式", self.app.motion_var.get())
        self.assertEqual(self.app.auto_button.cget("text"), "红球跟随 · 等待连接")
        for button in (self.app.auto_button, self.app.tracking_hold_button,
                       self.app.hold_button, self.app.send_button, self.app.home_button):
            self.assertTrue(button.instate(["disabled"]))
        self.assertEqual(len(self.device.requests(SET_MODE)), 1)

    def test_bluetooth_only_does_not_preselect_or_connect(self):
        with patch.object(self.controller, 'ports', return_value=[
            PortChoice('COM3', 'COM3 · 蓝牙串口（非开发板）', bluetooth=True),
            PortChoice('COM4', 'COM4 · 蓝牙串口（非开发板）', bluetooth=True),
        ]):
            self.app.refresh_ports()
        self.assertEqual(self.app.port_box.current(), -1)
        self.assertEqual(self.app.port_var.get(), '')
        self.assertIn('当前只有蓝牙串口', self.app.detail_var.get())
        self.assertIn('USB-JTAG', self.app.detail_var.get())
        self.app.connect_button.invoke()
        self.assertEqual(self.controller.connection_requests, [])
        self.assertIn('尚未选择', self.app.message_var.get())
        self.assertFalse(self.controller.snapshot()['connected'])
        self.assertFalse(self.device.requests())
        appeared = PortChoice('COM45', 'COM45 · USB 调试器候选', True)
        with patch.object(self.controller, 'ports', return_value=[appeared]):
            self.app.refresh_ports()
        self.assertEqual(self.app._selected_port().device, 'COM45')
        self.assertFalse(self.app._ui_error)
        self.assertFalse(self.device.requests())

    def test_unique_candidate_preselected_without_connecting_or_motion(self):
        choices = [PortChoice('COM45', 'COM45 · USB 调试器候选', board_candidate=True),
                   PortChoice('COM3', 'COM3 · 蓝牙串口（非开发板）', bluetooth=True)]
        with patch.object(self.controller, 'ports', return_value=choices):
            self.app.refresh_ports()
        self.assertEqual(self.app._selected_port().device, 'COM45')
        self.assertIn('连接后才会确认', self.app.detail_var.get())
        self.assertEqual(self.controller.connection_requests, [])
        self.assertFalse(self.controller.snapshot()['connected'])
        self.assertFalse(self.controller.snapshot()['enabled'])
        self.assertFalse(self.device.requests())
        self.assertNotIn('已连接', self.app.badge.cget('text'))

    def test_multiple_candidates_require_choice_and_refresh_preserves_device(self):
        choices = [PortChoice('COM45', 'COM45 · USB 调试器候选', True),
                   PortChoice('COM46', 'COM46 · USB 调试器候选', True)]
        with patch.object(self.controller, 'ports', return_value=choices):
            self.app.refresh_ports()
        self.assertEqual(self.app.port_box.current(), -1)
        self.assertIn('多个', self.app.detail_var.get())
        self.app.port_box.current(1)
        renamed = PortChoice('COM46', 'COM46 · USB 调试器候选 · New description', True)
        with patch.object(self.controller, 'ports', return_value=[renamed, choices[0]]):
            self.app.refresh_ports()
        self.assertEqual(self.app._selected_port().device, 'COM46')
        self.assertEqual(self.app.port_var.get(), renamed.label)
        # Removal clears the old choice rather than silently switching devices.
        with patch.object(self.controller, 'ports', return_value=[choices[0]]):
            self.app.refresh_ports()
        self.assertEqual(self.app.port_box.current(), -1)
        self.assertEqual(self.app.port_var.get(), '')
        self.assertFalse(self.device.requests())

    def test_generic_port_manual_choice_and_demo_still_work(self):
        generic = PortChoice('COM55', 'COM55 · 其他 USB 串口')
        candidate = PortChoice('COM44', 'COM44 · USB 调试器候选', True)
        with patch.object(self.controller, 'ports', return_value=[generic]):
            self.app.refresh_ports()
        self.assertEqual(self.app.port_box.current(), -1)
        self.app.port_box.current(0)
        with patch.object(self.controller, 'ports', return_value=[candidate, generic]):
            self.app.refresh_ports()
        self.assertEqual(self.app._selected_port().device, 'COM55')
        self.assertIn('其他串口', self.app.detail_var.get())
        # Observe the requested port without opening any real or simulated link.
        with patch.object(self.controller, 'connect') as connect:
            self.app.connect_button.invoke()
        connect.assert_called_once_with(port='COM55', demo=False)
        self.assertFalse(self.device.requests())
        with patch.object(self.controller, 'ports', return_value=[]):
            self.app.refresh_ports()
        self.assertEqual(self.app.port_var.get(), '')
        self.connect_demo()
        with patch.object(self.controller, 'ports', side_effect=AssertionError('Must not enumerate while connected')):
            self.app.refresh_ports()
        self.assertTrue(self.controller.snapshot()['connected'])
        self.assertTrue(self.controller.snapshot()['demo'])
        self.assertFalse(self.device.requests(ENABLE))
        self.assertFalse(self.device.requests(SET_TARGET))

    def test_connecting_refresh_does_not_enumerate_or_change_selection(self):
        candidate = PortChoice('COM45', 'COM45 · USB 调试器候选', True)
        with patch.object(self.controller, 'ports', return_value=[candidate]):
            self.app.refresh_ports()
        with patch.object(self.controller, 'snapshot', return_value={'connecting': True}), \
             patch.object(self.controller, 'ports', side_effect=AssertionError('Must not enumerate while connecting')):
            self.app.refresh_ports()
        self.assertEqual(self.app._selected_port().device, 'COM45')
        self.assertFalse(self.device.requests())

    def test_window_close_closes_worker_and_transport(self):
        self.connect_demo()
        self.enable_output()
        self._cancel_timers()
        callback = self.root.protocol("WM_DELETE_WINDOW")
        self.root.tk.call(callback)  # Exercise the registered window-close action.
        self.assertTrue(self.app.closed)
        self.assertEqual(self.controller.close_calls, 1)
        self.assertTrue(self.device.closed)
        self.assertFalse(self.controller._thread.is_alive())
        self.assertFalse(self.controller.snapshot()["connected"])
        self.assertTrue(self.device.requests(HOLD))
        self.app.close()  # Repeated close must be harmless.
        self.assertEqual(self.controller.close_calls, 1)


if __name__ == "__main__":
    unittest.main()

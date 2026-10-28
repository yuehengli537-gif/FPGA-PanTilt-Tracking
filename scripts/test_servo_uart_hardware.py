#!/usr/bin/env python3
"""PT20 serial integration check; default mode only queries the FPGA.

The exercise option is an explicit assertion that the independent servo 5V
supply is disconnected. Returned positions are PWM commands, not measurements
of the physical shaft. Close the desktop UI before running this script.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from desktop_control.protocol import (  # noqa: E402
    ENABLE, ERROR, ERROR_MESSAGES, GET_LIMITS, GET_STATUS, HELLO, HELLO_REPLY,
    HOLD, HOME, LIMITS, SET_TARGET, STATUS, Frame, StreamParser, encode,
    pack_us, unpack_us, us_to_angle, validate_limits,
)
from desktop_control.transport import SerialTransport  # noqa: E402

NAMES = {HELLO: "HELLO", GET_LIMITS: "GET_LIMITS", GET_STATUS: "GET_STATUS",
         ENABLE: "ENABLE", SET_TARGET: "SET_TARGET", HOLD: "HOLD", HOME: "HOME",
         HELLO_REPLY: "HELLO_REPLY", LIMITS: "LIMITS", STATUS: "STATUS", ERROR: "ERROR"}
READ_ONLY = (HELLO, GET_LIMITS, GET_STATUS)


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


class Evidence:
    def __init__(self, port, exercise):
        self.started = time.monotonic()
        self.data = {
            "schema": "pt20-uart-hardware-test-v1", "started_utc": utc_now(),
            "port": port, "baud": 115200, "exercise_servo_power_off": exercise,
            "position_semantics": "PWM command estimates; not measured shaft angles",
            "watchdog_scope": "Checks the inactivity latch and HOLD clear; small targets may finish before timeout",
            "events": [], "assertions": [], "ok": False,
        }

    def event(self, event, **details):
        item = {"utc": utc_now(), "elapsed_s": round(time.monotonic() - self.started, 6),
                "event": event, **details}
        self.data["events"].append(item)
        return item

    def check(self, name, condition, **details):
        item = self.event("assertion", name=name, ok=bool(condition), **details)
        self.data["assertions"].append(item.copy())
        if not condition:
            raise AssertionError(name)


class Session:
    def __init__(self, transport, evidence, exercise=False):
        self.transport, self.evidence, self.exercise = transport, evidence, exercise
        self.parser, self.seq, self.identified = StreamParser(), 0, False

    def next_seq(self):
        self.seq = (self.seq + 1) & 0xFF
        return self.seq

    def send(self, packet, kind, seq, **details):
        if kind not in READ_ONLY and not (self.exercise and self.identified):
            raise RuntimeError("Control requires --exercise-servo-power-off and a valid PT20 handshake")
        self.evidence.event("tx", kind=NAMES.get(kind, hex(kind)), seq=seq,
                            hex=packet.hex(), **details)
        self.transport.write(packet)

    def read(self):
        chunk = self.transport.read()
        if chunk:
            self.evidence.event("rx_bytes", hex=chunk.hex())
        frames = self.parser.feed(chunk)
        for frame in frames:
            details = {"kind": NAMES.get(frame.kind, hex(frame.kind)), "seq": frame.seq,
                       "flags": frame.flags, "payload_hex": frame.payload.hex()}
            if frame.kind == STATUS:
                values = unpack_us(frame.payload)
                details.update(command_us=list(values), estimated_angles_deg=list(map(us_to_angle, values)),
                               enabled=bool(frame.flags & 1), moving=bool(frame.flags & 2),
                               watchdog=bool(frame.flags & 4))
            elif frame.kind == LIMITS:
                details["limits_us"] = list(unpack_us(frame.payload))
            self.evidence.event("rx_frame", **details)
        return frames

    def exchange(self, kind, payload=bytes(8), timeout=0.8):
        seq = self.next_seq()
        expected = {HELLO: HELLO_REPLY, GET_LIMITS: LIMITS}.get(kind, STATUS)
        self.send(encode(Frame(kind, seq, payload=payload)), kind, seq)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for frame in self.read():
                if frame.seq != seq:
                    self.evidence.event("ignored_reply", seq=frame.seq, expected_seq=seq)
                    continue
                if frame.kind == ERROR:
                    raise RuntimeError(f"{NAMES[kind]} rejected: {ERROR_MESSAGES.get(frame.payload[0], frame.payload[0])}")
                if frame.kind == expected:
                    return frame
        raise TimeoutError(f"No valid {NAMES[expected]} for {NAMES[kind]}, sequence {seq}")

    def status(self, frame=None):
        frame = frame if frame is not None else self.exchange(GET_STATUS)
        values = unpack_us(frame.payload)
        self.evidence.check("valid_status_pulse_range", frame.kind == STATUS and
                            all(500 <= value <= 2500 for value in values), command_us=list(values))
        return values

    def wait_target(self, pan, tilt, name):
        deadline, last = time.monotonic() + 3.0, None
        while time.monotonic() < deadline:
            frame = self.exchange(GET_STATUS)  # Also keeps the watchdog alive.
            last = self.status(frame)
            self.evidence.check(name + "_enabled", bool(frame.flags & 1), flags=frame.flags)
            if last == (pan, tilt, pan, tilt) and not frame.flags & 2:
                self.evidence.check(name, True, command_us=list(last))
                return
            time.sleep(0.05)
        self.evidence.check(name, False, last_command_us=last, expected_us=[pan, tilt, pan, tilt])

    def run(self):
        hello = self.exchange(HELLO)
        self.evidence.check("PT20_identity_and_manual_capability",
                            hello.payload[:4] == b"PT20" and bool(hello.payload[6] & 1),
                            payload_hex=hello.payload.hex())
        self.identified = True
        self.evidence.data["firmware"] = f"{hello.payload[4]}.{hello.payload[5]}"
        limits = unpack_us(self.exchange(GET_LIMITS).payload)
        try:
            validate_limits(limits)
        except ValueError:
            self.evidence.check("valid_limits", False, limits_us=list(limits))
        self.evidence.check("valid_limits", True, limits_us=list(limits))
        self.evidence.data["initial_limits_us"] = list(limits)
        self.evidence.data["initial_command_us"] = list(self.status())
        if not self.exercise:
            return

        # The explicit exercise flag is required even if readback says disabled.
        self.evidence.check("small_targets_within_existing_limits",
                            limits[0] <= 1520 <= limits[1] and limits[2] <= 1480 <= limits[3],
                            targets_us=[1520, 1480], limits_us=list(limits))
        off = self.exchange(ENABLE, bytes(8))
        self.evidence.check("baseline_output_disabled", not off.flags & 1, flags=off.flags)
        on = self.exchange(ENABLE, b"\x01" + bytes(7))
        self.evidence.check("enable_starts_neutral", bool(on.flags & 1) and
                            self.status(on) == (1500, 1500, 1500, 1500), flags=on.flags)

        seq = self.next_seq()
        damaged = bytearray(encode(Frame(SET_TARGET, seq, payload=pack_us(1520, 1480))))
        damaged[-1] ^= 1
        self.send(bytes(damaged), SET_TARGET, seq, deliberately_corrupted_crc=True)
        deadline, replies = time.monotonic() + 0.12, []
        while time.monotonic() < deadline:
            replies.extend(frame for frame in self.read() if frame.seq == seq)
        self.evidence.check("bad_crc_no_reply", not replies, reply_count=len(replies))
        self.evidence.check("bad_crc_did_not_change_target", self.status() == (1500, 1500, 1500, 1500))

        self.exchange(SET_TARGET, pack_us(1520, 1480))
        self.wait_target(1520, 1480, "small_target_reached")
        self.exchange(HOME)
        self.wait_target(1500, 1500, "home_reached")

        self.exchange(SET_TARGET, pack_us(1520, 1480))
        time.sleep(0.06)
        held = self.exchange(HOLD)
        frozen = self.status(held)
        self.evidence.check("hold_freezes_command", frozen[:2] == frozen[2:] and
                            bool(held.flags & 1) and not held.flags & 2, command_us=list(frozen))
        time.sleep(0.08)
        self.evidence.check("hold_stays_still", self.status() == frozen, expected_us=list(frozen))

        self.exchange(SET_TARGET, pack_us(1520, 1480))
        self.evidence.event("watchdog_idle_start", seconds=1.2, sends_during_wait=False)
        time.sleep(1.2)  # Intentionally no heartbeat during this interval.
        expired = self.exchange(GET_STATUS)
        stopped = self.status(expired)
        self.evidence.check("watchdog_latched_and_holding", bool(expired.flags & 4) and
                            bool(expired.flags & 1) and not expired.flags & 2 and stopped[:2] == stopped[2:],
                            flags=expired.flags, command_us=list(stopped))
        recovered = self.exchange(HOLD)
        self.evidence.check("hold_clears_watchdog", not recovered.flags & 4 and
                            bool(recovered.flags & 1) and self.status(recovered) == stopped,
                            flags=recovered.flags)
        after_limits = unpack_us(self.exchange(GET_LIMITS).payload)
        self.evidence.check("limits_unchanged", after_limits == limits, limits_us=list(after_limits))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Board UART port, for example COM12")
    parser.add_argument("--output", required=True, type=Path, help="JSON evidence file")
    parser.add_argument("--exercise-servo-power-off", action="store_true",
                        help="Assert servo 5V is disconnected; permit PWM command tests and final disable")
    args = parser.parse_args(argv)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    evidence = Evidence(args.port, args.exercise_servo_power_off)
    transport, session, exit_code = None, None, 0
    try:
        transport = SerialTransport(args.port)
        evidence.event("port_opened")
        session = Session(transport, evidence, args.exercise_servo_power_off)
        session.run()
    except (Exception, KeyboardInterrupt) as exc:
        exit_code = 130 if isinstance(exc, KeyboardInterrupt) else 1
        evidence.data["error"] = {"type": type(exc).__name__, "message": str(exc)}
        evidence.event("failure", **evidence.data["error"])
    finally:
        if session and args.exercise_servo_power_off and session.identified:
            try:
                # A late reply to the failed command must finish before disable.
                time.sleep(0.01)
                reply = session.exchange(ENABLE, bytes(8))
                evidence.check("finally_output_disabled", not reply.flags & 1, flags=reply.flags)
                evidence.data["final_disable_confirmed"] = True
            except (Exception, KeyboardInterrupt) as exc:
                exit_code = exit_code or 1
                evidence.data["final_disable_confirmed"] = False
                evidence.event("final_disable_failed", error=str(exc),
                               note="PWM off is unconfirmed; keep servo 5V disconnected")
        if transport:
            try:
                transport.close()
                evidence.event("port_closed")
            except Exception as exc:
                exit_code = exit_code or 1
                evidence.event("port_close_failed", error=str(exc))
        evidence.data.update(ok=exit_code == 0, finished_utc=utc_now(),
                             duration_s=round(time.monotonic() - evidence.started, 6),
                             assertion_count=len(evidence.data["assertions"]))
        args.output.write_text(json.dumps(evidence.data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"{'PASS' if exit_code == 0 else 'FAIL'}: {args.output.resolve()}")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())

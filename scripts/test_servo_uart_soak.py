#!/usr/bin/env python3
"""Read-only PT20 UART soak: close the desktop UI before opening its port.

Only HELLO, GET_LIMITS and GET_STATUS are sent, including after packet loss.
Queries feed the firmware watchdog; no enable, target, mode or HOLD is sent.
"""
import argparse
import json
import math
from pathlib import Path
import time

from test_servo_uart_hardware import (
    Evidence, GET_STATUS, NAMES, READ_ONLY, SerialTransport, Session, utc_now,
)


def record_transport_error(evidence, operation, exc):
    evidence.data["statistics"]["transport_error_count"] += 1
    timed_out = isinstance(exc, TimeoutError) or type(exc).__name__ == "SerialTimeoutException"
    error_kind = f"transport_{operation}_{'timeout' if timed_out else 'error'}"
    evidence.data["error_kind"] = error_kind
    evidence.event("transport_error", operation=operation, error_kind=error_kind,
                   type=type(exc).__name__, message=str(exc))


class SoakSession(Session):
    def __init__(self, transport, evidence, timeout):
        super().__init__(transport, evidence, exercise=False)
        self.timeout = timeout
        self.stats = evidence.data["statistics"]
        self.io_stage = None

    def send(self, packet, kind, seq, **details):
        self.io_stage = "write"
        super().send(packet, kind, seq, **details)
        self.io_stage = None

    def read(self):
        self.io_stage = "read"
        frames = super().read()
        self.io_stage = None
        return frames

    def next_seq(self):
        seq = super().next_seq()
        if seq == 0:
            self.stats["sequence_wrap_count"] += 1
        return seq

    def exchange(self, kind, payload=bytes(8), timeout=None):
        if kind not in READ_ONLY or payload != bytes(8):
            raise ValueError("The soak probe only permits empty read-only requests")
        started = time.perf_counter()
        self.stats["request_count"] += 1
        try:
            reply = super().exchange(kind, payload, self.timeout if timeout is None else timeout)
        except Exception as exc:
            if self.io_stage is not None and isinstance(exc, OSError):
                record_transport_error(self.evidence, self.io_stage, exc)
            elif isinstance(exc, TimeoutError):
                self.stats["timeout_count"] += 1
                self.evidence.data["error_kind"] = "response_timeout"
                self.evidence.event("request_timeout", kind=NAMES[kind], seq=self.seq,
                                    error_kind="response_timeout", error=str(exc),
                                    elapsed_ms=round((time.perf_counter() - started) * 1000, 3))
            raise
        elapsed_ms = round((time.perf_counter() - started) * 1000, 3)
        self.stats["success_count"] += 1
        self.stats["max_latency_ms"] = max(self.stats["max_latency_ms"], elapsed_ms)
        self.evidence.event("request_completed", kind=NAMES[kind], seq=self.seq,
                            latency_ms=elapsed_ms)
        return reply


def positive_seconds(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be a finite positive number of seconds")
    return number


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Board UART, for example COM12")
    parser.add_argument("--output", required=True, type=Path, help="JSON evidence file")
    parser.add_argument("--duration", type=positive_seconds, default=120.0)
    parser.add_argument("--interval", type=positive_seconds, default=0.25)
    parser.add_argument("--timeout", type=positive_seconds, default=0.7)
    args = parser.parse_args(argv)
    evidence = Evidence(args.port, exercise=False)
    evidence.data.update(
        schema="pt20-uart-soak-v1", read_only=True, error_kind=None,
        requested_duration_s=args.duration, poll_interval_s=args.interval,
        request_timeout_s=args.timeout,
        watchdog_scope="Read-only queries feed the watchdog; no control commands are sent",
        latency_scope="Successful request round trips, including transport write and read",
        latency_clock="time.perf_counter",
        statistics=dict(request_count=0, success_count=0, timeout_count=0,
                        transport_error_count=0, max_latency_ms=0.0,
                        sequence_wrap_count=0, status_sample_count=0),
    )
    stats = evidence.data["statistics"]
    transport, handshake_done, completed, exit_code = None, False, False, 0
    try:
        transport = SerialTransport(args.port)
        evidence.event("port_opened")
        session = SoakSession(transport, evidence, args.timeout)
        deadline = time.monotonic() + args.duration
        next_progress = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            poll_started = time.monotonic()
            try:
                if not handshake_done:
                    session.run()  # Inherited read-only HELLO/LIMITS/STATUS validation.
                    handshake_done = True
                    evidence.event("handshake_complete")
                else:
                    session.status(session.exchange(GET_STATUS))
                stats["status_sample_count"] += 1
            except TimeoutError:
                if session.io_stage is not None:
                    raise  # A transport timeout is not a missing response.
                # Preserve raw RX evidence and parser state, then keep diagnosing.
                pass
            now = time.monotonic()
            if now >= next_progress:
                print(f"{now - evidence.started:.1f}s: replies={stats['success_count']} "
                      f"timeouts={stats['timeout_count']} max={stats['max_latency_ms']:.3f}ms", flush=True)
                next_progress = now + 10.0
            time.sleep(max(0.0, min(poll_started + args.interval, deadline) - now))
        completed = True
    except (Exception, KeyboardInterrupt) as exc:
        exit_code = 130 if isinstance(exc, KeyboardInterrupt) else 1
        if transport is None and isinstance(exc, OSError):
            record_transport_error(evidence, "open", exc)
        if not (isinstance(exc, OSError) and stats["transport_error_count"]):
            evidence.data["error_kind"] = "interrupted" if isinstance(exc, KeyboardInterrupt) else "probe_error"
        evidence.data["error"] = dict(type=type(exc).__name__, message=str(exc),
                                     error_kind=evidence.data["error_kind"])
        evidence.event("failure", **evidence.data["error"])
    finally:
        if transport is not None:
            try:
                transport.close()
                evidence.event("port_closed")
            except Exception as exc:
                exit_code = exit_code or 1
                record_transport_error(evidence, "close", exc)
                evidence.event("port_close_failed", error=str(exc))
        if not completed or not handshake_done or stats["timeout_count"]:
            exit_code = exit_code or 1
        evidence.data.update(ok=exit_code == 0, completed=completed,
                             handshake_complete=handshake_done, finished_utc=utc_now(),
                             duration_s=round(time.monotonic() - evidence.started, 6),
                             assertion_count=len(evidence.data["assertions"]))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(evidence.data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"{'PASS' if exit_code == 0 else 'FAIL'}: {args.output.resolve()}")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())

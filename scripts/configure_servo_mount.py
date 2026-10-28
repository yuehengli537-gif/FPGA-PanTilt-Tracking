"""Apply mount-coordinate limits over UART with PWM disabled; never enable/move.

Run from repository root with the desktop UI closed:
  .tools/servo-ui-venv/Scripts/python.exe scripts/configure_servo_mount.py --port COM12 --servo-power-off
No firmware download is needed. Board reset restores its conservative limits.
"""
import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from desktop_control.controller import ServoController


def wait_for(control, predicate, timeout=4):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = control.snapshot()
        if state["error"]:
            raise RuntimeError(state["error"])
        if predicate(state):
            return state
        time.sleep(0.02)
    raise TimeoutError("Device did not acknowledge the configuration")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--servo-power-off", action="store_true", required=True,
                        help="Confirm the independent servo 5V supply is disconnected")
    args = parser.parse_args()
    control = ServoController(directions_path=ROOT / "reports/runtime/desktop_control/mount_directions.json")
    result = {"port": args.port, "operation": "disable PWM and configure mount limits only",
              "enable_sent": False, "target_sent": False, "physical_endpoint_tested": False,
              "servo_power_off_confirmed": args.servo_power_off,
              "mount_angle_limits": [0, 180, -45, 45]}
    try:
        control.connect(args.port)
        before = wait_for(control, lambda state: state["connected"])
        result["before"] = before
        # Explicitly disable even when initial readback already says off.
        control.enable(False)
        disabled = wait_for(control, lambda state: not state["pending"] and not state["enabled"])
        result["disabled_before_configuration"] = disabled
        # Direction confirmed by user for this assembly on 2026-10-24.
        control.set_directions(False, False)
        control.apply_mount_range()
        limit_ack = wait_for(control, lambda state: not state["pending"] and state["profile_ready"])
        # LIMITS carries no position fields. Wait for the next GET_STATUS reply,
        # rather than mistaking the cached pre-configuration position for proof.
        after = wait_for(control, lambda state: state["status_revision"] > limit_ack["status_revision"])
        assert not after["enabled"]
        assert (after["pan_min"], after["pan_max"], after["tilt_min"], after["tilt_max"]) == (0, 180, -45, 45)
        for key in ("pan_pulse_us", "tilt_pulse_us", "target_pan_pulse_us", "target_tilt_pulse_us"):
            assert after[key] == disabled[key], f"Unexpected change in {key}"
        result.update(passed=True, after=after, pwm_disabled_verified=True,
                      position_command_unchanged=True, fresh_status_after_limits_verified=True)
    except Exception as exc:
        result.update(passed=False, error=str(exc))
    finally:
        control.close()
    rendered = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    # ASCII JSON on the console also works with Windows GBK redirected output.
    print(json.dumps(result, ensure_ascii=True, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

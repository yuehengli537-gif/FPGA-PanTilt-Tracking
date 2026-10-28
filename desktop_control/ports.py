"""Descriptions and hints for enumerated serial ports; never opens a device."""
from dataclasses import dataclass


@dataclass(frozen=True)
class PortChoice:
    device: str
    label: str
    board_candidate: bool = False
    bluetooth: bool = False


def describe_ports(devices):
    """VID/PID is only a hint: PT20 handshake still verifies the actual firmware."""
    choices = []
    for item in devices:
        candidate = (getattr(item, "vid", None), getattr(item, "pid", None)) == (0x0403, 0x6010)
        description = getattr(item, "description", "") or "串口"
        hwid = getattr(item, "hwid", "") or ""
        bluetooth = hwid.upper().startswith("BTHENUM\\") or "bluetooth" in description.lower() or "蓝牙" in description
        kind = "USB 调试器候选" if candidate else "蓝牙串口（非开发板）" if bluetooth else "其他 USB 串口" if getattr(item, "vid", None) is not None else "其他串口"
        choices.append(PortChoice(item.device, f"{item.device} · {kind} · {description}", candidate, bluetooth))
    return sorted(choices, key=lambda p: (not p.board_candidate, p.bluetooth, p.device))
#!/usr/bin/env python

"""Simple open/close test for the DH AG-95 gripper on BiDobot Nova5.

This script talks only to the selected arm's Dashboard port and the DH gripper
Modbus adapter. It does not send arm motion commands.

Examples:
    python src/lerobot/robots/bi_dobot_nova5_dh/test_dh_gripper_open_close.py
    python src/lerobot/robots/bi_dobot_nova5_dh/test_dh_gripper_open_close.py \
        --side left --left-ip 192.168.5.101
    python src/lerobot/robots/bi_dobot_nova5_dh/test_dh_gripper_open_close.py --cycles 5 --hold-s 1.0
"""

from __future__ import annotations

import argparse
import re
import time
from contextlib import suppress

from lerobot.robots.bi_dobot_nova5_dh.bi_dobot_nova5_dh import _DobotModbusRTU
from lerobot.robots.bi_dobot_nova5_dh.config_dh_gripper_integrated import (
    DHGripperIntegratedConfig,
)
from lerobot.robots.bi_dobot_nova5_dh.dh_gripper_integrated import DHGripperIntegrated
from lerobot.robots.bi_dobot_nova5_dh.TCP_IP_Python_V4.dobot_api import (
    DobotApiDashboard,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Open/close test for the DH AG-95 gripper via Dobot Nova5 end-effector RS485."
    )
    parser.add_argument(
        "--side",
        choices=("left", "right"),
        default="right",
        help="Arm/gripper side to test.",
    )
    parser.add_argument(
        "--left-ip", default="192.168.5.101", help="Left arm IP address."
    )
    parser.add_argument(
        "--right-ip", default="192.168.5.102", help="Right arm IP address."
    )
    parser.add_argument(
        "--dashboard-port", type=int, default=29999, help="Dobot Dashboard TCP port."
    )
    parser.add_argument(
        "--master-ip",
        default="192.168.201.1",
        help="Dobot ModbusCreate master IP for the end-effector RS485 proxy.",
    )
    parser.add_argument(
        "--master-port",
        type=int,
        default=60000,
        help="Dobot ModbusCreate master port for the end-effector RS485 proxy.",
    )
    parser.add_argument(
        "--slave-id", type=int, default=1, help="DH gripper Modbus slave ID."
    )
    parser.add_argument(
        "--baudrate", type=int, default=115200, help="End-effector RS485 baudrate."
    )
    parser.add_argument(
        "--parity",
        choices=("N", "E", "O"),
        default="N",
        help="RS485 parity. DH AG-95 is normally N.",
    )
    parser.add_argument(
        "--stop-bit", type=int, choices=(1, 2), default=1, help="RS485 stop bit."
    )
    parser.add_argument(
        "--identify",
        type=int,
        choices=(1, 2),
        default=1,
        help="Optional aviation connector index for SetToolPower/SetTool485.",
    )
    parser.add_argument(
        "--skip-tool-power",
        action="store_true",
        help="Do not send SetToolPower(1) before Modbus commands.",
    )
    parser.add_argument(
        "--power-cycle",
        action="store_true",
        help="Power-cycle the end tool before testing: SetToolPower(0), wait, SetToolPower(1).",
    )
    parser.add_argument(
        "--power-wait-s",
        type=float,
        default=2.0,
        help="Seconds to wait after turning end-tool power on.",
    )
    parser.add_argument(
        "--force", type=int, default=30, help="Gripper force percent, 20-100."
    )
    parser.add_argument(
        "--cycles", type=int, default=3, help="Number of close/open cycles."
    )
    parser.add_argument(
        "--hold-s", type=float, default=1.5, help="Seconds to wait after each command."
    )
    parser.add_argument(
        "--closed-pos",
        type=float,
        default=0.0,
        help="Normalized closed position. 0.0 means fully closed.",
    )
    parser.add_argument(
        "--open-pos",
        type=float,
        default=1.0,
        help="Normalized open position. 1.0 means fully open.",
    )
    parser.add_argument(
        "--no-final-open",
        action="store_true",
        help="Do not send a final open command before disconnecting.",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if not 20 <= args.force <= 100:
        raise ValueError(f"--force must be between 20 and 100, got {args.force}.")
    if args.cycles < 1:
        raise ValueError(f"--cycles must be >= 1, got {args.cycles}.")
    if args.hold_s < 0:
        raise ValueError(f"--hold-s must be >= 0, got {args.hold_s}.")
    if args.power_wait_s < 0:
        raise ValueError(f"--power-wait-s must be >= 0, got {args.power_wait_s}.")
    for name in ("closed_pos", "open_pos"):
        value = getattr(args, name)
        if not 0.0 <= value <= 1.0:
            raise ValueError(
                f"--{name.replace('_', '-')} must be in [0, 1], got {value}."
            )


def move_and_report(
    gripper: DHGripperIntegrated, target: float, hold_s: float, label: str
) -> None:
    print(f"{label}: command position={target:.3f}")
    gripper.set_gripper_position(target)
    time.sleep(hold_s)
    print(f"{label}: cached position={gripper.get_gripper_position():.3f}")


def parse_dobot_response(response: str) -> tuple[int, list[str]]:
    match = re.match(r"\s*(-?\d+)\s*,\s*\{([^}]*)\}", response)
    if match is None:
        raise RuntimeError(f"Unexpected Dobot response: {response!r}")
    error_id = int(match.group(1))
    values = [v.strip() for v in match.group(2).split(",") if v.strip()]
    return error_id, values


def print_response(label: str, response: str) -> tuple[int, list[str]]:
    error_id, values = parse_dobot_response(response)
    print(f"{label}: error_id={error_id}, values={values}, raw={response.strip()}")
    return error_id, values


def configure_tool_power(
    dashboard: DobotApiDashboard,
    power_cycle: bool,
    power_wait_s: float,
    identify: int,
) -> None:
    if power_cycle:
        response = dashboard.SetToolPower(0, identify)
        print_response("SetToolPower off", response)
        time.sleep(0.5)
    response = dashboard.SetToolPower(1, identify)
    print_response("SetToolPower on", response)
    time.sleep(power_wait_s)


def configure_tool_mode(dashboard: DobotApiDashboard, identify: int) -> None:
    response = dashboard.SetToolMode(1, 1, identify)
    print_response("SetToolMode", response)


def configure_tool485(
    dashboard: DobotApiDashboard,
    baudrate: int,
    parity: str,
    stop_bit: int,
    identify: int,
) -> None:
    response = dashboard.SetTool485(baudrate, parity, stop_bit, identify)
    print_response("SetTool485", response)


def probe_modbus(dashboard: DobotApiDashboard, modbus: _DobotModbusRTU) -> bool:
    response = dashboard.GetHoldRegs(modbus._index, 0x0200, 1)
    read_error, _ = print_response("GetHoldRegs init_state 0x0200", response)
    response = dashboard.SetHoldRegs(modbus._index, 0x0100, 1, "{165}")
    write_error, _ = print_response("SetHoldRegs init 0x0100=165", response)
    return read_error == 0 and write_error == 0


def main() -> None:
    args = parse_args()
    validate_args(args)

    robot_ip = args.left_ip if args.side == "left" else args.right_ip
    dashboard: DobotApiDashboard | None = None
    modbus: _DobotModbusRTU | None = None
    gripper: DHGripperIntegrated | None = None

    try:
        print(
            f"Connecting {args.side} Dashboard at {robot_ip}:{args.dashboard_port} ..."
        )
        dashboard = DobotApiDashboard(robot_ip, args.dashboard_port)
        if not args.skip_tool_power:
            configure_tool_power(
                dashboard, args.power_cycle, args.power_wait_s, args.identify
            )
        configure_tool_mode(dashboard, args.identify)
        configure_tool485(
            dashboard, args.baudrate, args.parity, args.stop_bit, args.identify
        )

        config = DHGripperIntegratedConfig(
            slave_id=args.slave_id,
            baudrate=args.baudrate,
            gripper_force=args.force,
            init_open=True,
        )
        modbus = _DobotModbusRTU(
            dashboard,
            args.master_ip,
            args.master_port,
            config.slave_id,
        )
        print(f"ModbusCreate: master_index={modbus._index}")
        if not probe_modbus(dashboard, modbus):
            print(
                "Modbus probe failed. Check end-tool power, slave ID, baudrate/parity, "
                "RS485 wiring, and that the gripper is connected to the selected arm."
            )
            return
        gripper = DHGripperIntegrated(config, name=args.side)
        gripper.connect(modbus)

        move_and_report(gripper, args.open_pos, args.hold_s, "initial open")
        for i in range(1, args.cycles + 1):
            print(f"\nCycle {i}/{args.cycles}")
            move_and_report(gripper, args.closed_pos, args.hold_s, "close")
            move_and_report(gripper, args.open_pos, args.hold_s, "open")

        if not args.no_final_open:
            print("\nFinal open before disconnect")
            move_and_report(gripper, args.open_pos, args.hold_s, "final open")

    finally:
        if gripper is not None:
            with suppress(Exception):
                gripper.disconnect()
            modbus = None
        if modbus is not None:
            with suppress(Exception):
                modbus.close()
        if dashboard is not None:
            dashboard.close()
            dashboard.socket_dobot = 0
        print("Done.")


if __name__ == "__main__":
    main()

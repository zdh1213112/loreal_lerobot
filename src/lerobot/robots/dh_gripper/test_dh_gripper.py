#!/usr/bin/env python3

# Copyright 2025 The XenseRobotics Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Standalone test script for the DH Robotics AG-95 gripper.

Runs three phases in sequence:
  1. Connection + hardware initialisation check
  2. Automatic motion sequence  (open → close → half → open)
  3. Interactive keyboard control with live position display

Usage
-----
    python test_dh_gripper.py --port /dev/ttyDHRight
    python test_dh_gripper.py --port /dev/ttyUSB0 --slave-id 1 --baudrate 115200
    python test_dh_gripper.py --port /dev/ttyUSB0 --no-auto   # skip auto sequence
    python test_dh_gripper.py --port /dev/ttyUSB0 --no-interactive

Interactive commands (single-keystroke, no Enter needed)
---------------------------------------------------------
    o        — move to fully open  (normalized 1.0)
    c        — move to fully closed (normalized 0.0)
    +        — open by 0.1
    -        — close by 0.1
    0–9 / .  — start typing a decimal position, confirm with Enter
    Backspace — delete last digit from buffer
    q        — exit
"""

import argparse
import os
import select
import signal
import sys
import termios
import time
import tty

# Allow running from the repo root without installing the package
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))

from lerobot.robots.dh_gripper.config_dh_gripper import DHGripperConfig
from lerobot.robots.dh_gripper.dh_gripper import DHGripper

# ── Global state ──────────────────────────────────────────────────────────────
_running = True


def _signal_handler(sig, frame):
    global _running
    print("\nCtrl+C received — stopping...")
    _running = False


signal.signal(signal.SIGINT, _signal_handler)


# ── Terminal helpers ──────────────────────────────────────────────────────────

def _read_char_nonblocking() -> str | None:
    """Return one character from stdin if available, else None. Non-blocking."""
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


# ── Display ───────────────────────────────────────────────────────────────────

def _pos_bar(normalized: float, width: int = 24) -> str:
    filled = int(max(0.0, min(1.0, normalized)) * width)
    return "█" * filled + "░" * (width - filled)


def _print_status(
    gripper: DHGripper,
    frame: int,
    fps: float,
    input_buf: str,
    last_msg: str,
) -> None:
    pos = gripper.get_gripper_position()
    bar = _pos_bar(pos)
    # Move cursor to top-left and overwrite — avoids flickering from full clear
    sys.stdout.write("\033[H")
    lines = [
        "=" * 60,
        f"  DH AG-95 Gripper Test  |  frame {frame:6d}  |  {fps:5.1f} Hz",
        "=" * 60,
        f"  Port      : {gripper._config.port}",
        f"  Slave ID  : {gripper._config.slave_id}  |  Baud: {gripper._config.baudrate}",
        "",
        f"  Position  : [{bar}] {pos:.3f}",
        f"              (0.0 = closed  ←  →  1.0 = open)",
        "",
        "─" * 60,
        "  o=open  c=close  +/-=step  0-9/. +Enter=position  q=quit",
        "─" * 60,
        f"  Input   : {input_buf}_" if input_buf else "  Input   : (type a command)",
        f"  Message : {last_msg}" if last_msg else "  Message : —",
        "=" * 60,
        "",  # extra blank line so any stray terminal output doesn't overlap
    ]
    sys.stdout.write("\n".join(lines))
    sys.stdout.flush()


# ── Test phases ───────────────────────────────────────────────────────────────

def _phase_auto_sequence(gripper: DHGripper) -> None:
    """Run a fixed open/close/half sequence with timing."""
    steps = [
        (1.0, "Open  "),
        (0.0, "Close "),
        (0.5, "Half  "),
        (1.0, "Open  "),
    ]
    print()
    print("─" * 60)
    print("  Phase 2: Automatic motion sequence")
    print("─" * 60)
    for target, label in steps:
        print(f"  [{label}] → {target:.1f}  ... ", end="", flush=True)
        t0 = time.monotonic()
        gripper.set_gripper_position_sync(target, timeout=8.0)
        elapsed = time.monotonic() - t0
        pos = gripper.get_gripper_position()
        print(f"done in {elapsed:.2f} s  (read-back {pos:.3f})")
        time.sleep(0.3)
    print("  Auto sequence complete.")


def _phase_interactive(gripper: DHGripper) -> None:
    """Live display with single-keystroke input via tty.setcbreak + select."""
    global _running

    print()
    print("─" * 60)
    print("  Phase 3: Interactive control")
    print("─" * 60)
    time.sleep(0.5)

    # Reserve screen space once before entering raw mode
    print("\033[2J", end="")   # clear screen
    sys.stdout.flush()

    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())  # characters available immediately, no Enter needed

        frame = 0
        fps = 0.0
        last_fps_time = time.monotonic()
        fps_window = 30
        frame_times: list[float] = []

        input_buf = ""   # accumulates digit/dot characters for numeric entry
        last_msg = "Connected. Type a command."
        current_pos = gripper.get_gripper_position()

        while _running:
            loop_start = time.monotonic()

            # ── Read all available keystrokes (drain the buffer) ──────────────
            while True:
                ch = _read_char_nonblocking()
                if ch is None:
                    break

                if ch in ("q", "Q"):
                    _running = False
                    break

                elif ch == "o":
                    gripper.set_gripper_position(1.0)
                    input_buf = ""
                    last_msg = "→ open (1.0)"

                elif ch == "c":
                    gripper.set_gripper_position(0.0)
                    input_buf = ""
                    last_msg = "→ close (0.0)"

                elif ch == "+":
                    target = min(1.0, current_pos + 0.1)
                    gripper.set_gripper_position(round(target, 1))
                    input_buf = ""
                    last_msg = f"→ +0.1  ({target:.1f})"

                elif ch == "-":
                    target = max(0.0, current_pos - 0.1)
                    gripper.set_gripper_position(round(target, 1))
                    input_buf = ""
                    last_msg = f"→ -0.1  ({target:.1f})"

                elif ch in "0123456789.":
                    input_buf += ch

                elif ch in ("\n", "\r"):
                    # Confirm numeric entry
                    if input_buf:
                        try:
                            val = float(input_buf)
                            if 0.0 <= val <= 1.0:
                                gripper.set_gripper_position(val)
                                last_msg = f"→ {val:.3f}"
                            else:
                                last_msg = f"[!] {val:.3f} out of range [0, 1]"
                        except ValueError:
                            last_msg = f"[!] '{input_buf}' is not a number"
                        input_buf = ""

                elif ch in ("\x7f", "\x08"):
                    # Backspace
                    input_buf = input_buf[:-1]

            # ── Update display ────────────────────────────────────────────────
            current_pos = gripper.get_gripper_position()
            frame += 1

            now = time.monotonic()
            frame_times.append(now)
            # Keep only the last fps_window timestamps
            cutoff = now - 1.0
            frame_times = [t for t in frame_times if t > cutoff]
            if len(frame_times) >= 2:
                fps = (len(frame_times) - 1) / (frame_times[-1] - frame_times[0])

            _print_status(gripper, frame, fps, input_buf, last_msg)

            # ── Rate limit to ~50 Hz ──────────────────────────────────────────
            elapsed = time.monotonic() - loop_start
            sleep = max(0.0, 0.02 - elapsed)
            time.sleep(sleep)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        # Restore cursor to a clean line
        print("\n")


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Standalone test for DH Robotics AG-95 gripper",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--port",      default="/dev/ttyUSB0", help="Serial port (default: /dev/ttyUSB0)")
    parser.add_argument("--slave-id",  type=int,  default=1,      dest="slave_id",  help="Modbus slave ID (default: 1)")
    parser.add_argument("--baudrate",  type=int,  default=115200,  choices=[9600, 19200, 38400, 115200], help="Baud rate (default: 115200)")
    parser.add_argument("--speed",     type=int,  default=0,     help="Gripper speed, which does not take effect (default: 0)")
    parser.add_argument("--force",     type=int,  default=50,     help="Gripper force 20–100 %% (default: 50)")
    parser.add_argument("--no-auto",         action="store_true", dest="no_auto",         help="Skip automatic motion sequence (phase 2)")
    parser.add_argument("--no-interactive",  action="store_true", dest="no_interactive",  help="Skip interactive control phase (phase 3)")
    args = parser.parse_args()

    print("=" * 60)
    print("  DH AG-95 Gripper — Standalone Test")
    print("=" * 60)
    print(f"  Port      : {args.port}")
    print(f"  Slave ID  : {args.slave_id}")
    print(f"  Baud rate : {args.baudrate}")
    print(f"  Speed     : {args.speed} %")
    print(f"  Force     : {args.force} %")
    print("=" * 60)

    # ── Phase 1: Connect ──────────────────────────────────────────────────────
    print()
    print("─" * 60)
    print("  Phase 1: Connection + hardware initialisation")
    print("─" * 60)

    config = DHGripperConfig(
        port=args.port,
        slave_id=args.slave_id,
        baudrate=args.baudrate,
        gripper_speed=args.speed,
        gripper_force=args.force,
        init_open=True,
    )
    gripper = DHGripper(config)

    try:
        print(f"  Connecting to {args.port} ...")
        gripper.connect()
        print(f"  Connected. Initial position: {gripper.get_gripper_position():.3f}")
    except Exception as e:
        print(f"  [ERROR] Failed to connect: {e}")
        sys.exit(1)

    try:
        # ── Phase 2: Auto sequence ────────────────────────────────────────────
        if not args.no_auto:
            _phase_auto_sequence(gripper)

        # ── Phase 3: Interactive ──────────────────────────────────────────────
        if not args.no_interactive:
            _phase_interactive(gripper)

    except Exception as e:
        print(f"\n[ERROR] {e}")
    finally:
        print()
        print("─" * 60)
        print("  Disconnecting...")
        try:
            gripper.disconnect()
            print("  Done.")
        except Exception as e:
            print(f"  Disconnect error (non-fatal): {e}")
        print("=" * 60)


if __name__ == "__main__":
    main()

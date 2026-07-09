#!/usr/bin/env python

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

"""Configuration for DH Robotics AG-95 gripper (Modbus RTU over RS485/USB-serial)."""

import warnings
from dataclasses import dataclass


@dataclass
class DHGripperConfig:
    """Configuration for the DH Robotics AG-95 gripper.

    Communicates over RS485/USB-serial using Modbus RTU protocol.
    No third-party Python SDK required — the driver speaks the wire
    protocol directly via pyserial.

    Position convention (DH hardware):
        0    = fully open
        1000 = fully closed

    Normalized position (lerobot convention):
        0.0  = fully closed
        1.0  = fully open

    Attributes:
        port:            Serial port path (e.g. ``"/dev/ttyUSB0"``).
        slave_id:        Modbus RTU slave ID (default 1).
        baudrate:        Baud rate (default 115200, also 9600/19200/38400).
        serial_timeout:  Per-read timeout in seconds (default 0.2).

        gripper_speed:   Target speed. This parameter does not take effect.
        gripper_force:   Target force, 20–100 % (default 30)(max 160N).

        init_open:       If True, move gripper to fully open after connect().
        init_timeout:    Seconds to wait for hardware self-initialisation
                         (register 0x0100 → 0xA5 sequence, default 10.0 s).
    """

    # ── Identification ─────────────────────────────────────────────────────────
    port: str = "/dev/ttyUSB0"
    slave_id: int = 1

    # ── Serial connection ──────────────────────────────────────────────────────
    baudrate: int = 115200
    serial_timeout: float = 0.2   # seconds; used as pyserial read timeout

    # ── Motion parameters (percentage 0–100) ──────────────────────────────────
    gripper_speed: int = 0
    gripper_force: int = 30

    # ── Initialization ─────────────────────────────────────────────────────────
    init_open: bool = True
    init_timeout: float = 10.0    # seconds to wait for hardware init

    def __post_init__(self):
        if not self.port:
            raise ValueError("DHGripperConfig: 'port' must not be empty.")
        if not 1 <= self.slave_id <= 247:
            raise ValueError(
                f"DHGripperConfig: slave_id {self.slave_id} out of Modbus range [1, 247]."
            )
        if self.baudrate not in (9600, 19200, 38400, 115200):
            raise ValueError(
                f"DHGripperConfig: unsupported baudrate {self.baudrate}. "
                "Choose from 9600, 19200, 38400, 115200."
            )
        if self.gripper_speed != 0:
            warnings.warn(
                "DHGripperConfig: gripper_speed does not take effect and will be ignored.",
                UserWarning,
                stacklevel=2,
            )
        if not 20 <= self.gripper_force <= 100:
            raise ValueError(
                f"DHGripperConfig: gripper_force {self.gripper_force} out of range [20, 100]."
            )
        if self.init_timeout <= 0:
            raise ValueError(
                f"DHGripperConfig: init_timeout must be positive, got {self.init_timeout}."
            )

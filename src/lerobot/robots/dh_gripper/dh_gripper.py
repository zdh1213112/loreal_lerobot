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

"""DH Robotics AG-95 gripper driver for Dobot Nova5.

Communicates via Modbus RTU over RS485 (USB-serial port) using pyserial.
No third-party DH SDK is required — this module implements the wire protocol
directly, following the reference C++ driver (ag95_driver).

Modbus RTU register map
-----------------------
Write (FC=0x06):
    0x0100  Initialization  — write 0xA5 to trigger hardware self-init
    0x0101  Target force    — 20–100 (%)
    0x0103  Target position — 0 (open) … 1000 (closed)
    0x0104  Target speed    — 0–100 (%) (this parameter does not take effect)

Read (FC=0x03):
    0x0200  Init state      — 0 = not init, 1 = init done
    0x0201  Grip state      — 0 = in motion, 1 = reached, 2 = gripped object, 3 = dropped
    0x0202  Current position — 0 (open) … 1000 (closed)

Position convention
-------------------
DH hardware : 0 = fully closed, 1000 = fully open
lerobot norm: 0.0 = fully closed, 1.0 = fully open

Conversion:
    dh_pos     = int(round(normalized * 1000))
    normalized = dh_pos / 1000.0

Background poller
-----------------
A daemon thread continuously reads the current position register and
updates ``_cached_position`` so that ``get_gripper_position()`` never
blocks the teleoperation loop.
"""

import time
from threading import Thread, Lock

import serial

from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.robots.dh_gripper.config_dh_gripper import DHGripperConfig
from lerobot.utils.robot_utils import get_logger

# ── Modbus RTU registers ───────────────────────────────────────────────────────
_REG_INITIALIZE = 0x0100
_REG_FORCE      = 0x0101
_REG_POSITION   = 0x0103
_REG_SPEED      = 0x0104
_REG_INIT_STATE = 0x0200
_REG_GRIP_STATE = 0x0201
_REG_CUR_POS    = 0x0202

_INIT_TRIGGER   = 0xA5   # value to write to _REG_INITIALIZE
_INIT_DONE      = 1      # value returned by _REG_INIT_STATE when ready

# DH position limits (hardware convention: 0 = closed, 1000 = open)
_DH_POS_CLOSED = 0
_DH_POS_OPEN   = 1000

# Background poller tunables
_POLL_INTERVAL_S        = 0.02   # ~50 Hz
_POLL_FAIL_LOG_THRESHOLD  = 20
_POLL_FAIL_REPEAT_INTERVAL = 100

# Blocking sync-move poll interval
_SYNC_POLL_INTERVAL_S = 0.05

# Grip state values returned by _REG_GRIP_STATE
_GRIP_IN_MOTION = 0
_GRIP_REACHED   = 1   # target reached without contact
_GRIP_GRIPPED   = 2   # gripped an object
_GRIP_DROPPED   = 3   # object dropped

# Number of Modbus retries for write / read operations
_WRITE_RETRIES = 3
_READ_RETRIES  = 3


def _crc16(data: bytes) -> int:
    """Compute Modbus CRC-16 (same lookup table as the C++ reference driver)."""
    table = (
        0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
        0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
        0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
        0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
        0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBc1, 0xDA81, 0x1A40,
        0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
        0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
        0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
        0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
        0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
        0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
        0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
        0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
        0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
        0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
        0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
        0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
        0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
        0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
        0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
        0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
        0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
        0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
        0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
        0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
        0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
        0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
        0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
        0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
        0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
        0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
        0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040,
    )
    crc = 0xFFFF
    for byte in data:
        crc = (crc >> 8) ^ table[(crc ^ byte) & 0xFF]
    return crc


class DHGripper:
    """DH Robotics AG-95 gripper driver for use inside DobotNova5.

    Normalized position convention:
        0.0  →  fully closed  (DH register value = 1000)
        1.0  →  fully open    (DH register value = 0)

    Example::

        cfg = DHGripperConfig(port="/dev/ttyUSB0", slave_id=1)
        g = DHGripper(cfg)
        g.connect()
        g.set_gripper_position(0.5)   # half-closed
        print(g.get_gripper_position())
        g.disconnect()
    """

    config_class = DHGripperConfig

    def __init__(self, config: DHGripperConfig):
        self._config = config
        self._logger = get_logger(f"DHGripper-{config.port.split('/')[-1]}")
        self._is_connected: bool = False

        self._serial: serial.Serial | None = None
        self._serial_lock = Lock()   # guards all serial I/O

        # Cached position updated by background poller — avoids blocking get_observation().
        # Pre-seeded from init_open so the value is sensible before the first poll response.
        self._cached_position: float = 1.0 if config.init_open else 0.0

        self._poll_thread: Thread | None = None
        self._poll_running: bool = False

    # ── Internal Modbus RTU helpers ────────────────────────────────────────────

    def _build_write_frame(self, reg: int, value: int) -> bytes:
        """Build a Modbus FC=0x06 single-register write frame."""
        frame = bytes([
            self._config.slave_id,
            0x06,
            (reg >> 8) & 0xFF,
            reg & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF,
        ])
        crc = _crc16(frame)
        return frame + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

    def _build_read_frame(self, reg: int, count: int = 1) -> bytes:
        """Build a Modbus FC=0x03 read-registers request frame."""
        frame = bytes([
            self._config.slave_id,
            0x03,
            (reg >> 8) & 0xFF,
            reg & 0xFF,
            0x00,
            count & 0xFF,
        ])
        crc = _crc16(frame)
        return frame + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

    def _write_register(self, reg: int, value: int) -> bool:
        """Write a single Modbus register (FC=0x06).

        The hardware echoes the request back on success.
        Retries up to _WRITE_RETRIES times.

        Returns:
            True on success, False on failure.
        """
        request = self._build_write_frame(reg, value)
        with self._serial_lock:
            if self._serial is None:
                return False
            for _ in range(_WRITE_RETRIES):
                try:
                    self._serial.reset_input_buffer()
                    self._serial.write(request)
                    response = self._serial.read(len(request))
                    if response == request:
                        return True
                except Exception:
                    pass
        return False

    def _read_register(self, reg: int) -> int | None:
        """Read a single Modbus register (FC=0x03).

        Returns:
            Integer register value on success, None on failure.
        """
        request = self._build_read_frame(reg, count=1)
        expected_len = 5 + 2 * 1  # 7 bytes for 1 register
        with self._serial_lock:
            if self._serial is None:
                return None
            for _ in range(_READ_RETRIES):
                try:
                    self._serial.reset_input_buffer()
                    self._serial.write(request)
                    response = self._serial.read(expected_len)
                    if len(response) != expected_len:
                        continue
                    # Validate CRC
                    calc_crc = _crc16(response[:-2])
                    recv_crc = response[-2] | (response[-1] << 8)
                    if calc_crc != recv_crc:
                        continue
                    # Validate header
                    if response[0] != self._config.slave_id or response[1] != 0x03:
                        continue
                    # Extract value (big-endian 16-bit)
                    return (response[3] << 8) | response[4]
                except Exception:
                    pass
        return None

    # ── Hardware initialisation ────────────────────────────────────────────────

    def _hardware_initialize(self) -> None:
        """Trigger and wait for the DH gripper hardware self-initialisation.

        Sends 0xA5 to register 0x0100. Polls register 0x0200 until the
        gripper reports init-done (value = 1) or init_timeout is exceeded.

        Raises:
            RuntimeError: If the gripper does not finish initialising in time.
        """
        init_state = self._read_register(_REG_INIT_STATE)
        if init_state == _INIT_DONE:
            self._logger.info("DH gripper already initialised.")
            return

        self._logger.info("Triggering DH gripper hardware initialisation...")
        if not self._write_register(_REG_INITIALIZE, _INIT_TRIGGER):
            raise RuntimeError(
                f"DHGripper on {self._config.port}: failed to send initialisation command."
            )

        deadline = time.monotonic() + self._config.init_timeout
        while time.monotonic() < deadline:
            state = self._read_register(_REG_INIT_STATE)
            if state == _INIT_DONE:
                self._logger.info("DH gripper hardware initialisation complete.")
                return
            time.sleep(0.1)

        raise RuntimeError(
            f"DHGripper on {self._config.port}: hardware initialisation timed out "
            f"after {self._config.init_timeout:.1f} s."
        )

    # ── Background position poller ─────────────────────────────────────────────

    def _position_poll_loop(self) -> None:
        """Background thread: continuously refresh _cached_position."""
        consecutive_failures = 0
        while self._poll_running:
            raw = self._read_register(_REG_CUR_POS)
            if raw is None:
                consecutive_failures += 1
                self._maybe_log_poll_failure(consecutive_failures)
                time.sleep(_POLL_INTERVAL_S)
                continue

            if consecutive_failures >= _POLL_FAIL_LOG_THRESHOLD:
                self._logger.info(
                    f"DHGripper communication recovered on {self._config.port} "
                    f"after {consecutive_failures} consecutive failures."
                )
            consecutive_failures = 0

            raw_clamped = max(_DH_POS_CLOSED, min(raw, _DH_POS_OPEN))
            self._cached_position = raw_clamped / _DH_POS_OPEN
            time.sleep(_POLL_INTERVAL_S)

    def _maybe_log_poll_failure(self, consecutive_failures: int) -> None:
        if consecutive_failures == _POLL_FAIL_LOG_THRESHOLD:
            self._logger.error(
                f"DHGripper communication unhealthy on {self._config.port} — "
                f"position cache is stale after {consecutive_failures} consecutive failures."
            )
        elif (
            consecutive_failures > _POLL_FAIL_LOG_THRESHOLD
            and (consecutive_failures - _POLL_FAIL_LOG_THRESHOLD) % _POLL_FAIL_REPEAT_INTERVAL == 0
        ):
            self._logger.error(
                f"DHGripper still unhealthy on {self._config.port} — "
                f"{consecutive_failures} consecutive failures."
            )

    # ── Connection lifecycle ───────────────────────────────────────────────────

    def connect(self) -> None:
        """Open the serial port, initialise hardware, and start position poller."""
        if self._is_connected:
            raise DeviceAlreadyConnectedError(f"{self} is already connected.")

        self._logger.info(
            f"Connecting DH gripper on {self._config.port} "
            f"(baud={self._config.baudrate}, slave_id={self._config.slave_id})..."
        )
        try:
            self._serial = serial.Serial(
                port=self._config.port,
                baudrate=self._config.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self._config.serial_timeout,
            )
        except Exception as e:
            raise RuntimeError(
                f"DHGripper: failed to open serial port {self._config.port}: {e}"
            ) from e

        self._is_connected = True
        self._cached_position = 1.0 if self._config.init_open else 0.0

        # Hardware initialisation (blocks until gripper is ready)
        try:
            self._hardware_initialize()
        except Exception as e:
            self._logger.error(f"DHGripper hardware initialisation failed: {e}")
            self._serial.close()
            self._serial = None
            self._is_connected = False
            raise

        # Apply speed and force settings
        self._write_register(_REG_SPEED, self._config.gripper_speed)
        self._write_register(_REG_FORCE, self._config.gripper_force)

        # Optionally move to open position
        if self._config.init_open:
            self._logger.info("Opening gripper to initial position...")
            self._write_register(_REG_POSITION, _DH_POS_OPEN)

        # Start background position poller
        self._poll_running = True
        self._poll_thread = Thread(target=self._position_poll_loop, daemon=True)
        self._poll_thread.start()

        self._logger.info(f"DH gripper connected on {self._config.port}.")

    def disconnect(self) -> None:
        """Open gripper, stop poller, and close the serial port."""
        if not self._is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Open gripper before disconnecting so it does not stay closed
        self._logger.info("Opening DH gripper before disconnect...")
        self._write_register(_REG_POSITION, _DH_POS_OPEN)

        # Stop background poller
        self._poll_running = False
        if self._poll_thread is not None:
            self._poll_thread.join(timeout=1.0)
            self._poll_thread = None

        with self._serial_lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception as e:
                    self._logger.debug(f"Error closing serial port: {e}")
                self._serial = None

        self._is_connected = False
        self._logger.info("DH gripper disconnected.")

    # ── Position interface ─────────────────────────────────────────────────────

    def get_gripper_position(self) -> float:
        """Return normalized gripper position in [0, 1] from the background cache.

        Returns immediately (non-blocking).

        Returns:
            0.0 = fully closed, 1.0 = fully open.
        """
        if not self._is_connected:
            return 0.0
        return self._cached_position

    def set_gripper_position(self, normalized_pos: float) -> None:
        """Send a position command (non-blocking).

        Args:
            normalized_pos: Target in [0, 1]. 0.0 = closed, 1.0 = open.
        """
        if not self._is_connected:
            raise DeviceNotConnectedError("DH gripper is not connected.")
        if not 0.0 <= normalized_pos <= 1.0:
            raise ValueError(f"normalized_pos must be in [0, 1], got {normalized_pos}.")
        dh_pos = int(round(normalized_pos * _DH_POS_OPEN))
        self._write_register(_REG_POSITION, dh_pos)

    def set_gripper_position_sync(
        self,
        normalized_pos: float,
        timeout: float = 10.0,
    ) -> None:
        """Send a position command and block until the gripper stops moving.

        The method polls the grip-state register and returns as soon as the
        gripper reports target-reached (1), gripped-object (2), or object-dropped
        (3), or when ``timeout`` seconds elapse.

        Args:
            normalized_pos: Target in [0, 1]. 0.0 = closed, 1.0 = open.
            timeout:        Max wait time in seconds (default 10.0).
        """
        if not self._is_connected:
            raise DeviceNotConnectedError("DH gripper is not connected.")
        if not 0.0 <= normalized_pos <= 1.0:
            raise ValueError(f"normalized_pos must be in [0, 1], got {normalized_pos}.")

        dh_pos = int(round(normalized_pos * _DH_POS_OPEN))
        self._write_register(_REG_POSITION, dh_pos)

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            grip_state = self._read_register(_REG_GRIP_STATE)
            if grip_state is not None and grip_state != _GRIP_IN_MOTION:
                return
            time.sleep(_SYNC_POLL_INTERVAL_S)

        self._logger.warning(
            f"set_gripper_position_sync timed out after {timeout:.1f} s "
            f"(target normalized={normalized_pos:.3f}, dh_pos={dh_pos})."
        )

    def initialize_gripper_position(self, normalized_pos: float = 1.0) -> None:
        """Move gripper to ``normalized_pos`` using a blocking sync command.

        Intended for use during robot start-up (before the teleoperation loop).

        Args:
            normalized_pos: Target in [0, 1]. Defaults to 1.0 (fully open).
        """
        self.set_gripper_position_sync(normalized_pos)

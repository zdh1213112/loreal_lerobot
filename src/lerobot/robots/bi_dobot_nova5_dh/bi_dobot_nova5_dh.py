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

"""BiDobotNova5: Bimanual Dobot Nova5 robot (NRT, TCP-IP Python V4).

Action keys (CARTESIAN_MOTION):
    left_tcp.{x,y,z,r1-r6} + left_gripper.pos
    right_tcp.{x,y,z,r1-r6} + right_gripper.pos

Action keys (JOINT_MOTION):
    left_joint_{1..6}.pos + left_gripper.pos
    right_joint_{1..6}.pos + right_gripper.pos

Observation keys follow the same naming convention.
"""

from __future__ import annotations

import contextlib
import re
import threading
import time
from functools import cached_property
from typing import Any

import numpy as np

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.robots.robot import Robot
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.utils.robot_utils import (
    euler_to_quaternion,
    get_logger,
    quaternion_to_euler,
    quaternion_to_rotation_6d,
    rotation_6d_to_quaternion,
)

from .config_bi_dobot_nova5_dh import (
    BiDobotNova5DHConfig,
    ControlMode,
    ResetMoveOrder,
    ResetStrategy,
    ResetTarget,
)
from .dh_gripper_integrated import DHGripperIntegrated
from .TCP_IP_Python_V4.dobot_api import DobotApiDashboard, DobotApiFeedBack

JOINT_DOF = 6
MM_PER_METER = 1000.0

_MODBUS_RETRIES = 3


class _DobotModbusRTU:
    """Thin adapter that exposes a ModbusRTUProtocol interface over DobotApiDashboard.

    Calls ModbusCreate(..., isRTU=True) on construction to establish the RS485
    Modbus master, then forwards read_register / write_register calls through
    GetHoldRegs / SetHoldRegs. ModbusClose is issued in close().

    This class lives here (not in a separate module) because it is tightly coupled to
    DobotApiDashboard and only makes sense in the context of BiDobotNova5.
    """

    def __init__(
        self,
        robot: DobotApiDashboard,
        master_ip: str,
        master_port: int,
        slave_id: int,
        is_rtu: bool = True,
    ) -> None:
        self._robot = robot
        resp = self._robot.ModbusCreate(master_ip, master_port, slave_id, is_rtu)
        error_id, values = self._parse(resp)
        if error_id != 0 or not values:
            raise RuntimeError(
                f"ModbusCreate failed (error_id={error_id}): {resp.strip()}"
            )
        self._index: int = int(values[0])

    # ── ModbusRTUProtocol implementation ──────────────────────────────────────

    def read_register(self, reg: int) -> int | None:
        for _ in range(_MODBUS_RETRIES):
            try:
                resp = self._robot.GetHoldRegs(self._index, reg, 1)
                error_id, values = self._parse(resp)
                if error_id == 0 and values:
                    return int(values[0])
            except Exception:
                pass
        return None

    def write_register(self, reg: int, value: int) -> bool:
        val_str = "{" + str(value) + "}"
        for _ in range(_MODBUS_RETRIES):
            try:
                resp = self._robot.SetHoldRegs(self._index, reg, 1, val_str)
                error_id, _ = self._parse(resp)
                if error_id == 0:
                    return True
            except Exception:
                pass
        return False

    def close(self) -> None:
        with contextlib.suppress(Exception):
            self._robot.ModbusClose(self._index)

    # ── Internal ──────────────────────────────────────────────────────────────

    @staticmethod
    def _parse(resp: str) -> tuple[int, list[str]]:
        match = re.match(r"\s*(-?\d+)\s*,\s*\{([^}]*)\}", resp)
        if match is None:
            raise RuntimeError(f"Unexpected Dobot response: {resp!r}")
        error_id = int(match.group(1))
        values = [v.strip() for v in match.group(2).split(",") if v.strip()]
        return error_id, values


class _FeedState:
    def __init__(self) -> None:
        self.RobotMode = -1
        self.robotCurrentCommandID = -1
        self.MessageSize = -1
        self.DigitalInputs = -1
        self.DigitalOutputs = -1
        self.User = -1
        self.Tool = -1
        self.tcpPose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # xyz(mm), rpy(deg)
        self.qActual = [0.0] * JOINT_DOF


class BiDobotNova5DH(Robot):
    config_class = BiDobotNova5DHConfig
    name = "bi_dobot_nova5_dh"

    def __init__(self, config: BiDobotNova5DHConfig):
        super().__init__(config)
        self.config = config
        self.logger = get_logger("BiDobotNova5DH")

        self._left_robot: DobotApiDashboard | None = None
        self._right_robot: DobotApiDashboard | None = None
        self._left_feed: DobotApiFeedBack | None = None
        self._right_feed: DobotApiFeedBack | None = None

        self._left_feed_data = _FeedState()
        self._right_feed_data = _FeedState()
        self._left_feed_lock = threading.Lock()
        self._right_feed_lock = threading.Lock()

        self._left_feed_thread: threading.Thread | None = None
        self._right_feed_thread: threading.Thread | None = None

        self._is_connected = False
        self.rt_moving = False  # for teleop loop compatibility
        self._cartesian_ik_servoj_until_s = {"left": 0.0, "right": 0.0}
        self._last_cartesian_guard_log_s = {"left": 0.0, "right": 0.0}
        self._last_cartesian_command_debug: dict[str, dict[str, Any]] = {}
        self._last_servo_period_s = 1.0 / float(config.control_frequency)
        self._action_worker_thread: threading.Thread | None = None
        self._action_worker_running = False
        self._action_worker_paused = False
        self._action_worker_busy = False
        self._action_worker_lock = threading.Lock()
        self._action_worker_cv = threading.Condition(self._action_worker_lock)
        self._pending_async_action: dict[str, Any] | None = None
        self._latest_sent_async_action: dict[str, Any] | None = None
        self._last_async_action_timing: dict[str, float] = {}
        self._async_action_drop_count = 0
        self._async_action_send_count = 0
        self._async_action_error_count = 0

        self._left_gripper: DHGripperIntegrated | None = None
        if config.use_left_gripper:
            self._left_gripper = DHGripperIntegrated(
                config.left_dh_gripper, name="left"
            )

        self._right_gripper: DHGripperIntegrated | None = None
        if config.use_right_gripper:
            self._right_gripper = DHGripperIntegrated(
                config.right_dh_gripper, name="right"
            )
        self._left_gripper_connected = False
        self._right_gripper_connected = False

        self._left_gripper_key = "left_gripper.pos"
        self._right_gripper_key = "right_gripper.pos"

        if config.control_mode == ControlMode.JOINT_MOTION:
            self._init_joint_mode()
        elif config.control_mode == ControlMode.CARTESIAN_MOTION:
            self._init_cartesian_mode()
        else:
            raise ValueError(f"Unsupported control_mode: {config.control_mode}")

        self.cameras = make_cameras_from_configs(config.cameras)
        np.set_printoptions(precision=6, suppress=True)

    def _init_joint_mode(self) -> None:
        self._left_joint_pos_keys = tuple(
            f"left_joint_{i}.pos" for i in range(1, JOINT_DOF + 1)
        )
        self._right_joint_pos_keys = tuple(
            f"right_joint_{i}.pos" for i in range(1, JOINT_DOF + 1)
        )
        self._left_action_joint_keys = self._left_joint_pos_keys
        self._right_action_joint_keys = self._right_joint_pos_keys

        self._control_frequency = self.config.control_frequency
        self._aheadtime = self.config.aheadtime
        self._gain = self.config.gain

    def _init_cartesian_mode(self) -> None:
        self._left_tcp_pose_keys = (
            "left_tcp.x",
            "left_tcp.y",
            "left_tcp.z",
            "left_tcp.r1",
            "left_tcp.r2",
            "left_tcp.r3",
            "left_tcp.r4",
            "left_tcp.r5",
            "left_tcp.r6",
        )
        self._right_tcp_pose_keys = (
            "right_tcp.x",
            "right_tcp.y",
            "right_tcp.z",
            "right_tcp.r1",
            "right_tcp.r2",
            "right_tcp.r3",
            "right_tcp.r4",
            "right_tcp.r5",
            "right_tcp.r6",
        )
        self._left_action_tcp_pose_keys = self._left_tcp_pose_keys
        self._right_action_tcp_pose_keys = self._right_tcp_pose_keys

        self._control_frequency = self.config.control_frequency
        self._aheadtime = self.config.aheadtime
        self._gain = self.config.gain

    @property
    def _action_ft(self) -> dict[str, type]:
        features: dict[str, type] = {}
        if self.config.control_mode == ControlMode.JOINT_MOTION:
            features.update(dict.fromkeys(self._left_action_joint_keys, float))
            features.update(dict.fromkeys(self._right_action_joint_keys, float))
        elif self.config.control_mode == ControlMode.CARTESIAN_MOTION:
            features.update(dict.fromkeys(self._left_action_tcp_pose_keys, float))
            features.update(dict.fromkeys(self._right_action_tcp_pose_keys, float))
        else:
            raise ValueError(f"Unsupported control_mode: {self.config.control_mode}")
        features[self._left_gripper_key] = float
        features[self._right_gripper_key] = float
        return features

    @property
    def _proprioception_ft(self) -> dict[str, type]:
        features: dict[str, type] = {}
        if self.config.control_mode == ControlMode.JOINT_MOTION:
            features.update(dict.fromkeys(self._left_joint_pos_keys, float))
            features.update(dict.fromkeys(self._right_joint_pos_keys, float))
        elif self.config.control_mode == ControlMode.CARTESIAN_MOTION:
            features.update(dict.fromkeys(self._left_tcp_pose_keys, float))
            features.update(dict.fromkeys(self._right_tcp_pose_keys, float))
        else:
            raise ValueError(f"Unsupported control_mode: {self.config.control_mode}")
        features[self._left_gripper_key] = float
        features[self._right_gripper_key] = float
        return features

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        features: dict[str, tuple] = {}
        for cam_name in self.cameras:
            features[cam_name] = (
                self.config.cameras[cam_name].height,
                self.config.cameras[cam_name].width,
                3,
            )
        return features

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._proprioception_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._action_ft

    @property
    def is_connected(self) -> bool:
        return (
            self._is_connected
            and self._left_robot is not None
            and self._right_robot is not None
            and all(cam.is_connected for cam in self.cameras.values())
        )

    @property
    def is_calibrated(self) -> bool:
        return self.is_connected

    def calibrate(self) -> None:
        self.logger.info(
            "Dobot Nova5 is factory calibrated, no runtime calibration needed."
        )

    def configure(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        self.logger.info(
            f"Configuring robot for {self.config.control_mode.value} mode..."
        )
        self._configure_tool_coordinates()

    def set_servo_period(self, period_s: float) -> None:
        self._last_servo_period_s = float(np.clip(period_s, 0.004, 0.2))

    def _configure_tool_coordinates(self) -> None:
        if self._left_robot is None or self._right_robot is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        left_tool = (
            int(self.config.left_tool_coordinate_index)
            if self.config.use_tool_coordinate
            else 0
        )
        right_tool = (
            int(self.config.right_tool_coordinate_index)
            if self.config.use_tool_coordinate
            else 0
        )
        self._set_tool_coordinate_one_arm(
            self._left_robot, self._left_feed_data, "left", left_tool
        )
        self._set_tool_coordinate_one_arm(
            self._right_robot, self._right_feed_data, "right", right_tool
        )

    def _set_tool_coordinate_one_arm(
        self,
        robot: DobotApiDashboard,
        feed: _FeedState,
        side: str,
        tool_index: int,
    ) -> None:
        self._raise_if_dobot_error(robot, robot.Tool(tool_index), f"{side} Tool")
        self._wait_for_feedback_tool_index(feed, side, tool_index)
        self.logger.info(f"{side} global tool coordinate set to Tool({tool_index})")

    def _wait_for_feedback_tool_index(
        self,
        feed: _FeedState,
        side: str,
        tool_index: int,
        timeout_s: float = 1.0,
    ) -> None:
        start_time = time.time()
        while time.time() - start_time <= timeout_s:
            if int(feed.Tool) == int(tool_index):
                return
            time.sleep(0.02)
        self.logger.warn(
            f"{side} feedback Tool index did not update to {tool_index} within {timeout_s:.1f}s "
            f"(current={feed.Tool}). Continuing with dashboard Tool({tool_index}) set."
        )

    def _parse_dobot_response(self, value_recv: str) -> tuple[int, list[str]]:
        if not isinstance(value_recv, str):
            raise RuntimeError(
                f"Invalid Dobot response type: {type(value_recv).__name__}"
            )
        if "Not Tcp" in value_recv:
            raise RuntimeError("Robot is not in TCP control mode")
        match = re.match(r"\s*(-?\d+)\s*,\s*\{([^}]*)\}", value_recv)
        if match is None:
            raise RuntimeError(f"Could not parse Dobot response: {value_recv!r}")
        error_id = int(match.group(1))
        values = [value.strip() for value in match.group(2).split(",") if value.strip()]
        return error_id, values

    def _dobot_error_detail(self, robot: DobotApiDashboard | None) -> str:
        if robot is None:
            return ""
        try:
            return robot.GetErrorID().strip()
        except Exception as e:
            return f"failed to read GetErrorID: {e}"

    def _dashboard_robot_mode(self, robot: DobotApiDashboard | None) -> int | None:
        if robot is None:
            return None
        try:
            error_id, values = self._parse_dobot_response(robot.RobotMode())
        except Exception:
            return None
        if error_id != 0 or not values:
            return None
        try:
            return int(float(values[0]))
        except ValueError:
            return None

    def _raise_if_dobot_error(
        self, robot: DobotApiDashboard | None, response: str, command_name: str
    ) -> list[str]:
        error_id, values = self._parse_dobot_response(response)
        if error_id == 0:
            return values
        message = f"{command_name} failed with ErrorID {error_id}: {response.strip()}"
        error_detail = self._dobot_error_detail(robot)
        if error_detail:
            message = f"{message}; GetErrorID: {error_detail}"
        raise RuntimeError(message)

    def _format_array(self, values: np.ndarray | list[float], precision: int = 3) -> str:
        array = np.asarray(values, dtype=np.float64)
        return np.array2string(array, precision=precision, suppress_small=True)

    def _cartesian_failure_reason(
        self, side: str, target_pose_mm_deg: np.ndarray, error_detail: str
    ) -> str:
        x_mm, y_mm, z_mm = target_pose_mm_deg[:3]
        rx_deg, ry_deg, rz_deg = target_pose_mm_deg[3:]
        reasons: list[str] = []

        if "肩奇异" in error_detail or "shoulder singularity" in error_detail.lower():
            reasons.append("Dobot 报告接近肩奇异点")
        if "逆解算无解" in error_detail or "InverseKin" in error_detail:
            reasons.append("Dobot 报告目标位姿逆解无解")
        if abs(x_mm) < 120.0 and z_mm > 450.0:
            reasons.append(
                f"TCP 在高位 z={z_mm:.1f}mm 且 x={x_mm:.1f}mm 靠近机器人肩部奇异轴"
            )
        if abs(abs(rx_deg) - 180.0) < 8.0 and abs(ry_deg) < 10.0:
            reasons.append(
                f"姿态接近翻腕边界 rx={rx_deg:.1f}deg, ry={ry_deg:.1f}deg，可能让同一 TCP 位置的 IK 分支变窄"
            )

        debug = self._last_cartesian_command_debug.get(side, {})
        workspace_clipped = debug.get("workspace_clipped")
        step_limited = debug.get("step_limited")
        if workspace_clipped:
            reasons.append("目标先被 workspace 裁剪，裁剪后的边界点可能更接近奇异/不可达边界")
        if step_limited:
            reasons.append("目标被 max_cartesian_step_m 限速，当前帧只走到中间点")

        if not reasons:
            reasons.append("Dobot 返回错误，但未命中本地启发式；优先看下面 GetErrorID 原文")
        return "; ".join(reasons)

    def _log_cartesian_command_failure(
        self,
        side: str,
        robot: DobotApiDashboard | None,
        response: str | None,
        command_name: str,
    ) -> None:
        debug = self._last_cartesian_command_debug.get(side)
        if not debug:
            self.logger.error(f"{side} {command_name} failed, but no Cartesian debug context was recorded.")
            return

        target_pose = np.asarray(debug["target_pose_mm_deg"], dtype=np.float64)
        current_pose = np.asarray(debug["current_pose_mm_deg"], dtype=np.float64)
        current_joint = np.asarray(debug["current_joint_deg"], dtype=np.float64)
        requested_pos = np.asarray(debug["requested_pos_m"], dtype=np.float64)
        clipped_pos = np.asarray(debug["clipped_pos_m"], dtype=np.float64)
        target_pos_before_step = np.asarray(debug["target_pos_before_step_mm"], dtype=np.float64)
        error_detail = self._dobot_error_detail(robot)
        reason = self._cartesian_failure_reason(side, target_pose, error_detail)

        self.logger.error(
            f"{side} Cartesian command failed diagnostics:\n"
            f"  command={command_name}\n"
            f"  likely_reason={reason}\n"
            f"  response={response.strip() if isinstance(response, str) else response}\n"
            f"  GetErrorID={error_detail}\n"
            f"  requested_pos_m={self._format_array(requested_pos)}\n"
            f"  clipped_pos_m={self._format_array(clipped_pos)} workspace_clipped={debug['workspace_clipped']}\n"
            f"  current_pose_mm_deg={self._format_array(current_pose)}\n"
            f"  target_pos_before_step_mm={self._format_array(target_pos_before_step)}\n"
            f"  target_pose_mm_deg={self._format_array(target_pose)}\n"
            f"  current_joint_deg={self._format_array(current_joint)}\n"
            f"  step_limited={debug['step_limited']} max_step_mm={debug['max_step_mm']:.1f}"
        )

    def _wait_for_first_feedback(self, timeout_s: float = 3.0) -> None:
        start_time = time.time()
        while True:
            left_ready = (
                self._left_feed_data.MessageSize != -1
                or self._left_feed_data.RobotMode != -1
            )
            right_ready = (
                self._right_feed_data.MessageSize != -1
                or self._right_feed_data.RobotMode != -1
            )
            if left_ready and right_ready:
                return
            if time.time() - start_time > timeout_s:
                return
            time.sleep(0.02)

    def _wait_until_not_error_mode(
        self,
        robot: DobotApiDashboard,
        feed: _FeedState,
        side: str,
        timeout_s: float = 10.0,
    ) -> None:
        start_time = time.time()
        while int(feed.RobotMode) == 9:
            if time.time() - start_time > timeout_s:
                raise TimeoutError(
                    f"{side} arm stays in error mode (RobotMode=9) after ClearError. "
                    f"GetErrorID: {self._dobot_error_detail(robot)}"
                )
            time.sleep(0.1)

    def _wait_for_joint_target(
        self,
        feed: _FeedState,
        robot: DobotApiDashboard,
        target_joint_deg: list[float],
        description: str,
        side: str,
        tolerance_deg: float = 0.5,
        timeout_s: float = 60.0,
    ) -> None:
        target = np.asarray(target_joint_deg, dtype=np.float64)
        start_time = time.time()
        last_log_time = 0.0
        while True:
            robot_mode = int(feed.RobotMode)
            current_joint = np.asarray(feed.qActual, dtype=np.float64)
            max_abs_error = float(np.max(np.abs(current_joint - target)))
            if robot_mode == 9:
                raise RuntimeError(
                    f"{side} {description} failed: robot entered error mode. "
                    f"GetErrorID: {self._dobot_error_detail(robot)}"
                )
            if robot_mode == 5 and max_abs_error <= tolerance_deg:
                return
            now = time.time()
            if now - start_time > timeout_s:
                raise TimeoutError(
                    f"Timed out waiting for {side} {description}: max_joint_error={max_abs_error:.3f} deg, "
                    f"RobotMode={robot_mode}, target={target.tolist()}, current={current_joint.tolist()}"
                )
            if now - last_log_time >= 2.0:
                self.logger.info(
                    f"Waiting for {side} {description}: RobotMode={robot_mode}, "
                    f"max_joint_error={max_abs_error:.3f} deg"
                )
                last_log_time = now
            time.sleep(0.1)

    def _move_joint_movj(
        self,
        robot: DobotApiDashboard,
        feed: _FeedState,
        joint_degrees: list[float],
        description: str,
        side: str,
    ) -> None:
        target = [float(value) for value in joint_degrees]
        self._raise_if_dobot_error(
            robot, robot.SpeedFactor(int(self.config.start_vel_scale)), "SpeedFactor"
        )
        response = robot.MovJ(
            target[0],
            target[1],
            target[2],
            target[3],
            target[4],
            target[5],
            1,
            v=int(self.config.start_vel_scale),
        )
        self.logger.info(
            f"{side} {description} MovJ(joint) response: {response.strip()}"
        )
        values = self._raise_if_dobot_error(robot, response, "MovJ(joint)")
        if values:
            try:
                command_id = int(float(values[0]))
                self._wait_for_command_id(feed, robot, command_id, description, side)
                return
            except (ValueError, TypeError):
                self.logger.warn(
                    f"{side} {description} command id parse failed ({values}), "
                    "falling back to joint-error completion check."
                )
        self._wait_for_joint_target(feed, robot, target, description, side)

    def _send_joint_movj(
        self,
        robot: DobotApiDashboard,
        joint_degrees: list[float],
        description: str,
        side: str,
    ) -> int | None:
        target = [float(value) for value in joint_degrees]
        self._raise_if_dobot_error(
            robot, robot.SpeedFactor(int(self.config.start_vel_scale)), "SpeedFactor"
        )
        response = robot.MovJ(
            target[0],
            target[1],
            target[2],
            target[3],
            target[4],
            target[5],
            1,
            v=int(self.config.start_vel_scale),
        )
        self.logger.info(
            f"{side} {description} MovJ(joint) response: {response.strip()}"
        )
        values = self._raise_if_dobot_error(robot, response, "MovJ(joint)")
        if not values:
            return None
        try:
            return int(float(values[0]))
        except (ValueError, TypeError):
            self.logger.warn(
                f"{side} {description} command id parse failed ({values}), "
                "falling back to joint-error completion check."
            )
            return None

    def _move_both_arms_joint_movj_sync(
        self,
        left_joint_degrees: list[float],
        right_joint_degrees: list[float],
        description: str,
    ) -> None:
        if self._left_robot is None or self._right_robot is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        left_command_id = self._send_joint_movj(
            self._left_robot,
            left_joint_degrees,
            description,
            "left",
        )
        right_command_id = self._send_joint_movj(
            self._right_robot,
            right_joint_degrees,
            description,
            "right",
        )

        if left_command_id is not None:
            self._wait_for_command_id(
                self._left_feed_data,
                self._left_robot,
                left_command_id,
                description,
                "left",
            )
        else:
            self._wait_for_joint_target(
                self._left_feed_data,
                self._left_robot,
                np.asarray(left_joint_degrees, dtype=np.float64),
                description,
                "left",
            )

        if right_command_id is not None:
            self._wait_for_command_id(
                self._right_feed_data,
                self._right_robot,
                right_command_id,
                description,
                "right",
            )
        else:
            self._wait_for_joint_target(
                self._right_feed_data,
                self._right_robot,
                np.asarray(right_joint_degrees, dtype=np.float64),
                description,
                "right",
            )

    def _move_both_arms_joint_movj(
        self,
        left_joint_degrees: list[float],
        right_joint_degrees: list[float],
        description: str,
    ) -> None:
        if self._left_robot is None or self._right_robot is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        move_specs = [
            (self._left_robot, self._left_feed_data, left_joint_degrees, "left"),
            (self._right_robot, self._right_feed_data, right_joint_degrees, "right"),
        ]
        if self.config.reset_move_order == ResetMoveOrder.RIGHT_THEN_LEFT:
            move_specs.reverse()
        elif self.config.reset_move_order != ResetMoveOrder.LEFT_THEN_RIGHT:
            raise ValueError(f"Unsupported reset_move_order: {self.config.reset_move_order}")

        for robot, feed, joint_degrees, side in move_specs:
            self._move_joint_movj(robot, feed, joint_degrees, description, side)

    def _move_pose_movj(
        self,
        robot: DobotApiDashboard,
        pose: np.ndarray,
        description: str,
        side: str,
    ) -> int | None:
        pose_mm_deg = [float(value) for value in pose]
        self._raise_if_dobot_error(
            robot, robot.SpeedFactor(int(self.config.start_vel_scale)), "SpeedFactor"
        )
        response = robot.MovJ(
            pose_mm_deg[0],
            pose_mm_deg[1],
            pose_mm_deg[2],
            pose_mm_deg[3],
            pose_mm_deg[4],
            pose_mm_deg[5],
            0,
            v=int(self.config.start_vel_scale),
        )
        self.logger.info(
            f"{side} {description} MovJ(pose) response: {response.strip()}"
        )
        values = self._raise_if_dobot_error(robot, response, "MovJ(pose)")
        if not values:
            return None
        try:
            return int(float(values[0]))
        except (ValueError, TypeError):
            self.logger.warn(
                f"{side} {description} command id parse failed ({values}), "
                "falling back to fixed wait."
            )
            return None

    def _move_both_arms_y_clearance_sync(self) -> None:
        if self._left_robot is None or self._right_robot is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        clearance_m = float(self.config.reset_y_clearance_m)
        if clearance_m <= 0.0:
            return

        left_pose = np.asarray(self._left_feed_data.tcpPose, dtype=np.float64).copy()
        right_pose = np.asarray(self._right_feed_data.tcpPose, dtype=np.float64).copy()
        left_pose[1] += float(self.config.left_reset_y_clearance_sign) * clearance_m * MM_PER_METER
        right_pose[1] += float(self.config.right_reset_y_clearance_sign) * clearance_m * MM_PER_METER

        left_position_m = left_pose[:3] / MM_PER_METER
        right_position_m = right_pose[:3] / MM_PER_METER
        left_pose[:3] = self._clip_workspace_position("left", left_position_m) * MM_PER_METER
        right_pose[:3] = self._clip_workspace_position("right", right_position_m) * MM_PER_METER

        self.logger.info(
            f"Moving both arms to Y-clearance poses: left_y={left_pose[1] / MM_PER_METER:.3f}m, "
            f"right_y={right_pose[1] / MM_PER_METER:.3f}m"
        )
        left_command_id = self._move_pose_movj(
            self._left_robot, left_pose, "Y-clearance pose", "left"
        )
        right_command_id = self._move_pose_movj(
            self._right_robot, right_pose, "Y-clearance pose", "right"
        )

        if left_command_id is not None:
            self._wait_for_command_id(
                self._left_feed_data,
                self._left_robot,
                left_command_id,
                "Y-clearance pose",
                "left",
            )
        else:
            time.sleep(1.0)

        if right_command_id is not None:
            self._wait_for_command_id(
                self._right_feed_data,
                self._right_robot,
                right_command_id,
                "Y-clearance pose",
                "right",
            )
        else:
            time.sleep(1.0)

    def _wait_for_command_id(
        self,
        feed: _FeedState,
        robot: DobotApiDashboard,
        command_id: int,
        description: str,
        side: str,
        timeout_s: float = 60.0,
    ) -> None:
        start_time = time.time()
        last_log_time = 0.0
        command_id = int(command_id)
        while True:
            robot_mode = int(feed.RobotMode)
            current_command_id = int(feed.robotCurrentCommandID)
            if robot_mode == 9:
                raise RuntimeError(
                    f"{side} {description} failed: robot entered error mode while waiting for command "
                    f"{command_id}. GetErrorID: {self._dobot_error_detail(robot)}"
                )
            if robot_mode == 5 and current_command_id == command_id:
                return
            now = time.time()
            if now - start_time > timeout_s:
                raise TimeoutError(
                    f"Timed out waiting for {side} {description}: target command ID={command_id}, "
                    f"current command ID={current_command_id}, RobotMode={robot_mode}"
                )
            if now - last_log_time >= 2.0:
                self.logger.info(
                    f"Waiting for {side} {description}: RobotMode={robot_mode}, "
                    f"CurrentCommandId={current_command_id}, target={command_id}"
                )
                last_log_time = now
            time.sleep(0.1)

    def _feed_loop(
        self,
        feed_client: DobotApiFeedBack,
        feed_state: _FeedState,
        lock: threading.Lock,
    ) -> dict | None:
        while True:
            if (
                not self._is_connected
                and self._left_robot is None
                and self._right_robot is None
            ):
                return None
            try:
                feed_info = feed_client.feedBackData()
            except Exception:
                return None
            with lock:
                if feed_info is None:
                    continue
                if hex(feed_info["TestValue"][0]) != "0x123456789abcdef":
                    continue
                feed_state.MessageSize = feed_info["len"][0]
                feed_state.RobotMode = feed_info["RobotMode"][0]
                feed_state.DigitalInputs = feed_info["DigitalInputs"][0]
                feed_state.DigitalOutputs = feed_info["DigitalOutputs"][0]
                feed_state.User = int(feed_info["User"][0])
                feed_state.Tool = int(feed_info["Tool"][0])
                feed_state.robotCurrentCommandID = feed_info["CurrentCommandId"][0]
                feed_state.tcpPose = feed_info["ToolVectorActual"][0]
                feed_state.qActual = feed_info["QActual"][0]

    def _connect_one_arm(
        self,
        side: str,
        robot_ip: str,
        dashboard_port: int,
        feed_port: int,
    ) -> tuple[DobotApiDashboard, DobotApiFeedBack]:
        self.logger.info(f"Connecting {side} Dobot Nova5: {robot_ip}")
        robot = DobotApiDashboard(robot_ip, dashboard_port)
        feed = DobotApiFeedBack(robot_ip, feed_port)
        return robot, feed

    def connect(self, calibrate: bool = False, go_to_start: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(
                f"{self} already connected, do not run `robot.connect()` twice."
            )
        try:
            self._left_robot, self._left_feed = self._connect_one_arm(
                "left",
                self.config.left_robot_ip,
                self.config.left_dashboardPort,
                self.config.left_feedPortFour,
            )
            self._right_robot, self._right_feed = self._connect_one_arm(
                "right",
                self.config.right_robot_ip,
                self.config.right_dashboardPort,
                self.config.right_feedPortFour,
            )

            self._is_connected = True

            self._left_feed_thread = threading.Thread(
                target=self._feed_loop,
                args=(self._left_feed, self._left_feed_data, self._left_feed_lock),
                daemon=True,
            )
            self._right_feed_thread = threading.Thread(
                target=self._feed_loop,
                args=(self._right_feed, self._right_feed_data, self._right_feed_lock),
                daemon=True,
            )
            self._left_feed_thread.start()
            self._right_feed_thread.start()
            self._wait_for_first_feedback()

            for side, robot, feed in [
                ("left", self._left_robot, self._left_feed_data),
                ("right", self._right_robot, self._right_feed_data),
            ]:
                if int(feed.RobotMode) == 9:
                    self.logger.warn(
                        f"{side} robot in error mode before enabling, trying ClearError ..."
                    )
                    self._raise_if_dobot_error(robot, robot.ClearError(), "ClearError")
                    self._wait_until_not_error_mode(robot, feed, side)
                    self.logger.info(f"{side} robot fault cleared")

                self.logger.info(f"Enabling {side} robot...")
                enable_response = robot.EnableRobot()
                enable_error, _ = self._parse_dobot_response(enable_response)
                if enable_error != 0:
                    mode_response = robot.RobotMode()
                    mode_error, mode_values = self._parse_dobot_response(mode_response)
                    current_mode = (
                        int(float(mode_values[0]))
                        if mode_error == 0 and mode_values
                        else -1
                    )
                    if current_mode not in (5, 6, 7, 8):
                        raise RuntimeError(
                            f"{side} EnableRobot failed with ErrorID {enable_error}: {enable_response.strip()} "
                            f"(RobotMode={current_mode}); GetErrorID: {self._dobot_error_detail(robot)}"
                        )
                    self.logger.warn(
                        f"{side} EnableRobot returned {enable_error}, but RobotMode={current_mode}. "
                        "Proceeding with existing enabled/control state."
                    )
                else:
                    self.logger.info(f"{side} robot enabled successfully.")

            timeout = 30.0
            start_time = time.time()
            last_mode_log_time = 0.0
            while True:
                left_feed_mode = int(self._left_feed_data.RobotMode)
                right_feed_mode = int(self._right_feed_data.RobotMode)
                left_dashboard_mode = self._dashboard_robot_mode(self._left_robot)
                right_dashboard_mode = self._dashboard_robot_mode(self._right_robot)
                left_mode = left_dashboard_mode if left_dashboard_mode is not None else left_feed_mode
                right_mode = right_dashboard_mode if right_dashboard_mode is not None else right_feed_mode
                left_ready = left_mode == 5
                right_ready = right_mode == 5
                if left_ready and right_ready:
                    break
                now = time.time()
                if now - last_mode_log_time >= 2.0:
                    self.logger.info(
                        "Waiting for robots to become operational: "
                        f"left feed/dashboard={left_feed_mode}/{left_dashboard_mode}, "
                        f"right feed/dashboard={right_feed_mode}/{right_dashboard_mode}"
                    )
                    last_mode_log_time = now
                if time.time() - start_time > timeout:
                    raise RuntimeError(
                        "Both robots did not become operational within 30 seconds: "
                        f"left feed/dashboard={left_feed_mode}/{left_dashboard_mode}, "
                        f"right feed/dashboard={right_feed_mode}/{right_dashboard_mode}; "
                        f"left GetErrorID={self._dobot_error_detail(self._left_robot)}; "
                        f"right GetErrorID={self._dobot_error_detail(self._right_robot)}"
                    )
                time.sleep(0.1)

            if self._left_gripper and self.config.use_left_gripper:
                self.logger.info("Connecting left DH Gripper via robot RS485...")
                try:
                    self._raise_if_dobot_error(
                        self._left_robot,
                        self._left_robot.SetToolMode(1, 1, self.config.left_tool_identify),
                        "left SetToolMode",
                    )
                    self._raise_if_dobot_error(
                        self._left_robot,
                        self._left_robot.SetTool485(
                            self.config.left_dh_gripper_baudrate,
                            "N",
                            1,
                            self.config.left_tool_identify,
                        ),
                        "left SetTool485",
                    )
                    left_modbus = _DobotModbusRTU(
                        self._left_robot,
                        self.config.left_master_ip,
                        self.config.left_master_port,
                        self.config.left_dh_gripper.slave_id,
                    )
                    self._left_gripper.connect(left_modbus)
                    self._left_gripper_connected = True
                except Exception as e:
                    self._left_gripper_connected = False
                    self.logger.error(
                        f"Failed to connect left DH Gripper, continuing without left gripper control: {e}"
                    )
            if self._right_gripper and self.config.use_right_gripper:
                self.logger.info("Connecting right DH Gripper via robot RS485...")
                try:
                    self._raise_if_dobot_error(
                        self._right_robot,
                        self._right_robot.SetToolMode(1, 1, self.config.right_tool_identify),
                        "right SetToolMode",
                    )
                    self._raise_if_dobot_error(
                        self._right_robot,
                        self._right_robot.SetTool485(
                            self.config.right_dh_gripper_baudrate,
                            "N",
                            1,
                            self.config.right_tool_identify,
                        ),
                        "right SetTool485",
                    )
                    right_modbus = _DobotModbusRTU(
                        self._right_robot,
                        self.config.right_master_ip,
                        self.config.right_master_port,
                        self.config.right_dh_gripper.slave_id,
                    )
                    self._right_gripper.connect(right_modbus)
                    self._right_gripper_connected = True
                except Exception as e:
                    self._right_gripper_connected = False
                    self.logger.error(
                        f"Failed to connect right DH Gripper, continuing without right gripper control: {e}"
                    )

            for cam in self.cameras.values():
                cam.connect()

            self.config.go_to_start = (
                go_to_start if go_to_start is not None else self.config.go_to_start
            )
            if self.config.go_to_start:
                self._go_to_start()

            self.configure()
            self._start_action_worker()
            self.logger.info("✅ BiDobot Nova5 connected and ready.")
        except Exception:
            self._is_connected = False
            self._left_robot = None
            self._right_robot = None
            self._left_feed = None
            self._right_feed = None
            self._left_gripper_connected = False
            self._right_gripper_connected = False
            raise

    def _initialize_gripper_position(self) -> None:
        if (
            self._left_gripper
            and self.config.use_left_gripper
            and self._left_gripper_connected
        ):
            left_target = 1.0 if self.config.left_dh_gripper_init_open else 0.0
            try:
                self._left_gripper.initialize_gripper_position(left_target)
            except Exception as e:
                self._left_gripper_connected = False
                self.logger.error(
                    f"Left DH gripper init move failed, disabling left gripper commands: {e}"
                )
        if (
            self._right_gripper
            and self.config.use_right_gripper
            and self._right_gripper_connected
        ):
            right_target = 1.0 if self.config.right_dh_gripper_init_open else 0.0
            try:
                self._right_gripper.initialize_gripper_position(right_target)
            except Exception as e:
                self._right_gripper_connected = False
                self.logger.error(
                    f"Right DH gripper init move failed, disabling right gripper commands: {e}"
                )

    def _go_to_start(self) -> None:
        if (
            not self.is_connected
            or self._left_robot is None
            or self._right_robot is None
        ):
            raise DeviceNotConnectedError(f"{self} is not connected.")
        self.logger.info("Moving both arms to start position...")
        if self.config.reset_strategy == ResetStrategy.SYNC_Y_CLEARANCE_THEN_JOINT:
            self._move_both_arms_y_clearance_sync()
            self._move_both_arms_joint_movj_sync(
                self.config.left_start_position_degree,
                self.config.right_start_position_degree,
                "start position",
            )
        elif self.config.reset_strategy == ResetStrategy.SEQUENTIAL_JOINT:
            self._move_both_arms_joint_movj(
                self.config.left_start_position_degree,
                self.config.right_start_position_degree,
                "start position",
            )
        else:
            raise ValueError(f"Unsupported reset_strategy: {self.config.reset_strategy}")
        self._initialize_gripper_position()
        self.logger.info("✅ Both arms at start position.")

    def _go_to_home(self) -> None:
        if (
            not self.is_connected
            or self._left_robot is None
            or self._right_robot is None
        ):
            raise DeviceNotConnectedError(f"{self} is not connected.")
        self.logger.info("Moving both arms to home position...")
        if self.config.reset_strategy == ResetStrategy.SYNC_Y_CLEARANCE_THEN_JOINT:
            self._move_both_arms_y_clearance_sync()
            self._move_both_arms_joint_movj_sync(
                self.config.left_home_point_list,
                self.config.right_home_point_list,
                "home position",
            )
        elif self.config.reset_strategy == ResetStrategy.SEQUENTIAL_JOINT:
            self._move_both_arms_joint_movj(
                self.config.left_home_point_list,
                self.config.right_home_point_list,
                "home position",
            )
        else:
            raise ValueError(f"Unsupported reset_strategy: {self.config.reset_strategy}")
        self._initialize_gripper_position()
        self.logger.info("✅ Both arms at home position.")

    def reset_to_initial_position(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        with self._pause_action_worker():
            if self.config.reset_target == ResetTarget.START:
                self._go_to_start()
            elif self.config.reset_target == ResetTarget.HOME:
                self._go_to_home()
            else:
                raise ValueError(f"Unsupported reset_target: {self.config.reset_target}")

    def _read_tcp_pose_quat(self, feed: _FeedState) -> np.ndarray:
        tcp_pose = np.asarray(feed.tcpPose, dtype=np.float64)
        pos_m = tcp_pose[:3] / MM_PER_METER
        quat = euler_to_quaternion(
            np.deg2rad(tcp_pose[3]),
            np.deg2rad(tcp_pose[4]),
            np.deg2rad(tcp_pose[5]),
        )
        return np.array(
            [pos_m[0], pos_m[1], pos_m[2], quat[0], quat[1], quat[2], quat[3]],
            dtype=np.float32,
        )

    def _read_gripper_pos(self, side: str) -> float:
        if side == "left":
            if self._left_gripper and self.config.use_left_gripper:
                return float(self._left_gripper.get_gripper_position())
            return 0.0
        else:
            if self._right_gripper and self.config.use_right_gripper:
                return float(self._right_gripper.get_gripper_position())
            return 0.0

    def get_current_tcp_pose_quat(self) -> tuple[np.ndarray, np.ndarray]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        left_pose = self._read_tcp_pose_quat(self._left_feed_data)
        right_pose = self._read_tcp_pose_quat(self._right_feed_data)
        left_gripper = self._read_gripper_pos("left")
        right_gripper = self._read_gripper_pos("right")
        left = np.array([*left_pose, left_gripper], dtype=np.float32)
        right = np.array([*right_pose, right_gripper], dtype=np.float32)
        return left, right

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        obs_start_s = time.perf_counter()
        obs_timing: dict[str, float] = {}
        obs_dict: dict[str, Any] = {}

        if self.config.control_mode == ControlMode.JOINT_MOTION:
            section_s = time.perf_counter()
            for i, key in enumerate(self._left_joint_pos_keys):
                obs_dict[key] = self._left_feed_data.qActual[i]
            obs_timing["left_arm_ms"] = (time.perf_counter() - section_s) * 1e3

            section_s = time.perf_counter()
            for i, key in enumerate(self._right_joint_pos_keys):
                obs_dict[key] = self._right_feed_data.qActual[i]
            obs_timing["right_arm_ms"] = (time.perf_counter() - section_s) * 1e3
        elif self.config.control_mode == ControlMode.CARTESIAN_MOTION:
            section_s = time.perf_counter()
            left_pose = self._left_feed_data.tcpPose

            obs_dict["left_tcp.x"] = left_pose[0] / MM_PER_METER
            obs_dict["left_tcp.y"] = left_pose[1] / MM_PER_METER
            obs_dict["left_tcp.z"] = left_pose[2] / MM_PER_METER
            left_quat = euler_to_quaternion(
                np.deg2rad(left_pose[3]),
                np.deg2rad(left_pose[4]),
                np.deg2rad(left_pose[5]),
            )
            left_r6d = quaternion_to_rotation_6d(
                left_quat[0], left_quat[1], left_quat[2], left_quat[3]
            )
            obs_dict["left_tcp.r1"] = left_r6d[0]
            obs_dict["left_tcp.r2"] = left_r6d[1]
            obs_dict["left_tcp.r3"] = left_r6d[2]
            obs_dict["left_tcp.r4"] = left_r6d[3]
            obs_dict["left_tcp.r5"] = left_r6d[4]
            obs_dict["left_tcp.r6"] = left_r6d[5]
            obs_timing["left_arm_ms"] = (time.perf_counter() - section_s) * 1e3

            section_s = time.perf_counter()
            right_pose = self._right_feed_data.tcpPose

            obs_dict["right_tcp.x"] = right_pose[0] / MM_PER_METER
            obs_dict["right_tcp.y"] = right_pose[1] / MM_PER_METER
            obs_dict["right_tcp.z"] = right_pose[2] / MM_PER_METER
            right_quat = euler_to_quaternion(
                np.deg2rad(right_pose[3]),
                np.deg2rad(right_pose[4]),
                np.deg2rad(right_pose[5]),
            )
            right_r6d = quaternion_to_rotation_6d(
                right_quat[0], right_quat[1], right_quat[2], right_quat[3]
            )
            obs_dict["right_tcp.r1"] = right_r6d[0]
            obs_dict["right_tcp.r2"] = right_r6d[1]
            obs_dict["right_tcp.r3"] = right_r6d[2]
            obs_dict["right_tcp.r4"] = right_r6d[3]
            obs_dict["right_tcp.r5"] = right_r6d[4]
            obs_dict["right_tcp.r6"] = right_r6d[5]
            obs_timing["right_arm_ms"] = (time.perf_counter() - section_s) * 1e3
        else:
            raise ValueError(f"Unsupported control_mode: {self.config.control_mode}")

        section_s = time.perf_counter()
        obs_dict[self._left_gripper_key] = self._read_gripper_pos("left")
        obs_timing["left_grip_ms"] = (time.perf_counter() - section_s) * 1e3

        section_s = time.perf_counter()
        obs_dict[self._right_gripper_key] = self._read_gripper_pos("right")
        obs_timing["right_grip_ms"] = (time.perf_counter() - section_s) * 1e3

        cameras_start_s = time.perf_counter()
        for cam_key, cam in self.cameras.items():
            section_s = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            obs_timing[f"cam[{cam_key}]_ms"] = (time.perf_counter() - section_s) * 1e3
        obs_timing["cameras_ms"] = (time.perf_counter() - cameras_start_s) * 1e3
        obs_timing["total_ms"] = (time.perf_counter() - obs_start_s) * 1e3
        self._last_obs_timing = obs_timing

        return obs_dict

    def _send_joint_action_one_arm(
        self,
        robot: DobotApiDashboard,
        action: dict[str, Any],
        keys: tuple[str, ...],
        side: str,
    ) -> None:
        target_pos = [float(action[k]) for k in keys]
        response = robot.ServoJ(
            target_pos[0],
            target_pos[1],
            target_pos[2],
            target_pos[3],
            target_pos[4],
            target_pos[5],
        )
        self._raise_if_dobot_error(robot, response, f"{side} ServoJ")

    def _clip_workspace_position(self, side: str, position_m: np.ndarray) -> np.ndarray:
        if not self.config.enable_clip:
            return position_m

        if side == "left":
            min_xyz = self.config.left_workspace_min_xyz_m
            max_xyz = self.config.left_workspace_max_xyz_m
        elif side == "right":
            min_xyz = self.config.right_workspace_min_xyz_m
            max_xyz = self.config.right_workspace_max_xyz_m
        else:
            raise ValueError(f"Unsupported arm side for workspace clipping: {side}")

        return np.clip(
            position_m,
            np.asarray(min_xyz, dtype=np.float64),
            np.asarray(max_xyz, dtype=np.float64),
        )

    def _cartesian_feed_state(self, side: str) -> tuple[np.ndarray, np.ndarray]:
        if side == "left":
            lock = self._left_feed_lock
            feed = self._left_feed_data
        elif side == "right":
            lock = self._right_feed_lock
            feed = self._right_feed_data
        else:
            raise ValueError(f"Unsupported arm side: {side}")

        with lock:
            tcp_pose = np.asarray(feed.tcpPose, dtype=np.float64).copy()
            joint_pos = np.asarray(feed.qActual, dtype=np.float64).copy()
        return tcp_pose, joint_pos

    def _parse_ik_joint_values(
        self, robot: DobotApiDashboard, response: str, command_name: str
    ) -> np.ndarray | None:
        error_id, values = self._parse_dobot_response(response)
        if error_id != 0:
            return None
        if len(values) < JOINT_DOF:
            self.logger.warn(
                f"{command_name} returned too few joint values: {response.strip()}"
            )
            return None
        try:
            return np.asarray(values[:JOINT_DOF], dtype=np.float64)
        except ValueError:
            self.logger.warn(
                f"{command_name} returned invalid joint values: {response.strip()}"
            )
            return None

    def _can_fallback_cartesian_servo_error(self, response: str) -> bool:
        try:
            error_id, _ = self._parse_dobot_response(response)
        except RuntimeError:
            return False
        return error_id in (-2, -1)

    def _send_cartesian_servoj(
        self,
        robot: DobotApiDashboard,
        target_joint_deg: np.ndarray,
        side: str,
        command_name: str,
    ) -> None:
        response = robot.ServoJ(
            float(target_joint_deg[0]),
            float(target_joint_deg[1]),
            float(target_joint_deg[2]),
            float(target_joint_deg[3]),
            float(target_joint_deg[4]),
            float(target_joint_deg[5]),
        )
        self._raise_if_dobot_error(robot, response, command_name)

    def _inverse_kin_with_joint_near(
        self,
        robot: DobotApiDashboard,
        pose_mm_deg: np.ndarray,
        joint_near_deg: np.ndarray,
        side: str,
    ) -> np.ndarray | None:
        joint_near = "{" + ",".join(f"{float(v):.6f}" for v in joint_near_deg) + "}"
        response = robot.InverseKin(
            float(pose_mm_deg[0]),
            float(pose_mm_deg[1]),
            float(pose_mm_deg[2]),
            float(pose_mm_deg[3]),
            float(pose_mm_deg[4]),
            float(pose_mm_deg[5]),
            useJointNear=1,
            JointNear=joint_near,
        )
        return self._parse_ik_joint_values(robot, response, f"{side} InverseKin")

    def _resolve_cartesian_servo_target(
        self,
        robot: DobotApiDashboard,
        target_pose_mm_deg: np.ndarray,
        side: str,
    ) -> tuple[np.ndarray, np.ndarray | None]:
        if not self.config.enable_cartesian_ik_guard:
            return target_pose_mm_deg, None

        current_pose_mm_deg, current_joint_deg = self._cartesian_feed_state(side)
        target_joint_deg = self._inverse_kin_with_joint_near(
            robot, target_pose_mm_deg, current_joint_deg, side
        )
        if target_joint_deg is not None:
            return target_pose_mm_deg, target_joint_deg

        backoff_steps = int(self.config.cartesian_ik_backoff_steps)
        for step in range(backoff_steps, 0, -1):
            ratio = step / float(backoff_steps + 1)
            candidate_pose_mm_deg = current_pose_mm_deg + ratio * (
                target_pose_mm_deg - current_pose_mm_deg
            )
            target_joint_deg = self._inverse_kin_with_joint_near(
                robot, candidate_pose_mm_deg, current_joint_deg, side
            )
            if target_joint_deg is not None:
                now_s = time.perf_counter()
                if now_s - self._last_cartesian_guard_log_s[side] >= 1.0:
                    self.logger.warn(
                        f"{side} Cartesian target rejected by IK guard; backing off to {ratio:.2f} of requested step "
                        f"near pose={target_pose_mm_deg.tolist()}"
                    )
                    self._last_cartesian_guard_log_s[side] = now_s
                return candidate_pose_mm_deg, target_joint_deg

        now_s = time.perf_counter()
        if now_s - self._last_cartesian_guard_log_s[side] >= 1.0:
            self.logger.warn(
                f"{side} Cartesian target rejected by IK guard; holding current pose near singularity "
                f"for requested pose={target_pose_mm_deg.tolist()}"
            )
            self._last_cartesian_guard_log_s[side] = now_s
        return current_pose_mm_deg, current_joint_deg

    def _send_cart_action_one_arm(
        self,
        robot: DobotApiDashboard,
        action: dict[str, Any],
        prefix: str,
        side: str,
    ) -> None:
        arm_start_s = time.perf_counter()
        arm_timing: dict[str, float] = {}
        self._last_cartesian_command_debug[f"{side}_timing"] = arm_timing
        enabled_key = f"{prefix}_arm.enabled"
        if enabled_key in action and not bool(action[enabled_key]):
            arm_timing["disabled_ms"] = (time.perf_counter() - arm_start_s) * 1e3
            return

        section_s = time.perf_counter()
        requested_pos_m = np.array(
            [
                float(action[f"{prefix}_tcp.x"]),
                float(action[f"{prefix}_tcp.y"]),
                float(action[f"{prefix}_tcp.z"]),
            ],
            dtype=np.float64,
        )
        clipped_pos_m = self._clip_workspace_position(side, requested_pos_m)
        arm_timing["prep_pos_ms"] = (time.perf_counter() - section_s) * 1e3
        x_m, y_m, z_m = clipped_pos_m

        section_s = time.perf_counter()
        current_pose_mm_deg, current_joint_deg = self._cartesian_feed_state(side)
        arm_timing["feed_state_ms"] = (time.perf_counter() - section_s) * 1e3

        section_s = time.perf_counter()
        target_pos_mm = np.array(
            [x_m * MM_PER_METER, y_m * MM_PER_METER, z_m * MM_PER_METER],
            dtype=np.float64,
        )
        target_pos_before_step_mm = target_pos_mm.copy()
        max_step_mm = float(self.config.max_cartesian_step_m) * MM_PER_METER
        step_limited = False
        if max_step_mm > 0:
            delta_pos_mm = target_pos_mm - current_pose_mm_deg[:3]
            delta_norm_mm = float(np.linalg.norm(delta_pos_mm))
            if delta_norm_mm > max_step_mm:
                target_pos_mm = current_pose_mm_deg[:3] + delta_pos_mm * (
                    max_step_mm / delta_norm_mm
                )
                step_limited = True
        r6d = np.array(
            [
                action[f"{prefix}_tcp.r1"],
                action[f"{prefix}_tcp.r2"],
                action[f"{prefix}_tcp.r3"],
                action[f"{prefix}_tcp.r4"],
                action[f"{prefix}_tcp.r5"],
                action[f"{prefix}_tcp.r6"],
            ],
            dtype=np.float64,
        )
        quat = rotation_6d_to_quaternion(r6d)
        euler = quaternion_to_euler(quat[0], quat[1], quat[2], quat[3])
        target_pose_mm_deg = np.array(
            [
                target_pos_mm[0],
                target_pos_mm[1],
                target_pos_mm[2],
                float(np.rad2deg(euler[0])),
                float(np.rad2deg(euler[1])),
                float(np.rad2deg(euler[2])),
            ],
            dtype=np.float64,
        )
        arm_timing["prep_pose_ms"] = (time.perf_counter() - section_s) * 1e3
        self._last_cartesian_command_debug[side] = {
            "requested_pos_m": requested_pos_m.copy(),
            "clipped_pos_m": clipped_pos_m.copy(),
            "workspace_clipped": bool(
                not np.allclose(requested_pos_m, clipped_pos_m, rtol=0.0, atol=1e-9)
            ),
            "current_pose_mm_deg": current_pose_mm_deg.copy(),
            "current_joint_deg": current_joint_deg.copy(),
            "target_pos_before_step_mm": target_pos_before_step_mm.copy(),
            "target_pose_mm_deg": target_pose_mm_deg.copy(),
            "step_limited": step_limited,
            "max_step_mm": max_step_mm,
        }
        if (
            self.config.enable_cartesian_ik_guard
            and self.config.cartesian_ik_servoj
            and time.perf_counter() < self._cartesian_ik_servoj_until_s[side]
        ):
            section_s = time.perf_counter()
            target_pose_mm_deg, target_joint_deg = self._resolve_cartesian_servo_target(
                robot, target_pose_mm_deg, side
            )
            arm_timing["ik_guard_ms"] = (time.perf_counter() - section_s) * 1e3
            if target_joint_deg is not None:
                section_s = time.perf_counter()
                self._send_cartesian_servoj(
                    robot, target_joint_deg, side, f"{side} ServoJ IK guard"
                )
                arm_timing["servoj_ms"] = (time.perf_counter() - section_s) * 1e3
                arm_timing["total_ms"] = (time.perf_counter() - arm_start_s) * 1e3
                return

        section_s = time.perf_counter()
        response = robot.ServoP(
            float(target_pose_mm_deg[0]),
            float(target_pose_mm_deg[1]),
            float(target_pose_mm_deg[2]),
            float(target_pose_mm_deg[3]),
            float(target_pose_mm_deg[4]),
            float(target_pose_mm_deg[5]),
        )
        arm_timing["servop_ms"] = (time.perf_counter() - section_s) * 1e3

        section_s = time.perf_counter()
        response_error_id, _ = self._parse_dobot_response(response)
        arm_timing["parse_response_ms"] = (time.perf_counter() - section_s) * 1e3
        if response_error_id == 0:
            arm_timing["total_ms"] = (time.perf_counter() - arm_start_s) * 1e3
            return

        if not (
            self.config.enable_cartesian_ik_guard
            and self.config.cartesian_ik_servoj
            and response_error_id in (-2, -1)
        ):
            try:
                self._log_cartesian_command_failure(side, robot, response, f"{side} ServoP")
            except Exception as e:
                self.logger.warn(f"Failed to build {side} ServoP diagnostics: {e}")
            arm_timing["total_ms"] = (time.perf_counter() - arm_start_s) * 1e3
            self._raise_if_dobot_error(robot, response, f"{side} ServoP")
            return

        original_response = response
        self._cartesian_ik_servoj_until_s[side] = (
            time.perf_counter() + float(self.config.cartesian_ik_servoj_hold_s)
        )
        section_s = time.perf_counter()
        target_pose_mm_deg, target_joint_deg = self._resolve_cartesian_servo_target(
            robot, target_pose_mm_deg, side
        )
        arm_timing["fallback_ik_ms"] = (time.perf_counter() - section_s) * 1e3
        if target_joint_deg is None:
            arm_timing["total_ms"] = (time.perf_counter() - arm_start_s) * 1e3
            return

        try:
            section_s = time.perf_counter()
            self._send_cartesian_servoj(
                robot,
                target_joint_deg,
                side,
                f"{side} ServoJ after ServoP IK fallback",
            )
            arm_timing["fallback_servoj_ms"] = (time.perf_counter() - section_s) * 1e3
            arm_timing["total_ms"] = (time.perf_counter() - arm_start_s) * 1e3
        except Exception as e:
            try:
                self._log_cartesian_command_failure(
                    side, robot, original_response, f"{side} ServoJ fallback"
                )
            except Exception as diagnostics_error:
                self.logger.warn(
                    f"Failed to build {side} ServoJ fallback diagnostics: {diagnostics_error}"
                )
            self.logger.warn(f"{side} ServoJ fallback failed, skipping frame: {e}")
            arm_timing["total_ms"] = (time.perf_counter() - arm_start_s) * 1e3

    def _send_gripper_action(self, action: dict[str, Any]) -> None:
        if (
            self._left_gripper
            and self.config.use_left_gripper
            and self._left_gripper_connected
            and self._left_gripper_key in action
        ):
            try:
                self._left_gripper.set_gripper_position(
                    float(action[self._left_gripper_key])
                )
            except Exception as e:
                self._left_gripper_connected = False
                self.logger.error(
                    f"Left DH gripper command failed, disabling left gripper commands for this session: {e}"
                )
        if (
            self._right_gripper
            and self.config.use_right_gripper
            and self._right_gripper_connected
            and self._right_gripper_key in action
        ):
            try:
                self._right_gripper.set_gripper_position(
                    float(action[self._right_gripper_key])
                )
            except Exception as e:
                self._right_gripper_connected = False
                self.logger.error(
                    f"Right DH gripper command failed, disabling right gripper commands for this session: {e}"
                )

    def _action_worker_loop(self) -> None:
        min_period_s = 1.0 / max(float(self.config.async_action_worker_frequency), 1e-6)
        next_send_s = time.perf_counter()
        while True:
            with self._action_worker_cv:
                while (
                    self._action_worker_running
                    and (self._action_worker_paused or self._pending_async_action is None)
                ):
                    self._action_worker_cv.wait(timeout=0.1)
                if not self._action_worker_running:
                    break
                action = self._pending_async_action
                self._pending_async_action = None

            now_s = time.perf_counter()
            if now_s < next_send_s:
                time.sleep(next_send_s - now_s)
            send_start_s = time.perf_counter()
            try:
                with self._action_worker_cv:
                    self._action_worker_busy = True
                sent_action = self._send_action_sync(action)
                elapsed_ms = (time.perf_counter() - send_start_s) * 1e3
                with self._action_worker_lock:
                    self._latest_sent_async_action = sent_action
                    self._async_action_send_count += 1
                    self._last_async_action_timing = {
                        "worker_send_ms": elapsed_ms,
                        "queue_drop_count": float(self._async_action_drop_count),
                        "send_count": float(self._async_action_send_count),
                        "error_count": float(self._async_action_error_count),
                    }
            except Exception as e:
                with self._action_worker_lock:
                    self._async_action_error_count += 1
                    self._last_async_action_timing = {
                        "worker_error_count": float(self._async_action_error_count),
                        "queue_drop_count": float(self._async_action_drop_count),
                }
                self.logger.warn(f"Async action worker skipped failed action: {e}")
            finally:
                with self._action_worker_cv:
                    self._action_worker_busy = False
                    self._action_worker_cv.notify_all()
            next_send_s = max(next_send_s + min_period_s, time.perf_counter())

    def _start_action_worker(self) -> None:
        if not self.config.async_action_worker or self._action_worker_thread is not None:
            return
        self._action_worker_running = True
        self._action_worker_paused = False
        self._action_worker_thread = threading.Thread(
            target=self._action_worker_loop,
            name="BiDobotNova5DHActionWorker",
            daemon=True,
        )
        self._action_worker_thread.start()
        self.logger.info(
            f"Async action worker started at {self.config.async_action_worker_frequency:.1f} Hz; "
            "record loop will not block on ServoP."
        )

    def _stop_action_worker(self) -> None:
        with self._action_worker_cv:
            self._action_worker_running = False
            self._pending_async_action = None
            self._action_worker_cv.notify_all()
        if self._action_worker_thread is not None:
            self._action_worker_thread.join(timeout=2.0)
            self._action_worker_thread = None

    @contextlib.contextmanager
    def _pause_action_worker(self):
        if not self.config.async_action_worker:
            yield
            return
        with self._action_worker_cv:
            previous_paused = self._action_worker_paused
            self._action_worker_paused = True
            self._pending_async_action = None
            self._action_worker_cv.notify_all()
            while self._action_worker_busy:
                self._action_worker_cv.wait(timeout=0.05)
        try:
            yield
        finally:
            with self._action_worker_cv:
                self._action_worker_paused = previous_paused
                self._action_worker_cv.notify_all()

    def _submit_async_action(self, action: dict[str, Any]) -> dict[str, Any]:
        submit_start_s = time.perf_counter()
        action_copy = dict(action)
        with self._action_worker_cv:
            if self._pending_async_action is not None:
                self._async_action_drop_count += 1
            self._pending_async_action = action_copy
            last_sent = self._latest_sent_async_action
            worker_timing = dict(self._last_async_action_timing)
            self._action_worker_cv.notify()
        submit_ms = (time.perf_counter() - submit_start_s) * 1e3
        self._last_action_timing = {
            "async_submit_ms": submit_ms,
            "total_ms": submit_ms,
            **worker_timing,
        }
        return last_sent if last_sent is not None else action

    def _send_action_sync(self, action: dict[str, Any]) -> dict[str, Any]:
        if (
            not self.is_connected
            or self._left_robot is None
            or self._right_robot is None
        ):
            raise DeviceNotConnectedError(f"{self} is not connected.")
        action_start_s = time.perf_counter()
        action_timing: dict[str, float] = {}

        if int(self._left_feed_data.RobotMode) == 9:
            section_s = time.perf_counter()
            try:
                self._log_cartesian_command_failure(
                    "left", self._left_robot, None, "left RobotMode=9"
                )
            except Exception as e:
                self.logger.warn(f"Failed to build left fault diagnostics: {e}")
            action_timing["left_fault_diagnostics_ms"] = (time.perf_counter() - section_s) * 1e3
            self.logger.warn(
                f"Left robot fault detected, trying ClearError and skipping this frame. "
                f"GetErrorID: {self._dobot_error_detail(self._left_robot)}"
            )
            section_s = time.perf_counter()
            self.clear_fault()
            action_timing["clear_fault_ms"] = (time.perf_counter() - section_s) * 1e3
            action_timing["total_ms"] = (time.perf_counter() - action_start_s) * 1e3
            self._last_action_timing = action_timing
            return action
        if int(self._right_feed_data.RobotMode) == 9:
            section_s = time.perf_counter()
            try:
                self._log_cartesian_command_failure(
                    "right", self._right_robot, None, "right RobotMode=9"
                )
            except Exception as e:
                self.logger.warn(f"Failed to build right fault diagnostics: {e}")
            action_timing["right_fault_diagnostics_ms"] = (time.perf_counter() - section_s) * 1e3
            self.logger.warn(
                f"Right robot fault detected, trying ClearError and skipping this frame. "
                f"GetErrorID: {self._dobot_error_detail(self._right_robot)}"
            )
            section_s = time.perf_counter()
            self.clear_fault()
            action_timing["clear_fault_ms"] = (time.perf_counter() - section_s) * 1e3
            action_timing["total_ms"] = (time.perf_counter() - action_start_s) * 1e3
            self._last_action_timing = action_timing
            return action

        if self.config.control_mode == ControlMode.JOINT_MOTION:
            section_s = time.perf_counter()
            self._send_joint_action_one_arm(
                self._left_robot, action, self._left_action_joint_keys, "left"
            )
            action_timing["left_arm_ms"] = (time.perf_counter() - section_s) * 1e3

            section_s = time.perf_counter()
            self._send_joint_action_one_arm(
                self._right_robot, action, self._right_action_joint_keys, "right"
            )
            action_timing["right_arm_ms"] = (time.perf_counter() - section_s) * 1e3
        elif self.config.control_mode == ControlMode.CARTESIAN_MOTION:
            section_s = time.perf_counter()
            self._send_cart_action_one_arm(self._left_robot, action, "left", "left")
            action_timing["left_arm_ms"] = (time.perf_counter() - section_s) * 1e3

            section_s = time.perf_counter()
            self._send_cart_action_one_arm(self._right_robot, action, "right", "right")
            action_timing["right_arm_ms"] = (time.perf_counter() - section_s) * 1e3
        else:
            raise ValueError(f"Unsupported control_mode: {self.config.control_mode}")

        section_s = time.perf_counter()
        self._send_gripper_action(action)
        action_timing["grippers_ms"] = (time.perf_counter() - section_s) * 1e3
        action_timing["total_ms"] = (time.perf_counter() - action_start_s) * 1e3
        self._last_action_timing = action_timing
        return action

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if self.config.async_action_worker:
            return self._submit_async_action(action)
        return self._send_action_sync(action)

    def clear_fault(self) -> bool:
        if self._left_robot is None or self._right_robot is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        ok = True
        if int(self._left_feed_data.RobotMode) == 9:
            try:
                self._raise_if_dobot_error(
                    self._left_robot, self._left_robot.ClearError(), "left ClearError"
                )
            except Exception as e:
                self.logger.error(f"Failed to clear left fault: {e}")
                ok = False
        if int(self._right_feed_data.RobotMode) == 9:
            try:
                self._raise_if_dobot_error(
                    self._right_robot,
                    self._right_robot.ClearError(),
                    "right ClearError",
                )
            except Exception as e:
                self.logger.error(f"Failed to clear right fault: {e}")
                ok = False
        return ok

    def disconnect(self) -> None:
        if not self._is_connected:
            self.logger.warn(f"{self} is not connected, skipping disconnect.")
            return
        try:
            self.logger.info("Disconnecting BiDobot Nova5...")
            self._stop_action_worker()
            try:
                self._go_to_home()
            except Exception as e:
                self.logger.warn(f"Failed to move to home before disconnect: {e}")

            # Disconnect grippers first — they need the robot TCP to send
            # ModbusClose and the open-gripper position command.
            if (
                self._left_gripper
                and self.config.use_left_gripper
                and self._left_gripper_connected
            ):
                self._left_gripper.disconnect()
            if (
                self._right_gripper
                and self.config.use_right_gripper
                and self._right_gripper_connected
            ):
                self._right_gripper.disconnect()

            if self._left_robot is not None:
                self._left_robot.Stop()
                self._left_robot.close()
            if self._right_robot is not None:
                self._right_robot.Stop()
                self._right_robot.close()
            if self._left_feed is not None:
                self._left_feed.close()
            if self._right_feed is not None:
                self._right_feed.close()

            for cam in self.cameras.values():
                cam.disconnect()
        except Exception as e:
            self.logger.error(f"Error during disconnect: {e}")
        finally:
            self._left_robot = None
            self._right_robot = None
            self._left_feed = None
            self._right_feed = None
            self._left_gripper = None
            self._right_gripper = None
            self._left_gripper_connected = False
            self._right_gripper_connected = False
            self._is_connected = False
            self.logger.info("✅ BiDobot Nova5 disconnected.")

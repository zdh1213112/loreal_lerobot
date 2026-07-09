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

import re
import threading
import time
from functools import cached_property
from typing import Any

import numpy as np

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.robots.bi_dobot_nova5.config_bi_dobot_nova5 import (
    BiDobotNova5Config,
    ControlMode,
)
from lerobot.robots.bi_dobot_nova5.TCP_IP_Python_V4.dobot_api import (
    DobotApiDashboard,
    DobotApiFeedBack,
)
from lerobot.robots.dh_gripper import DHGripper
from lerobot.robots.robot import Robot
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.utils.robot_utils import (
    euler_to_quaternion,
    get_logger,
    quaternion_to_euler,
    quaternion_to_rotation_6d,
    rotation_6d_to_quaternion,
)

JOINT_DOF = 6
MM_PER_METER = 1000.0


class _FeedState:
    def __init__(self) -> None:
        self.RobotMode = -1
        self.robotCurrentCommandID = -1
        self.MessageSize = -1
        self.DigitalInputs = -1
        self.DigitalOutputs = -1
        self.tcpPose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # xyz(mm), rpy(deg)
        self.qActual = [0.0] * JOINT_DOF


class BiDobotNova5(Robot):
    config_class = BiDobotNova5Config
    name = "bi_dobot_nova5"

    def __init__(self, config: BiDobotNova5Config):
        super().__init__(config)
        self.config = config
        self.logger = get_logger("BiDobotNova5")

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

        self._left_gripper: DHGripper | None = None
        if config.use_left_gripper:
            self._left_gripper = DHGripper(config.left_dh_gripper)

        self._right_gripper: DHGripper | None = None
        if config.use_right_gripper:
            self._right_gripper = DHGripper(config.right_dh_gripper)
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
            while True:
                left_ready = int(self._left_feed_data.RobotMode) == 5
                right_ready = int(self._right_feed_data.RobotMode) == 5
                if left_ready and right_ready:
                    break
                if time.time() - start_time > timeout:
                    raise RuntimeError(
                        "Both robots did not become operational within 30 seconds"
                    )
                time.sleep(0.1)

            if self._left_gripper and self.config.use_left_gripper:
                self.logger.info("Connecting left DH Gripper...")
                try:
                    self._left_gripper.connect()
                    self._left_gripper_connected = True
                except Exception as e:
                    self._left_gripper_connected = False
                    self.logger.error(
                        f"Failed to connect left DH Gripper, continuing without left gripper control: {e}"
                    )
            if self._right_gripper and self.config.use_right_gripper:
                self.logger.info("Connecting right DH Gripper...")
                try:
                    self._right_gripper.connect()
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
        self._move_joint_movj(
            self._left_robot,
            self._left_feed_data,
            self.config.left_start_position_degree,
            "start position",
            "left",
        )
        self._move_joint_movj(
            self._right_robot,
            self._right_feed_data,
            self.config.right_start_position_degree,
            "start position",
            "right",
        )
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
        self._move_joint_movj(
            self._left_robot,
            self._left_feed_data,
            self.config.left_home_point_list,
            "home position",
            "left",
        )
        self._move_joint_movj(
            self._right_robot,
            self._right_feed_data,
            self.config.right_home_point_list,
            "home position",
            "right",
        )
        self._initialize_gripper_position()
        self.logger.info("✅ Both arms at home position.")

    def reset_to_initial_position(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        if self.config.go_to_start:
            self._go_to_start()
        else:
            self._go_to_home()

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
        obs_dict: dict[str, Any] = {}

        if self.config.control_mode == ControlMode.JOINT_MOTION:
            for i, key in enumerate(self._left_joint_pos_keys):
                obs_dict[key] = self._left_feed_data.qActual[i]
            for i, key in enumerate(self._right_joint_pos_keys):
                obs_dict[key] = self._right_feed_data.qActual[i]
        elif self.config.control_mode == ControlMode.CARTESIAN_MOTION:
            left_pose = self._left_feed_data.tcpPose
            right_pose = self._right_feed_data.tcpPose

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
        else:
            raise ValueError(f"Unsupported control_mode: {self.config.control_mode}")

        obs_dict[self._left_gripper_key] = self._read_gripper_pos("left")
        obs_dict[self._right_gripper_key] = self._read_gripper_pos("right")

        for cam_key, cam in self.cameras.items():
            obs_dict[cam_key] = cam.async_read()

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
            1.0 / self._control_frequency,
            self._aheadtime,
            self._gain,
        )
        self._raise_if_dobot_error(robot, response, f"{side} ServoJ")

    def _send_cart_action_one_arm(
        self,
        robot: DobotApiDashboard,
        action: dict[str, Any],
        prefix: str,
        side: str,
    ) -> None:
        x_m = float(action[f"{prefix}_tcp.x"])
        y_m = float(action[f"{prefix}_tcp.y"])
        z_m = float(action[f"{prefix}_tcp.z"])
        x_mm = x_m * MM_PER_METER
        y_mm = y_m * MM_PER_METER
        z_mm = z_m * MM_PER_METER
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
        response = robot.ServoP(
            x_mm,
            y_mm,
            z_mm,
            float(np.rad2deg(euler[0])),
            float(np.rad2deg(euler[1])),
            float(np.rad2deg(euler[2])),
        )
        self._raise_if_dobot_error(robot, response, f"{side} ServoP")

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

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if (
            not self.is_connected
            or self._left_robot is None
            or self._right_robot is None
        ):
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if int(self._left_feed_data.RobotMode) == 9:
            raise RuntimeError(
                f"Left robot fault detected. GetErrorID: {self._dobot_error_detail(self._left_robot)}"
            )
        if int(self._right_feed_data.RobotMode) == 9:
            raise RuntimeError(
                f"Right robot fault detected. GetErrorID: {self._dobot_error_detail(self._right_robot)}"
            )

        if self.config.control_mode == ControlMode.JOINT_MOTION:
            self._send_joint_action_one_arm(
                self._left_robot, action, self._left_action_joint_keys, "left"
            )
            self._send_joint_action_one_arm(
                self._right_robot, action, self._right_action_joint_keys, "right"
            )
        elif self.config.control_mode == ControlMode.CARTESIAN_MOTION:
            self._send_cart_action_one_arm(self._left_robot, action, "left", "left")
            self._send_cart_action_one_arm(self._right_robot, action, "right", "right")
        else:
            raise ValueError(f"Unsupported control_mode: {self.config.control_mode}")

        self._send_gripper_action(action)
        return action

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
            try:
                self._go_to_home()
            except Exception as e:
                self.logger.warn(f"Failed to move to home before disconnect: {e}")

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

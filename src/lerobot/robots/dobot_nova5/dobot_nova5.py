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

"""Dobot Nova5 robot implementation for LeRobot.

This module provides integration with Dobot Nova5 6-DOF collaborative robot,
supporting two control modes (NRT = Non-Real-Time for Python API):

1. JOINT_MOTION:
   - Action: joint positions (7D) + gripper (1D) = 8D
   - Observation: joint positions (7D) + gripper (1D) = 8D

2. CARTESIAN_MOTION:
   - Uses 6D rotation representation (r1-r6) for continuity and better learning
     - Action: TCP pose (9D: x,y,z + r1-r6) + gripper (1D) = 10D
     - Observation: TCP pose (9D) + gripper (1D) = 10D

   6D Rotation Representation:
   - r1, r2, r3: First column of rotation matrix
   - r4, r5, r6: Second column of rotation matrix
   - Reference: "On the Continuity of Rotation Representations in Neural Networks"

Note: Python API can only use NRT modes due to language timing limitations.

Reference: https://github.com/Dobot-Arm/TCP-IP-Python-V4
"""

import time
from functools import cached_property
from typing import Any, Optional
import re
import threading

from lerobot.robots.dobot_nova5.TCP_IP_Python_V4.dobot_api import DobotApiFeedBack,DobotApiDashboard
import numpy as np

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.robots.dobot_nova5.config_dobot_nova5 import ControlMode, DobotNova5Config
from lerobot.robots.dh_gripper import DHGripper
from lerobot.robots.robot import Robot
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.utils.robot_utils import (
    get_logger,
    quaternion_to_euler,
    quaternion_to_rotation_6d,
    euler_to_quaternion,
    rotation_6d_to_quaternion,
)

# Alias for dobot_api.Mode for convenience
# Mode = DobotApiDashboard.RequestControl()

# Constants from dobot_api
JOINT_DOF = 6  # Dobot Nova5 robot joint DOF
MM_PER_METER = 1000.0

class DobotNova5(Robot):
    """Dobot Nova5 6-DOF collaborative robot.

    This class implements the LeRobot Robot interface for the Dobot Nova5 robot,
    supporting two control modes for joint space and Cartesian space control.

    Control Modes (NRT only for Python API):
        - JOINT_MOTION: Joint motion control (maps to NRT_JOINT_MOTION)
        - CARTESIAN_MOTION: Cartesian motion control (maps to NRT_CARTESIAN_MOTION)

    Example:
        >>> from lerobot.robots.dobot_nova5.TCP_IP_Python_V4.dobot_api import DobotApiFeedBack,DobotApiDashboard
        >>> from lerobot.robots.dobot_nova5.config_dobot_nova5 import DobotNova5Config, ControlMode
        >>> # Joint motion control
        >>> config = DobotNova5Config(
        ...     robot_ip="192.168.1.1",
        ...     control_mode=ControlMode.JOINT_MOTION,
        ... )
        >>> # Cartesian motion control
        >>> config = DobotNova5Config(
        ...     robot_ip="192.168.1.1",
        ...     control_mode=ControlMode.CARTESIAN_MOTION,
        ... )
        >>> robot = DobotNova5(config)
        >>> robot.connect()
        >>> obs = robot.get_observation()
        >>> robot.send_action({"joint_1.pos": 0.0, ...})
        >>> robot.disconnect()
    """

    config_class = DobotNova5Config
    name = "dobot_nova5"

    def __init__(self, config: DobotNova5Config):
        super().__init__(config)
        self.config = config

        # Logger
        self.logger = get_logger("DobotNova5")

        # Robot interface (initialized on connect)
        # Note: Python API only supports NRT (Non-Real-Time) modes, no Scheduler needed
        self._robot: DobotApiDashboard | None = None
        self._feedFour: DobotApiFeedBack | None = None
        self._feedInfo = []
        self.__globalLockValue = threading.Lock()

        class item:
            def __init__(self):
                self.RobotMode = -1     #
                self.robotCurrentCommandID = 0
                self.MessageSize = -1
                self.DigitalInputs =-1
                self.DigitalOutputs = -1
                self.robotCurrentCommandID = -1
                self.tcpPose = [ 258.1406287 ,  -78.59815584, 734.56544667, -171.70077286,    4.8702051,  -84.81002799]
                self.qActual = [ 11.71805878,   23.65214996,  -77.142659,    -27.70232162, -266.17571411,    7.17711411]
                # 自定义添加所需反馈数据

        self.feedData = item()  # 定义结构对象

        self._is_connected = False

        self._gripper: DHGripper | None = None
        if config.use_gripper:
            self._gripper = DHGripper(config.dh_gripper)

        # Control state - stores the current dobot_api.Mode
        self._current_mode = None

        # Home TCP pose - stored after moving to home position
        # Format: [x, y, z, qw, qx, qy, qz] (7D) - SDK format with quaternion
        self._home_tcp_pose: np.ndarray | None = None

        # Gripper key (1D) - always used
        self._gripper_key = "gripper.pos"

        # Initialize keys and buffers based on control mode
        if config.control_mode == ControlMode.JOINT_MOTION:
            self._init_joint_mode()
        elif config.control_mode == ControlMode.CARTESIAN_MOTION:
            self._init_cartesian_mode()
        else:
            raise ValueError(f"Unsupported control_mode: {config.control_mode}")

        self.cameras = make_cameras_from_configs(config.cameras)
        np.set_printoptions(precision=6, suppress=True)

    def _init_joint_mode(self) -> None:
        """Initialize keys and buffers for joint motion control mode."""
        # Joint state observation keys: joint_{1-6}.{pos, vel, effort}
        self._joint_pos_keys = tuple(f"joint_{i}.pos" for i in range(1, JOINT_DOF + 1))

        # Joint action keys: joint_{1-6}.pos
        self._action_joint_keys = tuple(f"joint_{i}.pos" for i in range(1, JOINT_DOF + 1))

        # Pre-cache config values as lists (for API calls)
        self._control_frequency = self.config.control_frequency
        self._aheadtime = self.config.aheadtime
        self._gain = self.config.gain

    def _init_cartesian_mode(self) -> None:
        """Initialize keys and buffers for CARTESIAN_MOTION control mode.

        Uses 6D rotation representation (r1-r6) instead of quaternion for:
        - Continuity: No discontinuities like Euler angles (gimbal lock)
        - No double-cover: Unlike quaternions where q and -q represent same rotation
        - Better for neural networks: Continuous representation is easier to learn

        Reference: "On the Continuity of Rotation Representations in Neural Networks"
        """
        # TCP pose observation/action keys: tcp.{x, y, z, r1, r2, r3, r4, r5, r6}
        # 6D rotation: r1-r3 = first column, r4-r6 = second column of rotation matrix
        self._tcp_pose_keys = (
            "tcp.x",
            "tcp.y",
            "tcp.z",
            "tcp.r1",
            "tcp.r2",
            "tcp.r3",
            "tcp.r4",
            "tcp.r5",
            "tcp.r6",
        )

        # TCP pose action keys (same as observation keys for 6D rotation)
        self._action_tcp_pose_keys = self._tcp_pose_keys

        # Pre-cache config values as lists (for API calls)
        self._control_frequency = self.config.control_frequency
        self._aheadtime = self.config.aheadtime
        self._gain = self.config.gain

    @property
    def _action_ft(self) -> dict[str, type]:
        """Return action features based on control_mode.

        Action space (all include gripper):
        - JOINT_MOTION: joint positions (7D) + gripper (1D) = 8D
        - CARTESIAN_MOTION: TCP pose (9D: xyz + 6D rotation) + gripper (1D) = 10D
        """
        features = {}

        if self.config.control_mode == ControlMode.JOINT_MOTION:
            # Joint positions (7D)
            features.update(dict.fromkeys(self._action_joint_keys, float))

        elif self.config.control_mode == ControlMode.CARTESIAN_MOTION:
            # TCP pose (9D: xyz + 6D rotation)
            features.update(dict.fromkeys(self._action_tcp_pose_keys, float))

        else:
            raise ValueError(f"Unsupported control_mode: {self.config.control_mode}")

        # Always include gripper (1D)
        features[self._gripper_key] = float
        return features

    @property
    def _proprioception_ft(self) -> dict[str, type]:
        """Return observation features based on control_mode.

        Observation space (all include gripper):
        - JOINT_MOTION: joint pos (7D) + gripper pos (1D) = 8D
        - CARTESIAN_MOTION: TCP pose (9D: xyz + 6D rotation) + gripper (1D) = 10D
        """
        features = {}

        if self.config.control_mode == ControlMode.JOINT_MOTION:
            # Joint positions (7D)
            features.update(dict.fromkeys(self._joint_pos_keys, float))

        elif self.config.control_mode == ControlMode.CARTESIAN_MOTION:
            # TCP pose (9D: xyz + 6D rotation)
            features.update(dict.fromkeys(self._tcp_pose_keys, float))

        else:
            raise ValueError(f"Unsupported control_mode: {self.config.control_mode}")

        # Always include gripper position (1D)
        features[self._gripper_key] = float
        return features

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        """Return camera/image features from external cameras."""
        features = {}

        for cam in self.cameras:
            features[cam] = (self.config.cameras[cam].height, self.config.cameras[cam].width, 3)

        return features

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        """Return observation features (robot states + cameras)."""
        return {**self._proprioception_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        """Return action features based on control mode."""
        return self._action_ft

    @property
    def is_connected(self) -> bool:
        return (
            self._is_connected
            and self._robot is not None
            and all(cam.is_connected for cam in self.cameras.values())
        )

    @property
    def is_calibrated(self) -> bool:
        """Dobot Nova5 robots are factory calibrated."""
        return self.is_connected

    def calibrate(self) -> None:
        """Dobot Nova5 robots are factory calibrated, no runtime calibration needed."""
        self.logger.info("Dobot Nova5 is factory calibrated, no runtime calibration needed.")


    def configure(self) -> None:
        """Configure the robot based on control mode."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.logger.info(f"Configuring robot for {self.config.control_mode.value} mode...")

    def _parse_dobot_response(self, value_recv: str) -> tuple[int, list[str]]:
        """Parse Dobot TCP response: ErrorID,{ResultID or values},Command(...);"""
        if not isinstance(value_recv, str):
            raise RuntimeError(f"Invalid Dobot response type: {type(value_recv).__name__}")

        if "Not Tcp" in value_recv:
            raise RuntimeError("Robot is not in TCP control mode")

        match = re.match(r"\s*(-?\d+)\s*,\s*\{([^}]*)\}", value_recv)
        if match is None:
            raise RuntimeError(f"Could not parse Dobot response: {value_recv!r}")

        error_id = int(match.group(1))
        values = [value.strip() for value in match.group(2).split(",") if value.strip()]
        return error_id, values

    def parseResultId(self, valueRecv):
        error_id, values = self._parse_dobot_response(valueRecv)
        result = [error_id]
        for value in values:
            result.append(int(float(value)))
        return result

    def _dobot_error_detail(self) -> str:
        if self._robot is None:
            return ""
        try:
            return self._robot.GetErrorID().strip()
        except Exception as e:
            return f"failed to read GetErrorID: {e}"

    def _raise_if_dobot_error(self, response: str, command_name: str) -> list[str]:
        error_id, values = self._parse_dobot_response(response)
        if error_id == 0:
            return values

        message = f"{command_name} failed with ErrorID {error_id}: {response.strip()}"
        error_detail = self._dobot_error_detail()
        if error_detail:
            message = f"{message}; GetErrorID: {error_detail}"
        raise RuntimeError(message)

    def _require_command_id(self, response: str, command_name: str) -> int:
        values = self._raise_if_dobot_error(response, command_name)
        if not values:
            raise RuntimeError(f"{command_name} response did not include a command ID: {response.strip()}")
        return int(float(values[0]))

    def _wait_for_command(self, command_id: int, description: str, timeout_s: float = 60.0) -> None:
        start_time = time.time()
        last_log_time = 0.0
        command_id = int(command_id)

        while True:
            robot_mode = int(self.feedData.RobotMode)
            current_command_id = int(self.feedData.robotCurrentCommandID)

            if robot_mode == 9:
                raise RuntimeError(
                    f"{description} failed: robot entered error mode while waiting for command "
                    f"{command_id}. GetErrorID: {self._dobot_error_detail()}"
                )

            if robot_mode == 5 and current_command_id == command_id:
                return

            now = time.time()
            if now - start_time > timeout_s:
                raise TimeoutError(
                    f"Timed out waiting for {description}: target command ID={command_id}, "
                    f"current command ID={current_command_id}, RobotMode={robot_mode}"
                )

            if now - last_log_time >= 2.0:
                self.logger.info(
                    f"Waiting for {description}: RobotMode={robot_mode}, "
                    f"CurrentCommandId={current_command_id}, target={command_id}"
                )
                last_log_time = now

            time.sleep(0.1)

    def _wait_for_joint_target(
        self,
        target_joint_deg: list[float],
        description: str,
        tolerance_deg: float = 0.5,
        timeout_s: float = 60.0,
    ) -> None:
        target = np.asarray(target_joint_deg, dtype=np.float64)
        start_time = time.time()
        last_log_time = 0.0

        while True:
            robot_mode = int(self.feedData.RobotMode)
            current_joint = np.asarray(self.feedData.qActual, dtype=np.float64)
            max_abs_error = float(np.max(np.abs(current_joint - target)))

            if robot_mode == 9:
                raise RuntimeError(
                    f"{description} failed: robot entered error mode. "
                    f"GetErrorID: {self._dobot_error_detail()}"
                )

            if robot_mode == 5 and max_abs_error <= tolerance_deg:
                return

            now = time.time()
            if now - start_time > timeout_s:
                raise TimeoutError(
                    f"Timed out waiting for {description}: max_joint_error={max_abs_error:.3f} deg, "
                    f"RobotMode={robot_mode}, target={target.tolist()}, current={current_joint.tolist()}"
                )

            if now - last_log_time >= 2.0:
                self.logger.info(
                    f"Waiting for {description}: RobotMode={robot_mode}, "
                    f"max_joint_error={max_abs_error:.3f} deg"
                )
                last_log_time = now

            time.sleep(0.1)

    def _wait_for_first_feedback(self, timeout_s: float = 3.0) -> None:
        start_time = time.time()
        while self.feedData.MessageSize == -1 and self.feedData.RobotMode == -1:
            if time.time() - start_time > timeout_s:
                return
            time.sleep(0.02)

    def _wait_until_not_error_mode(self, timeout_s: float = 10.0) -> None:
        start_time = time.time()
        while int(self.feedData.RobotMode) == 9:
            if time.time() - start_time > timeout_s:
                raise TimeoutError(
                    f"Robot stays in error mode (RobotMode=9) after ClearError. "
                    f"GetErrorID: {self._dobot_error_detail()}"
                )
            time.sleep(0.1)

    def _send_movj_joint_v4(self, joint_degrees: list[float]) -> str:
        if self._robot is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        # V4 standard API: MovJ with coordinateMode=1 (joint target).
        # Use per-command velocity scale from config.start_vel_scale.
        target = [float(value) for value in joint_degrees]
        return self._robot.MovJ(
            target[0],
            target[1],
            target[2],
            target[3],
            target[4],
            target[5],
            1,  # coordinateMode=1 -> joint
            v=int(self.config.start_vel_scale),
        )

    def _move_joint_movj(self, joint_degrees: list[float], description: str) -> None:
        if self._robot is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        target = [float(value) for value in joint_degrees]
        self._raise_if_dobot_error(
            self._robot.SpeedFactor(int(self.config.start_vel_scale)),
            "SpeedFactor",
        )
        response = self._send_movj_joint_v4(target)
        self.logger.info(f"{description} MovJ(joint) response: {response.strip()}")
        values = self._raise_if_dobot_error(response, "MovJ(joint)")
        command_name = "MovJ(joint)"

        if values:
            try:
                command_id = int(float(values[0]))
                self._wait_for_command(command_id, description)
                return
            except (ValueError, TypeError):
                self.logger.warn(
                    f"{command_name} response did not provide a valid command ID ({values}), "
                    "falling back to joint-error completion check."
                )

        self._wait_for_joint_target(target, description)

    def _initialize_gripper_position(self) -> None:
        if self._gripper is None:
            return
        target = 1.0 if self.config.dh_gripper_init_open else 0.0
        self._gripper.initialize_gripper_position(target)

    def _current_gripper_position(self) -> float:
        if self._gripper and self.config.use_gripper:
            return float(self._gripper.get_gripper_position())
        return 0.0

    def _current_tcp_pose_quat_from_feedback(self) -> np.ndarray:
        tcp_pose = np.asarray(self.feedData.tcpPose, dtype=np.float64)
        # Dobot feedback uses mm for xyz. Convert to meters for teleop stack.
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

    def connect(self, calibrate: bool = False, go_to_start: bool = True) -> None:
        """Connect to the Dobot Nova5 robot.

        Args:
            calibrate: Ignored (Dobot Nova5 robots are factory calibrated)
            go_to_start: If provided, overrides config.go_to_start. If None, uses config.go_to_start.
        """
        if self.is_connected:
            raise DeviceAlreadyConnectedError(
                f"{self} already connected, do not run `robot.connect()` twice."
            )

        try:
            self.logger.info(f"Connecting to Dobot Nova5 robot: {self.config.robot_ip}")

            # Create robot interface
            self._robot = DobotApiDashboard(self.config.robot_ip, self.config.dashboardPort)
            self._feedFour = DobotApiFeedBack(self.config.robot_ip, self.config.feedPortFour)

            # Start feedback thread to continuously read robot state (for get_observation)
            # TODO: Needs testing
            feed_thread = threading.Thread(
                target=self.get_robot_state)  # robot state feedback thread
            feed_thread.daemon = True
            feed_thread.start()
            self._wait_for_first_feedback()

            # Clear fault before enabling if needed.
            if int(self.feedData.RobotMode) == 9:
                self.logger.warn("Robot is in error mode before enabling, trying ClearError ...")
                self._raise_if_dobot_error(self._robot.ClearError(), "ClearError")
                self._wait_until_not_error_mode()
                self.logger.info("Fault on the connected robot is cleared")

            # Enable the robot
            self.logger.info("Enabling robot...")
            if self.config.control_mode in (ControlMode.CARTESIAN_MOTION, ControlMode.JOINT_MOTION):
                enable_response = self._robot.EnableRobot()
                enable_error, _ = self._parse_dobot_response(enable_response)
                if enable_error != 0:
                    mode_response = self._robot.RobotMode()
                    mode_error, mode_values = self._parse_dobot_response(mode_response)
                    current_mode = int(float(mode_values[0])) if mode_error == 0 and mode_values else -1
                    if current_mode in (5, 6, 7, 8):
                        self.logger.warn(
                            f"EnableRobot returned {enable_error}, but RobotMode={current_mode}. "
                            "Proceeding with existing enabled/control state."
                        )
                    else:
                        raise RuntimeError(
                            f"EnableRobot failed with ErrorID {enable_error}: {enable_response.strip()} "
                            f"(RobotMode={current_mode}); GetErrorID: {self._dobot_error_detail()}"
                        )
                else:
                    if self.config.control_mode == ControlMode.CARTESIAN_MOTION:
                        self.logger.info("Robot TCP enabled successfully.")
                    else:
                        self.logger.info("Robot Joint enabled successfully.")
            else:
                raise ValueError(f"Unsupported control_mode: {self.config.control_mode}")

            if self.feedData.RobotMode == 5:
                self.logger.info("Robot is enabled and ready.")
            else:
                self.logger.warn(
                    f"Robot is in unexpected mode {self.feedData.RobotMode} after enabling. Check robot status."
                )

            # Wait for robot to become operational
            timeout = 30  # seconds
            start_time = time.time()
            while not self.feedData.RobotMode == 5:  # ROBOT_MODE_ENABLE  Enabled and idle
                if time.time() - start_time > timeout:
                    raise RuntimeError(f"Robot did not become operational within {timeout} seconds")
                time.sleep(0.1)

            self.logger.info("Robot is now operational.")

            # Connect DH Gripper end-effector
            if self._gripper and self.config.use_gripper:
                self.logger.info("Connecting DH Gripper...")
                self._gripper.connect()

            # Connect external cameras (e.g., scene cameras)
            for cam in self.cameras.values():
                cam.connect()

            # Set _is_connected to True before calling methods that check is_connected
            self._is_connected = True

            # Move to start position if requested (use parameter if provided, otherwise use config)
            self.config.go_to_start = go_to_start if go_to_start is not None else self.config.go_to_start
            if self.config.go_to_start:
                self._go_to_start()

            # Configure control parameters
            self.configure()

            mode_desc = self.config.control_mode.value
            gripper_status = "with DH Gripper" if (self._gripper and self.config.use_gripper) else "no gripper"
            self.logger.info(f"✅ Dobot Nova5 connected and ready in {mode_desc} mode ({gripper_status}).")

        except Exception as e:
            self.logger.error(f"Failed to connect to Dobot Nova5 robot: {e}")
            self._robot = None
            self._is_connected = False
            raise

    def _go_to_home(self) -> None:
        """Move robot to home position using MoveJ primitive.

        Uses ExecutePrimitive("MoveJ") to move to factory-defined home pose:
        - target: [0.0, -40.0, 0.0, 90.0, 0.0, 40.0, 0.0] degrees
        - jntVelScale: 20
        """
        if not self._is_connected or self._robot is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.logger.info("Moving to home position...")

        self._move_joint_movj(self.config.home_point_list, "home position")
        self._initialize_gripper_position()
        self._home_tcp_pose = self._current_tcp_pose_quat_from_feedback()
        self.logger.info(f"Home TCP pose: {self._home_tcp_pose}")
        self.logger.info("✅ Robot at home position.")
        if self._gripper is not None:
            self.logger.info(f"Gripper position: {self._gripper.get_gripper_position()}")

    def _go_to_start(self) -> None:
        """Move robot to start position using MoveJ primitive.

        Uses ExecutePrimitive("MoveJ") with configurable parameters:
        - jntVelScale: Joint velocity scale 1-100 (from config.start_vel_scale)
        - target: Start joint position in degrees (from config.start_position_degree)
        """
        if not self._is_connected or self._robot is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.logger.info("Moving to start position...")

        self._move_joint_movj(self.config.start_position_degree, "start position")
        self._initialize_gripper_position()
        self.logger.info("✅ Robot at start position.")
        if self._gripper is not None:
            self.logger.info(f"Gripper position: {self._gripper.get_gripper_position()}")

    def reset_to_initial_position(self) -> None:
        """Reset robot to initial position based on config.go_to_start.

        If config.go_to_start=True, calls _go_to_start().
        Otherwise, calls _go_to_home().
        """
        if not self._is_connected or self._robot is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if self.config.go_to_start:
            self.logger.info("Resetting to start position (config.go_to_start=True)")
            self._go_to_start()
        else:
            self.logger.info("Resetting to home position (config.go_to_start=False)")
            self._go_to_home()

    def get_robot_state(self) -> Optional[dict]:
        # 获取机器人状态
        while True:
            if self._feedFour is None:
                return None
            try:
                feedInfo = self._feedFour.feedBackData()
            except Exception:
                # Socket may be closing during disconnect; exit feedback thread quietly.
                return None
            with self.__globalLockValue:
                if feedInfo is not None:   
                    if hex((feedInfo['TestValue'][0])) == '0x123456789abcdef':
                        # 基础字段
                        self.feedData.MessageSize = feedInfo['len'][0]
                        self.feedData.RobotMode = feedInfo['RobotMode'][0]
                        self.feedData.DigitalInputs = feedInfo['DigitalInputs'][0]
                        self.feedData.DigitalOutputs = feedInfo['DigitalOutputs'][0]
                        self.feedData.robotCurrentCommandID = feedInfo['CurrentCommandId'][0]
                        self.feedData.tcpPose = feedInfo['ToolVectorActual'][0]
                        self.feedData.qActual = feedInfo['QActual'][0]
                        # 自定义添加所需反馈数据
                        '''
                        self.feedData.DigitalOutputs = int(feedInfo['DigitalOutputs'][0])
                        self.feedData.RobotMode = int(feedInfo['RobotMode'][0])
                        self.feedData.TimeStamp = int(feedInfo['TimeStamp'][0])
                        '''


    def get_observation(self) -> dict[str, Any]:
        """Get current robot observation based on control_mode.

        Returns a dictionary with observation data. The content depends on control_mode:
        - JOINT_MOTION: joint_1-7.{pos,vel,effort} (21D) + gripper.pos (1D) = 22D
        - CARTESIAN_MOTION: tcp.{x,y,z,r1-r6} (9D) + gripper (1D) = 10D

        Also includes camera images if configured.
        """
        if not self.is_connected or self._robot is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        obs_dict = {}

        if self.config.control_mode == ControlMode.JOINT_MOTION:
            # Joint positions (7D)
            for i, key in enumerate(self._joint_pos_keys):
                # TODO
                obs_dict[key] = self.feedData.qActual[i]

        elif self.config.control_mode == ControlMode.CARTESIAN_MOTION:
            # TCP pose from SDK: [x, y, z, qw, qx, qy, qz]
            # TODO
            tcp_pose = self.feedData.tcpPose

            # Position (3D)
            obs_dict["tcp.x"] = tcp_pose[0] / MM_PER_METER
            obs_dict["tcp.y"] = tcp_pose[1] / MM_PER_METER
            obs_dict["tcp.z"] = tcp_pose[2] / MM_PER_METER

            # Convert quaternion to 6D rotation representation
            quat = euler_to_quaternion(np.deg2rad(tcp_pose[3]), np.deg2rad(tcp_pose[4]), np.deg2rad(tcp_pose[5]))
            r6d = quaternion_to_rotation_6d(quat[0], quat[1], quat[2], quat[3])

            obs_dict["tcp.r1"] = r6d[0]
            obs_dict["tcp.r2"] = r6d[1]
            obs_dict["tcp.r3"] = r6d[2]
            obs_dict["tcp.r4"] = r6d[3]
            obs_dict["tcp.r5"] = r6d[4]
            obs_dict["tcp.r6"] = r6d[5]


        else:
            raise ValueError(f"Unsupported control_mode: {self.config.control_mode}")

        # Read gripper position
        if self._gripper is not None and self.config.use_gripper:
            obs_dict[self._gripper_key] = self._gripper.get_gripper_position()

        # External camera observations (scene cameras, etc.)
        for cam_key, cam in self.cameras.items():
            obs_dict[cam_key] = cam.async_read()

        return obs_dict

    def forward_kinematics(self, joint_positions_deg: list | np.ndarray) -> np.ndarray:
        """Compute forward kinematics: joint positions -> TCP pose.
        
        This is a pure mathematical computation that only requires the Robot object
        to be created (for accessing kinematic model), not necessarily connected/enabled.
        
        Args:
            joint_positions_deg: Joint positions in degrees (7D)
            
        Returns:
            TCP pose [x, y, z, qw, qx, qy, qz] (7D)
        """
        if self._robot is None:
            raise RuntimeError("Robot object not created. Call connect() first or create Robot manually.")
        
        # V4 API PositiveKin returns TCP pose in controller format:
        # [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg].
        response = self._robot.PositiveKin(
            float(joint_positions_deg[0]),
            float(joint_positions_deg[1]),
            float(joint_positions_deg[2]),
            float(joint_positions_deg[3]),
            float(joint_positions_deg[4]),
            float(joint_positions_deg[5]),
        )
        values = self._raise_if_dobot_error(response, "PositiveKin")
        if len(values) < 6:
            raise RuntimeError(
                f"PositiveKin response missing pose values: {response.strip()}"
            )

        x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg = [float(v) for v in values[:6]]
        quat = euler_to_quaternion(
            np.deg2rad(rx_deg),
            np.deg2rad(ry_deg),
            np.deg2rad(rz_deg),
        )
        return np.array(
            [
                x_mm / MM_PER_METER,
                y_mm / MM_PER_METER,
                z_mm / MM_PER_METER,
                quat[0],
                quat[1],
                quat[2],
                quat[3],
            ],
            dtype=np.float32,
        )

    def get_start_tcp_pose(self) -> np.ndarray:
        """Get TCP pose at start position (from config.start_position_degree).
        
        This can be called after Robot object is created, even before robot is enabled.
        
        Returns:
            TCP pose [x, y, z, qw, qx, qy, qz] (7D)
        """
        return self.forward_kinematics(self.config.start_position_degree)

    def get_start_tcp_pose_euler(self) -> np.ndarray:
        """Get TCP pose at start position in Euler format [x, y, z, roll, pitch, yaw, gripper_pos].
        
        This can be called after Robot object is created, even before robot is enabled.
        
        Returns:
            numpy array of shape (7,) with [x, y, z, roll, pitch, yaw, gripper_pos]
        """
        tcp_pose_quat = self.get_start_tcp_pose()
        
        # Convert quaternion to Euler angles
        euler = quaternion_to_euler(
            tcp_pose_quat[3], tcp_pose_quat[4], tcp_pose_quat[5], tcp_pose_quat[6]
        )
        
        # Get initial gripper position based on config (1.0 = open, 0.0 = closed)
        gripper_pos = 1.0 if self.config.dh_gripper_init_open else 0.0
        
        return np.array(
            [tcp_pose_quat[0], tcp_pose_quat[1], tcp_pose_quat[2], 
             euler[0], euler[1], euler[2], gripper_pos],
            dtype=np.float32,
        )

    def get_current_tcp_pose_euler(self) -> np.ndarray:
        """Get current TCP pose in Euler angles format [x, y, z, roll, pitch, yaw, gripper_pos].

        This method can be used for getting the current TCP pose in Euler angles format for initializing teleoperators (e.g., spacemouse) with the robot's
        current TCP pose. Only available in CARTESIAN_MOTION mode.

        Returns:
            numpy array of shape (7,) with [x, y, z, roll, pitch, yaw, gripper_pos]
        """
        if not self.is_connected or self._robot is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if self.config.control_mode != ControlMode.CARTESIAN_MOTION:
            raise ValueError("get_current_tcp_pose_euler requires CARTESIAN_MOTION mode")

        tcp_pose = np.asarray(self.feedData.tcpPose, dtype=np.float32)
        gripper_pos = self._current_gripper_position()

        return np.array(
            [
                tcp_pose[0] / MM_PER_METER,
                tcp_pose[1] / MM_PER_METER,
                tcp_pose[2] / MM_PER_METER,
                np.deg2rad(tcp_pose[3]),
                np.deg2rad(tcp_pose[4]),
                np.deg2rad(tcp_pose[5]),
                gripper_pos,
            ],
            dtype=np.float32,
        )

    def get_current_tcp_pose_quat(self) -> np.ndarray:
        """Get current TCP pose in quaternion format [x, y, z, qw, qx, qy, qz, gripper_pos].

        This method can be used for getting the current TCP pose in quaternion format for initializing teleoperators (e.g., pico4) with the robot's
        current TCP pose. Only available in CARTESIAN_MOTION mode.

        Returns:
            numpy array of shape (8,) with [x, y, z, qw, qx, qy, qz, gripper_pos]
        """
        if not self.is_connected or self._robot is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if self.config.control_mode != ControlMode.CARTESIAN_MOTION:
            raise ValueError("get_current_tcp_pose_quat requires CARTESIAN_MOTION mode")

        tcp_pose = self._current_tcp_pose_quat_from_feedback()
        gripper_pos = self._current_gripper_position()

        # Return [x, y, z, qw, qx, qy, qz, gripper_pos]
        return np.array(
            [*tcp_pose, gripper_pos],
            dtype=np.float32,
        )

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Send action command to the robot.

        The action format depends on the control_mode and use_force:
        - JOINT_MOTION: {joint_i.pos: float} for i in 1..7, + gripper.pos
        - CARTESIAN_MOTION + use_force=False: {tcp.x, tcp.y, tcp.z, tcp.r1-r6} + gripper.pos
        - CARTESIAN_MOTION + use_force=True: pose (9D) + wrench (6D) + gripper.pos

        Args:
            action: Dictionary of action values

        Returns:
            The action that was actually sent (may be clipped for safety)
        """
        if not self.is_connected or self._robot is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Check for fault
        if self.feedData.RobotMode == 9:
            raise RuntimeError(f"Robot fault detected. GetErrorID: {self._dobot_error_detail()}")

        # Send robot arm action
        if self.config.control_mode == ControlMode.JOINT_MOTION:
            result = self._send_joint_position_action(action)

        elif self.config.control_mode == ControlMode.CARTESIAN_MOTION:
            result = self._send_cartesian_pure_motion_action(action)

        else:
            raise ValueError(f"Unsupported control_mode: {self.config.control_mode}")

        # Send gripper action
        self._send_gripper_action(action)

        return result

    def _send_joint_position_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Send joint command.

        Uses SendJointPosition with motion constraints (max_vel, max_acc) from config.
        API signature: SendJointPosition(target_pos, target_vel, max_vel, max_acc)

        Note: target_vel is always [0.0] * DoF for impedance control.

        Action keys: action.joint_{1-7}.pos
        """
        # Extract target positions directly from action
        target_pos = [action[key] for key in self._action_joint_keys]

        # Send command using API
        response = self._robot.ServoJ(
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
        self._raise_if_dobot_error(response, "ServoJ")

        return action

    def _send_cartesian_pure_motion_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Send Cartesian pure motion command (NRT mode, no force control).

        Action keys: action.tcp.{x,y,z,r1,r2,r3,r4,r5,r6}

        The action uses 6D rotation representation, then converts it to Dobot's
        ServoP Cartesian pose format [x, y, z, rx, ry, rz].
        """
        # Extract position
        x_m, y_m, z_m = action["tcp.x"], action["tcp.y"], action["tcp.z"]
        # LeRobot actions use meters; Dobot ServoP expects millimeters.
        x_mm = float(x_m) * MM_PER_METER
        y_mm = float(y_m) * MM_PER_METER
        z_mm = float(z_m) * MM_PER_METER

        # Extract 6D rotation and convert to quaternion
        r6d = np.array(
            [
                action["tcp.r1"],
                action["tcp.r2"],
                action["tcp.r3"],
                action["tcp.r4"],
                action["tcp.r5"],
                action["tcp.r6"],
            ]
        )
        quat = rotation_6d_to_quaternion(r6d)  # Returns [qw, qx, qy, qz]
        # Convert quaternion to Euler angles
        euler = quaternion_to_euler(
            quat[0], quat[1], quat[2], quat[3]
        )
        self.logger.debug(
            f"ServoP xyz: [{x_m:.6f}, {y_m:.6f}, {z_m:.6f}] m -> "
            f"[{x_mm:.3f}, {y_mm:.3f}, {z_mm:.3f}] mm"
        )

        response = self._robot.ServoP(
            x_mm,
            y_mm,
            z_mm,
            float(np.rad2deg(euler[0])),
            float(np.rad2deg(euler[1])),
            float(np.rad2deg(euler[2])),
            # 1.0 / self._control_frequency,
            # self._aheadtime,
            # self._gain,
        )
        self._raise_if_dobot_error(response, "ServoP")

        return action

    def _send_gripper_action(self, action: dict[str, Any]) -> None:
        """Send gripper position command.

        Action key: gripper.pos (normalized 0-1)
        """
        if not self._gripper or not self.config.use_gripper:
            return

        if self._gripper_key not in action:
            return

        self._gripper.set_gripper_position(action[self._gripper_key])

    def clear_fault(self) -> bool:
        """Attempt to clear robot fault.

        Returns:
            True if fault was cleared, False otherwise
        """
        if self._robot is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if self.feedData.RobotMode != 9:
            self.logger.info("No fault to clear.")
            return True

        self.logger.info("Attempting to clear fault...")
        response = self._robot.ClearError()
        try:
            self._raise_if_dobot_error(response, "ClearError")
            self.logger.info("✅ Fault cleared successfully.")
            return True
        except RuntimeError as e:
            self.logger.error(f"Failed to clear fault: {e}")
            return False

    def disconnect(self) -> None:
        """Disconnect from the robot safely.

        This method ensures safe disconnection even if errors occur during the process.
        It will attempt to:
        1. Move robot to home position (if possible)
        2. Stop any ongoing motion
        3. Disconnect gripper and cameras
        4. Clean up all resources

        All errors are caught and logged, but the disconnect process continues to ensure cleanup.
        """
        if not self._is_connected:
            self.logger.warn(f"{self} is not connected, skipping disconnect.")
            return

        try:
            self.logger.info("Disconnecting from Dobot Nova5 robot...")

            # Move to home position before disconnecting
            try:
                self._go_to_home()
            except Exception as e:
                self.logger.warn(f"Failed to move to home before disconnect: {e}")

            # Stop any ongoing motion
            if self._robot is not None:
                self._robot.Stop()
                self._robot.close()
            if self._feedFour is not None:
                self._feedFour.close()

            # Disconnect DH Gripper
            if self._gripper and self.config.use_gripper:
                self._gripper.disconnect()

            # Disconnect external cameras
            for cam in self.cameras.values():
                cam.disconnect()

        except Exception as e:
            self.logger.error(f"Error during disconnect: {e}")
        finally:
            self._robot = None
            self._feedFour = None
            self._gripper = None
            self._is_connected = False
            self._current_mode = None
            self.logger.info("✅ Dobot Nova5 disconnected.")

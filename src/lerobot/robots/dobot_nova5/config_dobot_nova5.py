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

"""Configuration for Dobot Nova5 robot."""

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict

from lerobot.robots.dobot_nova5.TCP_IP_Python_V4.dobot_api import DobotApiFeedBack,DobotApiDashboard

from lerobot.cameras.configs import CameraConfig
from lerobot.cameras.realsense import RealSenseCameraConfig
from lerobot.cameras.xense import XenseTactileCameraConfig, XenseOutputType
from lerobot.robots.config import RobotConfig
from lerobot.robots.dh_gripper import DHGripperConfig  # noqa: F401


class ControlMode(str, Enum):
    """Control mode for Dobot Nova5.

    JOINT_MOTION:
        Joint motion control.
        - Action: joint positions (6D) + gripper (1D) = 7D
        - Observation: joint positions (6D) + gripper (1D) = 7D

    CARTESIAN_MOTION:
        Cartesian motion control.
        - Action: TCP pose (9D) + gripper (1D) = 10D
        - Observation: TCP pose (9D) + gripper (1D) = 10D
    """

    JOINT_MOTION = "joint_motion_control"
    CARTESIAN_MOTION = "cartesian_motion_control"


@RobotConfig.register_subclass("dobot_nova5")
@dataclass
class DobotNova5Config(RobotConfig):
    """Configuration for Dobot Nova5 robot.

    The Dobot Nova5 is a 6-DOF collaborative robot with force sensing capabilities.
    """

    # Robot identification
    robot_ip: str = "192.168.5.102"  # Robot IP address
    dashboardPort: int = 29999
    feedPortFour: int = 30004

    # Control settings
    control_mode: ControlMode = ControlMode.CARTESIAN_MOTION

    # NRT mode supports 1-100 Hz
    control_frequency: float = 100.0  # Hz

    # Connection behavior: if True, move to start_position_degree after connect
    go_to_start: bool = True

    aheadtime: float = 50.0  # PID controller D (20.0-100.0, default 50.0)
    gain: float = 500.0      # PID controller P (200.0-1000.0, default 500.0)

    if robot_ip == "192.168.5.102":
        home_point_list = [270, 0, 90, 0, -90, 0]
    elif robot_ip == "192.168.5.101":
        home_point_list = [-90, 0, -90, 0, 90, 0]

    # Start position parameters (for MoveJ primitive)
    # Joint positions in degrees (factory-defined home position)
    if robot_ip == "192.168.5.102":
        start_position_degree: list[float] = field(default_factory=lambda: [270, 0, 90, 0, -90, 0])
    elif robot_ip == "192.168.5.101":
        start_position_degree: list[float] = field(default_factory=lambda: [-90, 0, -90, 0, 90, 0])
    # Joint velocity scale for moving to start position (1-100, default 30)
    start_vel_scale: int = 30

    # ======================== DH Gripper (end-effector) settings ==========
    # Whether to use the DH Robotics AG-95 gripper end-effector
    use_gripper: bool = False

    # Serial port configuration
    dh_gripper_port: str = "/dev/ttyUSB0"
    dh_gripper_slave_id: int = 1
    dh_gripper_baudrate: int = 115200
    dh_gripper_force: int = 30       # Target force 20-100 %
    dh_gripper_init_open: bool = True

    # Auto-created in __post_init__ from dh_gripper_* parameters (do not set directly)
    dh_gripper: DHGripperConfig | None = field(default=None, init=False)

    # ======================== Tactile Sensor Configuration ========================
    # Set enable_tactile_sensors=True to include XenseTactileCameraConfig entries in cameras.
    # Sensors are keyed by their observation name (e.g. "left_tactile", "right_tactile").
    enable_tactile_sensors: bool = False

    # ======================== Camera Configuration ========================

    # RealSense cameras (2 cameras recommended: main + wrist).
    # Tactile sensors are added automatically via enable_tactile_sensors above.
    cameras: Dict[str, CameraConfig] = field(default_factory=lambda: {})

    def __post_init__(self):
        super().__post_init__()
        # self.cameras = {
            # "head": RealSenseCameraConfig(
            #     serial_number_or_name="135522074323",
            #     fps=30,
            #     width=640,
            #     height=480,
            #     color_mode=ColorMode.RGB
            # ),
            # # wrist camera
            # "wrist": RealSenseCameraConfig(
            #     serial_number_or_name="249322063436",
            #     fps=30,
            #     width=640,
            #     height=480,
            #     color_mode=ColorMode.RGB
            # )
        # }
        # Validate control frequency (NRT mode: 1-100 Hz)
        if not 1 <= self.control_frequency <= 100:
            raise ValueError(
                f"control_frequency must be between 1 and 100 Hz for NRT mode, got {self.control_frequency}"
            )

        # Validate start position parameters
        if len(self.start_position_degree) != 6:
            raise ValueError(
                f"start_position_degree must have 6 elements, got {len(self.start_position_degree)}"
            )
        if len(self.home_point_list) != 6:
            raise ValueError(
                f"home_point_list must have 6 elements, got {len(self.home_point_list)}"
            )
        if not 1 <= self.start_vel_scale <= 100:
            raise ValueError(f"start_vel_scale must be between 1 and 100, got {self.start_vel_scale}")

        # Create DHGripperConfig from exposed parameters (only if use_gripper=True)
        if self.use_gripper:
            self.dh_gripper = DHGripperConfig(
                port=self.dh_gripper_port,
                slave_id=self.dh_gripper_slave_id,
                baudrate=self.dh_gripper_baudrate,
                gripper_force=self.dh_gripper_force,
                init_open=self.dh_gripper_init_open,
            )
        else:
            self.dh_gripper = None

        # Inject tactile sensors into cameras dict (only if enable_tactile_sensors=True)
        if self.enable_tactile_sensors:
            self.cameras.update(
                {
                    "tactile_0": XenseTactileCameraConfig(
                        serial_number="OG000339",
                        fps=30,
                        output_types=[XenseOutputType.RECTIFY],
                        warmup_s=0.05,
                    ),
                    "tactile_1": XenseTactileCameraConfig(
                        serial_number="OG000450",
                        fps=30,
                        output_types=[XenseOutputType.RECTIFY],
                        warmup_s=0.05,
                    ),
                }
            )
        pass

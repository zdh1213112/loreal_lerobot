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

"""Configuration for BiDobot Nova5 robot."""

from dataclasses import dataclass, field
from enum import Enum

from lerobot.cameras.configs import CameraConfig
from lerobot.cameras.xense import XenseOutputType, XenseTactileCameraConfig
from lerobot.robots.config import RobotConfig
from lerobot.robots.dh_gripper import DHGripperConfig  # noqa: F401


class ControlMode(str, Enum):
    """Control mode for BiDobot Nova5.

    JOINT_MOTION:
        Joint motion control.
        - Action: joint positions (6D each) + gripper (1D each) = 14D
        - Observation: joint positions (6D each) + gripper (1D each) = 14D

    CARTESIAN_MOTION:
        Cartesian motion control.
        - Action: TCP pose (9D each) + gripper (1D each) = 20D
        - Observation: TCP pose (9D each) + gripper (1D each) = 20D
    """

    JOINT_MOTION = "joint_motion_control"
    CARTESIAN_MOTION = "cartesian_motion_control"


@RobotConfig.register_subclass("bi_dobot_nova5")
@dataclass
class BiDobotNova5Config(RobotConfig):
    """Configuration for BiDobot Nova5 robot.

    The BiDobot Nova5 is a bimanual system with two 6-DOF collaborative robots.
    """

    # Robot identification
    left_robot_ip: str = "192.168.5.101"
    left_dashboardPort: int = 29999
    left_feedPortFour: int = 30004
    right_robot_ip: str = "192.168.5.102"
    right_dashboardPort: int = 29999
    right_feedPortFour: int = 30004

    # Control settings
    control_mode: ControlMode = ControlMode.CARTESIAN_MOTION

    # NRT mode supports 1-100 Hz
    control_frequency: float = 100.0  # Hz

    # Connection behavior: if True, move to start positions after connect
    go_to_start: bool = True

    aheadtime: float = 50.0  # PID controller D (20.0-100.0, default 50.0)
    gain: float = 500.0  # PID controller P (200.0-1000.0, default 500.0)

    left_home_point_list: list[float] = field(
        default_factory=lambda: [-90, 0, -90, 0, 90, 0]
    )
    right_home_point_list: list[float] = field(
        default_factory=lambda: [270, 0, 90, 0, -90, 0]
    )

    # Start position parameters (for MoveJ primitive)
    left_start_position_degree: list[float] = field(
        default_factory=lambda: [-90, 0, -90, 0, 90, 0]
    )
    right_start_position_degree: list[float] = field(
        default_factory=lambda: [270, 0, 90, 0, -90, 0]
    )
    # Joint velocity scale for moving to start position (1-100, default 30)
    start_vel_scale: int = 30

    # ======================== DH Gripper (end-effector) settings ==========
    # Whether to use the DH Robotics AG-95 gripper on each arm
    use_left_gripper: bool = False
    use_right_gripper: bool = True

    # Left gripper serial port configuration
    left_dh_gripper_port: str = "/dev/ttyDHLeft"
    left_dh_gripper_slave_id: int = 1
    left_dh_gripper_baudrate: int = 115200
    left_dh_gripper_force: int = 30  # 20-100 %
    left_dh_gripper_init_open: bool = True

    # Right gripper serial port configuration
    right_dh_gripper_port: str = "/dev/ttyDHRight"
    right_dh_gripper_slave_id: int = 1
    right_dh_gripper_baudrate: int = 115200
    right_dh_gripper_force: int = 30  # 20-100 %
    right_dh_gripper_init_open: bool = True

    # Auto-created in __post_init__ from dh_gripper_* parameters (do not set directly)
    left_dh_gripper: DHGripperConfig | None = field(default=None, init=False)
    right_dh_gripper: DHGripperConfig | None = field(default=None, init=False)

    # ======================== Tactile Sensor Configuration ========================
    # Set enable_tactile_sensors=True to include XenseTactileCameraConfig entries in cameras.
    # Sensors are keyed by their observation name (e.g. "left_tactile_0", "right_tactile_0").
    enable_tactile_sensors: bool = False

    # ======================== Camera Configuration ========================

    # cameras (2 cameras recommended: main + wrist)
    cameras: dict[str, CameraConfig] = field(default_factory=lambda: {})

    def __post_init__(self):
        super().__post_init__()
        # self.cameras = {
        #     "head": RealSenseCameraConfig(
        #             serial_number_or_name="230322271365",
        #             fps=30,
        #             width=640,
        #             height=480,
        #             warmup_s=1.0,
        #         ),
        #         "left_wrist": RealSenseCameraConfig(
        #             serial_number_or_name="230422271416",
        #             fps=30,
        #             width=640,
        #             height=480,
        #             warmup_s=1.0,
        #         ),
        #         "right_wrist": RealSenseCameraConfig(
        #             serial_number_or_name="230322274234",
        #             fps=30,
        #             width=640,
        #             height=480,
        #             warmup_s=1.0,
        #         ),
        # }

        # Validate control frequency (NRT mode: 1-100 Hz)
        if not 1 <= self.control_frequency <= 100:
            raise ValueError(
                f"control_frequency must be between 1 and 100 Hz for NRT mode, got {self.control_frequency}"
            )

        # Validate start position parameters
        if len(self.left_start_position_degree) != 6:
            raise ValueError(
                f"left_start_position_degree must have 6 elements, got {len(self.left_start_position_degree)}"
            )
        if len(self.right_start_position_degree) != 6:
            raise ValueError(
                f"right_start_position_degree must have 6 elements, got {len(self.right_start_position_degree)}"
            )
        if len(self.left_home_point_list) != 6:
            raise ValueError(
                f"left_home_point_list must have 6 elements, got {len(self.left_home_point_list)}"
            )
        if len(self.right_home_point_list) != 6:
            raise ValueError(
                f"right_home_point_list must have 6 elements, got {len(self.right_home_point_list)}"
            )
        if not 1 <= self.start_vel_scale <= 100:
            raise ValueError(
                f"start_vel_scale must be between 1 and 100, got {self.start_vel_scale}"
            )

        # Create DHGripperConfig from exposed parameters (only if use_*_gripper=True)
        if self.use_left_gripper:
            self.left_dh_gripper = DHGripperConfig(
                port=self.left_dh_gripper_port,
                slave_id=self.left_dh_gripper_slave_id,
                baudrate=self.left_dh_gripper_baudrate,
                gripper_force=self.left_dh_gripper_force,
                init_open=self.left_dh_gripper_init_open,
            )
            if self.enable_tactile_sensors:
                self.cameras.update(
                    {
                        "left_tactile_0": XenseTactileCameraConfig(
                            serial_number="OG000352",
                            fps=30,
                            output_types=[XenseOutputType.RECTIFY],
                            warmup_s=0.05,
                        ),
                        "left_tactile_1": XenseTactileCameraConfig(
                            serial_number="OG000353",
                            fps=30,
                            output_types=[XenseOutputType.RECTIFY],
                            warmup_s=0.05,
                        ),
                    }
                )
        else:
            self.left_dh_gripper = None

        if self.use_right_gripper:
            self.right_dh_gripper = DHGripperConfig(
                port=self.right_dh_gripper_port,
                slave_id=self.right_dh_gripper_slave_id,
                baudrate=self.right_dh_gripper_baudrate,
                gripper_force=self.right_dh_gripper_force,
                init_open=self.right_dh_gripper_init_open,
            )
            if self.enable_tactile_sensors:
                self.cameras.update(
                    {
                        "right_tactile_0": XenseTactileCameraConfig(
                            serial_number="OG000339",
                            fps=30,
                            output_types=[XenseOutputType.RECTIFY],
                            warmup_s=0.05,
                        ),
                        "right_tactile_1": XenseTactileCameraConfig(
                            serial_number="OG000450",
                            fps=30,
                            output_types=[XenseOutputType.RECTIFY],
                            warmup_s=0.05,
                        ),
                    }
                )
        else:
            self.right_dh_gripper = None

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
from lerobot.cameras.xense.configuration_xense import XenseOutputType, XenseTactileCameraConfig
from lerobot.robots.config import RobotConfig

from .config_dh_gripper_integrated import DHGripperIntegratedConfig  # noqa: F401


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


class ResetMoveOrder(str, Enum):
    LEFT_THEN_RIGHT = "left_then_right"
    RIGHT_THEN_LEFT = "right_then_left"


class ResetStrategy(str, Enum):
    SEQUENTIAL_JOINT = "sequential_joint"
    SYNC_Y_CLEARANCE_THEN_JOINT = "sync_y_clearance_then_joint"


class ResetTarget(str, Enum):
    HOME = "home"
    START = "start"


@RobotConfig.register_subclass("bi_dobot_nova5_dh")
@dataclass
class BiDobotNova5DHConfig(RobotConfig):
    """Configuration for BiDobot Nova5 robot.

    The BiDobot Nova5 is a bimanual system with two 6-DOF collaborative robots.
    """

    # Robot identification
    left_robot_ip: str = "192.168.111.101"
    left_dashboardPort: int = 29999
    left_feedPortFour: int = 30004
    right_robot_ip: str = "192.168.111.102"
    right_dashboardPort: int = 29999
    right_feedPortFour: int = 30004

    # Control settings
    control_mode: ControlMode = ControlMode.CARTESIAN_MOTION

    # NRT mode supports 1-100 Hz
    control_frequency: float = 100.0  # Hz

    # Connection behavior: if True, move to start positions after connect
    go_to_start: bool = False
    reset_move_order: ResetMoveOrder = ResetMoveOrder.RIGHT_THEN_LEFT
    reset_strategy: ResetStrategy = ResetStrategy.SEQUENTIAL_JOINT
    reset_target: ResetTarget = ResetTarget.HOME
    reset_y_clearance_m: float = 0.035
    left_reset_y_clearance_sign: float = 1.0
    right_reset_y_clearance_sign: float = -1.0

    aheadtime: float = 50.0  # PID controller D (20.0-100.0, default 50.0)
    gain: float = 500.0  # PID controller P (200.0-1000.0, default 500.0)

    # Cartesian ServoP pre-check. Dobot ServoP can fail when the target pose is
    # near a singular IK branch; pre-checking with current joints as JointNear
    # lets us keep the same branch and back off to the last reachable pose.
    enable_cartesian_ik_guard: bool = True
    cartesian_ik_servoj: bool = True
    cartesian_ik_backoff_steps: int = 6
    cartesian_ik_servoj_hold_s: float = 0.0
    max_cartesian_step_m: float = 0.15

    # Keep record/teleop loops at camera FPS by enqueueing the newest action and
    # letting a background worker perform the blocking Dobot ServoP call.
    async_action_worker: bool = True
    async_action_worker_frequency: float = 30.0

    # Tool coordinate for Cartesian teleoperation. Tool 0 is the flange.
    use_tool_coordinate: bool = True
    left_tool_coordinate_index: int = 1
    right_tool_coordinate_index: int = 1

    # Cartesian safety workspace for teleoperation actions, in meters.
    # Format: [x, y, z].
    enable_clip: bool = True
    left_workspace_min_xyz_m: list[float] = field(
        default_factory=lambda: [-0.60, -0.62, -0.175]
    )
    left_workspace_max_xyz_m: list[float] = field(
        default_factory=lambda: [0.68, 0.15, 0.69]
    )
    right_workspace_min_xyz_m: list[float] = field(
        default_factory=lambda: [-0.60, -0.15, -0.167]
    )
    right_workspace_max_xyz_m: list[float] = field(
        default_factory=lambda: [0.68, 0.62, 0.69]
    )

    left_home_point_list: list[float] = field(
        default_factory=lambda: [-40, 0, -145, 100, 100, 0]
    )
    right_home_point_list: list[float] = field(
        default_factory=lambda: [220, 0, 135, -80, -88, 0]
    )

    # Start position parameters (for MoveJ primitive)
    left_start_position_degree: list[float] = field(
        default_factory=lambda: [-40, 0, -145, 100, 100, 0]
    )
    right_start_position_degree: list[float] = field(
        default_factory=lambda: [220, 0, 135, -80, -88, 0]
    )
    # Joint velocity scale for moving to start position (1-100, default 30)
    start_vel_scale: int = 30

    # ======================== DH Gripper (end-effector) settings ==========
    # Whether to use the DH Robotics AG-95 gripper on each arm.
    # Grippers communicate via the arm's built-in RS485 end-effector port (Modbus RTU).
    use_left_gripper: bool = True
    use_right_gripper: bool = True

    # Left gripper Modbus RTU configuration
    left_master_ip: str = "192.168.201.1"
    left_master_port: int = 60000
    left_tool_identify: int = 1

    left_dh_gripper_slave_id: int = 1
    left_dh_gripper_baudrate: int = 115200
    left_dh_gripper_force: int = 33  # 20-100 %
    left_dh_gripper_init_open: bool = True
    left_dh_gripper_worker_frequency: float = 40.0  # Hz, best effort
    left_dh_gripper_position_poll_frequency: float = 30.0  # Hz
    left_dh_gripper_command_epsilon: float = 0.0

    # Right gripper Modbus RTU configuration
    right_master_ip: str = "192.168.201.1"
    right_master_port: int = 60000
    right_tool_identify: int = 1
    right_dh_gripper_slave_id: int = 1
    right_dh_gripper_baudrate: int = 115200
    right_dh_gripper_force: int = 33  # 20-100 %
    right_dh_gripper_init_open: bool = True
    right_dh_gripper_worker_frequency: float = 40.0  # Hz, best effort
    right_dh_gripper_position_poll_frequency: float = 30.0  # Hz
    right_dh_gripper_command_epsilon: float = 0.0

    # Auto-created in __post_init__ from dh_gripper_* parameters (do not set directly)
    left_dh_gripper: DHGripperIntegratedConfig | None = field(default=None, init=False)
    right_dh_gripper: DHGripperIntegratedConfig | None = field(default=None, init=False)

    # ======================== Tactile Sensor Configuration ========================
    # Set enable_tactile_sensors=True to include XenseTactileCameraConfig entries in cameras.
    # Sensors are keyed by their observation name (e.g. "left_tactile_0", "right_tactile_0").
    enable_tactile_sensors: bool = False

    # ======================== Camera Configuration ========================

    # cameras (2 cameras recommended: main + wrist)
    cameras: dict[str, CameraConfig] = field(default_factory=lambda: {})

    def __post_init__(self):
        super().__post_init__()
        from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig

        self.cameras = {
            "head": RealSenseCameraConfig(
                serial_number_or_name="254622078230",
                fps=30,
                width=1280,
                height=720,
                warmup_s=1.0,
            ),
            "left_wrist": RealSenseCameraConfig(
                serial_number_or_name="409122274792",
                fps=30,
                width=640,
                height=480,
                warmup_s=1.0,
            ),
            "right_wrist": RealSenseCameraConfig(
                serial_number_or_name="352122272611",
                fps=30,
                width=640,
                height=480,
                warmup_s=1.0,
            ),
        }

        # Validate control frequency (NRT mode: 1-100 Hz)
        if not 1 <= self.control_frequency <= 100:
            raise ValueError(
                f"control_frequency must be between 1 and 100 Hz for NRT mode, got {self.control_frequency}"
            )
        if self.cartesian_ik_backoff_steps < 0:
            raise ValueError(
                f"cartesian_ik_backoff_steps must be >= 0, got {self.cartesian_ik_backoff_steps}"
            )
        if self.cartesian_ik_servoj_hold_s < 0:
            raise ValueError(
                f"cartesian_ik_servoj_hold_s must be >= 0, got {self.cartesian_ik_servoj_hold_s}"
            )
        if self.max_cartesian_step_m < 0:
            raise ValueError(
                f"max_cartesian_step_m must be >= 0, got {self.max_cartesian_step_m}"
            )
        self._validate_tool_coordinate_index(
            self.left_tool_coordinate_index, "left_tool_coordinate_index"
        )
        self._validate_tool_coordinate_index(
            self.right_tool_coordinate_index, "right_tool_coordinate_index"
        )
        self._validate_workspace_min_max(
            self.left_workspace_min_xyz_m,
            self.left_workspace_max_xyz_m,
            "left_workspace",
        )
        self._validate_workspace_min_max(
            self.right_workspace_min_xyz_m,
            self.right_workspace_max_xyz_m,
            "right_workspace",
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
        if self.left_tool_identify not in (1, 2):
            raise ValueError(
                f"left_tool_identify must be 1 or 2, got {self.left_tool_identify}"
            )
        if self.right_tool_identify not in (1, 2):
            raise ValueError(
                f"right_tool_identify must be 1 or 2, got {self.right_tool_identify}"
            )

        # Create DHGripperIntegratedConfig from exposed parameters (only if use_*_gripper=True)
        if self.use_left_gripper:
            self.left_dh_gripper = DHGripperIntegratedConfig(
                slave_id=self.left_dh_gripper_slave_id,
                baudrate=self.left_dh_gripper_baudrate,
                gripper_force=self.left_dh_gripper_force,
                init_open=self.left_dh_gripper_init_open,
                worker_frequency=self.left_dh_gripper_worker_frequency,
                position_poll_frequency=self.left_dh_gripper_position_poll_frequency,
                command_epsilon=self.left_dh_gripper_command_epsilon,
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
            self.right_dh_gripper = DHGripperIntegratedConfig(
                slave_id=self.right_dh_gripper_slave_id,
                baudrate=self.right_dh_gripper_baudrate,
                gripper_force=self.right_dh_gripper_force,
                init_open=self.right_dh_gripper_init_open,
                worker_frequency=self.right_dh_gripper_worker_frequency,
                position_poll_frequency=self.right_dh_gripper_position_poll_frequency,
                command_epsilon=self.right_dh_gripper_command_epsilon,
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

    @staticmethod
    def _validate_tool_coordinate_index(index: int, name: str) -> None:
        if not 0 <= int(index) <= 9:
            raise ValueError(f"{name} must be between 0 and 9, got {index}")

    @staticmethod
    def _validate_workspace_min_max(
        min_xyz: list[float], max_xyz: list[float], name: str
    ) -> None:
        if len(min_xyz) != 3:
            raise ValueError(f"{name}_min_xyz_m must have 3 elements, got {len(min_xyz)}")
        if len(max_xyz) != 3:
            raise ValueError(f"{name}_max_xyz_m must have 3 elements, got {len(max_xyz)}")
        axis_names = ("x", "y", "z")
        for axis_idx, axis_name in enumerate(axis_names):
            lower = float(min_xyz[axis_idx])
            upper = float(max_xyz[axis_idx])
            if lower >= upper:
                raise ValueError(
                    f"{name} has invalid {axis_name} bounds: min {lower} >= max {upper}"
                )

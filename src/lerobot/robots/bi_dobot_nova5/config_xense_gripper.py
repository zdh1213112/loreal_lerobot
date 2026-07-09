from dataclasses import dataclass, field
from typing import Dict

from enum import Enum

class SensorOutputType(Enum):
    """Output type for tactile sensors."""

    RECTIFY = "rectify"
    DIFFERENCE = "difference"


@dataclass
class GripperConfig:
    """Configuration for XenseGripper"""
    
    # Gripper identification
    mac_addr: str = "7ec0c7f50ea6"  # Gripper serial number
    
    # Sensor settings
    rectify_size: tuple[int, int] = (96, 160)
    sensor_output_type: SensorOutputType = SensorOutputType.RECTIFY
    sensor_keys: dict[str, str] = field(default_factory=dict)
    
    # Gripper control parameters for set_position()
    gripper_velocity: float = 100.0  # Maximum velocity (0-350 mm/s)
    gripper_force: float = 30.0  # Maximum force (0-60 N)

    # Example: MIN_OPEN=0 mm, MAX_OPEN=85 mm
    gripper_min_pos: float = 0.0
    gripper_max_pos: float = 85.0
    
    # Initialize gripper to fully open on connect
    init_open: bool = True

    def __post_init__(self):
        if not self.mac_addr:
            raise ValueError("mac_addr is required for XenseGripper")

        # Set default sensor_keys if not provided
        if not self.sensor_keys:
            self.sensor_keys = {
                "left_tactile": "OG000447",
                "right_tactile": "OG000454",
            }
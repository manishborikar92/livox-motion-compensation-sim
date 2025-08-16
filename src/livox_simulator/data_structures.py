"""
Data structures for Livox Mid-70 simulation framework
"""

from dataclasses import dataclass
from typing import Dict, Any
import numpy as np


@dataclass
class LiDARFrame:
    """Represents a single LiDAR frame with associated metadata."""
    timestamp: float          # Frame timestamp (seconds)
    points: np.ndarray       # Point cloud data (N×4: x,y,z,intensity)
    frame_id: int            # Sequential frame identifier
    device_info: Dict[str, Any]        # Device information
    quality_metrics: Dict[str, Any]    # Frame quality metrics


@dataclass
class IMUData:
    """Represents IMU measurement data."""
    timestamp: float         # Measurement timestamp (seconds)
    gyro_x: float           # Angular velocity X (rad/s)
    gyro_y: float           # Angular velocity Y (rad/s)
    gyro_z: float           # Angular velocity Z (rad/s)
    accel_x: float          # Acceleration X (m/s²)
    accel_y: float          # Acceleration Y (m/s²)
    accel_z: float          # Acceleration Z (m/s²)


@dataclass
class TrajectoryPoint:
    """Represents a point in the vehicle trajectory."""
    timestamp: float         # Time (seconds)
    position: np.ndarray    # Position [x, y, z] (meters)
    orientation: np.ndarray # Orientation [roll, pitch, yaw] (radians)
    velocity: np.ndarray    # Velocity [vx, vy, vz] (m/s)
    acceleration: np.ndarray # Acceleration [ax, ay, az] (m/s²)


@dataclass
class SimulationConfig:
    """Complete simulation configuration."""
    # Simulation parameters
    duration: float = 180.0
    random_seed: int = 42
    output_directory: str = './lidar_simulation_output'
    
    # LiDAR specifications (Livox Mid-70)
    fov_horizontal: float = 70.4
    fov_vertical: float = 77.2
    range_max: float = 90.0
    range_min: float = 0.05
    points_per_second: int = 100000
    frame_rate: int = 10
    angular_resolution: float = 0.28
    point_accuracy: float = 0.02
    
    # Motion parameters
    trajectory_type: str = 'urban_circuit'
    max_speed: float = 15.0
    max_acceleration: float = 2.0
    max_angular_velocity: float = 0.3
    
    # Environment settings
    environment_complexity: str = 'urban'
    building_density: float = 0.4
    vegetation_coverage: float = 0.15
    dynamic_objects: bool = True
    
    # Advanced features
    enable_motion_compensation: bool = True
    coordinate_system: str = 'utm'
    lvx_format: str = 'lvx2'
    
    # IMU configuration
    imu_update_rate: int = 200
    imu_gyro_noise: float = 0.01
    imu_accel_noise: float = 0.1
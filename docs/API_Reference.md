# API Reference

## Core Classes

### LiDARMotionSimulator

Main simulation class for Livox Mid-70 LiDAR systems with motion compensation.

```python
class LiDARMotionSimulator:
    def __init__(self, config: dict)
    def run_simulation(self) -> dict
    def set_performance_callback(self, callback: callable)
    def get_system_info(self) -> dict
```

#### Constructor

```python
def __init__(self, config: dict)
```

**Parameters:**
- `config` (dict): Configuration dictionary containing simulation parameters

**Example:**
```python
config = {
    'duration': 120.0,
    'trajectory_type': 'urban_circuit',
    'enable_motion_compensation': True
}
simulator = LiDARMotionSimulator(config)
```

#### Methods

##### run_simulation()

```python
def run_simulation(self) -> dict
```

Executes the complete simulation pipeline.

**Returns:**
- `dict`: Simulation results containing:
  - `frames`: List of generated LiDAR frames
  - `trajectory`: Vehicle trajectory data
  - `imu_data`: High-frequency IMU measurements
  - `processing_time`: Performance metrics
  - `quality_metrics`: Data quality assessment

**Example:**
```python
results = simulator.run_simulation()
print(f"Generated {len(results['frames'])} frames")
print(f"Processing time: {results['processing_time']:.2f}s")
```

##### set_performance_callback()

```python
def set_performance_callback(self, callback: callable)
```

Sets a callback function for real-time performance monitoring.

**Parameters:**
- `callback` (callable): Function called with performance metrics

**Example:**
```python
def monitor_performance(metrics):
    print(f"Latency: {metrics['latency_ms']:.1f}ms")
    print(f"Memory: {metrics['memory_mb']:.1f}MB")

simulator.set_performance_callback(monitor_performance)
```

### CoordinateTransformer

Handles coordinate system transformations between sensor, vehicle, local, and global frames.

```python
class CoordinateTransformer:
    def __init__(self, calibration_params: dict)
    def transform_points(self, points: np.ndarray, source: str, target: str) -> np.ndarray
    def set_transformation(self, source: str, target: str, translation: list, rotation: list)
    def get_transformation_matrix(self, source: str, target: str) -> np.ndarray
```

#### Constructor

```python
def __init__(self, calibration_params: dict)
```

**Parameters:**
- `calibration_params` (dict): Calibration parameters for sensor mounting

**Example:**
```python
calibration = {
    'lidar_to_vehicle': {
        'translation': [0.0, 0.0, 1.5],  # meters
        'rotation': [0.0, 0.0, 0.0]      # radians
    }
}
transformer = CoordinateTransformer(calibration)
```

#### Methods

##### transform_points()

```python
def transform_points(self, points: np.ndarray, source: str, target: str) -> np.ndarray
```

Transforms point cloud between coordinate systems.

**Parameters:**
- `points` (np.ndarray): Input points (N×3 or N×4)
- `source` (str): Source coordinate system ('sensor', 'vehicle', 'local', 'global')
- `target` (str): Target coordinate system

**Returns:**
- `np.ndarray`: Transformed points

**Example:**
```python
# Transform from sensor to global coordinates
global_points = transformer.transform_points(
    sensor_points, 'sensor', 'global'
)
```

### MotionCompensator

Applies motion compensation using IMU data to correct for vehicle movement during scanning.

```python
class MotionCompensator:
    def __init__(self, imu_config: dict)
    def compensate_frame(self, points: np.ndarray, imu_data: list, frame_time: float) -> np.ndarray
    def set_compensation_method(self, method: str)
    def get_compensation_statistics(self) -> dict
```

#### Constructor

```python
def __init__(self, imu_config: dict)
```

**Parameters:**
- `imu_config` (dict): IMU configuration parameters

**Example:**
```python
imu_config = {
    'update_rate': 200,  # Hz
    'gyro_noise': 0.01,  # rad/s
    'accel_noise': 0.1   # m/s²
}
compensator = MotionCompensator(imu_config)
```

#### Methods

##### compensate_frame()

```python
def compensate_frame(self, points: np.ndarray, imu_data: list, frame_time: float) -> np.ndarray
```

Applies motion compensation to a single LiDAR frame.

**Parameters:**
- `points` (np.ndarray): Raw point cloud data
- `imu_data` (list): IMU measurements during frame acquisition
- `frame_time` (float): Frame timestamp

**Returns:**
- `np.ndarray`: Motion-compensated points

### DataExporter

Handles export of simulation data to various formats.

```python
class DataExporter:
    def __init__(self, output_directory: str)
    def export_point_cloud(self, points: np.ndarray, format: str, filename: str)
    def export_trajectory(self, trajectory: dict, format: str, filename: str)
    def export_all_formats(self, data: dict) -> dict
```

#### Constructor

```python
def __init__(self, output_directory: str)
```

**Parameters:**
- `output_directory` (str): Base directory for output files

#### Methods

##### export_point_cloud()

```python
def export_point_cloud(self, points: np.ndarray, format: str, filename: str)
```

Exports point cloud data in specified format.

**Parameters:**
- `points` (np.ndarray): Point cloud data
- `format` (str): Output format ('pcd', 'las', 'ply', 'xyz', 'lvx2')
- `filename` (str): Output filename

**Example:**
```python
exporter = DataExporter('./output')
exporter.export_point_cloud(points, 'pcd', 'scan.pcd')
exporter.export_point_cloud(points, 'las', 'scan.las')
```

## Data Structures

### LiDARFrame

Represents a single LiDAR frame with associated metadata.

```python
@dataclass
class LiDARFrame:
    timestamp: float          # Frame timestamp (seconds)
    points: np.ndarray       # Point cloud data (N×4: x,y,z,intensity)
    frame_id: int            # Sequential frame identifier
    device_info: dict        # Device information
    quality_metrics: dict    # Frame quality metrics
```

**Example:**
```python
frame = LiDARFrame(
    timestamp=1234567890.123,
    points=np.array([[1.0, 2.0, 3.0, 128]]),
    frame_id=42,
    device_info={'serial': '3GGDJ6K00200101'},
    quality_metrics={'point_count': 95000}
)
```

### IMUData

Represents IMU measurement data.

```python
@dataclass
class IMUData:
    timestamp: float         # Measurement timestamp (seconds)
    gyro_x: float           # Angular velocity X (rad/s)
    gyro_y: float           # Angular velocity Y (rad/s)
    gyro_z: float           # Angular velocity Z (rad/s)
    accel_x: float          # Acceleration X (m/s²)
    accel_y: float          # Acceleration Y (m/s²)
    accel_z: float          # Acceleration Z (m/s²)
```

### TrajectoryPoint

Represents a point in the vehicle trajectory.

```python
@dataclass
class TrajectoryPoint:
    timestamp: float         # Time (seconds)
    position: np.ndarray    # Position [x, y, z] (meters)
    orientation: np.ndarray # Orientation [roll, pitch, yaw] (radians)
    velocity: np.ndarray    # Velocity [vx, vy, vz] (m/s)
    acceleration: np.ndarray # Acceleration [ax, ay, az] (m/s²)
```

## Configuration Schema

### Main Configuration

```python
config_schema = {
    # Simulation parameters
    'duration': float,                    # Simulation duration (seconds)
    'random_seed': int,                   # Random seed for reproducibility
    'output_directory': str,              # Output directory path
    
    # LiDAR specifications
    'lidar_specs': {
        'fov_horizontal': float,          # Horizontal FOV (degrees)
        'fov_vertical': float,            # Vertical FOV (degrees)
        'range_max': float,               # Maximum range (meters)
        'range_min': float,               # Minimum range (meters)
        'points_per_second': int,         # Point generation rate
        'frame_rate': int,                # Frame rate (Hz)
        'angular_resolution': float,      # Angular resolution (degrees)
        'point_accuracy': float,          # Point accuracy (meters)
    },
    
    # Motion parameters
    'trajectory': {
        'type': str,                      # Trajectory type
        'max_speed': float,               # Maximum speed (m/s)
        'max_acceleration': float,        # Maximum acceleration (m/s²)
        'max_angular_velocity': float,    # Maximum angular velocity (rad/s)
    },
    
    # Environment settings
    'environment': {
        'complexity': str,                # Environment complexity level
        'building_density': float,        # Building density (0-1)
        'vegetation_coverage': float,     # Vegetation coverage (0-1)
        'dynamic_objects': bool,          # Enable dynamic objects
    },
    
    # Advanced features
    'enable_motion_compensation': bool,   # Enable motion compensation
    'coordinate_system': str,            # Output coordinate system
    'lvx_format': str,                   # LVX format version
}
```

### Sensor Configuration

```python
sensor_config = {
    'lidar_builtin_imu': {
        'enabled': bool,
        'update_rate': int,               # Hz
        'gyro_range': float,              # ±degrees/second
        'accel_range': float,             # ±g
        'gyro_noise': float,              # rad/s
        'accel_noise': float,             # m/s²
        'bias_stability': float,          # Bias drift
    },
    
    'external_gnss_ins': {
        'enabled': bool,
        'system_type': str,               # System model
        'position_accuracy': float,       # meters
        'attitude_accuracy': float,       # degrees
        'update_rate': int,               # Hz
        'rtk_availability': float,        # 0-1 probability
    }
}
```

## Utility Functions

### Coordinate Conversions

```python
def wgs84_to_utm(latitude: float, longitude: float, altitude: float) -> tuple
def utm_to_wgs84(easting: float, northing: float, zone: int, hemisphere: str) -> tuple
def euler_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray
def rotation_matrix_to_euler(R: np.ndarray) -> tuple
```

### Data Processing

```python
def filter_point_cloud(points: np.ndarray, min_range: float, max_range: float) -> np.ndarray
def downsample_point_cloud(points: np.ndarray, voxel_size: float) -> np.ndarray
def calculate_point_density(points: np.ndarray, grid_size: float) -> dict
def assess_data_quality(points: np.ndarray, trajectory: list) -> dict
```

### File I/O

```python
def load_pcd_file(filename: str) -> np.ndarray
def save_pcd_file(points: np.ndarray, filename: str, binary: bool = True)
def load_las_file(filename: str) -> np.ndarray
def save_las_file(points: np.ndarray, filename: str, coordinate_system: str = 'utm')
def load_lvx_file(filename: str) -> list
def save_lvx_file(frames: list, filename: str, format_version: str = 'lvx2')
```

## Error Handling

### Exception Classes

```python
class SimulationError(Exception):
    """Base exception for simulation errors"""
    pass

class ConfigurationError(SimulationError):
    """Configuration parameter errors"""
    pass

class HardwareError(SimulationError):
    """Hardware communication errors"""
    pass

class ProcessingError(SimulationError):
    """Data processing errors"""
    pass

class ExportError(SimulationError):
    """Data export errors"""
    pass
```

### Error Handling Example

```python
try:
    simulator = LiDARMotionSimulator(config)
    results = simulator.run_simulation()
except ConfigurationError as e:
    print(f"Configuration error: {e}")
except ProcessingError as e:
    print(f"Processing error: {e}")
except Exception as e:
    print(f"Unexpected error: {e}")
    import traceback
    traceback.print_exc()
```

## Performance Monitoring

### Performance Metrics

```python
performance_metrics = {
    'processing_time': {
        'total_seconds': float,
        'points_per_second': float,
        'frames_per_second': float,
    },
    'memory_usage': {
        'peak_mb': float,
        'average_mb': float,
        'current_mb': float,
    },
    'accuracy_metrics': {
        'motion_compensation_error': float,
        'coordinate_transform_error': float,
        'timing_synchronization_error': float,
    },
    'quality_metrics': {
        'point_density': float,
        'coverage_completeness': float,
        'noise_level': float,
    }
}
```

### Monitoring Functions

```python
def monitor_system_resources() -> dict
def calculate_processing_latency(start_time: float, end_time: float) -> float
def assess_memory_usage() -> dict
def validate_output_quality(data: dict) -> dict
```

## Integration APIs

### ROS Integration

```python
class ROSInterface:
    def __init__(self, node_name: str)
    def publish_point_cloud(self, points: np.ndarray, frame_id: str)
    def publish_odometry(self, trajectory_point: TrajectoryPoint)
    def subscribe_to_commands(self, callback: callable)
```

### Network Interface

```python
class NetworkInterface:
    def __init__(self, host: str, port: int)
    def start_udp_server(self)
    def send_lidar_packet(self, packet: bytes)
    def receive_commands(self) -> dict
```

## Examples

### Basic Usage Example

```python
import numpy as np
from livox_simulator import LiDARMotionSimulator, DataExporter

# Configure simulation
config = {
    'duration': 60.0,
    'trajectory_type': 'circular',
    'environment_complexity': 'medium',
    'enable_motion_compensation': True,
    'coordinate_system': 'utm'
}

# Run simulation
simulator = LiDARMotionSimulator(config)
results = simulator.run_simulation()

# Export results
exporter = DataExporter('./output')
exporter.export_all_formats(results)

print(f"Simulation completed: {len(results['frames'])} frames generated")
```

### Advanced Processing Example

```python
from livox_simulator import (
    LiDARMotionSimulator, 
    CoordinateTransformer, 
    MotionCompensator
)

# Advanced configuration
config = {
    'duration': 300.0,
    'sensors': {
        'lidar_builtin_imu': {'enabled': True, 'update_rate': 200},
        'external_gnss_ins': {'enabled': True, 'system_type': 'novatel'}
    },
    'processing': {
        'real_time_mode': True,
        'quality_monitoring': True,
        'adaptive_processing': True
    }
}

# Initialize components
simulator = LiDARMotionSimulator(config)
transformer = CoordinateTransformer(config['calibration'])
compensator = MotionCompensator(config['sensors']['lidar_builtin_imu'])

# Performance monitoring
def performance_callback(metrics):
    if metrics['latency_ms'] > 100:
        print(f"Warning: High latency {metrics['latency_ms']:.1f}ms")

simulator.set_performance_callback(performance_callback)

# Run simulation with custom processing
results = simulator.run_simulation()

# Post-processing analysis
quality_report = assess_data_quality(
    results['point_clouds'], 
    results['trajectory']
)
print(f"Data quality score: {quality_report['overall_score']}/100")
```

This API reference provides comprehensive documentation for all classes, methods, and functions available in the Livox motion compensation simulation framework. Use this reference to develop custom applications and integrate the simulator with your specific requirements.
# Livox Mid-70 Implementation Guide

## Overview

This guide provides comprehensive instructions for implementing and using the Livox Mid-70 simulation system based on extensive research of Livox technology.

## Quick Start

### 1. Basic Usage

```python
# Run standard simulation
python lidar_motion_compensation.py
```

### 2. Standard Features Usage

```python
# Create simulator
config = {
    'duration': 60.0,
    'trajectory_type': 'urban_circuit',
    'environment_complexity': 'urban',
    'enable_motion_compensation': True,
    'enable_network_sim': True,
    'coordinate_system': 'utm',
    'lvx_format': 'lvx2'
}

simulator = LiDARMotionSimulator(config)
results = simulator.run_simulation()
```

## Advanced Features

### 1. High-Frequency IMU Simulation (200Hz)

The simulator generates realistic IMU data at 200Hz matching Livox Mid-70 specifications:

```python
# IMU data structure
@dataclass
class IMUData:
    timestamp: int      # nanoseconds
    gyro_x: float      # rad/s
    gyro_y: float      # rad/s  
    gyro_z: float      # rad/s
    accel_x: float     # m/s²
    accel_y: float     # m/s²
    accel_z: float     # m/s²

# Access IMU data from results
imu_data = results['imu_data']
for imu_sample in imu_data:
    print(f"Time: {imu_sample.timestamp}, Gyro: [{imu_sample.gyro_x:.3f}, {imu_sample.gyro_y:.3f}, {imu_sample.gyro_z:.3f}]")
```

### 2. Motion Compensation

Advanced motion compensation using IMU data:

```python
# Enable motion compensation
config['enable_motion_compensation'] = True

# The simulator automatically applies IMU-based motion compensation
# to each point cloud frame during scanning
```

### 3. LVX Formats

Support for multiple LVX format versions:

```python
# Choose LVX format version
config['lvx_format'] = 'lvx2'  # Options: 'lvx', 'lvx2', 'lvx3'

# LVX writer with full compatibility
writer = LivoxLVXWriter(format_version='lvx2')
writer.write_lvx_file('output.lvx', frames_data, device_info)
```

### 4. Coordinate System Transformations

Multiple coordinate system support:

```python
# Available coordinate systems
config['coordinate_system'] = 'utm'  # Options: 'sensor', 'vehicle', 'local', 'utm', 'wgs84'

# Setup custom transformations
transformer = CoordinateTransformer()
transformer.set_transformation(
    CoordinateSystem.SENSOR, CoordinateSystem.VEHICLE,
    translation=[0, 0, 1.5],  # 1.5m above ground
    rotation=[0, 0, 0]        # No rotation
)
```

### 5. Network Protocol Simulation

Simulate Livox UDP network protocol:

```python
# Enable network simulation
config['enable_network_sim'] = True

# Network simulator sends UDP packets matching Livox protocol
network_sim = NetworkSimulator(host="127.0.0.1", data_port=65000)
network_sim.start_streaming()
```

## Configuration Options

### Complete Configuration Reference

```python
config = {
    # Simulation parameters
    'duration': 180.0,                    # Simulation duration (seconds)
    'lidar_fps': 10,                     # LiDAR frame rate (Hz) - fixed for Mid-70
    'imu_rate': 200,                     # IMU sample rate (Hz)
    'gps_rate': 5,                       # GPS update rate (Hz)
    'random_seed': 42,                   # Reproducibility seed
    
    # Livox Mid-70 specifications
    'fov_horizontal': 70.4,              # Horizontal FOV (degrees)
    'fov_vertical': 77.2,                # Vertical FOV (degrees)
    'range_max': 90.0,                   # Maximum range (meters)
    'range_min': 0.05,                   # Minimum range (meters)
    'points_per_frame': 100000,          # Points per second
    'angular_resolution': 0.28,          # Angular resolution (degrees)
    'point_accuracy': 0.02,              # Point accuracy (meters)
    
    # Motion parameters
    'max_speed': 15.0,                   # Maximum vehicle speed (m/s)
    'max_angular_vel': 0.5,              # Maximum angular velocity (rad/s)
    'trajectory_type': 'urban_circuit',   # Trajectory pattern
    
    # Noise parameters
    'gps_noise_std': 0.03,               # GPS noise standard deviation (m)
    'imu_accel_noise': 0.1,              # IMU acceleration noise (m/s²)
    'imu_gyro_noise': 0.01,              # IMU gyroscope noise (rad/s)
    'lidar_range_noise': 0.02,           # LiDAR range noise (m)
    
    # Environment parameters
    'environment_complexity': 'urban',   # Environment type
    'ground_height': 0.0,                # Ground plane height (m)
    'obstacle_density': 0.1,             # Obstacle density factor
    
    # Advanced features
    'enable_motion_compensation': True,   # Enable IMU motion compensation
    'enable_network_sim': False,         # Enable UDP network simulation
    'coordinate_system': 'utm',          # Target coordinate system
    'lvx_format': 'lvx2',               # LVX format version
    
    # Device configuration
    'device_info': {
        'lidar_sn': '3GGDJ6K00200101',   # LiDAR serial number
        'device_type': 1,                # Mid-70 device type
        'extrinsic_enable': True,        # Enable extrinsic parameters
        'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,  # Mounting orientation
        'x': 0.0, 'y': 0.0, 'z': 1.5    # Mounting position
    }
}
```

### Trajectory Types

```python
# Available trajectory patterns
trajectory_options = {
    'linear': 'Straight line with gentle curves',
    'circular': 'Circular path with constant radius',
    'figure_eight': 'Figure-8 pattern for comprehensive coverage',
    'urban_circuit': 'Realistic urban driving with stops and turns'
}
```

### Environment Types

```python
# Available environment complexities
environment_options = {
    'simple': 'Basic ground plane with simple obstacles',
    'medium': 'Moderate complexity with buildings and trees',
    'urban': 'Dense urban environment with detailed architecture',
    'highway': 'Highway environment with guard rails and signs',
    'industrial': 'Industrial setting with large structures and equipment'
}
```

## Output Files

- `motion_data.csv` - GPS/IMU motion data synchronized with frames
- `trajectory.csv` - Complete vehicle trajectory data
- `merged_aligned.pcd` - Motion-compensated point cloud
- `merged_aligned.las` - LAS format output
- `lidar_data.lvx` - LVX legacy format file (.lvx)
- `lidar_data.lvx2` - LVX2 format file (.lvx2) 
- `lidar_data.lvx3` - LVX3 format file (.lvx3)
- `imu_data_200hz.csv` - High-frequency IMU data (200Hz)
- `performance_stats.json` - Detailed performance metrics
- `simulation_config.json` - Complete simulation configuration
- `imu_analysis.png` - IMU data visualization
- `simulation_report.md` - Comprehensive analysis report

## Integration with Real Hardware

### 1. LVX File Compatibility

Generated LVX files are compatible with:
- Livox Viewer 0.11.0 (legacy .lvx format)
- Livox Viewer 2.x (.lvx2 format)
- Livox Viewer latest (.lvx3 format)

### 2. Motion Data Format

The motion data CSV format matches real sensor integration requirements:

```csv
frame_id,timestamp,gps_lat,gps_lon,gps_alt,imu_roll,imu_pitch,imu_yaw,vel_x,vel_y,vel_z
0,0.0,40.000270,74.000000,1.5,0.001,-0.002,0.785,2.1,1.8,0.0
```

### 3. Network Protocol

UDP packets match Livox SDK2 protocol:
- Data Port: 65000
- Command Port: 65001  
- IMU Port: 65002
- Packet structure matches official specification

## Performance Optimization

### 1. Memory Usage

```python
# Optimize for large datasets
config['points_per_frame'] = 50000  # Reduce if memory limited
config['duration'] = 30.0           # Shorter duration for testing
```

### 2. Processing Speed

```python
# Disable features for faster processing
config['enable_motion_compensation'] = False
config['enable_network_sim'] = False
config['environment_complexity'] = 'simple'
```

### 3. Output Size

```python
# Reduce output file sizes
config['lidar_fps'] = 5             # Lower frame rate
config['imu_rate'] = 100            # Lower IMU rate
```

## Troubleshooting

### Common Issues

1. **Import Error**: Features not available
   ```
   Solution: Ensure livox_simulator.py is in the same directory
   ```

2. **Memory Error**: Large datasets
   ```
   Solution: Reduce points_per_frame or duration in config
   ```

3. **LVX Compatibility**: File won't open in Livox Viewer
   ```
   Solution: Try different lvx_format ('lvx', 'lvx2', 'lvx3')
   ```

4. **UTM Conversion Error**: GPS coordinates invalid
   ```
   Solution: Install utm package: pip install utm
   ```

### Debug Mode

```python
# Enable detailed logging
import logging
logging.basicConfig(level=logging.DEBUG)

# Run with error handling
try:
    results = simulator.run_simulation()
except Exception as e:
    print(f"Simulation failed: {e}")
    import traceback
    traceback.print_exc()
```

## Research References

This implementation is based on comprehensive research of:

1. **Official Livox Documentation**
   - Mid-70 Technical Specifications
   - LVX File Format Specification v1.1
   - Livox SDK2 Protocol Documentation

2. **Hardware Analysis**
   - Point cloud data packet structure
   - IMU data integration methods
   - Network communication protocols

3. **Software Ecosystem**
   - Livox Viewer compatibility requirements
   - ROS integration patterns
   - Coordinate system transformations

## Future Enhancements

Planned improvements:
- Real-time streaming interface
- Multi-sensor synchronization
- Advanced SLAM integration
- Cloud processing support
- Machine learning point classification
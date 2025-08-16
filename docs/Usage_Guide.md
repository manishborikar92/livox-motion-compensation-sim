# Usage Guide

## Quick Start

### Basic Simulation

Run a basic simulation with default parameters:

```bash
# Activate virtual environment (if using one)
source livox_env/bin/activate

# Test installation first
python test_simulation.py

# Run full simulation with defaults (180 seconds, urban circuit)
python lidar_motion_compensation.py

# Check output directory
ls lidar_simulation_output/
```

### Custom Configuration

Create a custom configuration file:

```python
# config/custom_config.py
config = {
    'duration': 120.0,                    # 2-minute simulation
    'trajectory_type': 'urban_circuit',   # Realistic urban driving
    'environment_complexity': 'urban',    # Dense urban environment
    'enable_motion_compensation': True,   # Enable IMU compensation
    'coordinate_system': 'utm',          # UTM coordinates
    'lvx_format': 'lvx2'                 # Current LVX format
}
```

Run with custom configuration:
```bash
python lidar_motion_compensation.py --config config/custom_config.py
```

## Configuration Options

### Core Parameters

```python
config = {
    # === SIMULATION PARAMETERS ===
    'duration': 180.0,                    # Simulation duration (seconds)
    'random_seed': 42,                    # Reproducible results
    'output_directory': './output',       # Output directory path
    
    # === LIVOX MID-70 SPECIFICATIONS ===
    'lidar_specs': {
        'fov_horizontal': 70.4,           # Circular FOV (degrees)
        'fov_vertical': 77.2,             # Full vertical coverage
        'range_max': 90.0,                # Maximum range (meters)
        'range_min': 0.05,                # 5cm minimal detection
        'points_per_second': 100000,      # Maximum point rate
        'frame_rate': 10,                 # Fixed 10Hz
        'angular_resolution': 0.28,       # Uniform resolution (degrees)
        'point_accuracy': 0.02,           # ±2cm accuracy
    },
    
    # === MOTION PARAMETERS ===
    'trajectory': {
        'type': 'urban_circuit',          # Trajectory pattern
        'max_speed': 15.0,                # Maximum speed (m/s)
        'max_acceleration': 2.0,          # Maximum acceleration (m/s²)
        'max_angular_velocity': 0.3,      # Maximum angular velocity (rad/s)
    },
    
    # === ENVIRONMENT ===
    'environment': {
        'complexity': 'urban',            # Environment complexity
        'building_density': 0.4,          # 40% built area
        'vegetation_coverage': 0.15,      # 15% vegetation
        'dynamic_objects': True,          # Moving objects
    },
    
    # === ADVANCED FEATURES ===
    'enable_motion_compensation': True,   # IMU-based compensation
    'coordinate_system': 'utm',          # Output coordinate system
    'lvx_format': 'lvx2',               # LVX format version
}
```

### Trajectory Types

| Type | Description | Use Case |
|------|-------------|----------|
| `linear` | Straight line with gentle curves | Corridor mapping, highway scanning |
| `circular` | Circular path with constant radius | Object scanning, calibration |
| `figure_eight` | Figure-8 for comprehensive coverage | Area mapping, algorithm testing |
| `urban_circuit` | Realistic urban driving patterns | Autonomous vehicle development |
| `survey_grid` | Systematic grid pattern | Surveying, mapping applications |

### Environment Complexity

| Level | Description | Features |
|-------|-------------|----------|
| `simple` | Basic ground plane | Minimal obstacles, fast processing |
| `medium` | Moderate complexity | Buildings, trees, varied terrain |
| `urban` | Dense urban environment | Detailed architecture, vehicles |
| `highway` | Highway environment | Guard rails, signs, overpasses |
| `industrial` | Industrial setting | Large structures, equipment |

## Running Simulations

### Command Line Interface

```bash
# Basic usage
python lidar_motion_compensation.py

# With custom duration
python lidar_motion_compensation.py --duration 300

# With specific trajectory
python lidar_motion_compensation.py --trajectory figure_eight

# With custom output directory
python lidar_motion_compensation.py --output ./my_simulation

# Verbose output
python lidar_motion_compensation.py --verbose

# Help and options
python lidar_motion_compensation.py --help
```

### Programmatic Usage

```python
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from livox_simulator import LiDARMotionSimulator

# Create simulator with configuration
config = {
    'duration': 60.0,
    'trajectory_type': 'circular',
    'environment_complexity': 'medium',
    'enable_motion_compensation': True
}

simulator = LiDARMotionSimulator(config)

# Run simulation
results = simulator.run_simulation()

# Access results
print(f"Generated {len(results['frames'])} frames")
print(f"Total points: {len(results['point_cloud']):,}")
print(f"Processing time: {results['processing_time']:.2f}s")
print(f"Output saved to: {results['config']['output_directory']}")
```

## Output Files

### Standard Output Structure

```
lidar_simulation_output/
├── raw_data/
│   ├── lidar_frames.lvx2         # Raw LiDAR data
│   ├── imu_data_200hz.csv        # High-frequency IMU data
│   ├── trajectory.csv            # Vehicle trajectory
│   └── motion_data.csv           # Synchronized motion data
├── processed_data/
│   ├── motion_compensated.pcd    # Compensated point cloud
│   ├── global_coordinates.las    # Georeferenced LAS file
│   └── merged_dataset.ply        # Visualization format
├── analysis/
│   ├── performance_stats.json    # Processing metrics
│   ├── quality_report.html       # Quality assessment
│   └── accuracy_analysis.png     # Accuracy plots
└── configuration/
    ├── simulation_config.json    # Complete configuration
    └── system_info.txt           # System information
```

### File Descriptions

**Raw Data Files:**
- `lidar_frames.lvx2`: Native Livox format, compatible with Livox Viewer 2
- `imu_data_200hz.csv`: High-frequency IMU data (200Hz) for motion analysis
- `trajectory.csv`: Complete vehicle trajectory with timestamps
- `motion_data.csv`: Synchronized GPS/IMU data aligned with LiDAR frames

**Processed Data Files:**
- `motion_compensated.pcd`: Point cloud with motion compensation applied
- `global_coordinates.las`: Industry-standard LAS format with georeferencing
- `merged_dataset.ply`: 3D visualization format for viewers like CloudCompare

**Analysis Files:**
- `performance_stats.json`: Processing performance metrics and timing
- `quality_report.html`: Comprehensive quality assessment report
- `accuracy_analysis.png`: Visualization of accuracy metrics

## Data Analysis

### Loading and Viewing Data

**Point Cloud Data (PCD format):**
```python
import open3d as o3d
import numpy as np

# Load point cloud
pcd = o3d.io.read_point_cloud("motion_compensated.pcd")
print(f"Point cloud has {len(pcd.points)} points")

# Visualize
o3d.visualization.draw_geometries([pcd])

# Convert to numpy array
points = np.asarray(pcd.points)
colors = np.asarray(pcd.colors)
```

**LAS Data (Survey format):**
```python
import laspy

# Load LAS file
las_file = laspy.read("global_coordinates.las")
print(f"LAS file has {len(las_file.points)} points")

# Access coordinates
x = las_file.x
y = las_file.y
z = las_file.z
intensity = las_file.intensity
```

**Motion Data (CSV format):**
```python
import pandas as pd
import matplotlib.pyplot as plt

# Load motion data
motion_data = pd.read_csv("motion_data.csv")
print(motion_data.head())

# Plot trajectory
plt.figure(figsize=(10, 8))
plt.plot(motion_data['gps_lon'], motion_data['gps_lat'])
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.title('Vehicle Trajectory')
plt.show()

# Plot IMU data
plt.figure(figsize=(12, 6))
plt.subplot(2, 1, 1)
plt.plot(motion_data['timestamp'], motion_data['imu_roll'], label='Roll')
plt.plot(motion_data['timestamp'], motion_data['imu_pitch'], label='Pitch')
plt.plot(motion_data['timestamp'], motion_data['imu_yaw'], label='Yaw')
plt.legend()
plt.title('Vehicle Attitude')

plt.subplot(2, 1, 2)
plt.plot(motion_data['timestamp'], motion_data['vel_x'], label='Vel X')
plt.plot(motion_data['timestamp'], motion_data['vel_y'], label='Vel Y')
plt.legend()
plt.title('Vehicle Velocity')
plt.tight_layout()
plt.show()
```

### Performance Analysis

```python
import json

# Load performance statistics
with open("performance_stats.json", "r") as f:
    stats = json.load(f)

print(f"Processing time: {stats['processing_time']['total_seconds']:.2f}s")
print(f"Points per second: {stats['processing_time']['points_per_second']:,.0f}")
print(f"Memory usage: {stats['memory_usage']['peak_mb']:.1f} MB")
print(f"Accuracy metrics: {stats['accuracy_metrics']}")
```

## Advanced Usage

### Multi-Sensor Configuration

```python
# Configure multiple sensors
config = {
    'sensors': {
        'lidar_builtin_imu': {
            'enabled': True,
            'update_rate': 200,
            'noise_level': 'realistic'
        },
        'external_gnss_ins': {
            'enabled': True,
            'system_type': 'novatel_pwrpak7',
            'rtk_availability': 0.95,
            'accuracy': 0.01  # 1cm
        },
        'vehicle_odometry': {
            'enabled': True,
            'update_rate': 100,
            'drift_rate': 0.02  # 2% per km
        }
    },
    'sensor_fusion': {
        'algorithm': 'extended_kalman_filter',
        'prediction_horizon': 0.1
    }
}
```

### Custom Environment Generation

```python
# Define custom environment
environment_config = {
    'type': 'custom',
    'bounds': {
        'x_min': -100, 'x_max': 100,
        'y_min': -100, 'y_max': 100,
        'z_min': 0, 'z_max': 20
    },
    'objects': [
        {
            'type': 'building',
            'position': [10, 20, 0],
            'size': [15, 10, 8],
            'material': 'concrete'
        },
        {
            'type': 'tree',
            'position': [-5, 15, 0],
            'height': 12,
            'crown_radius': 4
        }
    ],
    'ground_plane': {
        'height': 0,
        'roughness': 0.1,
        'material': 'asphalt'
    }
}
```

### Real-time Processing Simulation

```python
# Enable real-time processing mode
config = {
    'real_time_mode': True,
    'processing_latency_target': 0.05,  # 50ms target
    'buffer_size': 10,  # Frame buffer
    'quality_monitoring': True,
    'adaptive_processing': True  # Adjust quality for performance
}

# Monitor real-time performance
simulator = LiDARMotionSimulator(config)
simulator.set_performance_callback(performance_monitor)
results = simulator.run_simulation()
```

## Integration with External Tools

### ROS Integration

```python
# ROS node for simulation data
import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

class SimulationPublisher:
    def __init__(self):
        rospy.init_node('livox_simulator')
        self.pc_pub = rospy.Publisher('/livox/lidar', PointCloud2, queue_size=1)
        self.odom_pub = rospy.Publisher('/odometry', Odometry, queue_size=1)
    
    def publish_frame(self, point_cloud, odometry):
        # Convert and publish point cloud
        pc_msg = self.convert_to_pointcloud2(point_cloud)
        self.pc_pub.publish(pc_msg)
        
        # Publish odometry
        odom_msg = self.convert_to_odometry(odometry)
        self.odom_pub.publish(odom_msg)
```

### MATLAB Integration

```matlab
% Load simulation data in MATLAB
motion_data = readtable('motion_data.csv');
point_cloud = pcread('motion_compensated.pcd');

% Visualize trajectory
figure;
plot3(motion_data.gps_lon, motion_data.gps_lat, motion_data.gps_alt);
xlabel('Longitude'); ylabel('Latitude'); zlabel('Altitude');
title('Vehicle Trajectory');

% Visualize point cloud
figure;
pcshow(point_cloud);
title('Motion Compensated Point Cloud');
```

## Troubleshooting

### Common Issues

**1. Memory Issues with Large Datasets**
```python
# Reduce memory usage
config['points_per_frame'] = 50000  # Reduce from 100k
config['duration'] = 60.0           # Shorter simulation
config['environment_complexity'] = 'simple'  # Less complex environment
```

**2. Processing Performance Issues**
```python
# Optimize for speed
config['enable_motion_compensation'] = False  # Disable for testing
config['real_time_processing'] = False        # Batch processing
config['output_formats'] = ['pcd']            # Single format only
```

**3. LVX File Compatibility**
```python
# Try different LVX formats
config['lvx_format'] = 'lvx'    # Legacy format
config['lvx_format'] = 'lvx2'   # Current format
config['lvx_format'] = 'lvx3'   # Latest format
```

**4. Coordinate System Issues**
```python
# Debug coordinate transformations
config['coordinate_system'] = 'sensor'  # Start with sensor frame
config['debug_transforms'] = True       # Enable debug output
```

**5. Visualization Issues**
```bash
# Test visualization backends
python -c "
try:
    import open3d; print('✅ Open3D available')
except ImportError:
    print('❌ Open3D not available')

try:
    import plotly; print('✅ Plotly available')
except ImportError:
    print('❌ Plotly not available')
"

# Use fallback backend
python visualize_3d.py data.pcd --backend matplotlib

# Reduce points for performance
python visualize_3d.py large_file.pcd --max-points 10000
```

### Debug Mode

```python
import logging

# Enable debug logging
logging.basicConfig(level=logging.DEBUG)

# Run with error handling
try:
    simulator = LiDARMotionSimulator(config)
    results = simulator.run_simulation()
    print("Simulation completed successfully")
except Exception as e:
    print(f"Simulation failed: {e}")
    import traceback
    traceback.print_exc()
```

## Performance Optimization

### System Optimization

```bash
# Optimize system for real-time processing
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Increase network buffers
echo 'net.core.rmem_max = 134217728' | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

### Code Optimization

```python
# Use vectorized operations
import numpy as np

# Efficient point cloud processing
points = np.array(point_list)  # Vectorize operations
transformed_points = transform_matrix @ points.T

# Memory-efficient processing
for chunk in np.array_split(large_point_cloud, 10):
    process_chunk(chunk)
```

## 3D Data Visualization

The framework includes comprehensive 3D visualization capabilities for analyzing simulation results and other 3D datasets.

### Basic Visualization

```bash
# Visualize simulation output
python visualize_3d.py lidar_simulation_output/processed_data/motion_compensated.pcd

# Use different backends
python visualize_3d.py data.pcd --backend open3d    # High-performance 3D viewer
python visualize_3d.py data.pcd --backend plotly    # Interactive web-based
python visualize_3d.py data.pcd --backend matplotlib # Traditional plotting
```

### Advanced Visualization Options

```bash
# Show detailed statistics
python visualize_3d.py data.las --stats

# Customize appearance
python visualize_3d.py data.pcd --point-size 2 --opacity 0.8 --colorscale Plasma

# Performance optimization
python visualize_3d.py large_dataset.pcd --max-points 50000

# Compare multiple datasets
python visualize_3d.py file1.pcd file2.las file3.ply --compare
```

### Programmatic Visualization

```python
from livox_simulator import DataVisualizer3D

# Create visualizer with specific backend
visualizer = DataVisualizer3D(backend='plotly')

# Load and visualize data
data = visualizer.load_data('simulation_output.pcd')
visualizer.visualize(data, 
                    point_size=2, 
                    opacity=0.8, 
                    title='Simulation Results')

# Generate comprehensive statistics
stats = visualizer.generate_statistics(data)
print(f"Dataset contains {stats['num_points']:,} points")
print(f"Bounding box: {stats['dimensions']}")

# Create comparison visualization
datasets = ['before.pcd', 'after.pcd']
titles = ['Before Processing', 'After Processing']
visualizer.create_comparison_view(datasets, titles)
```

### Supported File Formats

| Format | Extension | Type | Description |
|--------|-----------|------|-------------|
| **PCD** | .pcd | Point Cloud | PCL Point Cloud Data |
| **PLY** | .ply | Point Cloud/Mesh | Polygon File Format |
| **LAS/LAZ** | .las/.laz | Point Cloud | LiDAR data exchange format |
| **XYZ** | .xyz | Point Cloud | ASCII coordinate data |
| **CSV** | .csv | Point Cloud | Comma-separated values |
| **OBJ** | .obj | Mesh | Wavefront OBJ format |
| **STL** | .stl | Mesh | Stereolithography format |
| **NPY/NPZ** | .npy/.npz | Array | NumPy array format |

## Next Steps

After mastering basic usage:

1. **Custom Applications**: Develop application-specific configurations
2. **Real Hardware Integration**: Connect to actual Livox Mid-70 hardware
3. **Algorithm Development**: Use simulation for algorithm testing
4. **Performance Tuning**: Optimize for your specific use case
5. **Advanced Visualization**: Explore 3D data analysis and comparison tools

Refer to the [API Reference](API_Reference.md) for detailed programming interface documentation.
# Livox Mid-70 Motion Compensation Simulation

[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://python.org)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-Linux%20%7C%20Windows%20%7C%20macOS-lightgrey.svg)](https://github.com)

A comprehensive **pure Python** simulation framework for Livox Mid-70 LiDAR systems with motion compensation and global coordinate alignment capabilities.

## 🎯 Overview

This project provides a complete **pure Python** simulation environment for testing and validating Livox Mid-70 LiDAR systems in mobile mapping applications. The simulator generates realistic point cloud data with motion compensation using IMU data and supports multiple output formats including native LVX files.

**🐍 Pure Python Implementation**: No external hardware dependencies, C++ compilation, or SDK installation required - everything runs in Python for maximum portability and ease of use.

## ✨ Key Features

- **🔬 Livox Mid-70 Simulation**: Accurate modeling of Mid-70 specifications and scanning patterns
- **📐 Motion Compensation**: IMU-based motion compensation algorithms (200Hz simulation)
- **🌍 Global Coordinates**: Support for UTM, WGS84, and local coordinate systems
- **📁 Multiple Formats**: Export to LVX2, PCD, LAS, PLY, XYZ formats
- **⚡ Real-time Processing**: Simulated real-time data streaming with performance monitoring
- **📊 Comprehensive Analysis**: Performance metrics and quality assessment
- **🎨 Advanced Visualization**: Multi-backend 3D visualization (Matplotlib, Open3D, Plotly)
- **🔍 Format Support**: Comprehensive support for point clouds, meshes, and array formats
- **🐍 Pure Python**: No compilation required - runs entirely in Python 3.8+

## 🚀 Quick Start

### Installation

```bash
# Clone the repository
git clone https://github.com/your-username/livox-motion-compensation-sim.git
cd livox-motion-compensation-sim

# Create virtual environment (recommended)
python3 -m venv livox_env
source livox_env/bin/activate  # On Windows: livox_env\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Test installation
python test_simulation.py
```

### Basic Usage

```bash
# Run basic simulation (180 seconds, urban circuit)
python lidar_motion_compensation.py

# Run with custom parameters
python lidar_motion_compensation.py --duration 300 --trajectory circular --verbose

# Visualize results with advanced 3D viewer
python visualize_3d.py lidar_simulation_output/processed_data/motion_compensated.pcd

# Compare multiple datasets
python visualize_3d.py *.pcd --compare --backend plotly

# View results
ls lidar_simulation_output/
```

### Programmatic Usage

```python
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from livox_simulator import LiDARMotionSimulator

# Configure simulation
config = {
    'duration': 60.0,
    'trajectory_type': 'circular',
    'environment_complexity': 'urban',
    'enable_motion_compensation': True,
    'coordinate_system': 'utm'
}

# Run simulation
simulator = LiDARMotionSimulator(config)
results = simulator.run_simulation()

print(f"Generated {len(results['frames'])} frames")
print(f"Total points: {len(results['point_cloud']):,}")
```

## 📋 Requirements

### System Requirements
- **Python 3.8+** (Pure Python implementation)
- **RAM**: 8GB minimum (16GB recommended for large datasets)
- **Storage**: 2GB free space for output files
- **OS**: Linux, Windows, or macOS

### Python Dependencies
All dependencies are Python packages installable via pip:

- **Core Libraries**: NumPy, SciPy, Matplotlib, Pandas
- **Point Cloud Processing**: Open3D
- **File Formats**: Laspy (LAS), UTM (coordinate conversion)
- **Utilities**: PyYAML, tqdm, psutil
- **Visualization**: Plotly (interactive), Trimesh (meshes), Kaleido (export)

**No compilation required** - everything is pure Python!

## 🏗️ Architecture

The simulation framework consists of several key components:

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Environment   │───▶│   LiDAR Sensor   │───▶│ Motion Compensation │
│   Generator     │    │   Simulation     │    │   Algorithm     │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                                │                        │
┌─────────────────┐             │               ┌─────────────────┐
│ Trajectory      │─────────────┘               │ Coordinate      │
│ Generator       │                             │ Transformation  │
└─────────────────┘                             └─────────────────┘
                                                          │
┌─────────────────┐    ┌──────────────────┐             │
│ Data Export     │◀───│ Global Alignment │◀────────────┘
│ (LVX/PCD/LAS)   │    │ & Georeferencing │
└─────────────────┘    └──────────────────┘
```

### Core Components

1. **LiDARMotionSimulator**: Main simulation engine
2. **CoordinateTransformer**: Multi-frame coordinate transformations
3. **MotionCompensator**: IMU-based motion compensation
4. **DataExporter**: Multi-format export capabilities

## 🔧 Configuration Options

### Trajectory Types
- `linear`: Straight line with gentle curves
- `circular`: Circular path with constant radius
- `figure_eight`: Figure-8 for comprehensive coverage
- `urban_circuit`: Realistic urban driving patterns
- `survey_grid`: Systematic grid pattern

### Environment Complexity
- `simple`: Basic ground plane, fast processing
- `medium`: Moderate complexity with buildings and trees
- `urban`: Dense urban environment with detailed architecture
- `highway`: Highway environment with guard rails and signs
- `industrial`: Industrial setting with large structures

### Output Formats
- **LVX2**: Native Livox format (compatible with Livox Viewer 2)
- **PCD**: Point Cloud Data format (PCL compatible)
- **LAS**: Industry-standard surveying format
- **PLY**: 3D visualization format
- **XYZ**: Simple ASCII point format

## 🎨 Advanced 3D Visualization

The framework includes a comprehensive 3D visualization system supporting multiple backends and file formats.

### Supported Visualization Backends
- **Open3D**: High-performance 3D visualization with advanced features
- **Plotly**: Interactive web-based 3D plots with zoom, pan, and hover
- **Matplotlib**: Traditional scientific plotting with 3D capabilities

### Supported 3D Data Formats
- **Point Clouds**: PCD, PLY, LAS/LAZ, XYZ, CSV, NPY/NPZ
- **Meshes**: OBJ, STL, OFF
- **Arrays**: NumPy arrays and compressed archives

### Visualization Examples

```bash
# Basic point cloud visualization
python visualize_3d.py data.pcd

# Interactive visualization with Plotly
python visualize_3d.py data.las --backend plotly --stats

# High-quality Open3D visualization
python visualize_3d.py mesh.ply --backend open3d

# Compare multiple datasets
python visualize_3d.py file1.pcd file2.las file3.ply --compare

# Customize visualization
python visualize_3d.py data.pcd --point-size 2 --opacity 0.8 --colorscale Plasma

# Process with wildcards
python visualize_3d.py *.pcd --compare --max-points 10000
```

### Programmatic Visualization

```python
from livox_simulator import DataVisualizer3D

# Create visualizer
visualizer = DataVisualizer3D(backend='plotly')

# Load and visualize data
data = visualizer.load_data('point_cloud.pcd')
visualizer.visualize(data, point_size=2, opacity=0.8)

# Generate statistics
stats = visualizer.generate_statistics(data)
print(f"Points: {stats['num_points']:,}")

# Compare multiple datasets
visualizer.create_comparison_view(['data1.pcd', 'data2.las'])
```

## 📊 Livox Mid-70 Specifications

The simulator accurately models the Livox Mid-70 LiDAR specifications:

| Parameter | Specification | Notes |
|-----------|---------------|-------|
| **Detection Range** | 90m @ 10% / 130m @ 20% reflectivity | Environmental conditions dependent |
| **Accuracy** | ±2cm (1σ @ 25m) | At 25°C, 30% reflectivity |
| **Field of View** | 70.4° circular | Zero blind spots |
| **Frame Rate** | 10Hz (fixed) | Hardware limitation |
| **Point Rate** | Up to 100,000 points/second | Maximum theoretical |
| **Minimal Range** | 5cm | Critical for close-range |
| **Built-in IMU** | 6-axis, 200Hz | Motion compensation |

## 📁 Output Structure

```
lidar_simulation_output/
├── raw_data/
│   ├── lidar_frames.lvx2         # Native Livox format
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

## 🎮 Command Line Interface

### Simulation Commands
```bash
# Basic usage
python lidar_motion_compensation.py

# Custom duration and trajectory
python lidar_motion_compensation.py --duration 300 --trajectory circular

# Specify output directory
python lidar_motion_compensation.py --output ./my_simulation

# Disable motion compensation
python lidar_motion_compensation.py --no-motion-compensation

# Different environment complexity
python lidar_motion_compensation.py --environment urban --verbose

# Help and all options
python lidar_motion_compensation.py --help
```

### Visualization Commands
```bash
# Visualize point cloud
python visualize_3d.py data.pcd

# Interactive visualization
python visualize_3d.py data.las --backend plotly --stats

# Compare multiple files
python visualize_3d.py *.pcd --compare

# Custom visualization settings
python visualize_3d.py data.ply --point-size 3 --opacity 0.6 --max-points 100000

# Help and all options
python visualize_3d.py --help
```

## 🔬 Applications

### Mobile Mapping
- Vehicle-mounted surveying and mapping
- Infrastructure inspection and monitoring
- High-definition map generation

### Autonomous Vehicles
- Perception system development
- Localization algorithm testing
- Sensor fusion validation

### Research and Development
- Algorithm prototyping and validation
- Performance benchmarking
- Educational and training applications

### Robotics
- Mobile robot localization and mapping
- SLAM algorithm development
- Navigation system testing

## 📚 Documentation

Comprehensive documentation is available in the `docs/` directory:

- **[System Overview](docs/System_Overview.md)** - Architecture and components
- **[Hardware Specifications](docs/Hardware_Specifications.md)** - Livox Mid-70 technical details
- **[Software Setup](docs/Software_Setup.md)** - Installation and configuration
- **[Usage Guide](docs/Usage_Guide.md)** - Running simulations and processing data
- **[API Reference](docs/API_Reference.md)** - Programming interface documentation

## 🧪 Testing

```bash
# Quick installation test
python test_simulation.py

# Run short test simulation
python lidar_motion_compensation.py --duration 10 --quiet

# Test visualization capabilities
python examples/visualization_demo.py

# Verify all dependencies
python -c "import numpy, scipy, matplotlib, open3d, laspy, utm; print('All dependencies OK')"
```

## 🔧 Advanced Usage

### Custom Configuration File

```python
# config/custom_config.py
config = {
    'duration': 300.0,
    'trajectory_type': 'figure_eight',
    'environment_complexity': 'urban',
    'enable_motion_compensation': True,
    'coordinate_system': 'utm',
    'points_per_second': 50000,  # Reduced for faster processing
    'imu_update_rate': 200,
    'output_directory': './custom_output'
}
```

### Performance Monitoring

```python
def performance_callback(metrics):
    print(f"Frame {metrics['frame_id']}: {metrics['points_generated']} points, "
          f"Memory: {metrics['memory_mb']:.1f} MB")

simulator.set_performance_callback(performance_callback)
```

### Multi-Sensor Configuration

```python
config = {
    'sensors': {
        'lidar_builtin_imu': {'enabled': True, 'update_rate': 200},
        'external_gnss_ins': {'enabled': True, 'system_type': 'novatel'},
        'vehicle_odometry': {'enabled': True, 'update_rate': 100}
    },
    'sensor_fusion': {
        'algorithm': 'extended_kalman_filter',
        'prediction_horizon': 0.1
    }
}
```

## 🐛 Troubleshooting

### Common Issues

**Memory Issues:**
```python
# Reduce memory usage
config['points_per_second'] = 50000  # Reduce from 100k
config['duration'] = 60.0            # Shorter simulation
```

**Performance Issues:**
```python
# Optimize for speed
config['environment_complexity'] = 'simple'
config['enable_motion_compensation'] = False  # For testing
```

**Import Errors:**
```bash
# Ensure all dependencies are installed
pip install -r requirements.txt

# Verify installation
python test_simulation.py
```

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request. For major changes, please open an issue first to discuss what you would like to change.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **Livox Technology** for the Mid-70 LiDAR specifications
- **Open3D** for point cloud processing capabilities
- **Python Scientific Computing Community** for the excellent libraries

## 📞 Support

For questions and support:

- **Issues**: Open an issue on GitHub for bug reports and feature requests
- **Discussions**: Use GitHub Discussions for general questions and community support
- **Documentation**: Check the comprehensive documentation in the `docs/` directory

## 🔗 Related Projects

- [Livox SDK](https://github.com/Livox-SDK/Livox-SDK) - Official Livox SDK for hardware integration
- [Open3D](https://github.com/isl-org/Open3D) - Point cloud processing library
- [PCL](https://pointclouds.org/) - Point Cloud Library

---

**🎉 Ready to simulate? Start with `python test_simulation.py` to verify your installation, then run `python lidar_motion_compensation.py` for your first simulation!**
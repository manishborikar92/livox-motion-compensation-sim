# Livox Motion Compensation Simulation

A comprehensive **Python-based** simulation framework for Livox Mid-70 LiDAR systems with motion compensation and global coordinate alignment capabilities.

## Overview

This project provides a complete **pure Python** simulation environment for testing and validating Livox Mid-70 LiDAR systems in mobile mapping applications. The simulator generates realistic point cloud data with motion compensation using IMU data and supports multiple output formats including native LVX files.

**🐍 Pure Python Implementation**: No external hardware dependencies or C++ compilation required - everything runs in Python for maximum portability and ease of use.

## Key Features

- **Livox Mid-70 Simulation**: Accurate modeling of Mid-70 specifications and scanning patterns
- **Motion Compensation**: IMU-based motion compensation algorithms (200Hz simulation)
- **Global Coordinates**: Support for UTM, WGS84, and local coordinate systems
- **Multiple Formats**: Export to LVX2, PCD, LAS, PLY, XYZ formats
- **Real-time Processing**: Simulated real-time data streaming with performance monitoring
- **Comprehensive Analysis**: Performance metrics and quality assessment
- **Pure Python**: No compilation required - runs entirely in Python 3.8+

## Quick Start

```bash
# Clone the repository
git clone <repository-url>
cd livox-motion-compensation-sim

# Install Python dependencies
pip install -r requirements.txt

# Run basic simulation (180 seconds, urban circuit)
python lidar_motion_compensation.py

# Run with custom parameters
python lidar_motion_compensation.py --duration 300 --trajectory circular --verbose

# View results in output directory
ls lidar_simulation_output/
```

## Documentation

- [System Overview](System_Overview.md) - Architecture and components
- [Hardware Specifications](Hardware_Specifications.md) - Livox Mid-70 technical details
- [Software Setup](Software_Setup.md) - Installation and configuration
- [Usage Guide](Usage_Guide.md) - Running simulations and processing data
- [API Reference](API_Reference.md) - Programming interface documentation

## Applications

- **Mobile Mapping**: Vehicle-mounted surveying and mapping
- **Autonomous Vehicles**: Navigation system development
- **Robotics**: Mobile robot localization and mapping
- **Research**: Algorithm development and validation

## Requirements

- **Python 3.8+** (Pure Python implementation)
- **Core Libraries**: NumPy, SciPy, Matplotlib, Pandas
- **Point Cloud Processing**: Open3D
- **File Formats**: Laspy (LAS), UTM (coordinate conversion)
- **Optional**: ROS/ROS2 for integration testing

**No compilation required** - all dependencies are Python packages installable via pip.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Support

For questions and support, please open an issue on GitHub or contact the development team.
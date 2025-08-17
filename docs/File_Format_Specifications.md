# File Format Specifications

This document provides comprehensive specifications for all file formats supported by the Livox Mid-70 Motion Compensation Simulation framework.

## Overview

The simulation framework supports multiple industry-standard and proprietary file formats for point cloud data, trajectory information, and sensor data. Each format has specific use cases and data structure requirements.

## Point Cloud Formats

### 1. PCD (Point Cloud Data) Format

**Extension**: `.pcd`  
**Type**: Point Cloud  
**Standard**: Point Cloud Library (PCL)  
**Binary/ASCII**: Both supported  

#### Data Structure
```
# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH [number_of_points]
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS [number_of_points]
DATA binary/ascii
[point_data]
```

#### Point Data Layout
- **x, y, z**: 32-bit float coordinates (meters)
- **intensity**: 32-bit float intensity value (0-255)
- **Memory Layout**: 16 bytes per point (4 × 4-byte floats)

#### Implementation Details
- Uses Open3D library for I/O operations
- Supports both binary and ASCII formats
- Intensity mapped to RGB colors for visualization
- Compatible with PCL, CloudCompare, and other tools

### 2. LAS (LiDAR Data Exchange) Format

**Extension**: `.las`, `.laz` (compressed)  
**Type**: Point Cloud  
**Standard**: ASPRS (American Society for Photogrammetry and Remote Sensing)  
**Binary/ASCII**: Binary only  

#### Data Structure
```c
// LAS Header (227 bytes minimum)
struct LASHeader {
    char file_signature[4];        // "LASF"
    uint16_t file_source_id;
    uint16_t global_encoding;
    uint32_t guid_data_1;
    uint16_t guid_data_2;
    uint16_t guid_data_3;
    uint8_t guid_data_4[8];
    uint8_t version_major;         // 1
    uint8_t version_minor;         // 4
    char system_identifier[32];
    char generating_software[32];
    uint16_t file_creation_day;
    uint16_t file_creation_year;
    uint16_t header_size;
    uint32_t offset_to_point_data;
    uint32_t number_of_vlr;
    uint8_t point_data_format;     // 3 (with RGB and time)
    uint16_t point_data_record_length;
    uint32_t number_of_points;
    uint32_t number_of_points_by_return[5];
    double x_scale_factor;
    double y_scale_factor;
    double z_scale_factor;
    double x_offset;
    double y_offset;
    double z_offset;
    double max_x, min_x;
    double max_y, min_y;
    double max_z, min_z;
};

// Point Data Record Format 3 (34 bytes)
struct LASPoint {
    int32_t x;                     // Scaled integer
    int32_t y;                     // Scaled integer
    int32_t z;                     // Scaled integer
    uint16_t intensity;
    uint8_t return_info;           // Return number, number of returns, scan direction, edge of flight line
    uint8_t classification;
    int8_t scan_angle_rank;
    uint8_t user_data;
    uint16_t point_source_id;
    double gps_time;               // GPS time of week
    uint16_t red;                  // RGB color
    uint16_t green;
    uint16_t blue;
};
```

#### Coordinate System Support
- **UTM**: Universal Transverse Mercator
- **Geographic**: WGS84 latitude/longitude
- **State Plane**: US State Plane Coordinate System
- **Custom**: User-defined coordinate reference systems

#### Implementation Details
- Uses laspy library for Python I/O
- Supports coordinate reference system (CRS) metadata
- Automatic scaling and offset for precision
- Compatible with GIS software (QGIS, ArcGIS, etc.)

### 3. PLY (Polygon File Format)

**Extension**: `.ply`  
**Type**: Point Cloud/Mesh  
**Standard**: Stanford University  
**Binary/ASCII**: Both supported  

#### Data Structure
```
ply
format binary_little_endian 1.0
comment Created by Livox Mid-70 Simulator
element vertex [number_of_points]
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
property float intensity
end_header
[binary_point_data]
```

#### Point Data Layout
- **x, y, z**: 32-bit float coordinates
- **red, green, blue**: 8-bit color values (0-255)
- **intensity**: 32-bit float intensity value
- **Memory Layout**: 19 bytes per point

### 4. XYZ (ASCII Point Cloud)

**Extension**: `.xyz`  
**Type**: Point Cloud  
**Standard**: Simple ASCII format  
**Binary/ASCII**: ASCII only  

#### Data Structure
```
# X Y Z Intensity
1.234567 2.345678 3.456789 128
1.234568 2.345679 3.456790 129
...
```

#### Implementation Details
- Simple space-separated values
- Optional intensity column
- Human-readable format
- Large file sizes due to ASCII encoding

### 5. LVX (Livox Data Format)

**Extensions**: `.lvx`, `.lvx2`, `.lvx3`  
**Type**: Point Cloud (Proprietary)  
**Standard**: Livox Technology  
**Binary/ASCII**: Binary only  

#### LVX2 Data Structure
```c
// LVX2 File Header
struct LVX2Header {
    char signature[10];            // "livox_tech"
    uint32_t version;              // 2 for LVX2
    uint32_t magic_code;           // 0xAC0EA767
    uint32_t device_count;
};

// Device Information
struct LVX2DeviceInfo {
    uint8_t device_type;           // 1 = Mid-70
    char serial_number[16];
    uint8_t lidar_type;
    uint8_t reserved[7];
};

// Frame Header
struct LVX2FrameHeader {
    uint64_t timestamp;            // Nanoseconds since epoch
    uint32_t point_count;
    uint8_t device_id;
    uint8_t reserved[3];
};

// Point Data (Cartesian)
struct LVX2Point {
    int32_t x;                     // Millimeters
    int32_t y;                     // Millimeters  
    int32_t z;                     // Millimeters
    uint8_t intensity;             // 0-255
    uint8_t tag;                   // Point tag
    uint16_t reserved;
};
```

#### LVX Format Comparison

| Feature | LVX (v1) | LVX2 | LVX3 |
|---------|----------|------|------|
| **Max File Size** | 2GB | 4GB | Unlimited |
| **Timestamp Precision** | Microseconds | Nanoseconds | Nanoseconds |
| **Multi-device Support** | Limited | Yes | Enhanced |
| **Compression** | None | Optional | Advanced |
| **Metadata** | Basic | Extended | Comprehensive |
| **Compatibility** | Legacy viewers | Livox Viewer 2 | Latest tools |

#### Implementation Details
- Native Livox format for maximum compatibility
- Hardware-synchronized timestamps
- Supports multiple LiDAR devices
- Optimized for Livox ecosystem tools

## Trajectory and Motion Data Formats

### 1. CSV (Comma-Separated Values)

**Extension**: `.csv`  
**Type**: Tabular Data  
**Standard**: RFC 4180  

#### Structure for Motion Data
```csv
timestamp,gps_lat,gps_lon,gps_alt,imu_roll,imu_pitch,imu_yaw,vel_x,vel_y,vel_z,accel_x,accel_y,accel_z
1234567890.123,40.7128,-74.0060,10.5,0.01,-0.02,1.57,5.2,0.1,0.0,0.5,-0.1,9.81
```

#### Columns Description
- **timestamp**: Unix timestamp (seconds)
- **gps_lat/lon/alt**: WGS84 coordinates
- **imu_roll/pitch/yaw**: Euler angles (radians)
- **vel_x/y/z**: Velocity components (m/s)
- **accel_x/y/z**: Acceleration components (m/s²)

### 2. JSON (JavaScript Object Notation)

**Extension**: `.json`  
**Type**: Structured Data  
**Standard**: RFC 7159  

#### Structure for Trajectory Data
```json
{
  "trajectory": [
    {
      "timestamp": 1234567890.123,
      "position": {
        "x": 123.456,
        "y": 234.567,
        "z": 345.678
      },
      "orientation": {
        "roll": 0.01,
        "pitch": -0.02,
        "yaw": 1.57
      },
      "velocity": {
        "x": 5.2,
        "y": 0.1,
        "z": 0.0
      },
      "gnss": {
        "latitude": 40.7128,
        "longitude": -74.0060,
        "altitude": 10.5,
        "fix_type": "RTK_FIXED",
        "satellites": 12,
        "hdop": 0.8
      }
    }
  ]
}
```

### 3. KML (Keyhole Markup Language)

**Extension**: `.kml`  
**Type**: Geographic Data  
**Standard**: OGC KML 2.2  

#### Structure for Trajectory Visualization
```xml
<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>Vehicle Trajectory</name>
    <Placemark>
      <name>Path</name>
      <LineString>
        <coordinates>
          -74.0060,40.7128,10.5
          -74.0061,40.7129,10.6
        </coordinates>
      </LineString>
    </Placemark>
  </Document>
</kml>
```

## Array and Mesh Formats

### 1. NPY/NPZ (NumPy Arrays)

**Extensions**: `.npy`, `.npz`  
**Type**: Numerical Arrays  
**Standard**: NumPy  

#### NPY Structure (Single Array)
- Binary format for single NumPy arrays
- Preserves data type and shape information
- Efficient for large numerical datasets

#### NPZ Structure (Multiple Arrays)
- ZIP archive containing multiple .npy files
- Allows storage of multiple related arrays
- Compressed format option available

### 2. OBJ (Wavefront OBJ)

**Extension**: `.obj`  
**Type**: 3D Mesh  
**Standard**: Wavefront Technologies  

#### Basic Structure
```
# Wavefront OBJ file
v 1.0 2.0 3.0    # Vertex
vn 0.0 1.0 0.0   # Normal
vt 0.5 0.5       # Texture coordinate
f 1 2 3          # Face
```

### 3. STL (Stereolithography)

**Extension**: `.stl`  
**Type**: 3D Mesh  
**Standard**: 3D Systems  

#### ASCII Structure
```
solid name
  facet normal 0.0 0.0 1.0
    outer loop
      vertex 0.0 0.0 0.0
      vertex 1.0 0.0 0.0
      vertex 0.0 1.0 0.0
    endloop
  endfacet
endsolid name
```

## Format Selection Guidelines

### Use Case Recommendations

| Application | Recommended Format | Reason |
|-------------|-------------------|---------|
| **GIS Analysis** | LAS/LAZ | Industry standard, CRS support |
| **Visualization** | PLY, PCD | Color support, wide compatibility |
| **Livox Ecosystem** | LVX2/LVX3 | Native format, full feature support |
| **Data Exchange** | PCD, LAS | Universal compatibility |
| **Web Applications** | JSON, PLY | Lightweight, web-friendly |
| **Scientific Analysis** | NPY/NPZ | Efficient numerical processing |
| **Simple Viewing** | XYZ | Human-readable, minimal |

### Performance Characteristics

| Format | File Size | Load Speed | Compatibility | Features |
|--------|-----------|------------|---------------|----------|
| **LVX2** | Small | Fast | Livox tools | Full metadata |
| **LAS** | Medium | Medium | GIS software | CRS, standards |
| **PCD** | Medium | Fast | PCL ecosystem | Colors, normals |
| **PLY** | Medium | Medium | General 3D | Flexible schema |
| **XYZ** | Large | Slow | Universal | Simple format |
| **NPY** | Small | Very Fast | Python/NumPy | Numerical data |

## Implementation Status

### Currently Implemented
- ✅ PCD (Point Cloud Data)
- ✅ LAS (LiDAR Data Exchange)
- ✅ PLY (Polygon File Format)
- ✅ XYZ (ASCII Point Cloud)
- ✅ LVX2 (Livox Format v2) - Basic implementation
- ✅ CSV (Trajectory data)
- ✅ JSON (Configuration and metadata)
- ✅ KML (Geographic visualization)

### Planned Enhancements
- 🔄 LVX3 (Latest Livox format)
- 🔄 Enhanced LVX2 with full metadata
- 🔄 LAZ (Compressed LAS)
- 🔄 NPY/NPZ optimization
- 🔄 Advanced mesh format support

### Format Validation

Each format implementation includes:
- Data integrity checks
- Format specification compliance
- Error handling and recovery
- Metadata preservation
- Performance optimization

## References

1. **PCD Format**: Point Cloud Library Documentation
2. **LAS Format**: ASPRS LAS Specification v1.4
3. **PLY Format**: Stanford PLY Format Specification
4. **LVX Format**: Livox SDK Documentation
5. **KML Format**: OGC KML 2.2 Specification
6. **NumPy Formats**: NumPy Array Format Specification
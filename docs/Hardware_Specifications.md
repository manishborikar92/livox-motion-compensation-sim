# Hardware Specifications

## Livox Mid-70 LiDAR Sensor

### Core Specifications

| Parameter | Specification | Notes |
|-----------|---------------|-------|
| **Detection Range** | 90m @ 10% reflectivity<br>130m @ 20% reflectivity | Environmental conditions dependent |
| **Accuracy** | ±2cm (1σ @ 25m) | At 25°C, 30% reflectivity |
| **Precision** | ≤2cm (1σ @ 25m) | Repeatability measurement |
| **Field of View** | 70.4° circular | Zero blind spots |
| **Angular Resolution** | 0.28° × 0.28° | Uniform across FOV |
| **Frame Rate** | 10Hz (fixed) | Hardware limitation |
| **Point Rate** | Up to 100,000 points/second | Maximum theoretical |
| **Wavelength** | 905nm | Class 1 laser safety |
| **Minimal Detection Range** | **5cm** | Critical for close-range |
| **Latest Firmware** | 03.08.0000 | Current stable release |

### Physical Specifications

| Parameter | Value | Details |
|-----------|--------|---------|
| **Power Consumption** | 8W normal / 40W cold start | Self-heating mode -20°C to 0°C |
| **Operating Temperature** | -20°C to +65°C | Full operational range |
| **IP Rating** | IP67 | Sensor unit only |
| **Weight** | 760g | Sensor unit only |
| **Dimensions** | 88mm × 69mm × 102mm | Compact form factor |
| **Mounting** | 1/4"-20 thread | Standard tripod compatible |
| **Interface** | Ethernet (100Mbps) | Power over Ethernet Plus |

### Built-in IMU

| Parameter | Specification | Purpose |
|-----------|---------------|---------|
| **Type** | 6-axis IMU | 3-axis gyro + 3-axis accel |
| **Update Rate** | 200Hz | 5ms sampling interval |
| **Gyroscope Range** | ±2000°/s | Angular velocity |
| **Accelerometer Range** | ±16g | Linear acceleration |
| **Synchronization** | Hardware synchronized | Motion compensation |
| **Coordinate Frame** | Right-handed | X: forward, Y: left, Z: up |

## Power Supply Requirements

### Option 1: Power over Ethernet Plus (PoE+)
- **Standard**: IEEE 802.3at
- **Power**: 25.5W maximum
- **Voltage**: 48V DC
- **Cable**: Cat6/Cat6a, up to 100m
- **Advantages**: Single cable solution

### Option 2: External DC Power
- **Input Voltage**: 10V to 15V DC
- **Recommended**: 12V DC
- **Current**: 5A minimum (cold weather: 4A continuous)
- **Protection**: Voltage regulation, surge protection
- **Advantages**: Flexible power source options

### Option 3: Vehicle Integration
- **Input**: 12V/24V vehicle systems
- **Converter**: DC-DC regulated output
- **Protection**: EMI filtering, isolation
- **Integration**: CAN bus monitoring optional

## Communication Protocols

### Network Configuration
- **Primary Interface**: Ethernet UDP
- **Data Port**: 65000 (point cloud data)
- **Command Port**: 65001 (device control)
- **IMU Port**: 65002 (motion data)
- **Packet Rate**: ~1000 packets/second
- **Maximum Packet Size**: 1400 bytes

### Data Packet Structure
```c
struct LivoxPointPacket {
    uint8_t  version;           // Protocol version
    uint8_t  slot_id;           // Slot identifier
    uint8_t  lidar_id;          // LiDAR identifier
    uint8_t  reserved;          // Reserved byte
    uint32_t status_code;       // Device status
    uint8_t  timestamp_type;    // Timestamp format
    uint8_t  data_type;         // Data type (Cartesian)
    uint64_t timestamp;         // Nanosecond timestamp
    LivoxPoint points[];        // Point data array
};
```

## GNSS/INS Integration Options

### Survey-Grade Systems (Sub-centimeter accuracy)

#### NovAtel PwrPak7-E1
- **Position Accuracy**: 1cm + 1ppm RTK
- **Attitude Accuracy**: 0.008° roll/pitch, 0.015° heading
- **Update Rate**: 200Hz position, 400Hz IMU
- **Operating Temperature**: -40°C to +75°C
- **Price Range**: $12,000-18,000

#### Trimble APX-18
- **Position Accuracy**: 2cm absolute
- **Attitude Accuracy**: 0.015° roll/pitch
- **Applications**: Survey mapping, precision agriculture
- **Price Range**: $15,000-20,000

### Industrial Systems (Centimeter accuracy)

#### VectorNav VN-300
- **Position Accuracy**: 2cm GNSS
- **Attitude Accuracy**: 0.05° attitude
- **Applications**: Mobile robotics, UAV navigation
- **Price Range**: $3,000-6,000

#### Xsens MTi-G-710
- **Position Accuracy**: 2cm GNSS
- **Attitude Accuracy**: 0.2° attitude
- **Applications**: Industrial automation, mapping
- **Price Range**: $4,000-8,000

## Computing Platform Requirements

### Minimum Specifications
- **CPU**: Intel i7-10th gen or AMD Ryzen 7 (8+ cores)
- **RAM**: 32GB DDR4 (64GB recommended)
- **Storage**: 2TB NVMe SSD (high-speed logging)
- **Network**: Dual Gigabit Ethernet
- **GPU**: Dedicated GPU (optional, real-time processing)
- **OS**: Ubuntu 20.04 LTS or Windows 10/11 Pro

### Software Stack
- **Base OS**: Ubuntu 20.04 LTS
- **ROS**: ROS Noetic (ROS1) or ROS2 Galactic/Humble
- **LiDAR Driver**: livox_ros_driver (ROS1) or livox_ros2_driver (ROS2)
- **Libraries**: PCL 1.10+, Eigen 3.3+, PROJ 7.0+

## Mounting and Installation

### Mechanical Requirements
- **Base Plate**: 500mm × 300mm × 10mm aluminum
- **Material**: 6061-T6 aluminum or carbon fiber
- **Vibration Isolation**: 4× rubber isolators (70 Shore A)
- **Sensor Separation**: >30cm GNSS to LiDAR
- **Weight Capacity**: 5kg total system weight

### Environmental Protection
- **Enclosure Rating**: IP65+ for mobile applications
- **Operating Conditions**: -25°C to +70°C
- **Humidity**: 0-95% non-condensing
- **Vibration**: 5G RMS, 5-150Hz
- **Shock**: 30G, 11ms duration

## Synchronization Requirements

### Time Synchronization
- **PPS Signal**: Pulse Per Second distribution
- **Accuracy**: <40ns typical, <100ns maximum
- **Interface**: TTL/CMOS (3.3V or 5V)
- **Cable**: RG-58 coaxial or equivalent

### Network Infrastructure
- **Primary Switch**: Managed Gigabit PoE+ switch
- **Ports Required**: 4+ (LiDAR, GNSS/INS, Computer, spare)
- **Power Budget**: 60W+ (PoE+ for multiple devices)
- **Features**: VLAN support, QoS, port monitoring

## Performance Specifications

### Accuracy Targets
- **Absolute Position**: <5cm (95% confidence)
- **Relative Position**: <2cm (1σ)
- **Attitude**: <0.02° (roll/pitch), <0.05° (heading)
- **Point Density**: >1,000 points/m² @ 10m
- **Data Completeness**: >98% valid points

### Environmental Limits
- **Temperature**: -20°C to +65°C (<1cm degradation)
- **Humidity**: 0-95% non-condensing (no impact)
- **Wind Speed**: <25 m/s sustained
- **Precipitation**: Light to moderate (range reduction)
- **Vehicle Speed**: <50 km/h optimal

## SDK Compatibility

### Supported SDKs
- **Livox SDK (Original)**: ✅ Full Mid-70 support
- **Livox SDK2**: ❌ Not compatible with Mid-70

### ROS Drivers
- **livox_ros_driver**: ✅ ROS1 support for Mid-70
- **livox_ros2_driver**: ✅ ROS2 support for Mid-70
- **livox_ros_driver2**: ❌ HAP/Mid-360 only

### Development Tools
- **Livox Viewer**: Legacy .lvx file support
- **Livox Viewer 2**: Current .lvx2/.lvx3 support
- **Programming Languages**: C++, Python (via SDK)

## Quality Assurance

### Calibration Requirements
- **Extrinsic Calibration**: ±1cm translation, ±0.1° rotation
- **Time Synchronization**: ±1ms accuracy
- **Validation Method**: Ground control points

### Performance Monitoring
- **Real-time Metrics**: Processing latency, point throughput
- **Quality Indicators**: GNSS fix type, satellite count
- **System Health**: Temperature, power, network status
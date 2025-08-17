# GNSS Simulation Guide

## Overview

The Livox Mid-70 Motion Compensation Simulation framework includes a comprehensive GNSS (Global Navigation Satellite System) and INS (Inertial Navigation System) simulation module. This guide provides detailed information about GNSS simulation capabilities, configuration options, and integration with the LiDAR simulation.

## GNSS Fundamentals

### Supported Constellations

The simulator supports all major GNSS constellations:

| Constellation | Country/Region | Satellites | Frequency Bands | Status |
|---------------|----------------|------------|-----------------|---------|
| **GPS** | United States | 32 active | L1, L2, L5 | Fully Operational |
| **GLONASS** | Russia | 24 active | L1, L2, L3 | Fully Operational |
| **Galileo** | European Union | 30 planned | E1, E5a, E5b, E6 | Operational |
| **BeiDou** | China | 35 active | B1, B2, B3 | Fully Operational |
| **QZSS** | Japan | 7 satellites | L1, L2, L5, L6 | Regional Service |

### Fix Types and Accuracy

| Fix Type | Description | Typical Accuracy | Requirements |
|----------|-------------|------------------|--------------|
| **No Fix** | Insufficient satellites | N/A | < 4 satellites |
| **Autonomous** | Standard GPS | 3-5 meters | ≥ 4 satellites |
| **DGPS** | Differential GPS | 1-3 meters | ≥ 5 satellites + corrections |
| **RTK Float** | Real-Time Kinematic (ambiguous) | 10-50 cm | ≥ 6 satellites + base station |
| **RTK Fixed** | Real-Time Kinematic (resolved) | 1-5 cm | ≥ 6 satellites + base station |
| **PPP** | Precise Point Positioning | 5-20 cm | ≥ 8 satellites + precise orbits |

## Configuration

### Basic GNSS Configuration

```python
gnss_config = {
    # Update rate
    'update_rate': 10,  # Hz (1-20 Hz typical)
    
    # Base accuracy parameters
    'base_accuracy': 3.0,  # meters (autonomous GPS)
    
    # RTK availability
    'rtk_availability': 0.95,  # 95% availability
    
    # Error modeling
    'multipath_enabled': True,
    'atmospheric_errors': True,
    
    # Constellation configuration
    'constellations': {
        'GPS': {
            'enabled': True,
            'satellites': 32
        },
        'GLONASS': {
            'enabled': True,
            'satellites': 24
        },
        'GALILEO': {
            'enabled': True,
            'satellites': 30
        },
        'BEIDOU': {
            'enabled': True,
            'satellites': 35
        },
        'QZSS': {
            'enabled': False,  # Regional system
            'satellites': 7
        }
    }
}
```

### Advanced Configuration

```python
advanced_gnss_config = {
    # Receiver characteristics
    'receiver': {
        'type': 'survey_grade',  # 'consumer', 'mapping', 'survey_grade'
        'channels': 220,         # Number of tracking channels
        'update_rate': 20,       # Hz
        'cold_start_time': 30,   # seconds
        'warm_start_time': 5,    # seconds
        'hot_start_time': 1      # seconds
    },
    
    # Antenna configuration
    'antenna': {
        'type': 'survey_grade',
        'gain': 3.0,            # dBi
        'phase_center_variation': True,
        'multipath_rejection': 'choke_ring'
    },
    
    # Environmental conditions
    'environment': {
        'ionospheric_activity': 'moderate',  # 'low', 'moderate', 'high'
        'tropospheric_conditions': 'standard',
        'multipath_environment': 'open_sky',  # 'open_sky', 'suburban', 'urban', 'canyon'
        'interference_level': 'low'
    },
    
    # RTK base station
    'rtk_base': {
        'enabled': True,
        'distance': 10.0,       # km from rover
        'data_rate': 1,         # Hz
        'communication_delay': 0.1,  # seconds
        'reliability': 0.98     # 98% uptime
    },
    
    # PPP service
    'ppp_service': {
        'enabled': True,
        'convergence_time': 1800,  # seconds (30 minutes)
        'orbit_accuracy': 0.05,    # meters
        'clock_accuracy': 0.1      # nanoseconds
    }
}
```

## INS Integration

### INS Configuration

```python
ins_config = {
    # Update rate
    'update_rate': 200,  # Hz (100-1000 Hz typical)
    
    # IMU specifications (Livox Mid-70 built-in)
    'imu_type': 'mems',  # 'mems', 'fiber_optic', 'ring_laser'
    
    # Gyroscope characteristics
    'gyro_bias_stability': 1.0,    # degrees/hour
    'gyro_noise': 0.01,            # rad/s (1-sigma)
    'gyro_range': 2000,            # degrees/second
    'gyro_resolution': 0.01,       # degrees/second
    
    # Accelerometer characteristics
    'accel_bias_stability': 0.1,   # mg (milli-g)
    'accel_noise': 0.1,            # m/s² (1-sigma)
    'accel_range': 16,             # g
    'accel_resolution': 0.001,     # g
    
    # Integration parameters
    'alignment_time': 300,         # seconds (initial alignment)
    'zupt_enabled': True,          # Zero velocity updates
    'zupt_threshold': 0.1,         # m/s
    
    # Error modeling
    'temperature_effects': True,
    'vibration_effects': True,
    'scale_factor_errors': True
}
```

### GNSS/INS Fusion

```python
fusion_config = {
    # Fusion algorithm
    'algorithm': 'extended_kalman_filter',  # 'ekf', 'ukf', 'particle_filter'
    
    # State vector (15-state model)
    'states': [
        'position',      # lat, lon, alt
        'velocity',      # north, east, up
        'attitude',      # roll, pitch, yaw
        'gyro_bias',     # x, y, z bias
        'accel_bias'     # x, y, z bias
    ],
    
    # Process noise tuning
    'process_noise': {
        'position': 1e-8,
        'velocity': 1e-4,
        'attitude': 1e-6,
        'gyro_bias': 1e-10,
        'accel_bias': 1e-8
    },
    
    # Measurement updates
    'gnss_update_rate': 10,        # Hz
    'ins_propagation_rate': 200,   # Hz
    
    # Quality control
    'innovation_gating': True,
    'chi_square_threshold': 9.21,  # 99% confidence (3 DOF)
    'outlier_rejection': True
}
```

## Error Models

### Atmospheric Errors

#### Ionospheric Delay
The ionosphere causes signal delays that vary with:
- **Time of day**: Maximum around 14:00 local time
- **Season**: Higher in summer, lower in winter
- **Solar activity**: Increases with solar flux
- **Latitude**: Higher at equatorial regions
- **Frequency**: Inversely proportional to frequency squared

```python
# Ionospheric error model
ionospheric_delay = base_delay * diurnal_factor * seasonal_factor * solar_factor * latitude_factor
```

#### Tropospheric Delay
The troposphere causes delays due to:
- **Dry component**: Predictable, altitude-dependent
- **Wet component**: Variable, weather-dependent
- **Elevation angle**: Higher delays at low elevations

```python
# Tropospheric error model (Saastamoinen)
zenith_delay = 0.002277 * pressure * (1 + 0.0026 * cos(2 * latitude))
slant_delay = zenith_delay / sin(elevation)
```

### Multipath Errors

Multipath occurs when GNSS signals reflect off surfaces before reaching the antenna:

| Environment | Typical Multipath Error | Characteristics |
|-------------|------------------------|-----------------|
| **Open Sky** | 0.1-0.3 meters | Minimal reflections |
| **Suburban** | 0.3-1.0 meters | Buildings, trees |
| **Urban** | 1.0-5.0 meters | High buildings, canyons |
| **Indoor** | 5.0+ meters | Multiple reflections |

### Satellite Geometry (DOP)

Dilution of Precision (DOP) quantifies how satellite geometry affects accuracy:

```python
# DOP calculation
GDOP = sqrt(PDOP² + TDOP²)  # Geometric DOP
PDOP = sqrt(HDOP² + VDOP²)  # Position DOP
HDOP = horizontal_accuracy   # Horizontal DOP
VDOP = vertical_accuracy     # Vertical DOP
TDOP = time_accuracy         # Time DOP
```

| DOP Value | Rating | Description |
|-----------|--------|-------------|
| **1-2** | Excellent | Ideal geometry |
| **2-5** | Good | Acceptable for most applications |
| **5-10** | Moderate | Adequate for navigation |
| **10-20** | Fair | Poor geometry |
| **>20** | Poor | Unreliable positioning |

## Usage Examples

### Basic GNSS Simulation

```python
from livox_simulator.gnss_simulation import GNSSSimulator, FixType

# Initialize GNSS simulator
gnss_config = {
    'update_rate': 10,
    'base_accuracy': 3.0,
    'rtk_availability': 0.9
}

gnss_sim = GNSSSimulator(gnss_config)

# Simulate measurement
true_position = (40.7128, -74.0060, 10.0)  # NYC coordinates
timestamp = 1234567890.0

measurement = gnss_sim.simulate_measurement(true_position, timestamp)

print(f"Fix Type: {measurement.fix_type}")
print(f"Position: {measurement.latitude:.6f}, {measurement.longitude:.6f}")
print(f"Accuracy: {measurement.horizontal_accuracy:.2f}m")
print(f"Satellites: {measurement.satellites_used}/{measurement.satellites_visible}")
```

### GNSS/INS Fusion

```python
from livox_simulator.gnss_simulation import GNSSSimulator, INSSimulator, GNSSINSFusion

# Initialize systems
gnss_sim = GNSSSimulator(gnss_config)
ins_sim = INSSimulator(ins_config)
fusion = GNSSINSFusion(fusion_config)

# Simulation loop
for t in range(0, 3600, 1):  # 1-hour simulation
    timestamp = start_time + t
    
    # True trajectory (from vehicle simulation)
    true_pos = get_true_position(timestamp)
    true_vel = get_true_velocity(timestamp)
    true_att = get_true_attitude(timestamp)
    
    # Simulate GNSS (10 Hz)
    if t % 10 == 0:
        gnss_data = gnss_sim.simulate_measurement(true_pos, timestamp, true_vel)
    
    # Simulate INS (200 Hz)
    ins_data = ins_sim.simulate_ins_data(true_pos, true_vel, true_att, timestamp)
    
    # Fuse data
    fused_solution = fusion.update(gnss_data, ins_data, 1.0)
    
    # Use fused solution for LiDAR motion compensation
    apply_motion_compensation(lidar_frame, fused_solution)
```

### Multi-Constellation Analysis

```python
# Compare different constellation configurations
configs = [
    {'GPS': True, 'GLONASS': False, 'GALILEO': False, 'BEIDOU': False},
    {'GPS': True, 'GLONASS': True, 'GALILEO': False, 'BEIDOU': False},
    {'GPS': True, 'GLONASS': True, 'GALILEO': True, 'BEIDOU': True}
]

results = []
for config in configs:
    gnss_sim = GNSSSimulator({'constellations': config})
    
    # Run simulation
    measurements = []
    for t in range(0, 3600, 10):
        measurement = gnss_sim.simulate_measurement(true_position, t)
        measurements.append(measurement)
    
    # Analyze results
    avg_accuracy = np.mean([m.horizontal_accuracy for m in measurements])
    rtk_availability = len([m for m in measurements if m.fix_type == FixType.RTK_FIXED]) / len(measurements)
    
    results.append({
        'config': config,
        'avg_accuracy': avg_accuracy,
        'rtk_availability': rtk_availability
    })

# Print comparison
for result in results:
    print(f"Config: {result['config']}")
    print(f"Average Accuracy: {result['avg_accuracy']:.2f}m")
    print(f"RTK Availability: {result['rtk_availability']:.1%}")
```

## Integration with LiDAR Simulation

### Motion Compensation Workflow

```python
def apply_gnss_ins_motion_compensation(lidar_frame, gnss_ins_data):
    """Apply motion compensation using GNSS/INS data."""
    
    # Extract motion parameters
    position = np.array([
        gnss_ins_data['latitude'],
        gnss_ins_data['longitude'], 
        gnss_ins_data['altitude']
    ])
    
    velocity = np.array([
        gnss_ins_data['velocity_north'],
        gnss_ins_data['velocity_east'],
        gnss_ins_data['velocity_up']
    ])
    
    attitude = np.array([
        gnss_ins_data['roll'],
        gnss_ins_data['pitch'],
        gnss_ins_data['yaw']
    ])
    
    # Calculate motion during frame acquisition
    frame_duration = 0.1  # 100ms for 10Hz LiDAR
    
    # Compensate each point based on acquisition time within frame
    compensated_points = []
    for point in lidar_frame.points:
        # Calculate point acquisition time
        point_time = calculate_point_time(point, lidar_frame)
        
        # Interpolate motion at point time
        point_position = interpolate_position(position, velocity, point_time)
        point_attitude = interpolate_attitude(attitude, angular_rates, point_time)
        
        # Apply motion compensation
        compensated_point = compensate_point_motion(point, point_position, point_attitude)
        compensated_points.append(compensated_point)
    
    return np.array(compensated_points)
```

### Coordinate System Integration

```python
def transform_to_global_coordinates(points, gnss_data):
    """Transform LiDAR points to global coordinates using GNSS data."""
    
    # Convert GNSS lat/lon to UTM
    utm_x, utm_y, utm_zone, utm_letter = utm.from_latlon(
        gnss_data.latitude, 
        gnss_data.longitude
    )
    
    # Create transformation matrix
    origin = np.array([utm_x, utm_y, gnss_data.altitude])
    
    # Apply transformation to each point
    global_points = []
    for point in points:
        # Transform from sensor frame to global UTM
        global_point = transform_point_to_utm(point, origin, gnss_data)
        global_points.append(global_point)
    
    return np.array(global_points)
```

## Quality Assessment

### GNSS Quality Indicators

```python
def assess_gnss_quality(measurement):
    """Assess GNSS measurement quality."""
    
    quality_score = 0
    
    # Fix type scoring
    fix_scores = {
        FixType.NO_FIX: 0,
        FixType.AUTONOMOUS: 20,
        FixType.DGPS: 40,
        FixType.RTK_FLOAT: 70,
        FixType.RTK_FIXED: 100,
        FixType.PPP: 80
    }
    quality_score += fix_scores[measurement.fix_type]
    
    # Satellite count scoring
    if measurement.satellites_used >= 8:
        quality_score += 20
    elif measurement.satellites_used >= 6:
        quality_score += 15
    elif measurement.satellites_used >= 4:
        quality_score += 10
    
    # DOP scoring
    if measurement.hdop < 2.0:
        quality_score += 15
    elif measurement.hdop < 5.0:
        quality_score += 10
    elif measurement.hdop < 10.0:
        quality_score += 5
    
    # Accuracy scoring
    if measurement.horizontal_accuracy < 0.1:
        quality_score += 15
    elif measurement.horizontal_accuracy < 1.0:
        quality_score += 10
    elif measurement.horizontal_accuracy < 5.0:
        quality_score += 5
    
    return min(quality_score, 100)  # Cap at 100%
```

### Performance Monitoring

```python
def monitor_gnss_performance(measurements):
    """Monitor GNSS performance over time."""
    
    metrics = {
        'availability': {},
        'accuracy': {},
        'continuity': {},
        'integrity': {}
    }
    
    # Availability (percentage of valid fixes)
    total_epochs = len(measurements)
    valid_fixes = len([m for m in measurements if m.fix_type != FixType.NO_FIX])
    metrics['availability']['overall'] = valid_fixes / total_epochs
    
    # Accuracy statistics
    accuracies = [m.horizontal_accuracy for m in measurements if m.fix_type != FixType.NO_FIX]
    if accuracies:
        metrics['accuracy']['mean'] = np.mean(accuracies)
        metrics['accuracy']['std'] = np.std(accuracies)
        metrics['accuracy']['95th_percentile'] = np.percentile(accuracies, 95)
    
    # Continuity (uninterrupted service)
    fix_gaps = calculate_fix_gaps(measurements)
    metrics['continuity']['max_gap'] = max(fix_gaps) if fix_gaps else 0
    metrics['continuity']['mean_gap'] = np.mean(fix_gaps) if fix_gaps else 0
    
    # Integrity (protection levels)
    rtk_measurements = [m for m in measurements if m.fix_type == FixType.RTK_FIXED]
    if rtk_measurements:
        metrics['integrity']['rtk_availability'] = len(rtk_measurements) / total_epochs
        metrics['integrity']['mean_rtk_accuracy'] = np.mean([m.horizontal_accuracy for m in rtk_measurements])
    
    return metrics
```

## Troubleshooting

### Common Issues

#### Poor GNSS Performance
```python
# Diagnostic checks
def diagnose_gnss_issues(measurement):
    issues = []
    
    if measurement.satellites_visible < 4:
        issues.append("Insufficient satellite visibility - check sky view")
    
    if measurement.hdop > 10:
        issues.append("Poor satellite geometry - wait for better constellation")
    
    if measurement.fix_type == FixType.NO_FIX:
        issues.append("No position fix - check antenna connection")
    
    if measurement.horizontal_accuracy > 10:
        issues.append("High position uncertainty - check for interference")
    
    return issues
```

#### RTK Issues
```python
def diagnose_rtk_issues(measurement, base_station_data):
    issues = []
    
    if measurement.age_of_corrections > 30:
        issues.append("Stale RTK corrections - check base station link")
    
    if measurement.fix_type == FixType.RTK_FLOAT:
        issues.append("RTK float solution - may need more time or better geometry")
    
    if base_station_data['distance'] > 50:
        issues.append("Base station too far - accuracy may be degraded")
    
    return issues
```

## References

1. **GNSS Standards**:
   - GPS Interface Specification (IS-GPS-200)
   - GLONASS Interface Control Document
   - Galileo Open Service Signal-in-Space ICD
   - BeiDou Navigation Satellite System Signal-in-Space ICD

2. **Error Modeling**:
   - Klobuchar Ionospheric Model
   - Saastamoinen Tropospheric Model
   - Multipath Error Analysis

3. **Integration Algorithms**:
   - Extended Kalman Filter for GNSS/INS
   - Tightly Coupled Integration Methods
   - Quality Control and Outlier Detection

4. **Performance Standards**:
   - RTCA DO-229 (GPS/WAAS Performance)
   - ISO 17123 (Surveying Instrument Standards)
   - IMO Resolution A.915 (Maritime GNSS Standards)
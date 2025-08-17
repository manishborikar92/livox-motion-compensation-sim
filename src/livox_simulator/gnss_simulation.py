"""
GNSS/INS Simulation Module for Livox Mid-70 Simulation Framework

This module provides comprehensive GNSS (Global Navigation Satellite System) and INS 
(Inertial Navigation System) simulation capabilities for realistic positioning and 
navigation data generation.

Supported Systems:
- GPS (Global Positioning System)
- GLONASS (Russian satellite system)
- Galileo (European satellite system)
- BeiDou (Chinese satellite system)
- QZSS (Japanese regional system)

Features:
- Multi-constellation GNSS simulation
- RTK (Real-Time Kinematic) positioning
- PPP (Precise Point Positioning)
- Atmospheric error modeling
- Multipath effects simulation
- Satellite visibility and geometry
- GNSS/INS sensor fusion
"""

import numpy as np
import math
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass
from enum import Enum
import random
from datetime import datetime, timezone


class GNSSConstellation(Enum):
    """GNSS constellation types."""
    GPS = "GPS"
    GLONASS = "GLONASS"
    GALILEO = "GALILEO"
    BEIDOU = "BEIDOU"
    QZSS = "QZSS"


class FixType(Enum):
    """GNSS fix quality types."""
    NO_FIX = 0
    AUTONOMOUS = 1
    DGPS = 2
    RTK_FLOAT = 4
    RTK_FIXED = 5
    PPP = 6


@dataclass
class GNSSSatellite:
    """Represents a GNSS satellite."""
    prn: int                    # Pseudo-random number (satellite ID)
    constellation: GNSSConstellation
    elevation: float            # Elevation angle (degrees)
    azimuth: float             # Azimuth angle (degrees)
    snr: float                 # Signal-to-noise ratio (dB-Hz)
    used_in_solution: bool     # Whether satellite is used in position solution
    health: bool               # Satellite health status


@dataclass
class GNSSMeasurement:
    """GNSS measurement data."""
    timestamp: float           # GPS time (seconds)
    latitude: float           # WGS84 latitude (degrees)
    longitude: float          # WGS84 longitude (degrees)
    altitude: float           # Height above ellipsoid (meters)
    
    # Accuracy estimates
    horizontal_accuracy: float # Horizontal position accuracy (meters)
    vertical_accuracy: float   # Vertical position accuracy (meters)
    
    # Velocity
    velocity_north: float      # North velocity (m/s)
    velocity_east: float       # East velocity (m/s)
    velocity_up: float         # Up velocity (m/s)
    velocity_accuracy: float   # Velocity accuracy (m/s)
    
    # Quality indicators
    fix_type: FixType
    satellites_used: int       # Number of satellites used
    satellites_visible: int    # Number of satellites visible
    hdop: float               # Horizontal dilution of precision
    vdop: float               # Vertical dilution of precision
    pdop: float               # Position dilution of precision
    
    # Additional data
    age_of_corrections: float  # Age of differential corrections (seconds)
    base_station_id: int      # RTK base station ID
    satellites: List[GNSSSatellite] = None


@dataclass
class INSData:
    """Inertial Navigation System data."""
    timestamp: float
    
    # Position (integrated from IMU)
    latitude: float
    longitude: float
    altitude: float
    
    # Velocity
    velocity_north: float
    velocity_east: float
    velocity_up: float
    
    # Attitude
    roll: float               # Roll angle (radians)
    pitch: float              # Pitch angle (radians)
    yaw: float                # Yaw angle (radians)
    
    # Angular rates
    angular_rate_x: float     # Body frame angular rate (rad/s)
    angular_rate_y: float
    angular_rate_z: float
    
    # Accelerations
    acceleration_x: float     # Body frame acceleration (m/s²)
    acceleration_y: float
    acceleration_z: float
    
    # Quality indicators
    position_accuracy: float  # Position accuracy estimate (meters)
    velocity_accuracy: float  # Velocity accuracy estimate (m/s)
    attitude_accuracy: float  # Attitude accuracy estimate (radians)


class GNSSSimulator:
    """Comprehensive GNSS simulation engine."""
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize GNSS simulator.
        
        Args:
            config: GNSS simulation configuration
        """
        self.config = config
        self.constellation_config = config.get('constellations', {
            'GPS': {'enabled': True, 'satellites': 32},
            'GLONASS': {'enabled': True, 'satellites': 24},
            'GALILEO': {'enabled': True, 'satellites': 30},
            'BEIDOU': {'enabled': True, 'satellites': 35},
            'QZSS': {'enabled': False, 'satellites': 7}
        })
        
        # Simulation parameters
        self.update_rate = config.get('update_rate', 10)  # Hz
        self.base_accuracy = config.get('base_accuracy', 3.0)  # meters
        self.rtk_availability = config.get('rtk_availability', 0.95)
        self.multipath_enabled = config.get('multipath_enabled', True)
        self.atmospheric_errors = config.get('atmospheric_errors', True)
        
        # Initialize satellite constellations
        self._initialize_constellations()
        
        # Error models
        self.ionospheric_model = IonosphericModel()
        self.tropospheric_model = TroposphericModel()
        self.multipath_model = MultipathModel()
        
        # Current state
        self.current_time = 0.0
        self.last_fix_type = FixType.NO_FIX
        
    def _initialize_constellations(self):
        """Initialize satellite constellation data."""
        self.satellites = {}
        
        for constellation_name, config in self.constellation_config.items():
            if not config['enabled']:
                continue
                
            constellation = GNSSConstellation(constellation_name)
            satellite_count = config['satellites']
            
            self.satellites[constellation] = []
            
            for i in range(satellite_count):
                # Generate pseudo-random satellite positions
                prn = i + 1
                if constellation == GNSSConstellation.GPS:
                    prn = i + 1  # GPS PRN 1-32
                elif constellation == GNSSConstellation.GLONASS:
                    prn = i + 1  # GLONASS slot numbers
                elif constellation == GNSSConstellation.GALILEO:
                    prn = i + 1  # Galileo SVID
                elif constellation == GNSSConstellation.BEIDOU:
                    prn = i + 1  # BeiDou PRN
                elif constellation == GNSSConstellation.QZSS:
                    prn = 193 + i  # QZSS PRN 193-199
                
                satellite = GNSSSatellite(
                    prn=prn,
                    constellation=constellation,
                    elevation=0.0,
                    azimuth=0.0,
                    snr=0.0,
                    used_in_solution=False,
                    health=True
                )
                
                self.satellites[constellation].append(satellite)
    
    def simulate_measurement(self, true_position: Tuple[float, float, float], 
                           timestamp: float, velocity: Optional[Tuple[float, float, float]] = None) -> GNSSMeasurement:
        """
        Simulate GNSS measurement at given position and time.
        
        Args:
            true_position: True position (lat, lon, alt) in degrees and meters
            timestamp: GPS time in seconds
            velocity: True velocity (north, east, up) in m/s
            
        Returns:
            Simulated GNSS measurement
        """
        self.current_time = timestamp
        lat_true, lon_true, alt_true = true_position
        
        # Update satellite positions and visibility
        visible_satellites = self._update_satellite_visibility(lat_true, lon_true, timestamp)
        
        # Determine fix type based on conditions
        fix_type = self._determine_fix_type(visible_satellites, timestamp)
        
        # Calculate position errors based on fix type and conditions
        position_error = self._calculate_position_error(fix_type, visible_satellites, lat_true, lon_true, alt_true)
        
        # Apply errors to true position
        lat_measured = lat_true + position_error[0] / 111320.0  # Convert meters to degrees
        lon_measured = lon_true + position_error[1] / (111320.0 * math.cos(math.radians(lat_true)))
        alt_measured = alt_true + position_error[2]
        
        # Calculate accuracy estimates
        horizontal_accuracy, vertical_accuracy = self._calculate_accuracy_estimates(fix_type, visible_satellites)
        
        # Simulate velocity if provided
        if velocity:
            vel_north, vel_east, vel_up = velocity
            vel_error = self._calculate_velocity_error(fix_type)
            vel_north += vel_error[0]
            vel_east += vel_error[1]
            vel_up += vel_error[2]
            velocity_accuracy = self._calculate_velocity_accuracy(fix_type)
        else:
            vel_north = vel_east = vel_up = 0.0
            velocity_accuracy = 0.1
        
        # Calculate dilution of precision
        hdop, vdop, pdop = self._calculate_dop(visible_satellites)
        
        # Count satellites
        satellites_used = len([sat for sat in visible_satellites if sat.used_in_solution])
        satellites_visible = len(visible_satellites)
        
        # RTK-specific parameters
        age_of_corrections = 0.0
        base_station_id = 0
        if fix_type in [FixType.RTK_FLOAT, FixType.RTK_FIXED]:
            age_of_corrections = random.uniform(0.1, 2.0)
            base_station_id = random.randint(1000, 9999)
        
        return GNSSMeasurement(
            timestamp=timestamp,
            latitude=lat_measured,
            longitude=lon_measured,
            altitude=alt_measured,
            horizontal_accuracy=horizontal_accuracy,
            vertical_accuracy=vertical_accuracy,
            velocity_north=vel_north,
            velocity_east=vel_east,
            velocity_up=vel_up,
            velocity_accuracy=velocity_accuracy,
            fix_type=fix_type,
            satellites_used=satellites_used,
            satellites_visible=satellites_visible,
            hdop=hdop,
            vdop=vdop,
            pdop=pdop,
            age_of_corrections=age_of_corrections,
            base_station_id=base_station_id,
            satellites=visible_satellites
        )
    
    def _update_satellite_visibility(self, lat: float, lon: float, timestamp: float) -> List[GNSSSatellite]:
        """Update satellite positions and determine visibility."""
        visible_satellites = []
        
        for constellation, satellites in self.satellites.items():
            for satellite in satellites:
                # Simulate satellite orbital motion
                elevation, azimuth = self._calculate_satellite_position(
                    satellite.prn, constellation, lat, lon, timestamp
                )
                
                satellite.elevation = elevation
                satellite.azimuth = azimuth
                
                # Check if satellite is visible (above horizon with margin)
                if elevation > 5.0:  # 5-degree elevation mask
                    # Calculate signal strength
                    satellite.snr = self._calculate_snr(elevation, constellation)
                    
                    # Determine if satellite is healthy and usable
                    satellite.health = random.random() > 0.02  # 2% unhealthy rate
                    satellite.used_in_solution = (
                        satellite.health and 
                        elevation > 10.0 and  # Higher threshold for solution
                        satellite.snr > 35.0   # Minimum SNR threshold
                    )
                    
                    visible_satellites.append(satellite)
        
        return visible_satellites
    
    def _calculate_satellite_position(self, prn: int, constellation: GNSSConstellation, 
                                    lat: float, lon: float, timestamp: float) -> Tuple[float, float]:
        """Calculate satellite elevation and azimuth angles."""
        # Simplified satellite position calculation
        # In reality, this would use precise orbital elements
        
        # Create pseudo-random but consistent satellite motion
        seed = hash((prn, constellation.value)) % 1000000
        random.seed(seed + int(timestamp / 3600))  # Change every hour
        
        # Simulate orbital period (approximately 12 hours for GPS)
        orbital_period = 12 * 3600  # seconds
        if constellation == GNSSConstellation.GLONASS:
            orbital_period = 11.25 * 3600
        elif constellation == GNSSConstellation.GALILEO:
            orbital_period = 14.08 * 3600
        elif constellation == GNSSConstellation.BEIDOU:
            orbital_period = 12.63 * 3600
        
        # Calculate satellite position in sky
        phase = (timestamp % orbital_period) / orbital_period * 2 * math.pi
        phase += (prn - 1) * 2 * math.pi / 32  # Distribute satellites
        
        # Simplified elevation and azimuth calculation
        elevation = 15 + 60 * (0.5 + 0.5 * math.sin(phase + lat * math.pi / 180))
        azimuth = (phase * 180 / math.pi + lon + prn * 30) % 360
        
        return elevation, azimuth
    
    def _calculate_snr(self, elevation: float, constellation: GNSSConstellation) -> float:
        """Calculate signal-to-noise ratio based on elevation and constellation."""
        # Base SNR values for different constellations
        base_snr = {
            GNSSConstellation.GPS: 45.0,
            GNSSConstellation.GLONASS: 43.0,
            GNSSConstellation.GALILEO: 46.0,
            GNSSConstellation.BEIDOU: 44.0,
            GNSSConstellation.QZSS: 47.0
        }
        
        # SNR increases with elevation
        snr = base_snr[constellation] + (elevation - 5) * 0.3
        
        # Add random noise
        snr += random.gauss(0, 2.0)
        
        return max(snr, 25.0)  # Minimum SNR
    
    def _determine_fix_type(self, visible_satellites: List[GNSSSatellite], timestamp: float) -> FixType:
        """Determine GNSS fix type based on conditions."""
        usable_satellites = [sat for sat in visible_satellites if sat.used_in_solution]
        
        if len(usable_satellites) < 4:
            return FixType.NO_FIX
        
        # RTK availability simulation
        if random.random() < self.rtk_availability:
            if len(usable_satellites) >= 6:
                # RTK Fixed has higher requirements
                if random.random() < 0.8:  # 80% chance of fixed solution
                    return FixType.RTK_FIXED
                else:
                    return FixType.RTK_FLOAT
        
        # PPP simulation (requires longer convergence time)
        if len(usable_satellites) >= 8 and timestamp > 1800:  # 30 minutes convergence
            if random.random() < 0.3:  # 30% chance of PPP
                return FixType.PPP
        
        # DGPS simulation
        if len(usable_satellites) >= 5:
            if random.random() < 0.6:  # 60% chance of DGPS
                return FixType.DGPS
        
        return FixType.AUTONOMOUS
    
    def _calculate_position_error(self, fix_type: FixType, satellites: List[GNSSSatellite],
                                lat: float, lon: float, alt: float) -> Tuple[float, float, float]:
        """Calculate position error based on fix type and conditions."""
        # Base error for different fix types (meters)
        base_errors = {
            FixType.NO_FIX: (100.0, 100.0, 200.0),
            FixType.AUTONOMOUS: (3.0, 3.0, 5.0),
            FixType.DGPS: (1.0, 1.0, 2.0),
            FixType.RTK_FLOAT: (0.3, 0.3, 0.5),
            FixType.RTK_FIXED: (0.02, 0.02, 0.05),
            FixType.PPP: (0.1, 0.1, 0.2)
        }
        
        base_error = base_errors[fix_type]
        
        # Apply atmospheric errors
        if self.atmospheric_errors:
            iono_error = self.ionospheric_model.get_error(lat, lon, self.current_time)
            tropo_error = self.tropospheric_model.get_error(lat, alt)
            
            atmospheric_factor = 1.0 + (iono_error + tropo_error) / 10.0
        else:
            atmospheric_factor = 1.0
        
        # Apply multipath errors
        if self.multipath_enabled:
            multipath_factor = self.multipath_model.get_factor(satellites)
        else:
            multipath_factor = 1.0
        
        # Calculate final errors
        error_north = random.gauss(0, base_error[0] * atmospheric_factor * multipath_factor)
        error_east = random.gauss(0, base_error[1] * atmospheric_factor * multipath_factor)
        error_up = random.gauss(0, base_error[2] * atmospheric_factor * multipath_factor)
        
        return error_north, error_east, error_up
    
    def _calculate_accuracy_estimates(self, fix_type: FixType, 
                                    satellites: List[GNSSSatellite]) -> Tuple[float, float]:
        """Calculate horizontal and vertical accuracy estimates."""
        # Base accuracy for different fix types
        base_accuracies = {
            FixType.NO_FIX: (50.0, 100.0),
            FixType.AUTONOMOUS: (2.5, 4.0),
            FixType.DGPS: (0.8, 1.5),
            FixType.RTK_FLOAT: (0.25, 0.4),
            FixType.RTK_FIXED: (0.015, 0.03),
            FixType.PPP: (0.08, 0.15)
        }
        
        horizontal_acc, vertical_acc = base_accuracies[fix_type]
        
        # Adjust based on satellite geometry
        usable_sats = len([sat for sat in satellites if sat.used_in_solution])
        if usable_sats > 8:
            geometry_factor = 0.8
        elif usable_sats > 6:
            geometry_factor = 0.9
        elif usable_sats > 4:
            geometry_factor = 1.0
        else:
            geometry_factor = 1.5
        
        return horizontal_acc * geometry_factor, vertical_acc * geometry_factor
    
    def _calculate_velocity_error(self, fix_type: FixType) -> Tuple[float, float, float]:
        """Calculate velocity measurement errors."""
        # Base velocity error for different fix types (m/s)
        base_vel_errors = {
            FixType.NO_FIX: 1.0,
            FixType.AUTONOMOUS: 0.1,
            FixType.DGPS: 0.05,
            FixType.RTK_FLOAT: 0.02,
            FixType.RTK_FIXED: 0.005,
            FixType.PPP: 0.01
        }
        
        base_error = base_vel_errors[fix_type]
        
        vel_error_north = random.gauss(0, base_error)
        vel_error_east = random.gauss(0, base_error)
        vel_error_up = random.gauss(0, base_error * 1.5)  # Vertical velocity less accurate
        
        return vel_error_north, vel_error_east, vel_error_up
    
    def _calculate_velocity_accuracy(self, fix_type: FixType) -> float:
        """Calculate velocity accuracy estimate."""
        vel_accuracies = {
            FixType.NO_FIX: 0.5,
            FixType.AUTONOMOUS: 0.08,
            FixType.DGPS: 0.04,
            FixType.RTK_FLOAT: 0.015,
            FixType.RTK_FIXED: 0.003,
            FixType.PPP: 0.008
        }
        
        return vel_accuracies[fix_type]
    
    def _calculate_dop(self, satellites: List[GNSSSatellite]) -> Tuple[float, float, float]:
        """Calculate dilution of precision values."""
        usable_sats = [sat for sat in satellites if sat.used_in_solution]
        
        if len(usable_sats) < 4:
            return 99.9, 99.9, 99.9
        
        # Simplified DOP calculation based on satellite geometry
        # In reality, this would use the actual satellite positions
        
        # Better geometry with more satellites
        sat_count = len(usable_sats)
        base_dop = max(1.0, 8.0 / sat_count)
        
        # Add some randomness for realism
        hdop = base_dop * random.uniform(0.8, 1.2)
        vdop = hdop * random.uniform(1.2, 1.8)  # Vertical DOP typically higher
        pdop = math.sqrt(hdop**2 + vdop**2)
        
        return hdop, vdop, pdop


class INSSimulator:
    """Inertial Navigation System simulator."""
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize INS simulator."""
        self.config = config
        self.update_rate = config.get('update_rate', 200)  # Hz
        
        # IMU error characteristics
        self.gyro_bias_stability = config.get('gyro_bias_stability', 1.0)  # deg/hr
        self.accel_bias_stability = config.get('accel_bias_stability', 0.1)  # mg
        self.gyro_noise = config.get('gyro_noise', 0.01)  # rad/s
        self.accel_noise = config.get('accel_noise', 0.1)  # m/s²
        
        # Current biases (slowly varying)
        self.gyro_bias = np.array([0.0, 0.0, 0.0])
        self.accel_bias = np.array([0.0, 0.0, 0.0])
        
        # Integration state
        self.position = np.array([0.0, 0.0, 0.0])  # lat, lon, alt
        self.velocity = np.array([0.0, 0.0, 0.0])  # north, east, up
        self.attitude = np.array([0.0, 0.0, 0.0])  # roll, pitch, yaw
        
        self.last_update_time = 0.0
    
    def simulate_ins_data(self, true_position: Tuple[float, float, float],
                         true_velocity: Tuple[float, float, float],
                         true_attitude: Tuple[float, float, float],
                         timestamp: float) -> INSData:
        """
        Simulate INS data with realistic errors and drift.
        
        Args:
            true_position: True position (lat, lon, alt)
            true_velocity: True velocity (north, east, up)
            true_attitude: True attitude (roll, pitch, yaw)
            timestamp: Current time
            
        Returns:
            Simulated INS data
        """
        dt = timestamp - self.last_update_time if self.last_update_time > 0 else 1.0 / self.update_rate
        self.last_update_time = timestamp
        
        # Update biases (random walk)
        gyro_bias_change = np.random.normal(0, self.gyro_bias_stability * dt / 3600, 3)
        accel_bias_change = np.random.normal(0, self.accel_bias_stability * dt / 1000, 3)
        
        self.gyro_bias += gyro_bias_change
        self.accel_bias += accel_bias_change
        
        # Add measurement noise and bias to true values
        measured_angular_rates = np.array([
            (true_attitude[0] - self.attitude[0]) / dt,  # Approximate angular rates
            (true_attitude[1] - self.attitude[1]) / dt,
            (true_attitude[2] - self.attitude[2]) / dt
        ]) + self.gyro_bias + np.random.normal(0, self.gyro_noise, 3)
        
        # Calculate accelerations from velocity change
        true_accel = np.array([
            (true_velocity[0] - self.velocity[0]) / dt,
            (true_velocity[1] - self.velocity[1]) / dt,
            (true_velocity[2] - self.velocity[2]) / dt + 9.81  # Add gravity
        ])
        
        measured_accelerations = true_accel + self.accel_bias + np.random.normal(0, self.accel_noise, 3)
        
        # Integrate to get INS position, velocity, attitude (with accumulated errors)
        # This is a simplified integration - real INS uses complex navigation equations
        
        # Update attitude
        self.attitude += measured_angular_rates * dt
        
        # Update velocity
        self.velocity[:2] += measured_accelerations[:2] * dt  # Update horizontal velocity
        self.velocity[2] = true_velocity[2] + random.gauss(0, 0.1)  # Add some error
        
        # Update position (with accumulated drift)
        position_drift = np.array([
            random.gauss(0, 0.001 * timestamp),  # Drift increases with time
            random.gauss(0, 0.001 * timestamp),
            random.gauss(0, 0.002 * timestamp)
        ])
        
        self.position = np.array(true_position) + position_drift
        
        # Calculate accuracy estimates (degrade over time without GNSS updates)
        time_since_last_gnss = min(timestamp % 60, 60)  # Assume GNSS update every 60s
        position_accuracy = 0.1 + time_since_last_gnss * 0.05  # Drift 5cm per second
        velocity_accuracy = 0.01 + time_since_last_gnss * 0.002
        attitude_accuracy = 0.001 + time_since_last_gnss * 0.0001
        
        return INSData(
            timestamp=timestamp,
            latitude=self.position[0],
            longitude=self.position[1],
            altitude=self.position[2],
            velocity_north=self.velocity[0],
            velocity_east=self.velocity[1],
            velocity_up=self.velocity[2],
            roll=self.attitude[0],
            pitch=self.attitude[1],
            yaw=self.attitude[2],
            angular_rate_x=measured_angular_rates[0],
            angular_rate_y=measured_angular_rates[1],
            angular_rate_z=measured_angular_rates[2],
            acceleration_x=measured_accelerations[0],
            acceleration_y=measured_accelerations[1],
            acceleration_z=measured_accelerations[2],
            position_accuracy=position_accuracy,
            velocity_accuracy=velocity_accuracy,
            attitude_accuracy=attitude_accuracy
        )


class IonosphericModel:
    """Ionospheric delay error model."""
    
    def get_error(self, lat: float, lon: float, timestamp: float) -> float:
        """Calculate ionospheric delay error in meters."""
        # Simplified Klobuchar model
        # Real implementation would use broadcast parameters
        
        # Diurnal variation
        local_time = (timestamp / 3600) % 24
        diurnal_factor = 1.0 + 0.5 * math.sin(2 * math.pi * (local_time - 14) / 24)
        
        # Latitude dependence
        lat_factor = 1.0 + 0.3 * math.cos(math.radians(lat))
        
        # Solar activity (simplified)
        solar_factor = 1.0 + 0.2 * math.sin(2 * math.pi * timestamp / (365.25 * 24 * 3600))
        
        base_delay = 5.0  # meters
        return base_delay * diurnal_factor * lat_factor * solar_factor


class TroposphericModel:
    """Tropospheric delay error model."""
    
    def get_error(self, lat: float, alt: float) -> float:
        """Calculate tropospheric delay error in meters."""
        # Simplified Saastamoinen model
        
        # Standard atmosphere parameters
        p0 = 1013.25  # mbar
        t0 = 288.15   # K
        h0 = 0.0      # m
        
        # Pressure at altitude
        pressure = p0 * (1 - 0.0065 * alt / t0) ** 5.257
        
        # Zenith delay
        zenith_delay = 0.002277 * pressure
        
        # Latitude correction
        lat_correction = 1.0 + 0.0026 * math.cos(2 * math.radians(lat))
        
        return zenith_delay * lat_correction


class MultipathModel:
    """Multipath error model."""
    
    def get_factor(self, satellites: List[GNSSSatellite]) -> float:
        """Calculate multipath error factor based on satellite geometry."""
        if not satellites:
            return 1.0
        
        # Multipath is worse for low elevation satellites
        low_elevation_sats = len([sat for sat in satellites if sat.elevation < 30])
        total_sats = len(satellites)
        
        if total_sats == 0:
            return 1.0
        
        low_elevation_ratio = low_elevation_sats / total_sats
        multipath_factor = 1.0 + low_elevation_ratio * 0.5
        
        return multipath_factor


class GNSSINSFusion:
    """GNSS/INS sensor fusion using Extended Kalman Filter."""
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize GNSS/INS fusion system."""
        self.config = config
        
        # State vector: [lat, lon, alt, vel_n, vel_e, vel_u, roll, pitch, yaw]
        self.state = np.zeros(9)
        self.covariance = np.eye(9) * 0.1
        
        # Process noise
        self.process_noise = np.diag([1e-8, 1e-8, 1e-6, 1e-4, 1e-4, 1e-4, 1e-6, 1e-6, 1e-6])
        
        # Measurement noise (will be updated based on GNSS accuracy)
        self.gnss_noise = np.diag([1e-6, 1e-6, 1e-4, 1e-2, 1e-2, 1e-2])
        
    def update(self, gnss_data: GNSSMeasurement, ins_data: INSData, dt: float) -> Dict[str, Any]:
        """
        Update fused navigation solution.
        
        Args:
            gnss_data: GNSS measurement
            ins_data: INS data
            dt: Time step
            
        Returns:
            Fused navigation solution
        """
        # Prediction step (using INS)
        self._predict(ins_data, dt)
        
        # Update step (using GNSS when available)
        if gnss_data.fix_type != FixType.NO_FIX:
            self._update_with_gnss(gnss_data)
        
        # Extract fused solution
        fused_solution = {
            'timestamp': gnss_data.timestamp,
            'latitude': self.state[0],
            'longitude': self.state[1],
            'altitude': self.state[2],
            'velocity_north': self.state[3],
            'velocity_east': self.state[4],
            'velocity_up': self.state[5],
            'roll': self.state[6],
            'pitch': self.state[7],
            'yaw': self.state[8],
            'position_accuracy': math.sqrt(self.covariance[0, 0] + self.covariance[1, 1]),
            'velocity_accuracy': math.sqrt(self.covariance[3, 3] + self.covariance[4, 4]),
            'attitude_accuracy': math.sqrt(self.covariance[6, 6] + self.covariance[7, 7] + self.covariance[8, 8])
        }
        
        return fused_solution
    
    def _predict(self, ins_data: INSData, dt: float):
        """Prediction step using INS data."""
        # Simple state propagation
        self.state[0] = ins_data.latitude
        self.state[1] = ins_data.longitude
        self.state[2] = ins_data.altitude
        self.state[3] = ins_data.velocity_north
        self.state[4] = ins_data.velocity_east
        self.state[5] = ins_data.velocity_up
        self.state[6] = ins_data.roll
        self.state[7] = ins_data.pitch
        self.state[8] = ins_data.yaw
        
        # Propagate covariance
        self.covariance += self.process_noise * dt
    
    def _update_with_gnss(self, gnss_data: GNSSMeasurement):
        """Update step using GNSS measurement."""
        # Measurement vector
        z = np.array([
            gnss_data.latitude,
            gnss_data.longitude,
            gnss_data.altitude,
            gnss_data.velocity_north,
            gnss_data.velocity_east,
            gnss_data.velocity_up
        ])
        
        # Predicted measurement
        h = self.state[:6]  # Position and velocity states
        
        # Innovation
        y = z - h
        
        # Measurement Jacobian (identity for direct measurements)
        H = np.zeros((6, 9))
        H[:6, :6] = np.eye(6)
        
        # Update measurement noise based on GNSS accuracy
        self.gnss_noise[0, 0] = (gnss_data.horizontal_accuracy / 111320.0) ** 2
        self.gnss_noise[1, 1] = (gnss_data.horizontal_accuracy / 111320.0) ** 2
        self.gnss_noise[2, 2] = gnss_data.vertical_accuracy ** 2
        self.gnss_noise[3, 3] = gnss_data.velocity_accuracy ** 2
        self.gnss_noise[4, 4] = gnss_data.velocity_accuracy ** 2
        self.gnss_noise[5, 5] = gnss_data.velocity_accuracy ** 2
        
        # Innovation covariance
        S = H @ self.covariance @ H.T + self.gnss_noise
        
        # Kalman gain
        K = self.covariance @ H.T @ np.linalg.inv(S)
        
        # State update
        self.state += K @ y
        
        # Covariance update
        I = np.eye(9)
        self.covariance = (I - K @ H) @ self.covariance
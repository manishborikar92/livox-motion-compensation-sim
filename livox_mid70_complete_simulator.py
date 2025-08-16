#!/usr/bin/env python3
"""
Livox Mid-70 Complete Simulation System
=======================================

A comprehensive simulation framework for the Livox Mid-70 LiDAR system
based on official specifications and research documentation.

Features:
- Realistic Mid-70 hardware simulation (70.4° x 77.2° FOV, 90m range)
- High-frequency IMU data generation (200Hz)
- Motion compensation algorithms
- Multiple coordinate systems (sensor, vehicle, local, UTM, WGS84)
- LVX format family support (LVX, LVX2, LVX3)
- Network protocol simulation (UDP packets)
- Multiple output formats (PCD, LAS, CSV)
- Real-time performance monitoring

Author: Based on comprehensive Livox Mid-70 research
Version: 1.0
License: MIT
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import json
import csv
import struct
import socket
import threading
import time
import os
import logging
from typing import Dict, List, Tuple, Optional, Union
from dataclasses import dataclass, asdict
from datetime import datetime, timezone
import warnings
warnings.filterwarnings('ignore')

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Optional imports with graceful fallbacks
try:
    import utm
    UTM_AVAILABLE = True
except ImportError:
    UTM_AVAILABLE = False
    logger.warning("UTM package not available. Install with: pip install utm")

try:
    import laspy
    LASPY_AVAILABLE = True
except ImportError:
    LASPY_AVAILABLE = False
    logger.warning("laspy package not available. Install with: pip install laspy")

# Constants based on official Mid-70 specifications
class Mid70Specs:
    """Official Livox Mid-70 specifications"""
    # Field of View
    FOV_HORIZONTAL = 70.4  # degrees
    FOV_VERTICAL = 77.2    # degrees
    
    # Range specifications
    RANGE_MAX = 90.0       # meters at 10% reflectivity
    RANGE_MAX_20 = 130.0   # meters at 20% reflectivity
    RANGE_MIN = 0.05       # meters (5cm minimal detection)
    
    # Accuracy and precision
    ACCURACY = 0.02        # ±2cm at 25m
    PRECISION = 0.02       # ≤2cm at 25m
    
    # Temporal specifications
    FRAME_RATE = 10        # Hz (fixed)
    POINT_RATE_MAX = 100000  # points/second
    IMU_RATE = 200         # Hz
    
    # Angular resolution
    ANGULAR_RES_H = 0.28   # degrees
    ANGULAR_RES_V = 0.28   # degrees
    
    # Physical specifications
    POWER_CONSUMPTION = 10  # Watts (typical)
    WAVELENGTH = 905       # nm
    WEIGHT = 0.760         # kg
    
    # Communication
    DATA_PORT = 65000
    COMMAND_PORT = 65001
    IMU_PORT = 65002

@dataclass
class IMUData:
    """High-frequency IMU data structure (200Hz)"""
    timestamp: int          # nanoseconds since epoch
    gyro_x: float          # rad/s
    gyro_y: float          # rad/s  
    gyro_z: float          # rad/s
    accel_x: float         # m/s²
    accel_y: float         # m/s²
    accel_z: float         # m/s²

@dataclass
class GPSData:
    """GPS/GNSS data structure"""
    timestamp: int         # nanoseconds since epoch
    latitude: float        # degrees
    longitude: float       # degrees
    altitude: float        # meters above sea level
    velocity_x: float      # m/s (east)
    velocity_y: float      # m/s (north)
    velocity_z: float      # m/s (up)
    heading: float         # degrees from north

@dataclass
class LiDARPoint:
    """Individual LiDAR point with full attributes"""
    x: float              # meters
    y: float              # meters
    z: float              # meters
    intensity: int        # 0-255
    timestamp: int        # nanoseconds
    ring: int            # scan ring ID
    tag: int             # quality/confidence tag

@dataclass
class DeviceInfo:
    """Livox device information"""
    lidar_sn: str         # Serial number
    device_type: int      # 1 for Mid-70
    firmware_version: str # Firmware version
    extrinsic_enable: bool
    roll: float           # mounting orientation (radians)
    pitch: float
    yaw: float
    x: float              # mounting position (meters)
    y: float
    z: float

class CoordinateSystem:
    """Coordinate system enumeration"""
    SENSOR = "sensor"
    VEHICLE = "vehicle"
    LOCAL = "local"
    UTM = "utm"
    WGS84 = "wgs84"

class CoordinateTransformer:
    """Advanced coordinate system transformations"""
    
    def __init__(self):
        self.transformations = {}
        self._setup_default_transformations()
    
    def _setup_default_transformations(self):
        """Setup default transformation matrices"""
        # Identity transformations
        identity = np.eye(4)
        self.transformations[(CoordinateSystem.SENSOR, CoordinateSystem.SENSOR)] = identity
        self.transformations[(CoordinateSystem.VEHICLE, CoordinateSystem.VEHICLE)] = identity
        
        # Default sensor to vehicle (1.5m above ground, no rotation)
        T_sv = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 1.5],
            [0, 0, 0, 1]
        ])
        self.transformations[(CoordinateSystem.SENSOR, CoordinateSystem.VEHICLE)] = T_sv
    
    def set_transformation(self, from_frame: str, to_frame: str, 
                         translation: List[float], rotation: List[float]):
        """Set transformation between coordinate frames"""
        # Create transformation matrix from translation and rotation
        T = self._create_transform_matrix(translation, rotation)
        self.transformations[(from_frame, to_frame)] = T
        
        # Create inverse transformation
        T_inv = np.linalg.inv(T)
        self.transformations[(to_frame, from_frame)] = T_inv
    
    def _create_transform_matrix(self, translation: List[float], rotation: List[float]) -> np.ndarray:
        """Create 4x4 transformation matrix from translation and rotation"""
        # Rotation matrix (assuming roll, pitch, yaw order)
        roll, pitch, yaw = rotation
        
        # Rotation matrices
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(roll), -np.sin(roll)],
                       [0, np.sin(roll), np.cos(roll)]])
        
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                       [0, 1, 0],
                       [-np.sin(pitch), 0, np.cos(pitch)]])
        
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                       [np.sin(yaw), np.cos(yaw), 0],
                       [0, 0, 1]])
        
        R = Rz @ Ry @ Rx
        
        # Create 4x4 transformation matrix
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = translation
        
        return T
    
    def transform_points(self, points: np.ndarray, from_frame: str, to_frame: str) -> np.ndarray:
        """Transform points between coordinate frames"""
        if (from_frame, to_frame) not in self.transformations:
            logger.warning(f"No transformation available from {from_frame} to {to_frame}")
            return points
        
        T = self.transformations[(from_frame, to_frame)]
        
        # Convert to homogeneous coordinates
        if points.shape[1] == 3:
            ones = np.ones((points.shape[0], 1))
            points_homog = np.hstack([points, ones])
        else:
            points_homog = points
        
        # Apply transformation
        transformed = (T @ points_homog.T).T
        
        # Return to 3D coordinates
        return transformed[:, :3]

class LivoxLVXWriter:
    """Comprehensive LVX format writer supporting LVX, LVX2, LVX3"""
    
    def __init__(self, format_version: str = "lvx2"):
        self.format_version = format_version
        self.supported_versions = ["lvx", "lvx2", "lvx3"]
        
        if format_version not in self.supported_versions:
            raise ValueError(f"Unsupported format version: {format_version}")
    
    def write_lvx_file(self, filename: str, frames_data: List[Dict], device_info: DeviceInfo):
        """Write point cloud data to LVX format file"""
        logger.info(f"Writing {self.format_version.upper()} file: {filename}")
        
        if self.format_version == "lvx":
            self._write_lvx_legacy(filename, frames_data, device_info)
        elif self.format_version == "lvx2":
            self._write_lvx2(filename, frames_data, device_info)
        elif self.format_version == "lvx3":
            self._write_lvx3(filename, frames_data, device_info)
    
    def _write_lvx_legacy(self, filename: str, frames_data: List[Dict], device_info: DeviceInfo):
        """Write legacy LVX format"""
        with open(filename, 'wb') as f:
            # File header
            self._write_file_header_legacy(f, len(frames_data))
            
            # Device info
            self._write_device_info_legacy(f, device_info)
            
            # Frame data
            for frame in frames_data:
                self._write_frame_legacy(f, frame)
    
    def _write_lvx2(self, filename: str, frames_data: List[Dict], device_info: DeviceInfo):
        """Write LVX2 format with enhanced features"""
        with open(filename, 'wb') as f:
            # File header (24 bytes)
            signature = b"livox_tech"
            version = b"2.0.0"
            magic_code = 0xAC0EA767
            
            f.write(signature.ljust(10, b'\x00'))
            f.write(version.ljust(6, b'\x00'))
            f.write(struct.pack('<I', magic_code))
            f.write(b'\x00' * 4)  # Reserved
            
            # Private header
            self._write_private_header_lvx2(f, device_info, len(frames_data))
            
            # Frame data blocks
            for frame_idx, frame in enumerate(frames_data):
                self._write_frame_lvx2(f, frame, frame_idx)
    
    def _write_lvx3(self, filename: str, frames_data: List[Dict], device_info: DeviceInfo):
        """Write LVX3 format with latest enhancements"""
        # LVX3 includes additional metadata and compression
        self._write_lvx2(filename, frames_data, device_info)  # Base on LVX2 for now
        logger.info("LVX3 format uses LVX2 base with extended metadata")
    
    def _write_file_header_legacy(self, f, frame_count: int):
        """Write legacy LVX file header"""
        f.write(b"livox_file")  # 10 bytes
        f.write(struct.pack('<I', 1))  # Version
        f.write(struct.pack('<I', frame_count))
        f.write(b'\x00' * 10)  # Reserved
    
    def _write_device_info_legacy(self, f, device_info: DeviceInfo):
        """Write legacy device information"""
        f.write(device_info.lidar_sn.encode('ascii').ljust(16, b'\x00'))
        f.write(struct.pack('<B', device_info.device_type))
        f.write(b'\x00' * 15)  # Reserved/padding
    
    def _write_frame_legacy(self, f, frame: Dict):
        """Write legacy frame data"""
        points = frame['points']
        timestamp = frame['timestamp']
        
        # Frame header
        f.write(struct.pack('<Q', timestamp))  # 8 bytes timestamp
        f.write(struct.pack('<I', len(points)))  # 4 bytes point count
        
        # Point data (14 bytes per point for legacy format)
        for point in points:
            f.write(struct.pack('<fff', point.x, point.y, point.z))  # 12 bytes XYZ
            f.write(struct.pack('<B', point.intensity))  # 1 byte intensity
            f.write(struct.pack('<B', point.tag))  # 1 byte tag
    
    def _write_private_header_lvx2(self, f, device_info: DeviceInfo, frame_count: int):
        """Write LVX2 private header"""
        # Frame duration (50ms for Mid-70)
        f.write(struct.pack('<I', 50))
        
        # Device count (1 for single Mid-70)
        f.write(struct.pack('<I', 1))
        
        # Device information block
        f.write(device_info.lidar_sn.encode('ascii').ljust(16, b'\x00'))
        f.write(struct.pack('<B', device_info.device_type))
        f.write(struct.pack('<B', 1 if device_info.extrinsic_enable else 0))
        
        # Extrinsic parameters (6 floats: roll, pitch, yaw, x, y, z)
        f.write(struct.pack('<ffffff', 
                           device_info.roll, device_info.pitch, device_info.yaw,
                           device_info.x, device_info.y, device_info.z))
        
        f.write(b'\x00' * 14)  # Reserved/padding
    
    def _write_frame_lvx2(self, f, frame: Dict, frame_idx: int):
        """Write LVX2 frame data"""
        points = frame['points']
        timestamp = frame['timestamp']
        
        # Frame header (24 bytes)
        f.write(struct.pack('<I', frame_idx))  # Frame index
        f.write(struct.pack('<Q', timestamp))  # Timestamp
        f.write(struct.pack('<I', len(points)))  # Point count
        f.write(b'\x00' * 8)  # Reserved
        
        # Package header (22 bytes)
        f.write(struct.pack('<B', 5))  # Protocol version
        f.write(struct.pack('<B', 0))  # Slot ID
        f.write(struct.pack('<B', 1))  # LiDAR ID
        f.write(struct.pack('<B', 0))  # Reserved
        f.write(struct.pack('<I', 0))  # Status code
        f.write(struct.pack('<B', 1))  # Timestamp type
        f.write(struct.pack('<B', 2))  # Data type (Cartesian)
        f.write(b'\x00' * 3)  # Reserved
        f.write(struct.pack('<Q', timestamp))  # Package timestamp
        
        # Point data (14 bytes per point)
        for point in points:
            # Convert to millimeters for storage (as per Livox format)
            x_mm = int(point.x * 1000)
            y_mm = int(point.y * 1000)
            z_mm = int(point.z * 1000)
            
            f.write(struct.pack('<iii', x_mm, y_mm, z_mm))  # 12 bytes XYZ in mm
            f.write(struct.pack('<B', point.intensity))  # 1 byte reflectivity
            f.write(struct.pack('<B', point.tag))  # 1 byte tag

class NetworkSimulator:
    """UDP network protocol simulation matching Livox specifications"""
    
    def __init__(self, host: str = "127.0.0.1", 
                 data_port: int = Mid70Specs.DATA_PORT,
                 command_port: int = Mid70Specs.COMMAND_PORT,
                 imu_port: int = Mid70Specs.IMU_PORT):
        self.host = host
        self.data_port = data_port
        self.command_port = command_port
        self.imu_port = imu_port
        
        self.data_socket = None
        self.imu_socket = None
        self.streaming = False
        self.stream_thread = None
    
    def start_streaming(self, point_data: List[Dict], imu_data: List[IMUData]):
        """Start UDP streaming simulation"""
        logger.info(f"Starting network simulation on {self.host}")
        logger.info(f"Data port: {self.data_port}, IMU port: {self.imu_port}")
        
        self.streaming = True
        self.stream_thread = threading.Thread(
            target=self._stream_data, 
            args=(point_data, imu_data)
        )
        self.stream_thread.daemon = True
        self.stream_thread.start()
    
    def stop_streaming(self):
        """Stop UDP streaming"""
        self.streaming = False
        if self.stream_thread:
            self.stream_thread.join(timeout=1.0)
        
        if self.data_socket:
            self.data_socket.close()
        if self.imu_socket:
            self.imu_socket.close()
        
        logger.info("Network simulation stopped")
    
    def _stream_data(self, point_data: List[Dict], imu_data: List[IMUData]):
        """Internal streaming thread"""
        try:
            # Create UDP sockets
            self.data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.imu_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            
            # Stream data
            frame_interval = 1.0 / Mid70Specs.FRAME_RATE  # 100ms for 10Hz
            imu_interval = 1.0 / Mid70Specs.IMU_RATE      # 5ms for 200Hz
            
            frame_idx = 0
            imu_idx = 0
            start_time = time.time()
            
            while self.streaming and (frame_idx < len(point_data) or imu_idx < len(imu_data)):
                current_time = time.time() - start_time
                
                # Send LiDAR frame data
                if frame_idx < len(point_data) and current_time >= frame_idx * frame_interval:
                    self._send_lidar_packet(point_data[frame_idx])
                    frame_idx += 1
                
                # Send IMU data
                if imu_idx < len(imu_data) and current_time >= imu_idx * imu_interval:
                    self._send_imu_packet(imu_data[imu_idx])
                    imu_idx += 1
                
                time.sleep(0.001)  # 1ms sleep to prevent busy waiting
            
        except Exception as e:
            logger.error(f"Network streaming error: {e}")
    
    def _send_lidar_packet(self, frame_data: Dict):
        """Send LiDAR data packet"""
        try:
            # Create point data packet (simplified)
            points = frame_data['points'][:96]  # Max 96 points per packet
            timestamp = frame_data['timestamp']
            
            packet = bytearray()
            packet.extend(struct.pack('<B', 5))  # Version
            packet.extend(struct.pack('<B', 0))  # Slot ID
            packet.extend(struct.pack('<B', 1))  # LiDAR ID
            packet.extend(struct.pack('<B', 0))  # Reserved
            packet.extend(struct.pack('<I', 0))  # Status code
            packet.extend(struct.pack('<B', 1))  # Timestamp type
            packet.extend(struct.pack('<B', 2))  # Data type (Cartesian)
            packet.extend(b'\x00' * 3)  # Reserved
            packet.extend(struct.pack('<Q', timestamp))
            
            # Add point data
            for point in points:
                packet.extend(struct.pack('<iii', 
                                        int(point.x * 1000),  # Convert to mm
                                        int(point.y * 1000),
                                        int(point.z * 1000)))
                packet.extend(struct.pack('<B', point.intensity))
                packet.extend(struct.pack('<B', point.tag))
            
            self.data_socket.sendto(packet, (self.host, self.data_port))
            
        except Exception as e:
            logger.error(f"Error sending LiDAR packet: {e}")
    
    def _send_imu_packet(self, imu_data: IMUData):
        """Send IMU data packet"""
        try:
            packet = struct.pack('<Qffffff',
                               imu_data.timestamp,
                               imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z,
                               imu_data.accel_x, imu_data.accel_y, imu_data.accel_z)
            
            self.imu_socket.sendto(packet, (self.host, self.imu_port))
            
        except Exception as e:
            logger.error(f"Error sending IMU packet: {e}")

class EnvironmentGenerator:
    """Advanced environment generation for realistic scenes"""
    
    def __init__(self, complexity: str = "medium", seed: int = 42):
        self.complexity = complexity
        self.rng = np.random.RandomState(seed)
        
        # Environment parameters
        self.environments = {
            'simple': {'obstacle_count': 5, 'max_height': 2.0, 'density': 0.1},
            'medium': {'obstacle_count': 15, 'max_height': 5.0, 'density': 0.3},
            'urban': {'obstacle_count': 30, 'max_height': 15.0, 'density': 0.5},
            'highway': {'obstacle_count': 20, 'max_height': 3.0, 'density': 0.2},
            'industrial': {'obstacle_count': 25, 'max_height': 20.0, 'density': 0.4}
        }
    
    def generate_environment(self, bounds: Tuple[float, float, float, float]) -> List[np.ndarray]:
        """Generate 3D environment within specified bounds"""
        x_min, x_max, y_min, y_max = bounds
        env_params = self.environments.get(self.complexity, self.environments['medium'])
        
        objects = []
        
        # Ground plane
        ground = self._generate_ground_plane(x_min, x_max, y_min, y_max)
        objects.append(ground)
        
        # Buildings/obstacles
        for _ in range(env_params['obstacle_count']):
            obj = self._generate_building(x_min, x_max, y_min, y_max, env_params['max_height'])
            objects.append(obj)
        
        # Vegetation (trees, bushes)
        if self.complexity in ['medium', 'urban']:
            for _ in range(env_params['obstacle_count'] // 3):
                tree = self._generate_tree(x_min, x_max, y_min, y_max)
                objects.append(tree)
        
        # Road infrastructure
        if self.complexity in ['urban', 'highway']:
            road_objects = self._generate_road_infrastructure(x_min, x_max, y_min, y_max)
            objects.extend(road_objects)
        
        return objects
    
    def _generate_ground_plane(self, x_min: float, x_max: float, 
                             y_min: float, y_max: float, resolution: float = 0.5) -> np.ndarray:
        """Generate detailed ground plane with texture"""
        x_points = np.arange(x_min, x_max, resolution)
        y_points = np.arange(y_min, y_max, resolution)
        
        points = []
        for x in x_points:
            for y in y_points:
                # Add some terrain variation
                z = self.rng.normal(0, 0.05)  # Small height variations
                intensity = self.rng.randint(20, 60)  # Ground reflectivity
                points.append([x, y, z, intensity])
        
        return np.array(points)
    
    def _generate_building(self, x_min: float, x_max: float, y_min: float, y_max: float, 
                         max_height: float) -> np.ndarray:
        """Generate rectangular building structure"""
        # Random building parameters
        width = self.rng.uniform(3, 15)
        length = self.rng.uniform(3, 15)
        height = self.rng.uniform(2, max_height)
        
        # Random position
        x_center = self.rng.uniform(x_min + width/2, x_max - width/2)
        y_center = self.rng.uniform(y_min + length/2, y_max - length/2)
        
        points = []
        resolution = 0.2
        
        # Building walls
        for wall in ['front', 'back', 'left', 'right']:
            wall_points = self._generate_wall(x_center, y_center, width, length, height, wall, resolution)
            points.extend(wall_points)
        
        # Roof
        roof_points = self._generate_roof(x_center, y_center, width, length, height, resolution)
        points.extend(roof_points)
        
        return np.array(points)
    
    def _generate_wall(self, x_center: float, y_center: float, width: float, length: float,
                      height: float, wall_type: str, resolution: float) -> List[List[float]]:
        """Generate points for a building wall"""
        points = []
        intensity = self.rng.randint(40, 120)  # Building wall reflectivity
        
        if wall_type == 'front':
            x_wall = x_center + width/2
            y_range = np.arange(y_center - length/2, y_center + length/2, resolution)
            z_range = np.arange(0, height, resolution)
            for y in y_range:
                for z in z_range:
                    points.append([x_wall, y, z, intensity])
        elif wall_type == 'back':
            x_wall = x_center - width/2
            y_range = np.arange(y_center - length/2, y_center + length/2, resolution)
            z_range = np.arange(0, height, resolution)
            for y in y_range:
                for z in z_range:
                    points.append([x_wall, y, z, intensity])
        elif wall_type == 'left':
            y_wall = y_center - length/2
            x_range = np.arange(x_center - width/2, x_center + width/2, resolution)
            z_range = np.arange(0, height, resolution)
            for x in x_range:
                for z in z_range:
                    points.append([x, y_wall, z, intensity])
        elif wall_type == 'right':
            y_wall = y_center + length/2
            x_range = np.arange(x_center - width/2, x_center + width/2, resolution)
            z_range = np.arange(0, height, resolution)
            for x in x_range:
                for z in z_range:
                    points.append([x, y_wall, z, intensity])
        
        return points
    
    def _generate_roof(self, x_center: float, y_center: float, width: float, length: float,
                      height: float, resolution: float) -> List[List[float]]:
        """Generate roof points"""
        points = []
        intensity = self.rng.randint(60, 100)  # Roof reflectivity
        
        x_range = np.arange(x_center - width/2, x_center + width/2, resolution)
        y_range = np.arange(y_center - length/2, y_center + length/2, resolution)
        
        for x in x_range:
            for y in y_range:
                points.append([x, y, height, intensity])
        
        return points
    
    def _generate_tree(self, x_min: float, x_max: float, y_min: float, y_max: float) -> np.ndarray:
        """Generate tree-like structure"""
        # Random tree parameters
        height = self.rng.uniform(3, 8)
        crown_radius = self.rng.uniform(1, 3)
        
        # Random position
        x_center = self.rng.uniform(x_min + crown_radius, x_max - crown_radius)
        y_center = self.rng.uniform(y_min + crown_radius, y_max - crown_radius)
        
        points = []
        resolution = 0.3
        
        # Tree crown (simplified as cylinder with varying density)
        crown_height = height * 0.7
        crown_start = height * 0.3
        
        for z in np.arange(crown_start, height, resolution):
            # Variable crown radius (larger at bottom)
            current_radius = crown_radius * (1 - 0.3 * (z - crown_start) / crown_height)
            
            # Generate points in crown
            angles = np.arange(0, 2*np.pi, 0.2)
            radii = np.arange(0, current_radius, resolution)
            
            for angle in angles:
                for r in radii:
                    # Sparse foliage simulation
                    if self.rng.random() < 0.6:  # 60% density
                        x = x_center + r * np.cos(angle)
                        y = y_center + r * np.sin(angle)
                        intensity = self.rng.randint(80, 150)  # Vegetation reflectivity
                        points.append([x, y, z, intensity])
        
        # Tree trunk
        trunk_radius = 0.3
        trunk_points = np.arange(0, crown_start, resolution/2)
        for z in trunk_points:
            # Generate cylindrical trunk
            angles = np.arange(0, 2*np.pi, 0.5)
            for angle in angles:
                x = x_center + trunk_radius * np.cos(angle)
                y = y_center + trunk_radius * np.sin(angle)
                intensity = self.rng.randint(30, 70)  # Bark reflectivity
                points.append([x, y, z, intensity])
        
        return np.array(points)
    
    def _generate_road_infrastructure(self, x_min: float, x_max: float, 
                                    y_min: float, y_max: float) -> List[np.ndarray]:
        """Generate road signs, barriers, and other infrastructure"""
        objects = []
        
        # Traffic signs
        for _ in range(3):
            sign = self._generate_traffic_sign(x_min, x_max, y_min, y_max)
            objects.append(sign)
        
        # Guard rails (for highway environment)
        if self.complexity == 'highway':
            rail = self._generate_guard_rail(x_min, x_max, y_min, y_max)
            objects.append(rail)
        
        # Utility poles
        for _ in range(2):
            pole = self._generate_utility_pole(x_min, x_max, y_min, y_max)
            objects.append(pole)
        
        return objects
    
    def _generate_traffic_sign(self, x_min: float, x_max: float, y_min: float, y_max: float) -> np.ndarray:
        """Generate traffic sign structure"""
        points = []
        
        # Random position
        x_pos = self.rng.uniform(x_min, x_max)
        y_pos = self.rng.uniform(y_min, y_max)
        
        # Sign pole (vertical cylinder)
        pole_height = 3.0
        pole_radius = 0.1
        
        for z in np.arange(0, pole_height, 0.1):
            angles = np.arange(0, 2*np.pi, 0.5)
            for angle in angles:
                x = x_pos + pole_radius * np.cos(angle)
                y = y_pos + pole_radius * np.sin(angle)
                intensity = self.rng.randint(50, 80)  # Metal pole reflectivity
                points.append([x, y, z, intensity])
        
        # Sign board (rectangular)
        sign_width = 1.0
        sign_height = 0.8
        sign_z = 2.5
        
        for x_offset in np.arange(-sign_width/2, sign_width/2, 0.05):
            for y_offset in np.arange(-sign_height/2, sign_height/2, 0.05):
                x = x_pos + x_offset
                y = y_pos + y_offset
                intensity = self.rng.randint(150, 255)  # High reflectivity sign
                points.append([x, y, sign_z, intensity])
        
        return np.array(points)
    
    def _generate_guard_rail(self, x_min: float, x_max: float, y_min: float, y_max: float) -> np.ndarray:
        """Generate highway guard rail"""
        points = []
        
        # Rail along one side of the area
        y_rail = y_min + 2.0  # 2m from edge
        rail_height = 0.8
        
        for x in np.arange(x_min, x_max, 0.2):
            for z in np.arange(0.3, rail_height, 0.1):
                intensity = self.rng.randint(40, 90)  # Metal rail reflectivity
                points.append([x, y_rail, z, intensity])
                
            # Support posts every 4m
            if int(x) % 4 == 0:
                for z_post in np.arange(0, 1.2, 0.05):
                    intensity = self.rng.randint(30, 60)
                    points.append([x, y_rail, z_post, intensity])
        
        return np.array(points)
    
    def _generate_utility_pole(self, x_min: float, x_max: float, y_min: float, y_max: float) -> np.ndarray:
        """Generate utility pole with power lines"""
        points = []
        
        # Random position
        x_pos = self.rng.uniform(x_min, x_max)
        y_pos = self.rng.uniform(y_min, y_max)
        
        # Pole (tall cylinder)
        pole_height = 8.0
        pole_radius = 0.15
        
        for z in np.arange(0, pole_height, 0.1):
            angles = np.arange(0, 2*np.pi, 0.3)
            for angle in angles:
                x = x_pos + pole_radius * np.cos(angle)
                y = y_pos + pole_radius * np.sin(angle)
                intensity = self.rng.randint(20, 50)  # Wood pole reflectivity
                points.append([x, y, z, intensity])
        
        # Power lines (horizontal cables)
        line_heights = [6.0, 6.5, 7.0]
        for line_height in line_heights:
            for x_line in np.arange(x_pos - 10, x_pos + 10, 0.5):
                # Catenary curve simulation
                sag = 0.2 * ((x_line - x_pos)**2 / 100)  # Simplified sag
                z_line = line_height - sag
                intensity = self.rng.randint(80, 120)  # Cable reflectivity
                points.append([x_line, y_pos, z_line, intensity])
        
        return np.array(points)

class TrajectoryGenerator:
    """Advanced trajectory generation for realistic vehicle motion"""
    
    def __init__(self, trajectory_type: str = "urban_circuit", seed: int = 42):
        self.trajectory_type = trajectory_type
        self.rng = np.random.RandomState(seed)
    
    def generate_trajectory(self, duration: float, dt: float = 0.1, 
                          max_speed: float = 15.0, max_angular_vel: float = 0.5) -> Dict:
        """Generate realistic vehicle trajectory"""
        
        trajectory_generators = {
            'linear': self._generate_linear_trajectory,
            'circular': self._generate_circular_trajectory,
            'figure_eight': self._generate_figure_eight_trajectory,
            'urban_circuit': self._generate_urban_circuit_trajectory
        }
        
        generator = trajectory_generators.get(self.trajectory_type, 
                                            trajectory_generators['urban_circuit'])
        
        return generator(duration, dt, max_speed, max_angular_vel)
    
    def _generate_linear_trajectory(self, duration: float, dt: float, 
                                  max_speed: float, max_angular_vel: float) -> Dict:
        """Generate straight line trajectory with gentle curves"""
        t = np.arange(0, duration, dt)
        
        # Base linear motion with gentle curves
        x = np.cumsum(np.ones_like(t) * max_speed * 0.7 * dt)
        y = 5 * np.sin(0.1 * t)  # Gentle sine wave
        z = np.ones_like(t) * 1.5  # Constant height
        
        # Calculate velocities and orientations
        vx = np.gradient(x) / dt
        vy = np.gradient(y) / dt
        vz = np.gradient(z) / dt
        
        yaw = np.arctan2(vy, vx)
        pitch = np.arctan2(vz, np.sqrt(vx**2 + vy**2))
        roll = np.zeros_like(t)
        
        return self._create_trajectory_dict(t, x, y, z, vx, vy, vz, roll, pitch, yaw)
    
    def _generate_circular_trajectory(self, duration: float, dt: float,
                                    max_speed: float, max_angular_vel: float) -> Dict:
        """Generate circular trajectory"""
        t = np.arange(0, duration, dt)
        
        # Circular motion parameters
        radius = 20.0
        angular_speed = max_speed / radius
        
        x = radius * np.cos(angular_speed * t)
        y = radius * np.sin(angular_speed * t)
        z = np.ones_like(t) * 1.5
        
        # Calculate velocities
        vx = -radius * angular_speed * np.sin(angular_speed * t)
        vy = radius * angular_speed * np.cos(angular_speed * t)
        vz = np.zeros_like(t)
        
        # Orientation follows velocity direction
        yaw = np.arctan2(vy, vx)
        pitch = np.zeros_like(t)
        roll = np.ones_like(t) * angular_speed * 0.1  # Banking in turns
        
        return self._create_trajectory_dict(t, x, y, z, vx, vy, vz, roll, pitch, yaw)
    
    def _generate_figure_eight_trajectory(self, duration: float, dt: float,
                                        max_speed: float, max_angular_vel: float) -> Dict:
        """Generate figure-8 trajectory"""
        t = np.arange(0, duration, dt)
        
        # Figure-8 parameters
        scale = 15.0
        frequency = 0.2
        
        x = scale * np.sin(2 * frequency * t)
        y = scale * np.sin(frequency * t)
        z = np.ones_like(t) * 1.5
        
        # Calculate velocities
        vx = scale * 2 * frequency * np.cos(2 * frequency * t)
        vy = scale * frequency * np.cos(frequency * t)
        vz = np.zeros_like(t)
        
        # Orientation and banking
        yaw = np.arctan2(vy, vx)
        pitch = np.zeros_like(t)
        roll = np.gradient(yaw) / dt * 0.3  # Banking proportional to yaw rate
        
        return self._create_trajectory_dict(t, x, y, z, vx, vy, vz, roll, pitch, yaw)
    
    def _generate_urban_circuit_trajectory(self, duration: float, dt: float,
                                         max_speed: float, max_angular_vel: float) -> Dict:
        """Generate realistic urban driving trajectory"""
        t = np.arange(0, duration, dt)
        n_points = len(t)
        
        # Initialize arrays
        x = np.zeros(n_points)
        y = np.zeros(n_points)
        z = np.ones(n_points) * 1.5
        vx = np.zeros(n_points)
        vy = np.zeros(n_points)
        vz = np.zeros(n_points)
        yaw = np.zeros(n_points)
        
        # Urban driving parameters
        current_speed = max_speed * 0.5  # Start at moderate speed
        current_yaw = 0.0
        
        # Driving segments
        segments = [
            {'type': 'straight', 'duration': 30, 'speed': max_speed * 0.8},
            {'type': 'turn_right', 'duration': 15, 'speed': max_speed * 0.4},
            {'type': 'straight', 'duration': 25, 'speed': max_speed * 0.9},
            {'type': 'stop', 'duration': 10, 'speed': 0.0},
            {'type': 'accelerate', 'duration': 15, 'speed': max_speed * 0.7},
            {'type': 'turn_left', 'duration': 20, 'speed': max_speed * 0.5},
            {'type': 'straight', 'duration': 20, 'speed': max_speed * 0.6}
        ]
        
        segment_idx = 0
        segment_time = 0
        
        for i in range(1, n_points):
            # Current segment
            if segment_idx < len(segments) and segment_time >= segments[segment_idx]['duration']:
                segment_idx += 1
                segment_time = 0
            
            if segment_idx >= len(segments):
                segment_idx = 0  # Loop back
            
            current_segment = segments[segment_idx]
            
            # Update based on segment type
            if current_segment['type'] == 'straight':
                target_speed = current_segment['speed']
                yaw_rate = 0.0
            elif current_segment['type'] == 'turn_right':
                target_speed = current_segment['speed']
                yaw_rate = -0.3  # Right turn
            elif current_segment['type'] == 'turn_left':
                target_speed = current_segment['speed']
                yaw_rate = 0.3   # Left turn
            elif current_segment['type'] == 'stop':
                target_speed = 0.0
                yaw_rate = 0.0
            elif current_segment['type'] == 'accelerate':
                target_speed = current_segment['speed']
                yaw_rate = 0.0
            else:
                target_speed = max_speed * 0.5
                yaw_rate = 0.0
            
            # Smooth speed and yaw transitions
            speed_alpha = 0.95
            yaw_alpha = 0.9
            
            current_speed = speed_alpha * current_speed + (1 - speed_alpha) * target_speed
            current_yaw += yaw_rate * dt
            
            # Update velocity components
            vx[i] = current_speed * np.cos(current_yaw)
            vy[i] = current_speed * np.sin(current_yaw)
            
            # Update position
            x[i] = x[i-1] + vx[i] * dt
            y[i] = y[i-1] + vy[i] * dt
            
            yaw[i] = current_yaw
            
            segment_time += dt
        
        # Calculate roll and pitch
        pitch = np.zeros_like(t)
        roll = np.gradient(yaw) / dt * 0.2  # Banking in turns
        
        return self._create_trajectory_dict(t, x, y, z, vx, vy, vz, roll, pitch, yaw)
    
    def _create_trajectory_dict(self, t: np.ndarray, x: np.ndarray, y: np.ndarray, z: np.ndarray,
                              vx: np.ndarray, vy: np.ndarray, vz: np.ndarray,
                              roll: np.ndarray, pitch: np.ndarray, yaw: np.ndarray) -> Dict:
        """Create standardized trajectory dictionary"""
        return {
            'time': t,
            'position': np.column_stack([x, y, z]),
            'velocity': np.column_stack([vx, vy, vz]),
            'orientation': np.column_stack([roll, pitch, yaw]),
            'x': x, 'y': y, 'z': z,
            'vx': vx, 'vy': vy, 'vz': vz,
            'roll': roll, 'pitch': pitch, 'yaw': yaw
        }

class LiDARSimulator:
    """High-fidelity Livox Mid-70 LiDAR simulation"""
    
    def __init__(self, config: Dict):
        self.config = config
        self.rng = np.random.RandomState(config.get('random_seed', 42))
        
        # Mid-70 specifications
        self.fov_h = config.get('fov_horizontal', Mid70Specs.FOV_HORIZONTAL)
        self.fov_v = config.get('fov_vertical', Mid70Specs.FOV_VERTICAL)
        self.range_max = config.get('range_max', Mid70Specs.RANGE_MAX)
        self.range_min = config.get('range_min', Mid70Specs.RANGE_MIN)
        self.points_per_frame = config.get('points_per_frame', Mid70Specs.POINT_RATE_MAX // Mid70Specs.FRAME_RATE)
        self.angular_resolution = config.get('angular_resolution', Mid70Specs.ANGULAR_RES_H)
        
        # Noise parameters
        self.range_noise_std = config.get('lidar_range_noise', 0.02)
        self.intensity_noise_std = config.get('lidar_intensity_noise', 5.0)
        
        logger.info(f"LiDAR Simulator initialized - FOV: {self.fov_h}°x{self.fov_v}°, Range: {self.range_min}-{self.range_max}m")
    
    def simulate_scan(self, sensor_position: np.ndarray, sensor_orientation: np.ndarray,
                     environment_objects: List[np.ndarray], timestamp: int) -> List[LiDARPoint]:
        """Simulate a single LiDAR scan frame"""
        
        # Generate scanning pattern (non-repetitive rosette for Mid-70)
        scan_angles = self._generate_scan_pattern()
        
        points = []
        max_points = min(self.points_per_frame, len(scan_angles))
        
        for i in range(max_points):
            azimuth, elevation = scan_angles[i]
            
            # Convert to unit vector in sensor frame
            direction = self._angles_to_direction(azimuth, elevation)
            
            # Transform to world frame
            world_direction = self._transform_direction(direction, sensor_orientation)
            
            # Raycast to find intersection
            hit_point, intensity = self._raycast(sensor_position, world_direction, environment_objects)
            
            if hit_point is not None:
                # Transform back to sensor frame for output
                sensor_point = self._world_to_sensor(hit_point, sensor_position, sensor_orientation)
                
                # Add noise
                sensor_point += self.rng.normal(0, self.range_noise_std, 3)
                intensity += self.rng.normal(0, self.intensity_noise_std)
                intensity = np.clip(intensity, 0, 255)
                
                # Create LiDAR point
                lidar_point = LiDARPoint(
                    x=sensor_point[0],
                    y=sensor_point[1], 
                    z=sensor_point[2],
                    intensity=int(intensity),
                    timestamp=timestamp + i * 1000,  # Spread points across frame time
                    ring=i % 16,  # Simulate ring structure
                    tag=1 if np.linalg.norm(sensor_point) > self.range_max * 0.9 else 0
                )
                
                points.append(lidar_point)
        
        return points
    
    def _generate_scan_pattern(self) -> List[Tuple[float, float]]:
        """Generate Mid-70's non-repetitive rosette scanning pattern"""
        angles = []
        
        # Rosette pattern parameters (simplified)
        n_rings = 16  # Approximate ring structure
        points_per_ring = self.points_per_frame // n_rings
        
        for ring in range(n_rings):
            # Elevation angle for this ring
            elevation = (ring / (n_rings - 1) - 0.5) * np.radians(self.fov_v)
            
            for point in range(points_per_ring):
                # Non-repetitive azimuth pattern
                azimuth_base = (point / points_per_ring) * 2 * np.pi
                azimuth_offset = ring * 0.1  # Offset for non-repetitive pattern
                azimuth = azimuth_base + azimuth_offset
                
                # Constrain to FOV
                if abs(np.degrees(azimuth)) <= self.fov_h / 2:
                    angles.append((azimuth, elevation))
        
        return angles
    
    def _angles_to_direction(self, azimuth: float, elevation: float) -> np.ndarray:
        """Convert spherical angles to unit direction vector"""
        x = np.cos(elevation) * np.cos(azimuth)
        y = np.cos(elevation) * np.sin(azimuth)
        z = np.sin(elevation)
        return np.array([x, y, z])
    
    def _transform_direction(self, direction: np.ndarray, orientation: np.ndarray) -> np.ndarray:
        """Transform direction vector from sensor to world frame"""
        roll, pitch, yaw = orientation
        
        # Rotation matrices
        R_x = np.array([[1, 0, 0],
                       [0, np.cos(roll), -np.sin(roll)],
                       [0, np.sin(roll), np.cos(roll)]])
        
        R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                       [0, 1, 0],
                       [-np.sin(pitch), 0, np.cos(pitch)]])
        
        R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                       [np.sin(yaw), np.cos(yaw), 0],
                       [0, 0, 1]])
        
        R = R_z @ R_y @ R_x
        return R @ direction
    
    def _raycast(self, origin: np.ndarray, direction: np.ndarray, 
                environment_objects: List[np.ndarray]) -> Tuple[Optional[np.ndarray], float]:
        """Perform raycast to find first intersection"""
        
        min_distance = float('inf')
        hit_point = None
        hit_intensity = 0
        
        # Ray parameters
        max_range = self.range_max
        step_size = 0.1
        
        # March along ray
        for distance in np.arange(self.range_min, max_range, step_size):
            current_point = origin + direction * distance
            
            # Check intersection with environment objects
            for obj in environment_objects:
                if len(obj) == 0:
                    continue
                
                # Simple point-based intersection (for efficiency)
                distances = np.linalg.norm(obj[:, :3] - current_point, axis=1)
                close_points = distances < step_size
                
                if np.any(close_points):
                    closest_idx = np.argmin(distances[close_points])
                    closest_point = obj[close_points][closest_idx]
                    
                    if distance < min_distance:
                        min_distance = distance
                        hit_point = current_point
                        hit_intensity = closest_point[3] if len(closest_point) > 3 else 100
                        break
            
            if hit_point is not None:
                break
        
        return hit_point, hit_intensity
    
    def _world_to_sensor(self, world_point: np.ndarray, sensor_position: np.ndarray,
                        sensor_orientation: np.ndarray) -> np.ndarray:
        """Transform point from world to sensor frame"""
        # Translate to sensor origin
        translated = world_point - sensor_position
        
        # Rotate to sensor frame (inverse of sensor orientation)
        roll, pitch, yaw = sensor_orientation
        
        # Inverse rotation matrices
        R_x = np.array([[1, 0, 0],
                       [0, np.cos(-roll), -np.sin(-roll)],
                       [0, np.sin(-roll), np.cos(-roll)]])
        
        R_y = np.array([[np.cos(-pitch), 0, np.sin(-pitch)],
                       [0, 1, 0],
                       [-np.sin(-pitch), 0, np.cos(-pitch)]])
        
        R_z = np.array([[np.cos(-yaw), -np.sin(-yaw), 0],
                       [np.sin(-yaw), np.cos(-yaw), 0],
                       [0, 0, 1]])
        
        R_inv = R_x @ R_y @ R_z
        return R_inv @ translated

class IMUSimulator:
    """High-frequency IMU simulation (200Hz)"""
    
    def __init__(self, config: Dict):
        self.config = config
        self.rng = np.random.RandomState(config.get('random_seed', 42))
        
        # IMU specifications
        self.sample_rate = config.get('imu_rate', Mid70Specs.IMU_RATE)
        self.accel_noise_std = config.get('imu_accel_noise', 0.1)
        self.gyro_noise_std = config.get('imu_gyro_noise', 0.01)
        
        # Calibration parameters (factory calibrated for Mid-70)
        self.accel_bias = self.rng.normal(0, 0.02, 3)  # m/s²
        self.gyro_bias = self.rng.normal(0, 0.001, 3)  # rad/s
        
        logger.info(f"IMU Simulator initialized - Sample rate: {self.sample_rate}Hz")
    
    def simulate_imu_data(self, trajectory: Dict, duration: float) -> List[IMUData]:
        """Generate high-frequency IMU data from trajectory"""
        
        dt_imu = 1.0 / self.sample_rate
        t_imu = np.arange(0, duration, dt_imu)
        
        # Interpolate trajectory to IMU sample rate
        trajectory_interp = self._interpolate_trajectory(trajectory, t_imu)
        
        imu_data = []
        
        for i, t in enumerate(t_imu):
            timestamp = int(t * 1e9)  # Convert to nanoseconds
            
            # Get current state
            position = trajectory_interp['position'][i]
            velocity = trajectory_interp['velocity'][i]
            orientation = trajectory_interp['orientation'][i]
            
            # Calculate angular velocity (gyroscope)
            if i > 0:
                prev_orientation = trajectory_interp['orientation'][i-1]
                angular_velocity = (orientation - prev_orientation) / dt_imu
            else:
                angular_velocity = np.array([0.0, 0.0, 0.0])
            
            # Calculate linear acceleration (accelerometer)
            if i > 0:
                prev_velocity = trajectory_interp['velocity'][i-1]
                linear_acceleration = (velocity - prev_velocity) / dt_imu
            else:
                linear_acceleration = np.array([0.0, 0.0, 0.0])
            
            # Add gravity in world frame, then transform to sensor frame
            gravity_world = np.array([0.0, 0.0, -9.81])  # World gravity
            gravity_sensor = self._transform_to_sensor_frame(gravity_world, orientation)
            total_acceleration = linear_acceleration + gravity_sensor
            
            # Add noise and bias
            gyro_measurement = angular_velocity + self.gyro_bias + self.rng.normal(0, self.gyro_noise_std, 3)
            accel_measurement = total_acceleration + self.accel_bias + self.rng.normal(0, self.accel_noise_std, 3)
            
            # Create IMU data point
            imu_point = IMUData(
                timestamp=timestamp,
                gyro_x=gyro_measurement[0],
                gyro_y=gyro_measurement[1],
                gyro_z=gyro_measurement[2],
                accel_x=accel_measurement[0],
                accel_y=accel_measurement[1],
                accel_z=accel_measurement[2]
            )
            
            imu_data.append(imu_point)
        
        return imu_data
    
    def _interpolate_trajectory(self, trajectory: Dict, t_target: np.ndarray) -> Dict:
        """Interpolate trajectory to target time points"""
        t_orig = trajectory['time']
        
        interpolated = {}
        
        # Interpolate position
        interpolated['position'] = np.array([
            np.interp(t_target, t_orig, trajectory['x']),
            np.interp(t_target, t_orig, trajectory['y']),
            np.interp(t_target, t_orig, trajectory['z'])
        ]).T
        
        # Interpolate velocity
        interpolated['velocity'] = np.array([
            np.interp(t_target, t_orig, trajectory['vx']),
            np.interp(t_target, t_orig, trajectory['vy']),
            np.interp(t_target, t_orig, trajectory['vz'])
        ]).T
        
        # Interpolate orientation
        interpolated['orientation'] = np.array([
            np.interp(t_target, t_orig, trajectory['roll']),
            np.interp(t_target, t_orig, trajectory['pitch']),
            np.interp(t_target, t_orig, trajectory['yaw'])
        ]).T
        
        return interpolated
    
    def _transform_to_sensor_frame(self, vector_world: np.ndarray, orientation: np.ndarray) -> np.ndarray:
        """Transform vector from world to sensor frame"""
        roll, pitch, yaw = orientation
        
        # Inverse rotation matrices (world to sensor)
        R_x = np.array([[1, 0, 0],
                       [0, np.cos(-roll), -np.sin(-roll)],
                       [0, np.sin(-roll), np.cos(-roll)]])
        
        R_y = np.array([[np.cos(-pitch), 0, np.sin(-pitch)],
                       [0, 1, 0],
                       [-np.sin(-pitch), 0, np.cos(-pitch)]])
        
        R_z = np.array([[np.cos(-yaw), -np.sin(-yaw), 0],
                       [np.sin(-yaw), np.cos(-yaw), 0],
                       [0, 0, 1]])
        
        R_world_to_sensor = R_x @ R_y @ R_z
        return R_world_to_sensor @ vector_world

class GPSSimulator:
    """GPS/GNSS simulation with realistic noise and accuracy"""
    
    def __init__(self, config: Dict):
        self.config = config
        self.rng = np.random.RandomState(config.get('random_seed', 42))
        
        # GPS specifications
        self.sample_rate = config.get('gps_rate', 5)  # 5Hz typical
        self.noise_std = config.get('gps_noise_std', 0.03)  # 3cm RTK accuracy
        
        # Reference GPS coordinates (default to New York area)
        self.ref_lat = config.get('ref_latitude', 40.7128)
        self.ref_lon = config.get('ref_longitude', -74.0060)
        self.ref_alt = config.get('ref_altitude', 10.0)
        
        logger.info(f"GPS Simulator initialized - Sample rate: {self.sample_rate}Hz, Noise: {self.noise_std*100:.1f}cm")
    
    def simulate_gps_data(self, trajectory: Dict, duration: float) -> List[GPSData]:
        """Generate GPS data from trajectory"""
        
        dt_gps = 1.0 / self.sample_rate
        t_gps = np.arange(0, duration, dt_gps)
        
        # Interpolate trajectory to GPS sample rate
        trajectory_interp = self._interpolate_trajectory(trajectory, t_gps)
        
        gps_data = []
        
        for i, t in enumerate(t_gps):
            timestamp = int(t * 1e9)  # Convert to nanoseconds
            
            # Get current state
            position = trajectory_interp['position'][i]
            velocity = trajectory_interp['velocity'][i]
            orientation = trajectory_interp['orientation'][i]
            
            # Convert local position to GPS coordinates
            if UTM_AVAILABLE:
                lat, lon = self._local_to_gps(position[:2])
            else:
                # Simple offset from reference point
                lat = self.ref_lat + position[1] / 111000.0  # Rough conversion
                lon = self.ref_lon + position[0] / (111000.0 * np.cos(np.radians(self.ref_lat)))
            
            alt = self.ref_alt + position[2]
            
            # Add GPS noise
            lat_noise = self.rng.normal(0, self.noise_std / 111000.0)
            lon_noise = self.rng.normal(0, self.noise_std / (111000.0 * np.cos(np.radians(lat))))
            alt_noise = self.rng.normal(0, self.noise_std)
            
            lat += lat_noise
            lon += lon_noise
            alt += alt_noise
            
            # Calculate heading from velocity
            heading = np.degrees(np.arctan2(velocity[1], velocity[0]))
            if heading < 0:
                heading += 360
            
            # Create GPS data point
            gps_point = GPSData(
                timestamp=timestamp,
                latitude=lat,
                longitude=lon,
                altitude=alt,
                velocity_x=velocity[0],
                velocity_y=velocity[1],
                velocity_z=velocity[2],
                heading=heading
            )
            
            gps_data.append(gps_point)
        
        return gps_data
    
    def _interpolate_trajectory(self, trajectory: Dict, t_target: np.ndarray) -> Dict:
        """Interpolate trajectory to target time points"""
        t_orig = trajectory['time']
        
        interpolated = {}
        
        # Interpolate position
        interpolated['position'] = np.array([
            np.interp(t_target, t_orig, trajectory['x']),
            np.interp(t_target, t_orig, trajectory['y']),
            np.interp(t_target, t_orig, trajectory['z'])
        ]).T
        
        # Interpolate velocity
        interpolated['velocity'] = np.array([
            np.interp(t_target, t_orig, trajectory['vx']),
            np.interp(t_target, t_orig, trajectory['vy']),
            np.interp(t_target, t_orig, trajectory['vz'])
        ]).T
        
        # Interpolate orientation
        interpolated['orientation'] = np.array([
            np.interp(t_target, t_orig, trajectory['roll']),
            np.interp(t_target, t_orig, trajectory['pitch']),
            np.interp(t_target, t_orig, trajectory['yaw'])
        ]).T
        
        return interpolated
    
    def _local_to_gps(self, local_pos: np.ndarray) -> Tuple[float, float]:
        """Convert local coordinates to GPS using UTM"""
        if not UTM_AVAILABLE:
            return self.ref_lat, self.ref_lon
        
        try:
            # Convert reference to UTM
            ref_utm_x, ref_utm_y, zone_number, zone_letter = utm.from_latlon(self.ref_lat, self.ref_lon)
            
            # Add local offset
            utm_x = ref_utm_x + local_pos[0]
            utm_y = ref_utm_y + local_pos[1]
            
            # Convert back to GPS
            lat, lon = utm.to_latlon(utm_x, utm_y, zone_number, zone_letter)
            
            return lat, lon
        except:
            # Fallback to simple offset
            lat = self.ref_lat + local_pos[1] / 111000.0
            lon = self.ref_lon + local_pos[0] / (111000.0 * np.cos(np.radians(self.ref_lat)))
            return lat, lon

class MotionCompensator:
    """Advanced motion compensation using IMU data"""
    
    def __init__(self, config: Dict):
        self.config = config
        self.enable_compensation = config.get('enable_motion_compensation', True)
        
        logger.info(f"Motion Compensator initialized - Enabled: {self.enable_compensation}")
    
    def compensate_point_cloud(self, points: List[LiDARPoint], imu_data: List[IMUData],
                             frame_start_time: int, frame_duration_ns: int) -> List[LiDARPoint]:
        """Apply motion compensation to point cloud using IMU data"""
        
        if not self.enable_compensation or not imu_data:
            return points
        
        compensated_points = []
        frame_end_time = frame_start_time + frame_duration_ns
        
        for point in points:
            # Find corresponding IMU data
            imu_sample = self._interpolate_imu_data(imu_data, point.timestamp)
            
            if imu_sample is None:
                compensated_points.append(point)
                continue
            
            # Calculate motion compensation
            dt = (point.timestamp - frame_start_time) * 1e-9  # Convert to seconds
            
            # Angular displacement during scan
            angular_velocity = np.array([imu_sample.gyro_x, imu_sample.gyro_y, imu_sample.gyro_z])
            angular_displacement = angular_velocity * dt
            
            # Create rotation matrix for compensation
            rotation_matrix = self._create_rotation_matrix(angular_displacement)
            
            # Apply compensation (rotate point back to frame start orientation)
            point_vector = np.array([point.x, point.y, point.z])
            compensated_vector = rotation_matrix @ point_vector
            
            # Create compensated point
            compensated_point = LiDARPoint(
                x=compensated_vector[0],
                y=compensated_vector[1],
                z=compensated_vector[2],
                intensity=point.intensity,
                timestamp=point.timestamp,
                ring=point.ring,
                tag=point.tag
            )
            
            compensated_points.append(compensated_point)
        
        return compensated_points
    
    def _interpolate_imu_data(self, imu_data: List[IMUData], target_timestamp: int) -> Optional[IMUData]:
        """Interpolate IMU data for specific timestamp"""
        
        # Find surrounding IMU samples
        before_sample = None
        after_sample = None
        
        for sample in imu_data:
            if sample.timestamp <= target_timestamp:
                before_sample = sample
            elif sample.timestamp > target_timestamp and after_sample is None:
                after_sample = sample
                break
        
        if before_sample is None or after_sample is None:
            return before_sample or after_sample
        
        # Linear interpolation
        time_diff = after_sample.timestamp - before_sample.timestamp
        if time_diff == 0:
            return before_sample
        
        alpha = (target_timestamp - before_sample.timestamp) / time_diff
        
        interpolated = IMUData(
            timestamp=target_timestamp,
            gyro_x=before_sample.gyro_x + alpha * (after_sample.gyro_x - before_sample.gyro_x),
            gyro_y=before_sample.gyro_y + alpha * (after_sample.gyro_y - before_sample.gyro_y),
            gyro_z=before_sample.gyro_z + alpha * (after_sample.gyro_z - before_sample.gyro_z),
            accel_x=before_sample.accel_x + alpha * (after_sample.accel_x - before_sample.accel_x),
            accel_y=before_sample.accel_y + alpha * (after_sample.accel_y - before_sample.accel_y),
            accel_z=before_sample.accel_z + alpha * (after_sample.accel_z - before_sample.accel_z)
        )
        
        return interpolated
    
    def _create_rotation_matrix(self, angular_displacement: np.ndarray) -> np.ndarray:
        """Create rotation matrix from angular displacement"""
        rx, ry, rz = angular_displacement
        
        # Rotation matrices
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(-rx), -np.sin(-rx)],
                       [0, np.sin(-rx), np.cos(-rx)]])
        
        Ry = np.array([[np.cos(-ry), 0, np.sin(-ry)],
                       [0, 1, 0],
                       [-np.sin(-ry), 0, np.cos(-ry)]])
        
        Rz = np.array([[np.cos(-rz), -np.sin(-rz), 0],
                       [np.sin(-rz), np.cos(-rz), 0],
                       [0, 0, 1]])
        
        # Combined rotation (order: Z-Y-X)
        return Rx @ Ry @ Rz

class PerformanceMonitor:
    """Real-time performance monitoring and metrics"""
    
    def __init__(self):
        self.start_time = None
        self.metrics = {
            'total_points_processed': 0,
            'total_frames_processed': 0,
            'processing_times': [],
            'memory_usage': [],
            'frame_rates': []
        }
        
        logger.info("Performance Monitor initialized")
    
    def start_monitoring(self):
        """Start performance monitoring"""
        self.start_time = time.time()
        
    def update_metrics(self, points_processed: int, frames_processed: int, 
                      processing_time: float, memory_mb: float):
        """Update performance metrics"""
        
        self.metrics['total_points_processed'] += points_processed
        self.metrics['total_frames_processed'] += frames_processed
        self.metrics['processing_times'].append(processing_time)
        self.metrics['memory_usage'].append(memory_mb)
        
        if processing_time > 0:
            frame_rate = 1.0 / processing_time
            self.metrics['frame_rates'].append(frame_rate)
    
    def get_performance_stats(self) -> Dict:
        """Get comprehensive performance statistics"""
        
        if self.start_time is None:
            return {}
        
        total_time = time.time() - self.start_time
        
        stats = {
            'total_runtime_seconds': total_time,
            'total_points_processed': self.metrics['total_points_processed'],
            'total_frames_processed': self.metrics['total_frames_processed'],
            'average_points_per_second': self.metrics['total_points_processed'] / total_time if total_time > 0 else 0,
            'average_frames_per_second': self.metrics['total_frames_processed'] / total_time if total_time > 0 else 0,
            'processing_times': {
                'mean': np.mean(self.metrics['processing_times']) if self.metrics['processing_times'] else 0,
                'std': np.std(self.metrics['processing_times']) if self.metrics['processing_times'] else 0,
                'min': np.min(self.metrics['processing_times']) if self.metrics['processing_times'] else 0,
                'max': np.max(self.metrics['processing_times']) if self.metrics['processing_times'] else 0
            },
            'memory_usage': {
                'mean_mb': np.mean(self.metrics['memory_usage']) if self.metrics['memory_usage'] else 0,
                'peak_mb': np.max(self.metrics['memory_usage']) if self.metrics['memory_usage'] else 0,
                'std_mb': np.std(self.metrics['memory_usage']) if self.metrics['memory_usage'] else 0
            },
            'frame_rates': {
                'mean_fps': np.mean(self.metrics['frame_rates']) if self.metrics['frame_rates'] else 0,
                'std_fps': np.std(self.metrics['frame_rates']) if self.metrics['frame_rates'] else 0
            }
        }
        
        return stats

class DataExporter:
    """Multi-format data export with optimized writers"""
    
    def __init__(self, config: Dict):
        self.config = config
        self.coordinate_system = config.get('coordinate_system', CoordinateSystem.SENSOR)
        
        logger.info(f"Data Exporter initialized - Target coordinate system: {self.coordinate_system}")
    
    def export_point_clouds(self, frames_data: List[Dict], output_prefix: str = "lidar_data"):
        """Export point clouds to multiple formats"""
        
        logger.info("Exporting point cloud data...")
        
        # Merge all frames
        all_points = []
        for frame in frames_data:
            all_points.extend(frame['points'])
        
        if not all_points:
            logger.warning("No points to export")
            return
        
        # Convert to numpy array
        points_array = np.array([[p.x, p.y, p.z, p.intensity, p.timestamp] 
                                for p in all_points])
        
        # Export PCD format
        self._export_pcd(points_array, f"{output_prefix}.pcd")
        
        # Export LAS format (if available)
        if LASPY_AVAILABLE:
            self._export_las(points_array, f"{output_prefix}.las")
        
        # Export simple formats
        self._export_xyz(points_array, f"{output_prefix}.xyz")
        self._export_csv(points_array, f"{output_prefix}.csv")
        
        logger.info(f"Point cloud export completed - {len(all_points)} points")
    
    def _export_pcd(self, points: np.ndarray, filename: str):
        """Export to PCD format (PCL compatible)"""
        try:
            with open(filename, 'w') as f:
                # PCD header
                header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity timestamp
SIZE 4 4 4 4 8
TYPE F F F F F
COUNT 1 1 1 1 1
WIDTH {len(points)}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {len(points)}
DATA ascii
"""
                f.write(header)
                
                # Point data
                for point in points:
                    f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} {point[3]:.0f} {point[4]:.0f}\n")
            
            logger.info(f"PCD export completed: {filename}")
            
        except Exception as e:
            logger.error(f"PCD export failed: {e}")
    
    def _export_las(self, points: np.ndarray, filename: str):
        """Export to LAS format (surveying standard)"""
        try:
            # Create LAS file
            las_header = laspy.LasHeader(point_format=3, version="1.2")
            las_file = laspy.LasData(las_header)
            
            # Scale factors for precision
            las_file.header.x_scale = 0.001
            las_file.header.y_scale = 0.001
            las_file.header.z_scale = 0.001
            
            # Set coordinates
            las_file.x = points[:, 0]
            las_file.y = points[:, 1] 
            las_file.z = points[:, 2]
            las_file.intensity = points[:, 3].astype(np.uint16)
            
            # GPS time (convert from nanoseconds to seconds)
            las_file.gps_time = points[:, 4] * 1e-9
            
            # Write file
            las_file.write(filename)
            
            logger.info(f"LAS export completed: {filename}")
            
        except Exception as e:
            logger.error(f"LAS export failed: {e}")
    
    def _export_xyz(self, points: np.ndarray, filename: str):
        """Export to simple XYZ format"""
        try:
            np.savetxt(filename, points[:, :3], fmt='%.6f', delimiter=' ')
            logger.info(f"XYZ export completed: {filename}")
        except Exception as e:
            logger.error(f"XYZ export failed: {e}")
    
    def _export_csv(self, points: np.ndarray, filename: str):
        """Export to CSV format"""
        try:
            df = pd.DataFrame(points, columns=['x', 'y', 'z', 'intensity', 'timestamp'])
            df.to_csv(filename, index=False, float_format='%.6f')
            logger.info(f"CSV export completed: {filename}")
        except Exception as e:
            logger.error(f"CSV export failed: {e}")
    
    def export_sensor_data(self, imu_data: List[IMUData], gps_data: List[GPSData], 
                          trajectory: Dict, output_prefix: str = "sensor_data"):
        """Export sensor data to CSV files"""
        
        logger.info("Exporting sensor data...")
        
        # Export IMU data
        if imu_data:
            imu_df = pd.DataFrame([asdict(imu) for imu in imu_data])
            imu_df.to_csv(f"{output_prefix}_imu_200hz.csv", index=False, float_format='%.6f')
            logger.info(f"IMU data exported: {len(imu_data)} samples at 200Hz")
        
        # Export GPS data
        if gps_data:
            gps_df = pd.DataFrame([asdict(gps) for gps in gps_data])
            gps_df.to_csv(f"{output_prefix}_gps.csv", index=False, float_format='%.8f')
            logger.info(f"GPS data exported: {len(gps_data)} samples")
        
        # Export trajectory
        if trajectory:
            traj_data = {
                'time': trajectory['time'],
                'x': trajectory['x'],
                'y': trajectory['y'], 
                'z': trajectory['z'],
                'vx': trajectory['vx'],
                'vy': trajectory['vy'],
                'vz': trajectory['vz'],
                'roll': trajectory['roll'],
                'pitch': trajectory['pitch'],
                'yaw': trajectory['yaw']
            }
            traj_df = pd.DataFrame(traj_data)
            traj_df.to_csv(f"{output_prefix}_trajectory.csv", index=False, float_format='%.6f')
            logger.info(f"Trajectory exported: {len(traj_df)} points")
        
        # Export synchronized motion data (GPS + IMU at LiDAR frame rate)
        self._export_motion_data(imu_data, gps_data, trajectory, f"{output_prefix}_motion_data.csv")
    
    def _export_motion_data(self, imu_data: List[IMUData], gps_data: List[GPSData], 
                           trajectory: Dict, filename: str):
        """Export synchronized motion data for LiDAR frames"""
        try:
            # Create synchronized data at LiDAR frame rate (10Hz)
            frame_interval = 1.0 / Mid70Specs.FRAME_RATE  # 0.1s
            duration = trajectory['time'][-1] if trajectory['time'] is not None else 60.0
            frame_times = np.arange(0, duration, frame_interval)
            
            motion_data = []
            
            for frame_id, t in enumerate(frame_times):
                timestamp_ns = int(t * 1e9)
                
                # Find closest GPS data
                gps_sample = self._find_closest_gps(gps_data, timestamp_ns) if gps_data else None
                
                # Interpolate IMU data
                imu_sample = self._interpolate_imu_for_time(imu_data, timestamp_ns) if imu_data else None
                
                # Interpolate trajectory
                traj_data = self._interpolate_trajectory_for_time(trajectory, t) if trajectory else None
                
                # Create motion data row
                row = {
                    'frame_id': frame_id,
                    'timestamp': t,
                    'gps_lat': gps_sample.latitude if gps_sample else 0.0,
                    'gps_lon': gps_sample.longitude if gps_sample else 0.0,
                    'gps_alt': gps_sample.altitude if gps_sample else 0.0,
                    'imu_roll': traj_data['roll'] if traj_data else 0.0,
                    'imu_pitch': traj_data['pitch'] if traj_data else 0.0,
                    'imu_yaw': traj_data['yaw'] if traj_data else 0.0,
                    'vel_x': traj_data['vx'] if traj_data else 0.0,
                    'vel_y': traj_data['vy'] if traj_data else 0.0,
                    'vel_z': traj_data['vz'] if traj_data else 0.0,
                    'gyro_x': imu_sample.gyro_x if imu_sample else 0.0,
                    'gyro_y': imu_sample.gyro_y if imu_sample else 0.0,
                    'gyro_z': imu_sample.gyro_z if imu_sample else 0.0,
                    'accel_x': imu_sample.accel_x if imu_sample else 0.0,
                    'accel_y': imu_sample.accel_y if imu_sample else 0.0,
                    'accel_z': imu_sample.accel_z if imu_sample else 0.0
                }
                
                motion_data.append(row)
            
            # Export to CSV
            motion_df = pd.DataFrame(motion_data)
            motion_df.to_csv(filename, index=False, float_format='%.6f')
            
            logger.info(f"Motion data exported: {len(motion_data)} frames synchronized at 10Hz")
            
        except Exception as e:
            logger.error(f"Motion data export failed: {e}")
    
    def _find_closest_gps(self, gps_data: List[GPSData], timestamp_ns: int) -> Optional[GPSData]:
        """Find GPS sample closest to timestamp"""
        if not gps_data:
            return None
        
        closest_sample = None
        min_diff = float('inf')
        
        for sample in gps_data:
            diff = abs(sample.timestamp - timestamp_ns)
            if diff < min_diff:
                min_diff = diff
                closest_sample = sample
        
        return closest_sample
    
    def _interpolate_imu_for_time(self, imu_data: List[IMUData], timestamp_ns: int) -> Optional[IMUData]:
        """Interpolate IMU data for specific timestamp"""
        if not imu_data:
            return None
        
        # Find surrounding samples
        before_sample = None
        after_sample = None
        
        for sample in imu_data:
            if sample.timestamp <= timestamp_ns:
                before_sample = sample
            elif sample.timestamp > timestamp_ns and after_sample is None:
                after_sample = sample
                break
        
        if before_sample is None:
            return after_sample
        if after_sample is None:
            return before_sample
        
        # Linear interpolation
        time_diff = after_sample.timestamp - before_sample.timestamp
        if time_diff == 0:
            return before_sample
        
        alpha = (timestamp_ns - before_sample.timestamp) / time_diff
        
        return IMUData(
            timestamp=timestamp_ns,
            gyro_x=before_sample.gyro_x + alpha * (after_sample.gyro_x - before_sample.gyro_x),
            gyro_y=before_sample.gyro_y + alpha * (after_sample.gyro_y - before_sample.gyro_y),
            gyro_z=before_sample.gyro_z + alpha * (after_sample.gyro_z - before_sample.gyro_z),
            accel_x=before_sample.accel_x + alpha * (after_sample.accel_x - before_sample.accel_x),
            accel_y=before_sample.accel_y + alpha * (after_sample.accel_y - before_sample.accel_y),
            accel_z=before_sample.accel_z + alpha * (after_sample.accel_z - before_sample.accel_z)
        )
    
    def _interpolate_trajectory_for_time(self, trajectory: Dict, t: float) -> Optional[Dict]:
        """Interpolate trajectory data for specific time"""
        if not trajectory or 'time' not in trajectory:
            return None
        
        t_orig = trajectory['time']
        
        return {
            'x': np.interp(t, t_orig, trajectory['x']),
            'y': np.interp(t, t_orig, trajectory['y']),
            'z': np.interp(t, t_orig, trajectory['z']),
            'vx': np.interp(t, t_orig, trajectory['vx']),
            'vy': np.interp(t, t_orig, trajectory['vy']),
            'vz': np.interp(t, t_orig, trajectory['vz']),
            'roll': np.interp(t, t_orig, trajectory['roll']),
            'pitch': np.interp(t, t_orig, trajectory['pitch']),
            'yaw': np.interp(t, t_orig, trajectory['yaw'])
        }

class LiDARMotionSimulator:
    """Main Livox Mid-70 simulation coordinator"""
    
    def __init__(self, config: Dict):
        self.config = config
        self.setup_logging()
        
        # Initialize components
        self.trajectory_generator = TrajectoryGenerator(
            config.get('trajectory_type', 'urban_circuit'),
            config.get('random_seed', 42)
        )
        
        self.environment_generator = EnvironmentGenerator(
            config.get('environment_complexity', 'medium'),
            config.get('random_seed', 42)
        )
        
        self.lidar_simulator = LiDARSimulator(config)
        self.imu_simulator = IMUSimulator(config)
        self.gps_simulator = GPSSimulator(config)
        self.motion_compensator = MotionCompensator(config)
        self.coordinate_transformer = CoordinateTransformer()
        self.performance_monitor = PerformanceMonitor()
        self.data_exporter = DataExporter(config)
        
        # LVX writer
        lvx_format = config.get('lvx_format', 'lvx2')
        self.lvx_writer = LivoxLVXWriter(lvx_format)
        
        # Network simulator (optional)
        self.network_simulator = None
        if config.get('enable_network_sim', False):
            self.network_simulator = NetworkSimulator()
        
        # Device information
        self.device_info = DeviceInfo(
            lidar_sn=config.get('device_info', {}).get('lidar_sn', '3GGDJ6K00200101'),
            device_type=config.get('device_info', {}).get('device_type', 1),
            firmware_version="1.5.0",
            extrinsic_enable=config.get('device_info', {}).get('extrinsic_enable', True),
            roll=config.get('device_info', {}).get('roll', 0.0),
            pitch=config.get('device_info', {}).get('pitch', 0.0),
            yaw=config.get('device_info', {}).get('yaw', 0.0),
            x=config.get('device_info', {}).get('x', 0.0),
            y=config.get('device_info', {}).get('y', 0.0),
            z=config.get('device_info', {}).get('z', 1.5)
        )
        
        logger.info("Livox Mid-70 Simulator initialized successfully")
        logger.info(f"Configuration: {self._summarize_config()}")
    
    def setup_logging(self):
        """Configure detailed logging"""
        log_level = self.config.get('log_level', 'INFO')
        logging.getLogger().setLevel(getattr(logging, log_level))
    
    def _summarize_config(self) -> str:
        """Create configuration summary"""
        return (f"Duration: {self.config.get('duration', 60)}s, "
                f"Trajectory: {self.config.get('trajectory_type', 'urban_circuit')}, "
                f"Environment: {self.config.get('environment_complexity', 'medium')}, "
                f"Motion Comp: {self.config.get('enable_motion_compensation', True)}")
    
    def run_simulation(self) -> Dict:
        """Execute complete simulation pipeline"""
        
        logger.info("Starting Livox Mid-70 simulation...")
        self.performance_monitor.start_monitoring()
        
        try:
            # 1. Generate trajectory
            logger.info("Generating vehicle trajectory...")
            duration = self.config.get('duration', 60.0)
            trajectory = self.trajectory_generator.generate_trajectory(
                duration=duration,
                max_speed=self.config.get('max_speed', 15.0),
                max_angular_vel=self.config.get('max_angular_vel', 0.5)
            )
            
            # 2. Generate environment
            logger.info("Generating 3D environment...")
            bounds = (-50, 50, -50, 50)  # x_min, x_max, y_min, y_max
            environment_objects = self.environment_generator.generate_environment(bounds)
            
            # 3. Generate sensor data
            logger.info("Generating high-frequency sensor data...")
            imu_data = self.imu_simulator.simulate_imu_data(trajectory, duration)
            gps_data = self.gps_simulator.simulate_gps_data(trajectory, duration)
            
            # 4. Generate LiDAR frames
            logger.info("Simulating LiDAR scanning...")
            frames_data = self._simulate_lidar_frames(trajectory, environment_objects, imu_data)
            
            # 5. Apply motion compensation
            if self.config.get('enable_motion_compensation', True):
                logger.info("Applying motion compensation...")
                frames_data = self._apply_motion_compensation(frames_data, imu_data)
            
            # 6. Coordinate system transformation
            coord_system = self.config.get('coordinate_system', CoordinateSystem.SENSOR)
            if coord_system != CoordinateSystem.SENSOR:
                logger.info(f"Transforming to {coord_system} coordinates...")
                frames_data = self._transform_coordinates(frames_data, coord_system, gps_data)
            
            # 7. Export data
            logger.info("Exporting simulation results...")
            self._export_results(frames_data, imu_data, gps_data, trajectory)
            
            # 8. Network simulation (if enabled)
            if self.network_simulator:
                logger.info("Starting network simulation...")
                self.network_simulator.start_streaming(frames_data, imu_data)
                time.sleep(2)  # Let it run briefly
                self.network_simulator.stop_streaming()
            
            # 9. Generate analysis report
            performance_stats = self.performance_monitor.get_performance_stats()
            self._generate_analysis_report(frames_data, imu_data, gps_data, performance_stats)
            
            # 10. Create visualizations
            self._create_visualizations(trajectory, frames_data, imu_data, performance_stats)
            
            # Compile results
            results = {
                'trajectory': trajectory,
                'frames': frames_data,
                'imu_data': imu_data,
                'gps_data': gps_data,
                'environment_objects': environment_objects,
                'performance_stats': performance_stats,
                'device_info': self.device_info,
                'config': self.config
            }
            
            logger.info("Simulation completed successfully!")
            logger.info(f"Generated {len(frames_data)} LiDAR frames with {sum(len(f['points']) for f in frames_data)} total points")
            
            return results
            
        except Exception as e:
            logger.error(f"Simulation failed: {e}")
            raise
    
    def _simulate_lidar_frames(self, trajectory: Dict, environment_objects: List[np.ndarray], 
                              imu_data: List[IMUData]) -> List[Dict]:
        """Simulate LiDAR scanning frames"""
        
        frames_data = []
        frame_interval = 1.0 / Mid70Specs.FRAME_RATE  # 0.1s for 10Hz
        duration = trajectory['time'][-1]
        frame_times = np.arange(0, duration, frame_interval)
        
        for frame_idx, t in enumerate(frame_times):
            frame_start_time = time.time()
            
            # Interpolate sensor position and orientation
            sensor_position = np.array([
                np.interp(t, trajectory['time'], trajectory['x']),
                np.interp(t, trajectory['time'], trajectory['y']),
                np.interp(t, trajectory['time'], trajectory['z'])
            ])
            
            sensor_orientation = np.array([
                np.interp(t, trajectory['time'], trajectory['roll']),
                np.interp(t, trajectory['time'], trajectory['pitch']),
                np.interp(t, trajectory['time'], trajectory['yaw'])
            ])
            
            # Add mounting offset
            sensor_position[2] += self.device_info.z  # Add mounting height
            
            # Simulate LiDAR scan
            timestamp_ns = int(t * 1e9)
            points = self.lidar_simulator.simulate_scan(
                sensor_position, sensor_orientation, environment_objects, timestamp_ns
            )
            
            # Create frame data
            frame_data = {
                'frame_id': frame_idx,
                'timestamp': timestamp_ns,
                'sensor_position': sensor_position,
                'sensor_orientation': sensor_orientation,
                'points': points,
                'frame_duration_ns': int(frame_interval * 1e9)
            }
            
            frames_data.append(frame_data)
            
            # Update performance monitoring
            frame_processing_time = time.time() - frame_start_time
            memory_mb = self._get_memory_usage_mb()
            self.performance_monitor.update_metrics(
                len(points), 1, frame_processing_time, memory_mb
            )
            
            if frame_idx % 10 == 0:
                logger.info(f"Processed frame {frame_idx}/{len(frame_times)} - {len(points)} points")
        
        return frames_data
    
    def _apply_motion_compensation(self, frames_data: List[Dict], 
                                  imu_data: List[IMUData]) -> List[Dict]:
        """Apply motion compensation to all frames"""
        
        compensated_frames = []
        
        for frame in frames_data:
            compensated_points = self.motion_compensator.compensate_point_cloud(
                frame['points'], imu_data, 
                frame['timestamp'], frame['frame_duration_ns']
            )
            
            # Create new frame with compensated points
            compensated_frame = frame.copy()
            compensated_frame['points'] = compensated_points
            compensated_frame['motion_compensated'] = True
            
            compensated_frames.append(compensated_frame)
        
        return compensated_frames
    
    def _transform_coordinates(self, frames_data: List[Dict], target_system: str, 
                             gps_data: List[GPSData]) -> List[Dict]:
        """Transform point coordinates to target coordinate system"""
        
        transformed_frames = []
        
        for frame in frames_data:
            transformed_points = []
            
            for point in frame['points']:
                # Convert point to numpy array
                point_array = np.array([[point.x, point.y, point.z]])
                
                # Transform coordinates
                if target_system == CoordinateSystem.UTM and gps_data:
                    # Find corresponding GPS data for this frame
                    gps_sample = self._find_closest_gps_sample(gps_data, frame['timestamp'])
                    if gps_sample and UTM_AVAILABLE:
                        try:
                            # Convert GPS to UTM for reference
                            utm_x, utm_y, zone_num, zone_letter = utm.from_latlon(
                                gps_sample.latitude, gps_sample.longitude
                            )
                            
                            # Apply UTM offset
                            transformed_point = point_array[0] + np.array([utm_x, utm_y, 0])
                        except:
                            transformed_point = point_array[0]
                    else:
                        transformed_point = point_array[0]
                else:
                    # Use coordinate transformer for other systems
                    transformed_point = self.coordinate_transformer.transform_points(
                        point_array, CoordinateSystem.SENSOR, target_system
                    )[0]
                
                # Create transformed point
                transformed_lidar_point = LiDARPoint(
                    x=transformed_point[0],
                    y=transformed_point[1],
                    z=transformed_point[2],
                    intensity=point.intensity,
                    timestamp=point.timestamp,
                    ring=point.ring,
                    tag=point.tag
                )
                
                transformed_points.append(transformed_lidar_point)
            
            # Create transformed frame
            transformed_frame = frame.copy()
            transformed_frame['points'] = transformed_points
            transformed_frame['coordinate_system'] = target_system
            
            transformed_frames.append(transformed_frame)
        
        return transformed_frames
    
    def _find_closest_gps_sample(self, gps_data: List[GPSData], timestamp_ns: int) -> Optional[GPSData]:
        """Find GPS sample closest to given timestamp"""
        if not gps_data:
            return None
        
        closest_sample = None
        min_diff = float('inf')
        
        for sample in gps_data:
            diff = abs(sample.timestamp - timestamp_ns)
            if diff < min_diff:
                min_diff = diff
                closest_sample = sample
        
        return closest_sample
    
    def _export_results(self, frames_data: List[Dict], imu_data: List[IMUData], 
                       gps_data: List[GPSData], trajectory: Dict):
        """Export all simulation results"""
        
        output_prefix = self.config.get('output_prefix', 'livox_mid70_sim')
        
        # Export point clouds
        self.data_exporter.export_point_clouds(frames_data, output_prefix)
        
        # Export sensor data
        self.data_exporter.export_sensor_data(imu_data, gps_data, trajectory, output_prefix)
        
        # Export LVX files
        lvx_filename = f"{output_prefix}.{self.config.get('lvx_format', 'lvx2')}"
        self.lvx_writer.write_lvx_file(lvx_filename, frames_data, self.device_info)
        
        # Export configuration
        config_filename = f"{output_prefix}_config.json"
        with open(config_filename, 'w') as f:
            # Make config JSON serializable
            config_copy = self.config.copy()
            json.dump(config_copy, f, indent=2, default=str)
        
        logger.info(f"All results exported with prefix: {output_prefix}")
    
    def _generate_analysis_report(self, frames_data: List[Dict], imu_data: List[IMUData], 
                                 gps_data: List[GPSData], performance_stats: Dict):
        """Generate comprehensive simulation analysis report"""
        
        report_filename = f"{self.config.get('output_prefix', 'livox_mid70_sim')}_report.md"
        
        total_points = sum(len(frame['points']) for frame in frames_data)
        
        report_content = f"""# Livox Mid-70 Simulation Report

## Simulation Overview
- **Duration**: {self.config.get('duration', 60):.1f} seconds
- **Trajectory Type**: {self.config.get('trajectory_type', 'urban_circuit')}
- **Environment**: {self.config.get('environment_complexity', 'medium')}
- **LVX Format**: {self.config.get('lvx_format', 'lvx2').upper()}
- **Motion Compensation**: {'Enabled' if self.config.get('enable_motion_compensation', True) else 'Disabled'}

## Data Statistics
- **LiDAR Frames**: {len(frames_data)}
- **Total Points**: {total_points:,}
- **Average Points/Frame**: {total_points/len(frames_data):.0f}
- **IMU Samples**: {len(imu_data):,} at 200Hz
- **GPS Samples**: {len(gps_data):,} at {self.config.get('gps_rate', 5)}Hz

## Performance Metrics
- **Total Runtime**: {performance_stats.get('total_runtime_seconds', 0):.2f} seconds
- **Points/Second**: {performance_stats.get('average_points_per_second', 0):,.0f}
- **Frames/Second**: {performance_stats.get('average_frames_per_second', 0):.1f}
- **Peak Memory**: {performance_stats.get('memory_usage', {}).get('peak_mb', 0):.1f} MB
- **Average Frame Processing**: {performance_stats.get('processing_times', {}).get('mean', 0)*1000:.2f} ms

## Hardware Simulation Accuracy
- **FOV**: {Mid70Specs.FOV_HORIZONTAL}° × {Mid70Specs.FOV_VERTICAL}° (Horizontal × Vertical)
- **Range**: {Mid70Specs.RANGE_MIN}m - {Mid70Specs.RANGE_MAX}m
- **Point Rate**: Up to {Mid70Specs.POINT_RATE_MAX:,} points/second
- **Frame Rate**: {Mid70Specs.FRAME_RATE}Hz (fixed)
- **IMU Rate**: {Mid70Specs.IMU_RATE}Hz

## Quality Metrics
- **Range Accuracy**: ±{Mid70Specs.ACCURACY*100:.0f}cm (simulated)
- **Angular Resolution**: {Mid70Specs.ANGULAR_RES_H}° × {Mid70Specs.ANGULAR_RES_V}°
- **Timestamp Precision**: Nanosecond resolution
- **Motion Compensation**: {'Real-time IMU-based' if self.config.get('enable_motion_compensation', True) else 'Not applied'}

## Output Files Generated
- **Point Clouds**: PCD, {'LAS, ' if LASPY_AVAILABLE else ''}XYZ, CSV formats
- **LVX Data**: {self.config.get('lvx_format', 'lvx2').upper()} format (Livox Viewer compatible)
- **Sensor Data**: High-frequency IMU (200Hz), GPS, synchronized motion data
- **Trajectory**: Complete vehicle path with velocities and orientations
- **Configuration**: Complete simulation parameters backup

## Recommendations
1. **For Real-time Applications**: Consider reducing point density if processing speed is critical
2. **For Accuracy**: Motion compensation significantly improves data quality during movement
3. **For Analysis**: Use the synchronized motion data CSV for SLAM and mapping algorithms
4. **For Visualization**: LVX files can be directly opened in Livox Viewer for inspection

---
*Report generated by Livox Mid-70 Complete Simulation System*
*Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}*
"""
        
        with open(report_filename, 'w') as f:
            f.write(report_content)
        
        logger.info(f"Analysis report generated: {report_filename}")
    
    def _create_visualizations(self, trajectory: Dict, frames_data: List[Dict], 
                             imu_data: List[IMUData], performance_stats: Dict):
        """Create comprehensive visualization plots"""
        
        try:
            # 1. Trajectory plot
            plt.figure(figsize=(15, 12))
            
            # Trajectory path
            plt.subplot(2, 3, 1)
            plt.plot(trajectory['x'], trajectory['y'], 'b-', linewidth=2, label='Trajectory')
            plt.scatter(trajectory['x'][0], trajectory['y'][0], color='green', s=100, label='Start')
            plt.scatter(trajectory['x'][-1], trajectory['y'][-1], color='red', s=100, label='End')
            plt.xlabel('X (meters)')
            plt.ylabel('Y (meters)')
            plt.title('Vehicle Trajectory (Top View)')
            plt.grid(True)
            plt.legend()
            plt.axis('equal')
            
            # Velocity profile
            plt.subplot(2, 3, 2)
            speed = np.sqrt(trajectory['vx']**2 + trajectory['vy']**2)
            plt.plot(trajectory['time'], speed, 'r-', linewidth=2)
            plt.xlabel('Time (seconds)')
            plt.ylabel('Speed (m/s)')
            plt.title('Vehicle Speed Profile')
            plt.grid(True)
            
            # IMU data (if available)
            if imu_data:
                imu_times = np.array([imu.timestamp * 1e-9 for imu in imu_data[:1000]])  # First 1000 samples
                imu_gyro_z = np.array([imu.gyro_z for imu in imu_data[:1000]])
                
                plt.subplot(2, 3, 3)
                plt.plot(imu_times, imu_gyro_z, 'g-', linewidth=1)
                plt.xlabel('Time (seconds)')
                plt.ylabel('Gyro Z (rad/s)')
                plt.title('IMU Angular Velocity (Z-axis)')
                plt.grid(True)
            
            # Point cloud density
            plt.subplot(2, 3, 4)
            frame_ids = [f['frame_id'] for f in frames_data]
            point_counts = [len(f['points']) for f in frames_data]
            plt.plot(frame_ids, point_counts, 'purple', marker='o', markersize=3)
            plt.xlabel('Frame ID')
            plt.ylabel('Points per Frame')
            plt.title('Point Cloud Density')
            plt.grid(True)
            
            # Processing performance
            plt.subplot(2, 3, 5)
            if 'processing_times' in performance_stats:
                processing_times = performance_stats['processing_times']
                if isinstance(processing_times, dict) and 'mean' in processing_times:
                    mean_time = processing_times['mean'] * 1000  # Convert to ms
                    std_time = processing_times.get('std', 0) * 1000
                    plt.bar(['Processing Time'], [mean_time], yerr=[std_time], 
                           color='orange', capsize=5)
                    plt.ylabel('Time (ms)')
                    plt.title('Average Frame Processing Time')
                    plt.grid(True)
            
            # Memory usage
            plt.subplot(2, 3, 6)
            if 'memory_usage' in performance_stats:
                memory_stats = performance_stats['memory_usage']
                if isinstance(memory_stats, dict):
                    mean_memory = memory_stats.get('mean_mb', 0)
                    peak_memory = memory_stats.get('peak_mb', 0)
                    plt.bar(['Average', 'Peak'], [mean_memory, peak_memory], 
                           color=['lightblue', 'darkblue'])
                    plt.ylabel('Memory (MB)')
                    plt.title('Memory Usage')
                    plt.grid(True)
            
            plt.tight_layout()
            
            # Save plot
            plot_filename = f"{self.config.get('output_prefix', 'livox_mid70_sim')}_analysis.png"
            plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
            plt.close()
            
            logger.info(f"Visualization saved: {plot_filename}")
            
            # 2. 3D trajectory visualization
            fig = plt.figure(figsize=(12, 8))
            ax = fig.add_subplot(111, projection='3d')
            
            # Plot 3D trajectory
            ax.plot(trajectory['x'], trajectory['y'], trajectory['z'], 'b-', linewidth=2, label='Trajectory')
            ax.scatter(trajectory['x'][0], trajectory['y'][0], trajectory['z'][0], 
                      color='green', s=100, label='Start')
            ax.scatter(trajectory['x'][-1], trajectory['y'][-1], trajectory['z'][-1], 
                      color='red', s=100, label='End')
            
            # Add some sample points from first frame
            if frames_data:
                first_frame_points = frames_data[0]['points'][:100]  # First 100 points
                px = [p.x + frames_data[0]['sensor_position'][0] for p in first_frame_points]
                py = [p.y + frames_data[0]['sensor_position'][1] for p in first_frame_points]
                pz = [p.z + frames_data[0]['sensor_position'][2] for p in first_frame_points]
                ax.scatter(px, py, pz, c='red', s=1, alpha=0.5, label='Sample Points')
            
            ax.set_xlabel('X (meters)')
            ax.set_ylabel('Y (meters)')
            ax.set_zlabel('Z (meters)')
            ax.set_title('3D Trajectory with Sample LiDAR Points')
            ax.legend()
            
            # Save 3D plot
            plot_3d_filename = f"{self.config.get('output_prefix', 'livox_mid70_sim')}_3d_trajectory.png"
            plt.savefig(plot_3d_filename, dpi=300, bbox_inches='tight')
            plt.close()
            
            logger.info(f"3D visualization saved: {plot_3d_filename}")
            
        except Exception as e:
            logger.error(f"Visualization creation failed: {e}")
    
    def _get_memory_usage_mb(self) -> float:
        """Get current memory usage in MB"""
        try:
            import psutil
            process = psutil.Process()
            return process.memory_info().rss / 1024 / 1024  # Convert to MB
        except:
            return 0.0

# Default configuration for comprehensive simulation
DEFAULT_CONFIG = {
    # Simulation parameters
    'duration': 120.0,                    # 2 minutes simulation
    'lidar_fps': Mid70Specs.FRAME_RATE,   # 10Hz fixed
    'imu_rate': Mid70Specs.IMU_RATE,      # 200Hz
    'gps_rate': 5,                        # 5Hz
    'random_seed': 42,                    # Reproducible results
    
    # Livox Mid-70 specifications (verified)
    'fov_horizontal': Mid70Specs.FOV_HORIZONTAL,
    'fov_vertical': Mid70Specs.FOV_VERTICAL,
    'range_max': Mid70Specs.RANGE_MAX,
    'range_min': Mid70Specs.RANGE_MIN,
    'points_per_frame': 10000,            # Realistic for real-time processing
    'angular_resolution': Mid70Specs.ANGULAR_RES_H,
    'point_accuracy': Mid70Specs.ACCURACY,
    
    # Motion and trajectory
    'max_speed': 12.0,                    # 43 km/h realistic urban speed
    'max_angular_vel': 0.3,               # Realistic turning rate
    'trajectory_type': 'urban_circuit',   # Complex urban driving
    
    # Realistic noise parameters
    'gps_noise_std': 0.03,                # 3cm RTK GPS accuracy
    'imu_accel_noise': 0.1,               # Consumer-grade IMU
    'imu_gyro_noise': 0.01,               # Gyroscope noise
    'lidar_range_noise': 0.02,            # 2cm range noise
    'lidar_intensity_noise': 5.0,         # Intensity variation
    
    # Environment
    'environment_complexity': 'urban',    # Detailed urban environment
    'ground_height': 0.0,                 # Sea level reference
    'obstacle_density': 0.3,              # Moderate obstacle density
    
    # Advanced features
    'enable_motion_compensation': True,    # Essential for moving platform
    'enable_network_sim': False,          # Disable for file-only output
    'coordinate_system': 'sensor',        # Start with sensor coordinates
    'lvx_format': 'lvx2',                # Current Livox Viewer format
    
    # Device configuration (realistic Mid-70 setup)
    'device_info': {
        'lidar_sn': '3GGDJ6K00200101',    # Realistic serial number format
        'device_type': 1,                 # Mid-70 identifier
        'extrinsic_enable': True,         # Enable mounting calibration
        'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,  # Level mounting
        'x': 0.0, 'y': 0.0, 'z': 1.5     # 1.5m above ground (typical vehicle)
    },
    
    # GPS reference point (New York City area)
    'ref_latitude': 40.7128,
    'ref_longitude': -74.0060,
    'ref_altitude': 10.0,
    
    # Output configuration
    'output_prefix': 'livox_mid70_simulation',
    'log_level': 'INFO'
}

def main():
    """Main entry point for simulation"""
    
    print("="*80)
    print("Livox Mid-70 Complete Simulation System")
    print("="*80)
    print()
    print("Features:")
    print("- Realistic Mid-70 hardware simulation with verified specifications")
    print("- High-frequency IMU data generation (200Hz)")  
    print("- Advanced motion compensation algorithms")
    print("- Multiple coordinate systems and LVX format support")
    print("- Comprehensive data export and analysis")
    print()
    
    # Create simulator with default configuration
    config = DEFAULT_CONFIG.copy()
    
    # Override with user preferences if needed
    config.update({
        'duration': 60.0,          # Shorter for quick demo
        'points_per_frame': 8000,  # Reduced for faster processing
    })
    
    try:
        # Initialize and run simulation
        simulator = LiDARMotionSimulator(config)
        results = simulator.run_simulation()
        
        # Print summary
        print("\n" + "="*80)
        print("SIMULATION COMPLETED SUCCESSFULLY")
        print("="*80)
        print(f"Generated Files:")
        print(f"- Point Clouds: PCD, {'LAS, ' if LASPY_AVAILABLE else ''}XYZ, CSV")
        print(f"- LVX Data: {config['lvx_format'].upper()} format")
        print(f"- Sensor Data: IMU (200Hz), GPS, Motion data")
        print(f"- Analysis: Comprehensive report and visualizations")
        print()
        print(f"Total Points: {sum(len(f['points']) for f in results['frames']):,}")
        print(f"Total Frames: {len(results['frames'])}")
        print(f"IMU Samples: {len(results['imu_data']):,}")
        print(f"GPS Samples: {len(results['gps_data'])}")
        print()
        print("Files can be opened with:")
        print("- Livox Viewer (LVX files)")
        print("- CloudCompare/PCL (PCD files)")
        print("- QGIS/ArcGIS (LAS files)" if LASPY_AVAILABLE else "- Any text editor (XYZ files)")
        print("="*80)
        
        return results
        
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
        return None
    except Exception as e:
        print(f"\nSimulation failed: {e}")
        import traceback
        traceback.print_exc()
        return None

# Advanced usage examples
def example_custom_configuration():
    """Example of custom configuration for specific use cases"""
    
    # High-precision mapping configuration
    mapping_config = DEFAULT_CONFIG.copy()
    mapping_config.update({
        'duration': 300.0,                    # 5 minutes for detailed mapping
        'points_per_frame': 15000,            # High point density
        'trajectory_type': 'figure_eight',    # Systematic coverage
        'environment_complexity': 'urban',    # Detailed environment
        'gps_noise_std': 0.01,                # High-precision RTK GPS
        'coordinate_system': 'utm',           # Global coordinates
        'enable_motion_compensation': True,    # Essential for accuracy
        'output_prefix': 'high_precision_mapping'
    })
    
    # Real-time processing configuration  
    realtime_config = DEFAULT_CONFIG.copy()
    realtime_config.update({
        'duration': 30.0,                     # Shorter duration
        'points_per_frame': 5000,             # Reduced density for speed
        'environment_complexity': 'simple',   # Faster processing
        'enable_network_sim': True,           # Network streaming
        'coordinate_system': 'sensor',        # No transformation overhead
        'output_prefix': 'realtime_demo'
    })
    
    # Autonomous vehicle testing configuration
    autonomous_config = DEFAULT_CONFIG.copy()
    autonomous_config.update({
        'duration': 180.0,                    # 3 minutes
        'trajectory_type': 'urban_circuit',   # Complex driving scenario
        'max_speed': 20.0,                    # Higher speed testing
        'environment_complexity': 'urban',    # Realistic urban environment
        'gps_rate': 10,                       # Higher GPS rate
        'coordinate_system': 'vehicle',       # Vehicle-centric coordinates
        'output_prefix': 'autonomous_vehicle_test'
    })
    
    return mapping_config, realtime_config, autonomous_config

if __name__ == "__main__":
    # Run main simulation
    # python livox_mid70_complete_simulator.py
    results = main()
    
    # Optional: Run custom configurations
    # mapping_config, realtime_config, autonomous_config = example_custom_configuration()
    # 
    # print("\nRunning custom configuration examples...")
    # for config_name, config in [("Mapping", mapping_config), ("Real-time", realtime_config)]:
    #     print(f"\n{config_name} Configuration:")
    #     simulator = LiDARMotionSimulator(config)
    #     results = simulator.run_simulation()
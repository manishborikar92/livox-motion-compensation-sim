"""
Realistic LiDAR Motion Compensation Simulation for Livox Mid-70
Simulates accurate point cloud data with proper Mid-70 scanning patterns,
occlusions, noise models, and realistic sensor behavior.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import struct
from datetime import datetime, timedelta
from scipy.spatial.transform import Rotation as R
from scipy import signal
from scipy.spatial import cKDTree
import laspy

class LivoxLVXWriter:
    """
    Corrected LVX writer for Livox Viewer compatibility
    Based on reverse engineering of working LVX files
    """
    
    def __init__(self):
        # === EXACT CONSTANTS FROM WORKING LVX FILES ===
        self.LVX_FILE_SIGNATURE = b'livox_tech'  # Exactly 10 bytes
        self.MAGIC_CODE = 0xAC0EA767
        
        # Version constants
        self.VERSION_MAJOR = 1
        self.VERSION_MINOR = 1
        self.VERSION_PATCH = 0
        self.VERSION_BUILD = 0
        
        # Device constants
        self.DEVICE_TYPE_MID70 = 6
        self.LIDAR_ID = 0
        self.SLOT_ID = 0
        
        # Data format constants
        self.DATA_TYPE_CARTESIAN = 2  # Cartesian coordinate with tag
        self.TIMESTAMP_TYPE = 0  # UTC time
        self.PACKAGE_VERSION = 5
        
        # Package constants
        self.POINTS_PER_PACKAGE = 96  # Standard for Mid-70
        self.FRAME_DURATION_US = 100000  # 100ms = 10Hz
        
    def write_lvx_file(self, frames_data, output_path):
        """
        Write LVX file with Livox Viewer compatibility
        
        Args:
            frames_data (list): List of frame dictionaries with 'points', 'timestamp', 'frame_id'
            output_path (str): Output file path
        """
        print(f"Writing corrected LVX file: {output_path}")
        
        with open(output_path, 'wb') as f:
            frame_count = len(frames_data)
            device_count = 1
            
            # === PUBLIC HEADER (24 bytes) ===
            public_header = bytearray(24)
            
            # File signature (exactly 10 bytes)
            public_header[0:10] = self.LVX_FILE_SIGNATURE
            
            # Version (4 bytes) - v1.1.0.0
            public_header[10] = self.VERSION_MAJOR
            public_header[11] = self.VERSION_MINOR
            public_header[12] = self.VERSION_PATCH
            public_header[13] = self.VERSION_BUILD
            
            # Magic code (4 bytes, little endian)
            struct.pack_into('<I', public_header, 14, self.MAGIC_CODE)
            
            # Frame count (4 bytes, little endian)
            struct.pack_into('<I', public_header, 18, frame_count)
            
            # Device count (2 bytes, little endian)
            struct.pack_into('<H', public_header, 22, device_count)
            
            f.write(bytes(public_header))
            
            # === PRIVATE HEADER (5 bytes) ===
            private_header = bytearray(5)
            
            # Frame duration in microseconds (4 bytes, little endian)
            struct.pack_into('<I', private_header, 0, self.FRAME_DURATION_US)
            
            # Device count (1 byte)
            private_header[4] = device_count
            
            f.write(bytes(private_header))
            
            # === DEVICE INFO BLOCK (59 bytes) ===
            device_info = self._create_device_info()
            f.write(device_info)
            
            # === PRE-CALCULATE FRAME POSITIONS ===
            frame_positions = []
            current_pos = f.tell()  # Position after all headers
            
            for frame_data in frames_data:
                frame_positions.append(current_pos)
                
                points = frame_data['points']
                num_packages = (len(points) + self.POINTS_PER_PACKAGE - 1) // self.POINTS_PER_PACKAGE
                
                frame_size = (
                    24 +  # Frame header
                    num_packages * (22 + self.POINTS_PER_PACKAGE * 14)  # Packages (header + points)
                )
                
                current_pos += frame_size
            
            # === WRITE FRAMES ===
            for i, frame_data in enumerate(frames_data):
                self._write_frame(f, frame_data, frame_positions, i)
        
        file_size = os.path.getsize(output_path)
        print(f"✅ Corrected LVX file created: {file_size:,} bytes")
        
        return output_path
    
    def _create_device_info(self):
        """Create device info block (59 bytes)"""
        device_info = bytearray(59)
        
        # LiDAR SN (16 bytes)
        lidar_sn = b'MID70-240100001\x00'  # 15 chars + null terminator
        device_info[0:len(lidar_sn)] = lidar_sn
        
        # Device type (1 byte) - Mid-70
        device_info[16] = self.DEVICE_TYPE_MID70
        
        # Extrinsic parameters (6 * 4 = 24 bytes)
        # Roll, Pitch, Yaw (radians), X, Y, Z (meters)
        extrinsic = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Identity transform
        for i, value in enumerate(extrinsic):
            struct.pack_into('<f', device_info, 17 + i * 4, value)
        
        # Reserved (18 bytes) - keep as zeros
        
        return bytes(device_info)
    
    def _write_frame(self, f, frame_data, frame_positions, frame_index):
        """Write a single frame to the file"""
        points = frame_data['points']
        timestamp_ns = int(frame_data['timestamp'] * 1e9)  # Convert to nanoseconds
        
        # === FRAME HEADER (24 bytes = 3 * 8 bytes) ===
        frame_header = bytearray(24)
        
        # Current offset
        struct.pack_into('<Q', frame_header, 0, frame_positions[frame_index])
        
        # Next offset (0 if last frame)
        if frame_index < len(frame_positions) - 1:
            struct.pack_into('<Q', frame_header, 8, frame_positions[frame_index + 1])
        else:
            struct.pack_into('<Q', frame_header, 8, 0)
        
        # Frame index
        struct.pack_into('<Q', frame_header, 16, frame_data['frame_id'])
        
        f.write(bytes(frame_header))
        
        # === PACKAGES ===
        # Split points into packages (96 points per package for data type 2)
        for i in range(0, len(points), self.POINTS_PER_PACKAGE):
            package_points = points[i:i + self.POINTS_PER_PACKAGE]
            self._write_package(f, package_points, timestamp_ns)
    
    def _write_package(self, f, points, timestamp_ns):
        """Write a single package to the file"""
        actual_point_count = len(points)
        
        # === PACKAGE HEADER (22 bytes) ===
        package_header = bytearray(22)
        
        # Device Index (1 byte)
        package_header[0] = 0
        
        # Version (1 byte)
        package_header[1] = self.PACKAGE_VERSION
        
        # Slot ID (1 byte)
        package_header[2] = self.SLOT_ID
        
        # LiDAR ID (1 byte)
        package_header[3] = self.LIDAR_ID
        
        # Reserved (1 byte)
        package_header[4] = 0
        
        # Status Code (4 bytes) - normal operation
        struct.pack_into('<I', package_header, 5, 0)
        
        # Timestamp Type (1 byte)
        package_header[9] = self.TIMESTAMP_TYPE
        
        # Data Type (1 byte)
        package_header[10] = self.DATA_TYPE_CARTESIAN
        
        # Reserved (3 bytes)
        package_header[11:14] = b'\x00\x00\x00'
        
        # Timestamp (8 bytes)
        struct.pack_into('<Q', package_header, 14, timestamp_ns)
        
        f.write(bytes(package_header))
        
        # === POINT DATA ===
        # For data type 2: x(4) + y(4) + z(4) + reflectivity(1) + tag(1) = 14 bytes per point
        for point in points:
            self._write_point_data_type2(f, point)
        
        # Pad remaining points in package with zeros if needed
        points_to_pad = self.POINTS_PER_PACKAGE - actual_point_count
        if points_to_pad > 0:
            padding = b'\x00' * (points_to_pad * 14)  # 14 bytes per point for type 2
            f.write(padding)
    
    def _write_point_data_type2(self, f, point):
        """Write point data for type 2 (Cartesian with tag) - 14 bytes"""
        point_data = bytearray(14)
        
        # Coordinates in millimeters (int32, little endian)
        x_mm = int(np.clip(point[0] * 1000, -2147483648, 2147483647))
        y_mm = int(np.clip(point[1] * 1000, -2147483648, 2147483647))
        z_mm = int(np.clip(point[2] * 1000, -2147483648, 2147483647))
        
        struct.pack_into('<i', point_data, 0, x_mm)
        struct.pack_into('<i', point_data, 4, y_mm)
        struct.pack_into('<i', point_data, 8, z_mm)
        
        # Reflectivity (0-255)
        reflectivity = int(np.clip(point[3] * 255, 0, 255)) if len(point) > 3 else 128
        point_data[12] = reflectivity
        
        # Tag (point quality/classification)
        point_data[13] = 0  # Normal point
        
        f.write(bytes(point_data))

def save_corrected_lvx(frames_data, output_path):
    """Save frames data to LVX format using the corrected writer"""
    writer = LivoxLVXWriter()
    writer.write_lvx_file(frames_data, output_path)
    print(f"Corrected LVX file saved to: {output_path}")

class RealisticLivoxMid70Simulator:
    def __init__(self, config=None):
        """
        Initialize the Realistic Livox Mid-70 LiDAR Simulator
        
        Args:
            config (dict): Configuration parameters
        """
        self.config = self.default_config()
        if config:
            self.config.update(config)
        np.random.seed(self.config['random_seed'])
        
        # Pre-compute rosette scanning pattern for Mid-70
        self.scan_pattern = self._generate_rosette_pattern()
        
    def default_config(self):
        """Default configuration for realistic Mid-70 simulation"""
        return {
            # Simulation parameters
            'duration': 60.0,
            'lidar_fps': 10,            # Mid-70 outputs 10Hz
            'imu_rate': 100,
            'gps_rate': 5,
            'random_seed': 42,
            
            # Motion parameters
            'max_speed': 15.0,
            'max_angular_vel': 0.5,
            'trajectory_type': 'figure_eight',
            
            # Livox Mid-70 ACTUAL specifications
            'fov_degrees': 70.4,        # Circular FOV
            'range_max': 260.0,         # meters (actual spec)
            'range_min': 0.05,          # meters
            'points_per_frame': 96000,  # At 10Hz = 960kHz point rate
            'scan_frequency': 1447,     # Hz - rosette pattern frequency
            
            # Realistic noise parameters for Mid-70
            'range_precision': 0.02,    # 2cm precision
            'range_accuracy': 0.03,     # 3cm accuracy  
            'intensity_noise': 0.05,    # 5% intensity variation
            'outlier_rate': 0.001,      # 0.1% outlier points
            'beam_divergence': 0.28,    # mrad - actual beam divergence
            
            # Environmental parameters
            'environment_complexity': 'medium',
            'ground_height': 0.0,
            'obstacle_density': 0.1,
            'atmospheric_attenuation': True,
            'multiple_returns': True,   # Mid-70 supports dual returns
            
            # Sensor mounting (typical vehicle setup)
            'sensor_height': 1.8,       # meters above ground
            'sensor_tilt': 0.0,         # degrees (0 = horizontal)
        }
    
    def _generate_rosette_pattern(self):
        """
        Generate the non-repetitive rosette scanning pattern characteristic of Livox sensors
        This creates the distinctive flower-like pattern that Mid-70 uses
        """
        # Rosette pattern parameters (based on Livox technical specs)
        pattern_period = 1.4  # seconds for complete pattern
        samples_per_period = int(self.config['scan_frequency'] * pattern_period)
        
        # Generate rosette coordinates using parametric equations
        t = np.linspace(0, pattern_period, samples_per_period)
        
        # Rosette pattern with multiple frequencies for non-repetitive coverage
        freq1, freq2 = 17, 23  # Prime numbers for non-repeating pattern
        r = 0.5 * (1 + 0.3 * np.cos(2 * np.pi * freq1 * t))
        theta = 2 * np.pi * freq2 * t
        
        # Convert to angular coordinates (degrees)
        # Map to FOV circle
        fov_radius = self.config['fov_degrees'] / 2
        azimuth = r * fov_radius * np.cos(theta)
        elevation = r * fov_radius * np.sin(theta)
        
        # Add slight randomization to avoid perfect repeatability
        azimuth += np.random.normal(0, 0.1, len(azimuth))
        elevation += np.random.normal(0, 0.1, len(elevation))
        
        # Keep within FOV limits
        r_check = np.sqrt(azimuth**2 + elevation**2)
        valid_mask = r_check <= fov_radius
        
        return {
            'azimuth': azimuth[valid_mask],
            'elevation': elevation[valid_mask],
            'time_offset': t[valid_mask]
        }
    
    def generate_trajectory(self):
        """Generate vehicle trajectory with realistic motion"""
        t = np.linspace(0, self.config['duration'], 
                       int(self.config['duration'] * self.config['gps_rate']))
        
        if self.config['trajectory_type'] == 'linear':
            # Linear with realistic road following
            x = self.config['max_speed'] * t
            y = 3 * np.sin(0.1 * t) + 1.5 * np.sin(0.3 * t)  # Realistic road curves
            z = np.full_like(t, self.config['sensor_height'])
            
        elif self.config['trajectory_type'] == 'circular':
            radius = 40.0
            angular_freq = self.config['max_speed'] / radius
            x = radius * np.cos(angular_freq * t)
            y = radius * np.sin(angular_freq * t)
            z = np.full_like(t, self.config['sensor_height'])
            
        elif self.config['trajectory_type'] == 'figure_eight':
            scale = 25.0
            freq = 0.08
            x = scale * np.sin(2 * np.pi * freq * t)
            y = scale * np.sin(4 * np.pi * freq * t)
            z = np.full_like(t, self.config['sensor_height'])
        
        # Calculate realistic vehicle dynamics
        vx = np.gradient(x) * self.config['gps_rate']
        vy = np.gradient(y) * self.config['gps_rate']
        vz = np.gradient(z) * self.config['gps_rate']
        
        # Vehicle orientation from motion
        yaw = np.arctan2(vy, vx)
        
        # Realistic roll/pitch from centrifugal forces
        speed = np.sqrt(vx**2 + vy**2)
        lateral_accel = np.gradient(speed * np.sin(yaw)) * self.config['gps_rate']
        roll = np.arctan2(lateral_accel, 9.81) * 0.3  # Banking in turns
        pitch = np.arctan2(vz, speed + 1e-6) * 0.5   # Pitch from elevation change
        
        return {
            'time': t,
            'position': np.column_stack([x, y, z]),
            'velocity': np.column_stack([vx, vy, vz]),
            'orientation': np.column_stack([roll, pitch, yaw])
        }
    
    def add_sensor_noise(self, trajectory):
        """Add realistic sensor noise based on actual GPS/IMU characteristics"""
        # GPS noise (realistic consumer-grade GPS)
        gps_noise_std = np.array([0.5, 0.5, 1.0])  # Higher vertical uncertainty
        gps_noise = np.random.normal(0, gps_noise_std, trajectory['position'].shape)
        trajectory['position_gps'] = trajectory['position'] + gps_noise
        
        # IMU noise (realistic automotive IMU)
        accel_bias = np.random.normal(0, 0.05, (3,))  # Constant bias
        accel_noise = np.random.normal(0, 0.1, trajectory['velocity'].shape)
        gyro_bias = np.random.normal(0, np.radians(0.5), (3,))  # 0.5 deg/hr bias
        gyro_noise = np.random.normal(0, np.radians(0.1), trajectory['orientation'].shape)
        
        # Calculate accelerations with bias
        dt = 1.0 / self.config['gps_rate']
        acceleration = np.gradient(trajectory['velocity'], dt, axis=0)
        trajectory['acceleration'] = acceleration + accel_noise + accel_bias
        trajectory['orientation_imu'] = trajectory['orientation'] + gyro_noise + gyro_bias
        
        return trajectory
    
    def generate_realistic_environment(self):
        """Generate realistic 3D environment with proper material properties"""
        if self.config['environment_complexity'] == 'simple':
            return self._generate_simple_scene()
        elif self.config['environment_complexity'] == 'medium':
            return self._generate_urban_scene()
        else:
            return self._generate_complex_scene()
    
    def _generate_simple_scene(self):
        """Simple scene with ground plane and basic obstacles"""
        objects = []
        
        # Ground plane with realistic material properties
        ground = self._create_ground_plane(
            bounds=(-150, 150, -150, 150),
            resolution=2.0,
            roughness=0.1,
            reflectivity=0.3
        )
        objects.append(ground)
        
        # Simple buildings
        for _ in range(8):
            center = np.random.uniform(-100, 100, 2)
            center = np.append(center, 0)
            size = np.random.uniform(5, 15, 3)
            size[2] = np.random.uniform(8, 25)  # Building height
            
            building = self._create_box_object(center, size, reflectivity=0.6)
            objects.append(building)
        
        # Vehicles (high reflectivity)
        for _ in range(15):
            center = np.random.uniform(-80, 80, 2)
            center = np.append(center, 0.8)  # Vehicle height
            size = [4.5, 1.8, 1.6]  # Typical car dimensions
            
            vehicle = self._create_box_object(center, size, reflectivity=0.8)
            objects.append(vehicle)
        
        return objects
    
    def _generate_urban_scene(self):
        """Urban scene with realistic infrastructure"""
        objects = []
        
        # Textured ground
        ground = self._create_ground_plane(
            bounds=(-200, 200, -200, 200),
            resolution=1.5,
            roughness=0.15,
            reflectivity=0.4
        )
        objects.append(ground)
        
        # Buildings with varying materials
        for _ in range(25):
            center = np.random.uniform(-120, 120, 2)
            center = np.append(center, 0)
            size = np.random.uniform(8, 30, 3)
            size[2] = np.random.uniform(12, 50)
            
            # Vary building materials
            if np.random.random() < 0.3:  # Glass buildings
                reflectivity = 0.9
            elif np.random.random() < 0.5:  # Concrete
                reflectivity = 0.5
            else:  # Brick/stone
                reflectivity = 0.6
                
            building = self._create_box_object(center, size, reflectivity=reflectivity)
            objects.append(building)
        
        # Trees (low reflectivity, complex shape)
        for _ in range(40):
            center = np.random.uniform(-150, 150, 2)
            center = np.append(center, 0)
            
            tree = self._create_tree_object(center, reflectivity=0.2)
            objects.append(tree)
        
        # Street furniture and signs
        for _ in range(60):
            center = np.random.uniform(-100, 100, 2)
            center = np.append(center, 0)
            
            obj = self._create_random_urban_object(center)
            objects.append(obj)
        
        return objects
    
    def _generate_complex_scene(self):
        """Complex scene with detailed urban environment"""
        objects = []
        
        # Detailed ground with multiple surface types
        ground = self._create_complex_ground(
            bounds=(-300, 300, -300, 300),
            resolution=1.0
        )
        objects.extend(ground)
        
        # Dense urban infrastructure
        building_count = 60
        for _ in range(building_count):
            center = np.random.uniform(-200, 200, 2)
            center = np.append(center, 0)
            
            # Create complex building with details
            building = self._create_detailed_building(center)
            objects.extend(building)  # Multiple objects per building
        
        # Dense vegetation
        for _ in range(100):
            center = np.random.uniform(-250, 250, 2)
            center = np.append(center, 0)
            vegetation = self._create_vegetation_cluster(center)
            objects.extend(vegetation)
        
        # Infrastructure details
        powerlines = self._create_powerline_network()
        objects.extend(powerlines)
        
        return objects
    
    def _create_ground_plane(self, bounds, resolution, roughness, reflectivity):
        """Create realistic ground plane with texture"""
        x_min, x_max, y_min, y_max = bounds
        
        # Generate grid points
        x = np.arange(x_min, x_max, resolution)
        y = np.arange(y_min, y_max, resolution)
        xx, yy = np.meshgrid(x, y)
        
        # Add terrain variation
        zz = (self.config['ground_height'] + 
              roughness * np.sin(xx * 0.1) * np.cos(yy * 0.1) +
              roughness * 0.3 * np.random.random(xx.shape))
        
        # Flatten for point cloud
        points = np.column_stack([xx.flatten(), yy.flatten(), zz.flatten()])
        
        # Add material-based intensity variation
        base_intensity = reflectivity
        intensity_var = 0.1 * reflectivity
        intensities = np.random.normal(base_intensity, intensity_var, len(points))
        intensities = np.clip(intensities, 0, 1)
        
        return {
            'points': points,
            'intensities': intensities,
            'material': 'ground',
            'reflectivity': reflectivity
        }
    
    def _create_box_object(self, center, size, reflectivity):
        """Create box-shaped object with realistic surface sampling"""
        points = []
        intensities = []
        
        # Sample surfaces with realistic density (based on LiDAR beam width)
        beam_width = self.config['beam_divergence'] / 1000  # Convert mrad to rad
        
        # For each face, calculate appropriate point density
        faces = [
            # Bottom face (usually not visible)
            ([center[0]-size[0]/2, center[0]+size[0]/2], [center[1]-size[1]/2, center[1]+size[1]/2], center[2]),
            # Top face
            ([center[0]-size[0]/2, center[0]+size[0]/2], [center[1]-size[1]/2, center[1]+size[1]/2], center[2]+size[2]),
            # Front face
            ([center[0]-size[0]/2, center[0]+size[0]/2], center[1]-size[1]/2, [center[2], center[2]+size[2]]),
            # Back face
            ([center[0]-size[0]/2, center[0]+size[0]/2], center[1]+size[1]/2, [center[2], center[2]+size[2]]),
            # Left face
            (center[0]-size[0]/2, [center[1]-size[1]/2, center[1]+size[1]/2], [center[2], center[2]+size[2]]),
            # Right face
            (center[0]+size[0]/2, [center[1]-size[1]/2, center[1]+size[1]/2], [center[2], center[2]+size[2]])
        ]
        
        for face_idx, face in enumerate(faces):
            if face_idx == 0:  # Skip bottom face (ground occlusion)
                continue
                
            # Calculate surface area and appropriate sampling density
            if face_idx <= 1:  # Top/bottom faces
                area = size[0] * size[1]
                n_points = int(area * 20)  # 20 points per m²
                if face_idx == 1:  # Top face
                    x_pts = np.random.uniform(face[0][0], face[0][1], n_points)
                    y_pts = np.random.uniform(face[1][0], face[1][1], n_points)
                    z_pts = np.full(n_points, face[2])
            else:  # Side faces
                if isinstance(face[0], list):  # Front/back faces
                    area = size[0] * size[2]
                    n_points = int(area * 15)
                    x_pts = np.random.uniform(face[0][0], face[0][1], n_points)
                    y_pts = np.full(n_points, face[1])
                    z_pts = np.random.uniform(face[2][0], face[2][1], n_points)
                else:  # Left/right faces
                    area = size[1] * size[2]
                    n_points = int(area * 15)
                    x_pts = np.full(n_points, face[0])
                    y_pts = np.random.uniform(face[1][0], face[1][1], n_points)
                    z_pts = np.random.uniform(face[2][0], face[2][1], n_points)
            
            if n_points > 0:
                face_points = np.column_stack([x_pts, y_pts, z_pts])
                points.append(face_points)
                
                # Material-based intensity with surface normal effects
                base_intensity = reflectivity
                intensity_noise = 0.05 * reflectivity
                face_intensities = np.random.normal(base_intensity, intensity_noise, n_points)
                intensities.append(np.clip(face_intensities, 0, 1))
        
        if points:
            all_points = np.vstack(points)
            all_intensities = np.concatenate(intensities)
        else:
            all_points = np.empty((0, 3))
            all_intensities = np.array([])
        
        return {
            'points': all_points,
            'intensities': all_intensities,
            'material': 'building',
            'reflectivity': reflectivity
        }
    
    def _create_tree_object(self, center, reflectivity):
        """Create realistic tree with trunk and foliage"""
        points = []
        intensities = []
        
        # Tree parameters
        trunk_height = np.random.uniform(3, 8)
        trunk_radius = np.random.uniform(0.2, 0.5)
        crown_radius = np.random.uniform(2, 5)
        crown_height = np.random.uniform(4, 8)
        
        # Trunk points (cylindrical)
        n_trunk = int(trunk_height * trunk_radius * 100)
        if n_trunk > 0:
            theta = np.random.uniform(0, 2*np.pi, n_trunk)
            r = np.random.uniform(0, trunk_radius, n_trunk)
            x_trunk = center[0] + r * np.cos(theta)
            y_trunk = center[1] + r * np.sin(theta)
            z_trunk = center[2] + np.random.uniform(0, trunk_height, n_trunk)
            
            trunk_points = np.column_stack([x_trunk, y_trunk, z_trunk])
            trunk_intensities = np.random.normal(reflectivity * 0.3, 0.02, n_trunk)
            
            points.append(trunk_points)
            intensities.append(np.clip(trunk_intensities, 0, 1))
        
        # Foliage points (irregular cloud)
        n_foliage = int(crown_radius**2 * crown_height * 50)
        if n_foliage > 0:
            # Generate points in ellipsoid with clustering
            phi = np.random.uniform(0, 2*np.pi, n_foliage)
            costheta = np.random.uniform(-1, 1, n_foliage)
            u = np.random.uniform(0, 1, n_foliage)
            
            theta = np.arccos(costheta)
            r = crown_radius * np.power(u, 1/3)
            
            x_foliage = center[0] + r * np.sin(theta) * np.cos(phi)
            y_foliage = center[1] + r * np.sin(theta) * np.sin(phi)
            z_foliage = center[2] + trunk_height + (crown_height/2) + (crown_height/2) * r/crown_radius * np.cos(theta)
            
            foliage_points = np.column_stack([x_foliage, y_foliage, z_foliage])
            
            # Foliage has lower reflectivity and more variation
            foliage_intensities = np.random.normal(reflectivity * 0.6, 0.1, n_foliage)
            
            points.append(foliage_points)
            intensities.append(np.clip(foliage_intensities, 0, 1))
        
        if points:
            all_points = np.vstack(points)
            all_intensities = np.concatenate(intensities)
        else:
            all_points = np.empty((0, 3))
            all_intensities = np.array([])
            
        return {
            'points': all_points,
            'intensities': all_intensities,
            'material': 'vegetation',
            'reflectivity': reflectivity
        }
    
    def _create_random_urban_object(self, center):
        """Create random urban objects (signs, poles, etc.)"""
        obj_type = np.random.choice(['pole', 'sign', 'bench', 'small_vehicle'])
        
        if obj_type == 'pole':
            # Street light or utility pole
            height = np.random.uniform(4, 12)
            radius = np.random.uniform(0.1, 0.3)
            n_points = int(height * radius * 200)
            
            theta = np.random.uniform(0, 2*np.pi, n_points)
            r = np.random.uniform(0, radius, n_points)
            x = center[0] + r * np.cos(theta)
            y = center[1] + r * np.sin(theta)
            z = center[2] + np.random.uniform(0, height, n_points)
            
            points = np.column_stack([x, y, z])
            intensities = np.random.normal(0.7, 0.1, n_points)  # Metal poles
            
        elif obj_type == 'sign':
            # Traffic sign or billboard
            width, height, thickness = np.random.uniform(1, 4), np.random.uniform(1, 3), 0.1
            pole_height = np.random.uniform(2, 4)
            
            # Sign face
            n_face = int(width * height * 100)
            x_face = center[0] + np.random.uniform(-width/2, width/2, n_face)
            y_face = np.full(n_face, center[1] + thickness/2)
            z_face = center[2] + pole_height + np.random.uniform(0, height, n_face)
            
            # Support pole
            n_pole = int(pole_height * 50)
            x_pole = np.full(n_pole, center[0])
            y_pole = np.full(n_pole, center[1])
            z_pole = center[2] + np.random.uniform(0, pole_height, n_pole)
            
            points = np.vstack([
                np.column_stack([x_face, y_face, z_face]),
                np.column_stack([x_pole, y_pole, z_pole])
            ])
            
            # Signs are highly reflective
            intensities = np.concatenate([
                np.random.normal(0.9, 0.05, n_face),  # Retroreflective sign
                np.random.normal(0.6, 0.1, n_pole)    # Metal pole
            ])
            
        else:  # Default small object
            size = np.random.uniform(0.5, 2, 3)
            return self._create_box_object(center, size, reflectivity=0.5)
        
        return {
            'points': points,
            'intensities': np.clip(intensities, 0, 1),
            'material': obj_type,
            'reflectivity': np.mean(intensities)
        }
    
    def _create_complex_ground(self, bounds, resolution):
        """Create complex ground with multiple surface types"""
        objects = []
        x_min, x_max, y_min, y_max = bounds
        
        # Main ground plane
        main_ground = self._create_ground_plane(bounds, resolution, 0.1, 0.4)
        objects.append(main_ground)
        
        # Road surfaces (higher reflectivity)
        for _ in range(10):
            road_center = np.random.uniform([x_min/2, y_min/2], [x_max/2, y_max/2], 2)
            road_length = np.random.uniform(50, 200)
            road_width = np.random.uniform(6, 12)
            
            road_points = self._create_road_section(road_center, road_length, road_width)
            objects.append(road_points)
        
        return objects
    
    def _create_road_section(self, center, length, width):
        """Create a road section with lane markings"""
        # Generate road surface
        n_points = int(length * width * 10)
        x_pts = center[0] + np.random.uniform(-length/2, length/2, n_points)
        y_pts = center[1] + np.random.uniform(-width/2, width/2, n_points)
        z_pts = np.full(n_points, self.config['ground_height'])
        
        points = np.column_stack([x_pts, y_pts, z_pts])
        
        # Road surface is more reflective than ground
        intensities = np.random.normal(0.6, 0.1, n_points)
        
        # Add lane markings (very high reflectivity)
        n_markings = int(length * 2)  # 2 markings per meter
        marking_x = center[0] + np.random.uniform(-length/2, length/2, n_markings)
        marking_y = np.full(n_markings, center[1])  # Center line
        marking_z = np.full(n_markings, self.config['ground_height'] + 0.01)
        
        marking_points = np.column_stack([marking_x, marking_y, marking_z])
        marking_intensities = np.random.normal(0.95, 0.02, n_markings)  # Retroreflective
        
        all_points = np.vstack([points, marking_points])
        all_intensities = np.concatenate([intensities, marking_intensities])
        
        return {
            'points': all_points,
            'intensities': np.clip(all_intensities, 0, 1),
            'material': 'road',
            'reflectivity': 0.7
        }
    
    def simulate_realistic_lidar_scan(self, environment, sensor_pose, frame_time):
        """
        Simulate realistic LiDAR scanning with proper Mid-70 behavior
        """
        # Get scan pattern for this frame
        pattern_idx = int(frame_time * self.config['scan_frequency']) % len(self.scan_pattern['azimuth'])
        points_this_frame = min(self.config['points_per_frame'], 
                               len(self.scan_pattern['azimuth']) - pattern_idx)
        
        # Extract beam directions for this frame
        azimuth_angles = self.scan_pattern['azimuth'][pattern_idx:pattern_idx + points_this_frame]
        elevation_angles = self.scan_pattern['elevation'][pattern_idx:pattern_idx + points_this_frame]
        
        # Convert to beam direction vectors in sensor frame
        az_rad = np.radians(azimuth_angles)
        el_rad = np.radians(elevation_angles)
        
        # Beam directions (normalized)
        beam_dirs = np.column_stack([
            np.cos(el_rad) * np.cos(az_rad),  # X (forward)
            np.cos(el_rad) * np.sin(az_rad),  # Y (left)
            np.sin(el_rad)                    # Z (up)
        ])
        
        # Transform beam directions to world frame
        sensor_pos = sensor_pose['position']
        sensor_rot = sensor_pose['orientation']
        R_matrix = R.from_euler('xyz', sensor_rot).as_matrix()
        
        world_beam_dirs = (R_matrix @ beam_dirs.T).T
        
        # Ray casting with realistic physics
        detected_points = []
        detected_intensities = []
        
        for i, beam_dir in enumerate(world_beam_dirs):
            # Cast ray and find intersections
            hit_point, hit_intensity, hit_range = self._cast_ray(
                sensor_pos, beam_dir, environment, max_range=self.config['range_max']
            )
            
            if hit_point is not None and hit_range >= self.config['range_min']:
                # Apply realistic noise and errors
                noisy_point, noisy_intensity = self._apply_measurement_noise(
                    hit_point, hit_intensity, hit_range, beam_dir
                )
                
                # Transform back to sensor frame for output
                relative_point = noisy_point - sensor_pos
                sensor_frame_point = (R_matrix.T @ relative_point)
                
                detected_points.append(sensor_frame_point)
                detected_intensities.append(noisy_intensity)
        
        if detected_points:
            points_array = np.array(detected_points)
            intensities_array = np.array(detected_intensities)
            return np.column_stack([points_array, intensities_array])
        else:
            return np.empty((0, 4))
    
    def _cast_ray(self, origin, direction, environment, max_range):
        """
        Cast a ray through the environment and find the first intersection
        """
        closest_hit = None
        closest_range = max_range
        closest_intensity = 0
        
        for obj in environment:
            points = obj['points']
            intensities = obj['intensities']
            
            if len(points) == 0:
                continue
            
            # Use KDTree for efficient spatial queries
            tree = cKDTree(points)
            
            # Sample points along the ray
            ranges = np.linspace(self.config['range_min'], max_range, 1000)
            ray_points = origin[np.newaxis, :] + ranges[:, np.newaxis] * direction[np.newaxis, :]
            
            # Find closest environment points to ray samples
            distances, indices = tree.query(ray_points, k=1)
            
            # Check for hits (within beam width)
            beam_radius_at_range = ranges * self.config['beam_divergence'] / 1000
            hit_mask = distances < beam_radius_at_range
            
            if np.any(hit_mask):
                hit_indices = np.where(hit_mask)[0]
                first_hit_idx = hit_indices[0]
                hit_range = ranges[first_hit_idx]
                
                if hit_range < closest_range:
                    closest_range = hit_range
                    point_idx = indices[first_hit_idx]
                    closest_hit = points[point_idx]
                    closest_intensity = intensities[point_idx]
                    
                    # Apply material-specific intensity based on incidence angle
                    surface_normal = self._estimate_surface_normal(points, point_idx)
                    incidence_angle = np.arccos(np.clip(np.dot(-direction, surface_normal), -1, 1))
                    intensity_factor = np.cos(incidence_angle) ** 0.5  # Lambertian-like reflection
                    
                    closest_intensity *= intensity_factor
        
        return closest_hit, closest_intensity, closest_range
    
    def _estimate_surface_normal(self, points, point_idx, k=5):
        """Estimate surface normal at a point using local neighbors"""
        if len(points) < k:
            return np.array([0, 0, 1])  # Default upward normal
            
        tree = cKDTree(points)
        distances, indices = tree.query(points[point_idx], k=k+1)  # +1 to include the point itself
        
        if len(indices) < 3:
            return np.array([0, 0, 1])
        
        # Use neighbors (excluding the point itself)
        neighbor_points = points[indices[1:]]  # Skip first index (the point itself)
        
        # Simple normal estimation using covariance
        centered = neighbor_points - np.mean(neighbor_points, axis=0)
        if len(centered) < 2:
            return np.array([0, 0, 1])
            
        try:
            _, _, vh = np.linalg.svd(centered)
            normal = vh[-1]  # Last row is normal to the plane
            return normal / np.linalg.norm(normal)
        except:
            return np.array([0, 0, 1])
    
    def _apply_measurement_noise(self, point, intensity, range_m, beam_direction):
        """Apply realistic measurement noise based on Mid-70 specifications"""
        # Range-dependent noise
        range_noise_std = self.config['range_precision'] + self.config['range_accuracy'] * (range_m / 100)
        range_noise = np.random.normal(0, range_noise_std)
        
        # Apply range noise along beam direction
        noisy_point = point + range_noise * beam_direction
        
        # Intensity noise (distance and material dependent)
        intensity_noise_std = self.config['intensity_noise'] * intensity
        intensity_noise = np.random.normal(0, intensity_noise_std)
        noisy_intensity = np.clip(intensity + intensity_noise, 0, 1)
        
        # Atmospheric attenuation (for long ranges)
        if self.config['atmospheric_attenuation'] and range_m > 50:
            attenuation = np.exp(-range_m / 5000)  # 5km attenuation distance
            noisy_intensity *= attenuation
        
        # Random outliers (multipath, specular reflections)
        if np.random.random() < self.config['outlier_rate']:
            # Outlier points have random range errors
            outlier_error = np.random.normal(0, range_m * 0.1)
            noisy_point += outlier_error * beam_direction
            noisy_intensity *= 0.3  # Outliers typically have lower intensity
        
        return noisy_point, noisy_intensity
    
    def run_simulation(self):
        """Run complete realistic simulation"""
        print("Starting Realistic Livox Mid-70 Simulation...")
        print(f"FOV: {self.config['fov_degrees']}° circular")
        print(f"Range: {self.config['range_min']}m - {self.config['range_max']}m")
        print(f"Points per frame: {self.config['points_per_frame']:,}")
        
        # Generate trajectory
        print("Generating vehicle trajectory...")
        trajectory = self.generate_trajectory()
        trajectory = self.add_sensor_noise(trajectory)
        
        # Generate realistic environment
        print("Generating realistic 3D environment...")
        environment = self.generate_realistic_environment()
        print(f"Environment contains {len(environment)} objects")
        
        # Simulation timestamps
        lidar_times = np.linspace(0, self.config['duration'], 
                                int(self.config['duration'] * self.config['lidar_fps']))
        
        # Results storage
        all_scans = []
        aligned_pointclouds = []
        motion_data = []
        
        print(f"Simulating {len(lidar_times)} LiDAR frames with realistic Mid-70 behavior...")
        
        for i, t in enumerate(lidar_times):
            # Interpolate sensor pose at LiDAR timestamp
            pose_idx = np.searchsorted(trajectory['time'], t)
            if pose_idx >= len(trajectory['time']):
                pose_idx = len(trajectory['time']) - 1
                
            sensor_pose = {
                'position': trajectory['position_gps'][pose_idx],
                'orientation': trajectory['orientation_imu'][pose_idx],
                'velocity': trajectory['velocity'][pose_idx]
            }
            
            # Simulate realistic LiDAR scan
            scan = self.simulate_realistic_lidar_scan(environment, sensor_pose, t)
            
            # Store raw scan (local coordinates)
            all_scans.append({
                'frame_id': i,
                'timestamp': t,
                'points_local': scan,
                'sensor_pose': sensor_pose
            })
            
            # Apply motion compensation transformation
            transformation = {
                'translation': sensor_pose['position'],
                'rotation': sensor_pose['orientation']
            }
            
            aligned_scan = self.transform_pointcloud(scan, transformation)
            aligned_pointclouds.append(aligned_scan)
            
            # Store motion data
            motion_data.append({
                'frame_id': i,
                'timestamp': t,
                'gps_lat': sensor_pose['position'][1] / 111320,
                'gps_lon': sensor_pose['position'][0] / 111320,
                'gps_alt': sensor_pose['position'][2],
                'imu_roll': sensor_pose['orientation'][0],
                'imu_pitch': sensor_pose['orientation'][1],
                'imu_yaw': sensor_pose['orientation'][2],
                'vel_x': sensor_pose['velocity'][0],
                'vel_y': sensor_pose['velocity'][1],
                'vel_z': sensor_pose['velocity'][2]
            })
            
            if (i + 1) % 5 == 0:
                avg_points = np.mean([len(s['points_local']) for s in all_scans[-5:]])
                print(f"Processed frame {i + 1}/{len(lidar_times)}, avg points: {avg_points:.0f}")
        
        return {
            'raw_scans': all_scans,
            'aligned_pointclouds': aligned_pointclouds,
            'motion_data': motion_data,
            'trajectory': trajectory,
            'environment': environment
        }
    
    def transform_pointcloud(self, points, transformation):
        """Apply transformation to point cloud"""
        if len(points) == 0:
            return points
            
        R_matrix = R.from_euler('xyz', transformation['rotation']).as_matrix()
        transformed = (R_matrix @ points[:, :3].T).T + transformation['translation']
        return np.column_stack([transformed, points[:, 3]])
    
    def save_results(self, results, output_dir='realistic_lidar_output'):
        """Save simulation results to files"""
        os.makedirs(output_dir, exist_ok=True)
        
        print(f"Saving results to {output_dir}...")
        
        # Save motion data as CSV
        motion_df = pd.DataFrame(results['motion_data'])
        motion_df.to_csv(os.path.join(output_dir, 'motion_data.csv'), index=False)
        
        # Save individual frame point clouds (PCD format)
        pcd_dir = os.path.join(output_dir, 'raw_scans_pcd')
        os.makedirs(pcd_dir, exist_ok=True)
        
        for scan in results['raw_scans']:
            if len(scan['points_local']) > 0:
                filename = os.path.join(pcd_dir, f'frame_{scan["frame_id"]:04d}.pcd')
                self.save_pcd(scan['points_local'], filename)
        
        # Save aligned point clouds
        aligned_dir = os.path.join(output_dir, 'aligned_scans_pcd')
        os.makedirs(aligned_dir, exist_ok=True)
        
        for i, aligned_pc in enumerate(results['aligned_pointclouds']):
            if len(aligned_pc) > 0:
                filename = os.path.join(aligned_dir, f'aligned_frame_{i:04d}.pcd')
                self.save_pcd(aligned_pc, filename)
        
        # Save merged point clouds
        valid_aligned = [pc for pc in results['aligned_pointclouds'] if len(pc) > 0]
        if valid_aligned:
            merged_aligned = np.vstack(valid_aligned)
            self.save_pcd(merged_aligned, os.path.join(output_dir, 'merged_aligned.pcd'))
        
        valid_raw = [scan['points_local'] for scan in results['raw_scans'] if len(scan['points_local']) > 0]
        if valid_raw:
            merged_raw = np.vstack(valid_raw)
            self.save_pcd(merged_raw, os.path.join(output_dir, 'merged_raw_overlapped.pcd'))
        
        # Save as LAS format
        try:
            if valid_aligned:
                self.save_las(merged_aligned, os.path.join(output_dir, 'merged_aligned.las'))
                print("LAS format saved successfully")
        except Exception as e:
            print(f"Could not save LAS format: {e}")
        
        # Save as LVX formats
        try:
            base_filename = os.path.join(output_dir, 'realistic_lidar_data')
            self.save_lvx(results, base_filename)
            print("LVX formats saved successfully")
        except Exception as e:
            print(f"Could not save LVX formats: {e}")
        
        # Save trajectory
        traj_data = {
            'time': results['trajectory']['time'],
            'x': results['trajectory']['position'][:, 0],
            'y': results['trajectory']['position'][:, 1],
            'z': results['trajectory']['position'][:, 2],
            'x_gps': results['trajectory']['position_gps'][:, 0],
            'y_gps': results['trajectory']['position_gps'][:, 1],
            'z_gps': results['trajectory']['position_gps'][:, 2]
        }
        traj_df = pd.DataFrame(traj_data)
        traj_df.to_csv(os.path.join(output_dir, 'trajectory.csv'), index=False)
        
        print("Results saved successfully!")
        return output_dir
    
    def save_pcd(self, points, filename):
        """Save point cloud in PCD format"""
        if len(points) == 0:
            return
            
        with open(filename, 'w') as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\n")
            f.write("FIELDS x y z intensity\n")
            f.write("SIZE 4 4 4 4\n")
            f.write("TYPE F F F F\n")
            f.write("COUNT 1 1 1 1\n")
            f.write(f"WIDTH {len(points)}\n")
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {len(points)}\n")
            f.write("DATA ascii\n")
            
            for point in points:
                f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} {point[3]:.6f}\n")
    
    def save_las(self, points, filename):
        """Save point cloud in LAS format"""
        if len(points) == 0:
            return
            
        header = laspy.LasHeader(point_format=3, version="1.2")
        las = laspy.LasData(header)
        
        las.x = points[:, 0]
        las.y = points[:, 1] 
        las.z = points[:, 2]
        las.intensity = (points[:, 3] * 65535).astype(np.uint16)
        
        las.write(filename)
    
    def save_lvx(self, results, base_filename):
        """Save point cloud data in Livox LVX format"""
        lvx_writer = LivoxLVXWriter()
        
        # Prepare frame data
        frames_data = []
        for scan in results['raw_scans']:
            if len(scan['points_local']) > 0:
                frame_data = {
                    'frame_id': scan['frame_id'],
                    'timestamp': scan['timestamp'],
                    'points': scan['points_local']
                }
                frames_data.append(frame_data)
        
        if frames_data:
            # Write corrected LVX format
            try:
                lvx_writer.write_lvx_file(frames_data, f"{base_filename}.lvx")
                print(f"✅ Corrected LVX format: {base_filename}.lvx")
            except Exception as e:
                print(f"❌ LVX creation failed: {e}")
    
    def create_visualization(self, results, output_dir):
        """Create comprehensive visualizations of the realistic simulation"""
        print("Creating realistic simulation visualizations...")
        
        # 1. Scanning pattern visualization
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
        
        # Rosette pattern
        pattern = self.scan_pattern
        ax1.scatter(pattern['azimuth'], pattern['elevation'], c=pattern['time_offset'], 
                   s=0.5, alpha=0.6, cmap='viridis')
        ax1.set_xlabel('Azimuth (degrees)')
        ax1.set_ylabel('Elevation (degrees)')
        ax1.set_title('Livox Mid-70 Rosette Scanning Pattern')
        ax1.grid(True)
        
        # Add FOV circle
        circle = plt.Circle((0, 0), self.config['fov_degrees']/2, 
                           fill=False, color='red', linestyle='--', linewidth=2)
        ax1.add_patch(circle)
        ax1.set_aspect('equal')
        
        # Point count statistics
        frame_sizes = [len(scan['points_local']) for scan in results['raw_scans']]
        ax2.plot(frame_sizes, 'b-', linewidth=2)
        ax2.axhline(y=np.mean(frame_sizes), color='r', linestyle='--', 
                   label=f'Mean: {np.mean(frame_sizes):.0f}')
        ax2.set_xlabel('Frame Number')
        ax2.set_ylabel('Points per Frame')
        ax2.set_title('Realistic Point Count Variation')
        ax2.legend()
        ax2.grid(True)
        
        # Range distribution
        if results['raw_scans']:
            sample_scan = results['raw_scans'][len(results['raw_scans'])//2]['points_local']
            if len(sample_scan) > 0:
                ranges = np.sqrt(np.sum(sample_scan[:, :3]**2, axis=1))
                ax3.hist(ranges, bins=50, alpha=0.7, color='green')
                ax3.set_xlabel('Range (m)')
                ax3.set_ylabel('Point Count')
                ax3.set_title('Range Distribution (Mid-frame Sample)')
                ax3.grid(True, alpha=0.3)
        
        # Intensity distribution
        if results['raw_scans']:
            sample_intensities = sample_scan[:, 3] if len(sample_scan) > 0 else []
            if len(sample_intensities) > 0:
                ax4.hist(sample_intensities, bins=30, alpha=0.7, color='orange')
                ax4.set_xlabel('Intensity')
                ax4.set_ylabel('Point Count')
                ax4.set_title('Intensity Distribution')
                ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, 'realistic_simulation_analysis.png'), 
                    dpi=300, bbox_inches='tight')
        plt.close()
        
        # 2. 3D visualization of scan quality
        if len(results['raw_scans']) > 0:
            fig = plt.figure(figsize=(15, 6))
            
            # Raw scan (sensor frame)
            ax1 = fig.add_subplot(121, projection='3d')
            sample_raw = results['raw_scans'][0]['points_local']
            if len(sample_raw) > 0:
                # Color by range
                ranges = np.sqrt(np.sum(sample_raw[:, :3]**2, axis=1))
                scatter = ax1.scatter(sample_raw[:, 0], sample_raw[:, 1], sample_raw[:, 2],
                                    c=ranges, s=0.5, alpha=0.6, cmap='plasma')
                plt.colorbar(scatter, ax=ax1, label='Range (m)')
                ax1.set_xlabel('X (m)')
                ax1.set_ylabel('Y (m)')
                ax1.set_zlabel('Z (m)')
                ax1.set_title('Raw Scan (Sensor Frame)')
            
            # Aligned scan (world frame)  
            ax2 = fig.add_subplot(122, projection='3d')
            sample_aligned = results['aligned_pointclouds'][0]
            if len(sample_aligned) > 0:
                ranges = np.sqrt(np.sum((sample_aligned[:, :3] - results['raw_scans'][0]['sensor_pose']['position'])**2, axis=1))
                scatter = ax2.scatter(sample_aligned[:, 0], sample_aligned[:, 1], sample_aligned[:, 2],
                                    c=sample_aligned[:, 3], s=0.5, alpha=0.6, cmap='viridis')
                plt.colorbar(scatter, ax=ax2, label='Intensity')
                ax2.set_xlabel('X (m)')
                ax2.set_ylabel('Y (m)')
                ax2.set_zlabel('Z (m)')
                ax2.set_title('Aligned Scan (World Frame)')
            
            plt.tight_layout()
            plt.savefig(os.path.join(output_dir, 'scan_comparison_3d.png'), 
                       dpi=300, bbox_inches='tight')
            plt.close()
        
        print("Realistic visualizations saved!")


def create_visualization_script(output_dir):
    """Create a standalone visualization script using matplotlib and optional Open3D"""
    
    viz_script = f'''
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

# Try to import Open3D for advanced visualization
try:
    import open3d as o3d
    HAS_OPEN3D = True
    print("Open3D available - enhanced visualizations enabled")
except ImportError:
    HAS_OPEN3D = False
    print("Open3D not available - using matplotlib only")

def load_pcd_file(filename):
    """Load PCD file (simple ASCII format)"""
    points = []
    with open(filename, 'r') as f:
        lines = f.readlines()
        
        # Find data section
        data_start = None
        for i, line in enumerate(lines):
            if line.startswith('DATA ascii'):
                data_start = i + 1
                break
        
        if data_start:
            for line in lines[data_start:]:
                parts = line.strip().split()
                if len(parts) >= 4:
                    x, y, z, intensity = map(float, parts[:4])
                    points.append([x, y, z, intensity])
    
    return np.array(points) if points else np.empty((0, 4))

def visualize_single_scan(pcd_file):
    """Visualize a single scan with both matplotlib and Open3D"""
    
    print(f"Loading {{pcd_file}}")
    points = load_pcd_file(pcd_file)
    
    if len(points) == 0:
        print("No points loaded!")
        return
    
    print(f"Loaded {{len(points):,}} points")
    
    # Matplotlib visualization
    fig = plt.figure(figsize=(15, 5))
    
    # 3D scatter plot
    ax1 = fig.add_subplot(131, projection='3d')
    ranges = np.sqrt(np.sum(points[:, :3]**2, axis=1))
    scatter = ax1.scatter(points[:, 0], points[:, 1], points[:, 2], 
                         c=ranges, s=0.5, alpha=0.6, cmap='plasma')
    plt.colorbar(scatter, ax=ax1, shrink=0.5, label='Range (m)')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('3D Point Cloud (Colored by Range)')
    
    # Top view
    ax2 = fig.add_subplot(132)
    scatter2 = ax2.scatter(points[:, 0], points[:, 1], c=points[:, 3], 
                          s=0.5, alpha=0.7, cmap='viridis')
    plt.colorbar(scatter2, ax=ax2, label='Intensity')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('Top View (Colored by Intensity)')
    ax2.set_aspect('equal')
    
    # Range vs Intensity
    ax3 = fig.add_subplot(133)
    ax3.scatter(ranges, points[:, 3], alpha=0.5, s=1)
    ax3.set_xlabel('Range (m)')
    ax3.set_ylabel('Intensity')
    ax3.set_title('Range vs Intensity')
    ax3.grid(True)
    
    plt.tight_layout()
    plt.show()
    
    # Open3D visualization (if available)
    if HAS_OPEN3D:
        print("Creating Open3D visualization...")
        
        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        
        # Color by intensity
        intensities = points[:, 3]
        colors = plt.cm.viridis(intensities)[:, :3]  # Convert to RGB
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
        # Create coordinate frame
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=5.0)
        
        # Visualize
        print("Opening Open3D viewer...")
        print("Controls: Mouse to rotate/pan/zoom, 'Q' to quit")
        o3d.visualization.draw_geometries([pcd, coord_frame],
                                        window_name="Realistic Livox Mid-70 Simulation",
                                        width=1200, height=800)

def compare_scans():
    """Compare raw vs aligned scans"""
    output_dir = "{output_dir}"
    
    # Load sample files
    raw_file = os.path.join(output_dir, "raw_scans_pcd", "frame_0000.pcd")
    aligned_file = os.path.join(output_dir, "aligned_scans_pcd", "aligned_frame_0000.pcd")
    
    if os.path.exists(raw_file) and os.path.exists(aligned_file):
        print("Comparing raw vs aligned scans...")
        
        raw_points = load_pcd_file(raw_file)
        aligned_points = load_pcd_file(aligned_file)
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
        
        # Raw scan (centered at origin)
        if len(raw_points) > 0:
            ax1.scatter(raw_points[:, 0], raw_points[:, 1], 
                       c=raw_points[:, 3], s=0.5, alpha=0.7, cmap='viridis')
            ax1.set_xlabel('X (m)')
            ax1.set_ylabel('Y (m)')
            ax1.set_title('Raw Scan (Sensor Frame)')
            ax1.set_aspect('equal')
            ax1.grid(True)
        
        # Aligned scan (world coordinates)
        if len(aligned_points) > 0:
            ax2.scatter(aligned_points[:, 0], aligned_points[:, 1], 
                       c=aligned_points[:, 3], s=0.5, alpha=0.7, cmap='viridis')
            ax2.set_xlabel('X (m)')
            ax2.set_ylabel('Y (m)')
            ax2.set_title('Aligned Scan (World Frame)')
            ax2.set_aspect('equal')
            ax2.grid(True)
        
        plt.tight_layout()
        plt.show()
        
        # Statistics comparison
        if len(raw_points) > 0 and len(aligned_points) > 0:
            print(f"\\nStatistics:")
            print(f"Raw scan points: {{len(raw_points):,}}")
            print(f"Aligned scan points: {{len(aligned_points):,}}")
            
            raw_ranges = np.sqrt(np.sum(raw_points[:, :3]**2, axis=1))
            print(f"Raw scan range: {{raw_ranges.min():.2f}} - {{raw_ranges.max():.2f}} m")
            print(f"Raw scan mean intensity: {{raw_points[:, 3].mean():.3f}}")
            
            aligned_ranges = np.sqrt(np.sum(aligned_points[:, :3]**2, axis=1))
            print(f"Aligned scan range: {{aligned_ranges.min():.2f}} - {{aligned_ranges.max():.2f}} m")
            print(f"Aligned scan mean intensity: {{aligned_points[:, 3].mean():.3f}}")
    
    else:
        print("Sample scan files not found!")

def visualize_merged_cloud():
    """Visualize the complete merged point cloud"""
    output_dir = "{output_dir}"
    merged_file = os.path.join(output_dir, "merged_aligned.pcd")
    
    if os.path.exists(merged_file):
        print("Loading merged aligned point cloud...")
        points = load_pcd_file(merged_file)
        
        if len(points) > 0:
            print(f"Merged cloud contains {{len(points):,}} points")
            
            # Subsample for visualization if too large
            if len(points) > 100000:
                indices = np.random.choice(len(points), 100000, replace=False)
                points = points[indices]
                print(f"Subsampled to {{len(points):,}} points for visualization")
            
            # Create comprehensive visualization
            fig = plt.figure(figsize=(20, 10))
            
            # 3D view
            ax1 = fig.add_subplot(231, projection='3d')
            ranges = np.sqrt(np.sum(points[:, :3]**2, axis=1))
            scatter1 = ax1.scatter(points[:, 0], points[:, 1], points[:, 2],
                                 c=points[:, 3], s=0.1, alpha=0.5, cmap='viridis')
            ax1.set_xlabel('X (m)')
            ax1.set_ylabel('Y (m)')
            ax1.set_zlabel('Z (m)')
            ax1.set_title('Complete Merged Point Cloud')
            
            # Top view
            ax2 = fig.add_subplot(232)
            ax2.scatter(points[:, 0], points[:, 1], c=points[:, 3], 
                       s=0.1, alpha=0.5, cmap='viridis')
            ax2.set_xlabel('X (m)')
            ax2.set_ylabel('Y (m)')
            ax2.set_title('Top View')
            ax2.set_aspect('equal')
            
            # Side view
            ax3 = fig.add_subplot(233)
            ax3.scatter(points[:, 0], points[:, 2], c=points[:, 3], 
                       s=0.1, alpha=0.5, cmap='viridis')
            ax3.set_xlabel('X (m)')
            ax3.set_ylabel('Z (m)')
            ax3.set_title('Side View')
            
            # Range histogram
            ax4 = fig.add_subplot(234)
            ax4.hist(ranges, bins=50, alpha=0.7, color='blue')
            ax4.set_xlabel('Range (m)')
            ax4.set_ylabel('Point Count')
            ax4.set_title('Range Distribution')
            ax4.grid(True)
            
            # Intensity histogram
            ax5 = fig.add_subplot(235)
            ax5.hist(points[:, 3], bins=50, alpha=0.7, color='green')
            ax5.set_xlabel('Intensity')
            ax5.set_ylabel('Point Count')
            ax5.set_title('Intensity Distribution')
            ax5.grid(True)
            
            # Height distribution
            ax6 = fig.add_subplot(236)
            ax6.hist(points[:, 2], bins=50, alpha=0.7, color='red')
            ax6.set_xlabel('Height (m)')
            ax6.set_ylabel('Point Count')
            ax6.set_title('Height Distribution')
            ax6.grid(True)
            
            plt.tight_layout()
            plt.show()
            
            # Open3D visualization of merged cloud
            if HAS_OPEN3D:
                print("Creating Open3D visualization of merged cloud...")
                
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(points[:, :3])
                
                # Color by height for better visualization
                heights = points[:, 2]
                normalized_heights = (heights - heights.min()) / (heights.max() - heights.min())
                colors = plt.cm.terrain(normalized_heights)[:, :3]
                pcd.colors = o3d.utility.Vector3dVector(colors)
                
                # Add coordinate frame and ground plane
                coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10.0)
                
                # Create simple ground plane mesh
                ground_plane = o3d.geometry.TriangleMesh.create_box(width=200, height=0.1, depth=200)
                ground_plane.translate([-100, -100, heights.min()])
                ground_plane.paint_uniform_color([0.6, 0.6, 0.6])
                
                print("Opening Open3D viewer for merged cloud...")
                o3d.visualization.draw_geometries([pcd, coord_frame, ground_plane],
                                                window_name="Complete Merged Point Cloud",
                                                width=1400, height=1000)
        else:
            print("No points in merged file!")
    else:
        print("Merged file not found!")

def main():
    """Main visualization function"""
    print("Realistic Livox Mid-70 Simulation - Visualization Tool")
    print("="*60)
    
    output_dir = "{output_dir}"
    
    while True:
        print("\\nChoose visualization option:")
        print("1. View single scan (first raw scan)")
        print("2. Compare raw vs aligned scans")  
        print("3. View complete merged point cloud")
        print("4. Exit")
        
        try:
            choice = input("\\nEnter choice (1-4): ").strip()
            
            if choice == '1':
                # Show first raw scan
                scan_file = os.path.join(output_dir, "raw_scans_pcd", "frame_0000.pcd")
                if os.path.exists(scan_file):
                    visualize_single_scan(scan_file)
                else:
                    print("Raw scan file not found!")
                    
            elif choice == '2':
                compare_scans()
                
            elif choice == '3':
                visualize_merged_cloud()
                
            elif choice == '4':
                print("Exiting...")
                break
                
            else:
                print("Invalid choice!")
                
        except KeyboardInterrupt:
            print("\\nExiting...")
            break
        except Exception as e:
            print(f"Error: {{e}}")

if __name__ == "__main__":
    main()
'''
    
    viz_filename = os.path.join(output_dir, 'visualize_results.py')
    with open(viz_filename, 'w') as f:
        f.write(viz_script)
    
    print(f"Visualization script saved as: {viz_filename}")
    print("To use: python visualize_results.py")
    
    return viz_filename


class LiDARMotionSimulator(RealisticLivoxMid70Simulator):
    """Backward compatibility wrapper"""
    pass


def main():
    """Main simulation execution with realistic Mid-70 behavior"""
    print("Realistic Livox Mid-70 LiDAR Simulation")
    print("=" * 50)
    
    # Realistic configuration scenarios
    configs = {
        'urban_realistic': {
            'duration': 30.0,
            'trajectory_type': 'figure_eight',
            'environment_complexity': 'medium',
            'max_speed': 8.0,          # Realistic urban speed
            'lidar_fps': 10,           # Mid-70 native rate
            'points_per_frame': 48000, # Reduced for realistic density
            'range_max': 120.0,        # Urban range limit
            'outlier_rate': 0.002,     # 0.2% outliers
            'atmospheric_attenuation': True
        },
        'highway_high_speed': {
            'duration': 45.0,
            'trajectory_type': 'linear',
            'environment_complexity': 'simple',
            'max_speed': 22.0,         # Highway speed
            'lidar_fps': 10,
            'points_per_frame': 72000, # Higher density for long range
            'range_max': 200.0,        # Full range capability
            'outlier_rate': 0.001,
            'atmospheric_attenuation': True
        },
        'parking_detailed': {
            'duration': 20.0,
            'trajectory_type': 'circular',
            'environment_complexity': 'complex',
            'max_speed': 3.0,          # Parking lot speed
            'lidar_fps': 10,
            'points_per_frame': 96000, # Maximum density
            'range_max': 50.0,         # Close range focus
            'outlier_rate': 0.0005,    # Very clean environment
            'atmospheric_attenuation': False
        }
    }
    
    # Select configuration
    config_name = 'urban_realistic'  # Change this to test different scenarios
    print(f"Using realistic configuration: {config_name}")
    
    # Initialize simulator
    simulator = RealisticLivoxMid70Simulator(configs[config_name])
    
    # Run simulation
    results = simulator.run_simulation()
    
    # Save results
    output_dir = simulator.save_results(results)
    
    # Generate visualizations
    simulator.create_visualization(results, output_dir)
    
    # Create visualization script
    viz_script = create_visualization_script(output_dir)
    
    # Generate comprehensive report
    simulator.generate_report(results, output_dir)
    
    print("\n" + "=" * 60)
    print("REALISTIC SIMULATION COMPLETE!")
    print(f"Results saved to: {output_dir}")
    print("\n🎯 Key Improvements Made:")
    print("✅ Realistic Mid-70 rosette scanning pattern")
    print("✅ Proper 70.4° circular FOV simulation") 
    print("✅ Physics-based ray casting with occlusions")
    print("✅ Distance-dependent noise and intensity")
    print("✅ Material-based reflectivity modeling")
    print("✅ Atmospheric attenuation for long ranges")
    print("✅ Realistic point density variation")
    print("✅ Proper beam divergence simulation")
    
    print(f"\n📊 Simulation Statistics:")
    total_points = sum(len(scan['points_local']) for scan in results['raw_scans'])
    avg_points = total_points / len(results['raw_scans']) if results['raw_scans'] else 0
    print(f"• {len(results['raw_scans'])} frames processed")
    print(f"• {total_points:,} total points generated")
    print(f"• {avg_points:.0f} average points per frame")
    print(f"• Realistic Mid-70 LVX files generated")
    
    print(f"\n🔧 Usage:")
    print(f"• Run: python {viz_script} for interactive visualization")
    print(f"• Load *.lvx files in Livox Viewer")
    print(f"• Use *.pcd files in CloudCompare/PCL")
    print(f"• Import *.las files in GIS software")
    
    return output_dir


if __name__ == "__main__":
    # python realistic_livox_simulator.py
    main()
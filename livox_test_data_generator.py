#!/usr/bin/env python3
"""
Livox Mid-70 Motion Compensation Test Data Generator

This script generates comprehensive test data for testing motion compensation algorithms
for Livox Mid-70 LiDAR systems. It creates synthetic but realistic data including:
- Point cloud data (simulating LVX format)
- RTK GNSS positioning data
- IMU orientation and motion data
- Synchronized timestamps
- Motion trajectory simulation

Author: Generated for Livox Mid-70 research
Date: 2025-08-13
"""

import numpy as np
import pandas as pd
import json
import struct
import os
from datetime import datetime, timedelta
from typing import Tuple, List, Dict
import matplotlib.pyplot as plt
from pathlib import Path

class LivoxTestDataGenerator:
    """
    Generates comprehensive test data for Livox Mid-70 motion compensation testing
    """
    
    def __init__(self, output_dir: str = "livox_test_data"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        
        # Simulation parameters
        self.duration = 60.0  # seconds
        self.lidar_frequency = 10.0  # Hz
        self.gnss_frequency = 10.0   # Hz
        self.imu_frequency = 100.0   # Hz
        
        # Generate timestamps
        self.start_time = datetime.now()
        self.generate_timestamps()
        
        # Motion trajectory parameters
        self.trajectory_type = "figure_8"  # Options: "straight", "circular", "figure_8"
        self.velocity = 10.0  # m/s
        self.trajectory_scale = 50.0  # meters
        
        print(f"Initializing Livox Mid-70 test data generator")
        print(f"Output directory: {self.output_dir}")
        print(f"Duration: {self.duration}s")
        print(f"Trajectory: {self.trajectory_type}")
    
    def generate_timestamps(self):
        """Generate synchronized timestamps for all sensors"""
        # LiDAR timestamps
        self.lidar_times = np.arange(0, self.duration, 1.0/self.lidar_frequency)
        
        # GNSS timestamps
        self.gnss_times = np.arange(0, self.duration, 1.0/self.gnss_frequency)
        
        # IMU timestamps (high frequency)
        self.imu_times = np.arange(0, self.duration, 1.0/self.imu_frequency)
        
        # Convert to absolute timestamps
        self.lidar_timestamps = [self.start_time + timedelta(seconds=t) for t in self.lidar_times]
        self.gnss_timestamps = [self.start_time + timedelta(seconds=t) for t in self.gnss_times]
        self.imu_timestamps = [self.start_time + timedelta(seconds=t) for t in self.imu_times]
    
    def generate_trajectory(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Generate vehicle trajectory based on specified pattern
        Returns: positions (x, y, z), velocities, accelerations
        """
        # Starting position (latitude/longitude will be added later)
        start_pos = np.array([0.0, 0.0, 0.0])
        
        if self.trajectory_type == "straight":
            # Straight line with constant velocity
            x = self.velocity * self.gnss_times
            y = np.zeros_like(self.gnss_times)
            z = np.zeros_like(self.gnss_times)
            
        elif self.trajectory_type == "circular":
            # Circular trajectory
            angular_velocity = self.velocity / self.trajectory_scale
            x = self.trajectory_scale * np.sin(angular_velocity * self.gnss_times)
            y = self.trajectory_scale * np.cos(angular_velocity * self.gnss_times)
            z = np.zeros_like(self.gnss_times)
            
        elif self.trajectory_type == "figure_8":
            # Figure-8 trajectory
            omega = 2 * np.pi / 30.0  # Complete figure-8 every 30 seconds
            x = self.trajectory_scale * np.sin(omega * self.gnss_times)
            y = self.trajectory_scale * np.sin(2 * omega * self.gnss_times) / 2
            z = 2 * np.sin(omega * self.gnss_times / 2)  # Slight vertical motion
        
        positions = np.column_stack([x, y, z])
        
        # Calculate velocities (numerical differentiation)
        velocities = np.zeros_like(positions)
        velocities[1:] = np.diff(positions, axis=0) / np.diff(self.gnss_times).reshape(-1, 1)
        velocities[0] = velocities[1]  # First point same as second
        
        # Calculate accelerations
        accelerations = np.zeros_like(positions)
        accelerations[1:] = np.diff(velocities, axis=0) / np.diff(self.gnss_times).reshape(-1, 1)
        accelerations[0] = accelerations[1]
        
        return positions, velocities, accelerations
    
    def generate_rtk_gnss_data(self, positions: np.ndarray) -> Dict:
        """
        Generate RTK GNSS data with centimeter-level accuracy
        """
        # Base coordinates (example: somewhere in Germany)
        base_lat = 52.520008  # Berlin latitude
        base_lon = 13.404954  # Berlin longitude
        base_alt = 50.0       # meters
        
        # Convert local positions to lat/lon/alt
        # Approximate conversion (for testing purposes)
        lat_scale = 1.0 / 111000.0  # degrees per meter (latitude)
        lon_scale = 1.0 / (111000.0 * np.cos(np.radians(base_lat)))  # longitude
        
        latitudes = base_lat + positions[:, 0] * lat_scale
        longitudes = base_lon + positions[:, 1] * lon_scale
        altitudes = base_alt + positions[:, 2]
        
        # Add RTK precision noise (centimeter level)
        rtk_noise_std = 0.01  # 1cm standard deviation
        lat_noise = np.random.normal(0, rtk_noise_std * lat_scale, len(latitudes))
        lon_noise = np.random.normal(0, rtk_noise_std * lon_scale, len(longitudes))
        alt_noise = np.random.normal(0, rtk_noise_std, len(altitudes))
        
        gnss_data = {
            'timestamps': [ts.isoformat() for ts in self.gnss_timestamps],
            'latitude': latitudes + lat_noise,
            'longitude': longitudes + lon_noise,
            'altitude': altitudes + alt_noise,
            'fix_quality': [4] * len(latitudes),  # RTK Fixed
            'num_satellites': np.random.randint(12, 18, len(latitudes)),
            'hdop': np.random.uniform(0.5, 1.2, len(latitudes)),  # Horizontal dilution
            'vdop': np.random.uniform(0.6, 1.5, len(altitudes)),  # Vertical dilution
            'position_accuracy_h': np.full(len(latitudes), 0.01),  # 1cm horizontal
            'position_accuracy_v': np.full(len(latitudes), 0.015), # 1.5cm vertical
        }
        
        return gnss_data
    
    def generate_imu_data(self, positions: np.ndarray, velocities: np.ndarray, 
                         accelerations: np.ndarray) -> Dict:
        """
        Generate IMU data (accelerometer, gyroscope, magnetometer)
        """
        # Interpolate trajectory to IMU frequency
        imu_positions = np.zeros((len(self.imu_times), 3))
        imu_velocities = np.zeros((len(self.imu_times), 3))
        imu_accelerations = np.zeros((len(self.imu_times), 3))
        
        for i in range(3):  # x, y, z
            imu_positions[:, i] = np.interp(self.imu_times, self.gnss_times, positions[:, i])
            imu_velocities[:, i] = np.interp(self.imu_times, self.gnss_times, velocities[:, i])
            imu_accelerations[:, i] = np.interp(self.imu_times, self.gnss_times, accelerations[:, i])
        
        # Calculate orientation (roll, pitch, yaw) based on motion
        roll = np.zeros(len(self.imu_times))
        pitch = np.zeros(len(self.imu_times))
        yaw = np.zeros(len(self.imu_times))
        
        # Yaw from velocity direction
        for i in range(len(self.imu_times)):
            if np.linalg.norm(imu_velocities[i, :2]) > 0.1:  # Avoid division by zero
                yaw[i] = np.arctan2(imu_velocities[i, 1], imu_velocities[i, 0])
        
        # Roll and pitch from accelerations (simplified)
        for i in range(1, len(self.imu_times)):
            # Banking in turns
            centripetal_acc = np.linalg.norm(imu_accelerations[i, :2])
            roll[i] = np.arctan(centripetal_acc / 9.81) * 0.3  # Banking angle
            
            # Pitch from forward acceleration
            pitch[i] = np.arctan2(-imu_accelerations[i, 2], 9.81) * 0.1
        
        # Add IMU noise
        accel_noise_std = 0.1  # m/s^2
        gyro_noise_std = 0.01  # rad/s
        mag_noise_std = 0.5    # μT
        
        # Accelerometer data (body frame + gravity + noise)
        accel_x = imu_accelerations[:, 0] + np.random.normal(0, accel_noise_std, len(self.imu_times))
        accel_y = imu_accelerations[:, 1] + np.random.normal(0, accel_noise_std, len(self.imu_times))
        accel_z = imu_accelerations[:, 2] + 9.81 + np.random.normal(0, accel_noise_std, len(self.imu_times))
        
        # Gyroscope data (angular rates)
        gyro_x = np.gradient(roll, self.imu_times) + np.random.normal(0, gyro_noise_std, len(self.imu_times))
        gyro_y = np.gradient(pitch, self.imu_times) + np.random.normal(0, gyro_noise_std, len(self.imu_times))
        gyro_z = np.gradient(yaw, self.imu_times) + np.random.normal(0, gyro_noise_std, len(self.imu_times))
        
        # Magnetometer data (pointing roughly north with noise)
        mag_x = np.full(len(self.imu_times), 25.0) + np.random.normal(0, mag_noise_std, len(self.imu_times))
        mag_y = np.full(len(self.imu_times), 0.5) + np.random.normal(0, mag_noise_std, len(self.imu_times))
        mag_z = np.full(len(self.imu_times), -40.0) + np.random.normal(0, mag_noise_std, len(self.imu_times))
        
        imu_data = {
            'timestamps': [ts.isoformat() for ts in self.imu_timestamps],
            'accelerometer_x': accel_x,
            'accelerometer_y': accel_y,
            'accelerometer_z': accel_z,
            'gyroscope_x': gyro_x,
            'gyroscope_y': gyro_y,
            'gyroscope_z': gyro_z,
            'magnetometer_x': mag_x,
            'magnetometer_y': mag_y,
            'magnetometer_z': mag_z,
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
            'temperature': np.random.normal(25.0, 2.0, len(self.imu_times)),  # Celsius
        }
        
        return imu_data
    
    def generate_lidar_point_clouds(self, positions: np.ndarray, 
                                   velocities: np.ndarray) -> List[Dict]:
        """
        Generate synthetic LiDAR point cloud data
        """
        point_clouds = []
        
        # Livox Mid-70 specifications
        fov_horizontal = 70.4  # degrees
        fov_vertical = 77.2    # degrees
        max_range = 90.0       # meters
        points_per_frame = 24000  # approximate
        
        for i, timestamp in enumerate(self.lidar_timestamps):
            # Current sensor position and orientation
            pos = np.interp(self.lidar_times[i], self.gnss_times, positions.T).T
            vel = np.interp(self.lidar_times[i], self.gnss_times, velocities.T).T
            
            # Generate synthetic environment (walls, ground, obstacles)
            points = []
            
            # Ground plane
            for _ in range(points_per_frame // 3):
                angle_h = np.random.uniform(-fov_horizontal/2, fov_horizontal/2) * np.pi/180
                angle_v = np.random.uniform(-20, 5) * np.pi/180  # Ground below horizon
                range_val = np.random.uniform(2, max_range)
                
                # Local coordinates (before motion compensation)
                x_local = range_val * np.cos(angle_v) * np.cos(angle_h)
                y_local = range_val * np.cos(angle_v) * np.sin(angle_h)
                z_local = range_val * np.sin(angle_v)
                
                intensity = np.random.randint(50, 200)
                
                points.append({
                    'x': x_local,
                    'y': y_local,
                    'z': z_local,
                    'intensity': intensity,
                    'timestamp': timestamp.timestamp() * 1000000  # microseconds
                })
            
            # Vertical surfaces (walls/buildings)
            for _ in range(points_per_frame // 3):
                angle_h = np.random.uniform(-fov_horizontal/2, fov_horizontal/2) * np.pi/180
                angle_v = np.random.uniform(-10, 40) * np.pi/180
                range_val = np.random.uniform(5, max_range * 0.8)
                
                x_local = range_val * np.cos(angle_v) * np.cos(angle_h)
                y_local = range_val * np.cos(angle_v) * np.sin(angle_h)
                z_local = range_val * np.sin(angle_v)
                
                intensity = np.random.randint(80, 255)
                
                points.append({
                    'x': x_local,
                    'y': y_local,
                    'z': z_local,
                    'intensity': intensity,
                    'timestamp': timestamp.timestamp() * 1000000
                })
            
            # Random objects/vegetation
            for _ in range(points_per_frame // 3):
                angle_h = np.random.uniform(-fov_horizontal/2, fov_horizontal/2) * np.pi/180
                angle_v = np.random.uniform(-fov_vertical/2, fov_vertical/2) * np.pi/180
                range_val = np.random.uniform(1, max_range)
                
                x_local = range_val * np.cos(angle_v) * np.cos(angle_h)
                y_local = range_val * np.cos(angle_v) * np.sin(angle_h)
                z_local = range_val * np.sin(angle_v)
                
                intensity = np.random.randint(30, 180)
                
                points.append({
                    'x': x_local,
                    'y': y_local,
                    'z': z_local,
                    'intensity': intensity,
                    'timestamp': timestamp.timestamp() * 1000000
                })
            
            frame_data = {
                'frame_index': i,
                'timestamp': timestamp.isoformat(),
                'timestamp_us': int(timestamp.timestamp() * 1000000),
                'sensor_position': pos.tolist(),
                'sensor_velocity': vel.tolist(),
                'num_points': len(points),
                'points': points
            }
            
            point_clouds.append(frame_data)
        
        return point_clouds
    
    def save_data_files(self, gnss_data: Dict, imu_data: Dict, 
                       point_clouds: List[Dict]):
        """
        Save all generated data to various file formats
        """
        print("Saving data files...")
        
        # 1. Save GNSS data
        gnss_df = pd.DataFrame(gnss_data)
        gnss_df.to_csv(self.output_dir / 'rtk_gnss_data.csv', index=False)
        
        with open(self.output_dir / 'rtk_gnss_data.json', 'w') as f:
            json.dump(gnss_data, f, indent=2, default=str)
        
        # 2. Save IMU data
        imu_df = pd.DataFrame(imu_data)
        imu_df.to_csv(self.output_dir / 'imu_data.csv', index=False)
        
        with open(self.output_dir / 'imu_data.json', 'w') as f:
            json.dump(imu_data, f, indent=2, default=str)
        
        # 3. Save point cloud data
        with open(self.output_dir / 'lidar_point_clouds.json', 'w') as f:
            json.dump(point_clouds, f, indent=2, default=str)
        
        # 4. Save individual point cloud frames (simulating LVX format)
        pc_dir = self.output_dir / 'point_cloud_frames'
        pc_dir.mkdir(exist_ok=True)
        
        for frame in point_clouds:
            frame_file = pc_dir / f'frame_{frame["frame_index"]:04d}.json'
            with open(frame_file, 'w') as f:
                json.dump(frame, f, indent=2)
        
        # 5. Save binary LiDAR data (simplified LVX format simulation)
        self.save_binary_lidar_data(point_clouds)
        
        # 6. Generate calibration data
        self.save_calibration_data()
        
        # 7. Generate configuration files
        self.save_config_files()
        
        print(f"Data saved to: {self.output_dir}")
    
    def save_binary_lidar_data(self, point_clouds: List[Dict]):
        """
        Save point cloud data in binary format (simulating LVX structure)
        """
        binary_dir = self.output_dir / 'binary_data'
        binary_dir.mkdir(exist_ok=True)
        
        for frame in point_clouds:
            filename = binary_dir / f'frame_{frame["frame_index"]:04d}.bin'
            
            with open(filename, 'wb') as f:
                # Frame header (simplified)
                f.write(struct.pack('<I', frame['frame_index']))  # Frame ID
                f.write(struct.pack('<Q', frame['timestamp_us']))  # Timestamp
                f.write(struct.pack('<I', frame['num_points']))    # Number of points
                
                # Point data
                for point in frame['points']:
                    f.write(struct.pack('<fff', point['x'], point['y'], point['z']))
                    f.write(struct.pack('<B', point['intensity']))
                    f.write(struct.pack('<Q', int(point['timestamp'])))
    
    def save_calibration_data(self):
        """
        Generate and save sensor calibration data
        """
        calibration = {
            'sensor_setup': {
                'lidar_model': 'Livox Mid-70',
                'gnss_model': 'RTK GNSS Receiver',
                'imu_model': 'High-Precision IMU'
            },
            'extrinsic_calibration': {
                'lidar_to_imu_translation': [0.0, 0.0, 0.15],  # meters
                'lidar_to_imu_rotation': [0.0, 0.0, 0.0],      # roll, pitch, yaw (rad)
                'gnss_to_imu_translation': [0.0, 0.2, 0.05],   # meters
                'lever_arm': [0.0, 0.2, 0.05]                  # GNSS antenna offset
            },
            'intrinsic_calibration': {
                'lidar_fov_horizontal': 70.4,
                'lidar_fov_vertical': 77.2,
                'lidar_range_accuracy': 0.02,    # meters
                'imu_accelerometer_bias': [0.01, -0.005, 0.02],  # m/s^2
                'imu_gyroscope_bias': [0.001, 0.0005, -0.002],   # rad/s
                'gnss_antenna_offset': [0.0, 0.0, 1.8]          # meters
            },
            'timing_calibration': {
                'lidar_time_offset': 0.0,       # seconds
                'imu_time_offset': 0.001,       # seconds  
                'gnss_time_offset': 0.05,       # seconds
                'synchronization_accuracy': 0.001  # seconds
            }
        }
        
        with open(self.output_dir / 'calibration_data.json', 'w') as f:
            json.dump(calibration, f, indent=2)
    
    def save_config_files(self):
        """
        Generate configuration files for motion compensation algorithms
        """
        config = {
            'data_sources': {
                'lidar_data': 'lidar_point_clouds.json',
                'gnss_data': 'rtk_gnss_data.csv',
                'imu_data': 'imu_data.csv',
                'calibration': 'calibration_data.json'
            },
            'processing_parameters': {
                'motion_compensation_method': 'trajectory_interpolation',
                'coordinate_system': 'ECEF',
                'interpolation_method': 'linear',
                'outlier_rejection_threshold': 3.0,
                'temporal_window': 0.1  # seconds
            },
            'output_settings': {
                'corrected_point_clouds': 'output/corrected_point_clouds/',
                'trajectory_file': 'output/vehicle_trajectory.csv',
                'quality_report': 'output/motion_compensation_report.json'
            }
        }
        
        with open(self.output_dir / 'processing_config.json', 'w') as f:
            json.dump(config, f, indent=2)
        
        # Generate README with data descriptions
        readme_content = """
# Livox Mid-70 Motion Compensation Test Data

This directory contains synthetic test data for testing motion compensation algorithms for Livox Mid-70 LiDAR systems.

## Data Files

### Point Cloud Data
- `lidar_point_clouds.json`: Complete point cloud data with timestamps
- `point_cloud_frames/`: Individual frame files
- `binary_data/`: Binary point cloud data (simulating LVX format)

### Navigation Data
- `rtk_gnss_data.csv`: RTK GNSS positioning data (lat/lon/alt)
- `imu_data.csv`: IMU data (accelerometer, gyroscope, magnetometer, orientation)

### Calibration & Configuration
- `calibration_data.json`: Sensor calibration parameters
- `processing_config.json`: Processing configuration

## Data Specifications

### GNSS Data
- Frequency: 10 Hz
- Accuracy: ~1cm (RTK fixed)
- Fields: lat, lon, alt, fix_quality, num_satellites, accuracy

### IMU Data  
- Frequency: 100 Hz
- Fields: accel_xyz, gyro_xyz, mag_xyz, roll, pitch, yaw
- Includes realistic noise and bias

### LiDAR Data
- Frequency: 10 Hz
- Points per frame: ~24,000
- Fields: x, y, z, intensity, timestamp
- Simulates realistic environment (ground, walls, objects)

## Usage
Use this data to test and validate your motion compensation algorithms before processing real Livox Mid-70 data.
"""
        
        with open(self.output_dir / 'README.md', 'w') as f:
            f.write(readme_content)
    
    def generate_visualization(self, positions: np.ndarray, point_clouds: List[Dict]):
        """
        Generate visualization plots of the generated data
        """
        fig, axes = plt.subplots(2, 2, figsize=(15, 12))
        
        # 1. Vehicle trajectory
        axes[0, 0].plot(positions[:, 0], positions[:, 1], 'b-', linewidth=2)
        axes[0, 0].scatter(positions[0, 0], positions[0, 1], c='green', s=100, label='Start')
        axes[0, 0].scatter(positions[-1, 0], positions[-1, 1], c='red', s=100, label='End')
        axes[0, 0].set_xlabel('X (meters)')
        axes[0, 0].set_ylabel('Y (meters)')
        axes[0, 0].set_title('Vehicle Trajectory')
        axes[0, 0].legend()
        axes[0, 0].grid(True)
        axes[0, 0].axis('equal')
        
        # 2. Sample point cloud (first frame)
        if point_clouds:
            first_frame = point_clouds[0]
            points = first_frame['points']
            x_pts = [p['x'] for p in points]
            y_pts = [p['y'] for p in points]
            z_pts = [p['z'] for p in points]
            intensities = [p['intensity'] for p in points]
            
            scatter = axes[0, 1].scatter(x_pts, y_pts, c=intensities, cmap='viridis', s=1, alpha=0.6)
            axes[0, 1].set_xlabel('X (meters)')
            axes[0, 1].set_ylabel('Y (meters)')
            axes[0, 1].set_title('Sample Point Cloud (Frame 0)')
            plt.colorbar(scatter, ax=axes[0, 1], label='Intensity')
        
        # 3. Altitude profile
        axes[1, 0].plot(self.gnss_times, positions[:, 2], 'r-', linewidth=2)
        axes[1, 0].set_xlabel('Time (seconds)')
        axes[1, 0].set_ylabel('Altitude (meters)')
        axes[1, 0].set_title('Altitude Profile')
        axes[1, 0].grid(True)
        
        # 4. Speed profile
        speeds = np.linalg.norm(np.interp(self.gnss_times, self.gnss_times, 
                               np.gradient(positions, axis=0)), axis=1)
        axes[1, 1].plot(self.gnss_times, speeds, 'g-', linewidth=2)
        axes[1, 1].set_xlabel('Time (seconds)')
        axes[1, 1].set_ylabel('Speed (m/s)')
        axes[1, 1].set_title('Speed Profile')
        axes[1, 1].grid(True)
        
        plt.tight_layout()
        plt.savefig(self.output_dir / 'data_visualization.png', dpi=300, bbox_inches='tight')
        plt.close()
    
    def generate_all_data(self):
        """
        Main function to generate all test data
        """
        print("Generating motion trajectory...")
        positions, velocities, accelerations = self.generate_trajectory()
        
        print("Generating RTK GNSS data...")
        gnss_data = self.generate_rtk_gnss_data(positions)
        
        print("Generating IMU data...")
        imu_data = self.generate_imu_data(positions, velocities, accelerations)
        
        print("Generating LiDAR point clouds...")
        point_clouds = self.generate_lidar_point_clouds(positions, velocities)
        
        print("Saving data files...")
        self.save_data_files(gnss_data, imu_data, point_clouds)
        
        print("Generating visualizations...")
        self.generate_visualization(positions, point_clouds)
        
        # Print summary
        print("\n" + "="*60)
        print("DATA GENERATION COMPLETE")
        print("="*60)
        print(f"Duration: {self.duration} seconds")
        print(f"LiDAR frames: {len(point_clouds)}")
        print(f"GNSS samples: {len(gnss_data['timestamps'])}")
        print(f"IMU samples: {len(imu_data['timestamps'])}")
        print(f"Total points: {sum(frame['num_points'] for frame in point_clouds):,}")
        print(f"Output directory: {self.output_dir}")
        print("\nGenerated Files:")
        print("- rtk_gnss_data.csv/json (RTK GNSS positioning)")
        print("- imu_data.csv/json (IMU measurements)")
        print("- lidar_point_clouds.json (Complete LiDAR data)")
        print("- point_cloud_frames/ (Individual frame files)")
        print("- binary_data/ (Binary LiDAR data)")
        print("- calibration_data.json (Sensor calibration)")
        print("- processing_config.json (Processing configuration)")
        print("- data_visualization.png (Data plots)")
        print("- README.md (Documentation)")
        
        return {
            'gnss_data': gnss_data,
            'imu_data': imu_data,
            'point_clouds': point_clouds,
            'positions': positions,
            'velocities': velocities,
            'accelerations': accelerations
        }


def main():
    """
    Main execution function with different trajectory options
    """
    import argparse
    
    parser = argparse.ArgumentParser(description='Generate Livox Mid-70 test data')
    parser.add_argument('--duration', type=float, default=60.0, 
                       help='Duration in seconds (default: 60)')
    parser.add_argument('--trajectory', choices=['straight', 'circular', 'figure_8'], 
                       default='figure_8', help='Trajectory type (default: figure_8)')
    parser.add_argument('--velocity', type=float, default=10.0, 
                       help='Vehicle velocity in m/s (default: 10)')
    parser.add_argument('--output', type=str, default='livox_test_data', 
                       help='Output directory (default: livox_test_data)')
    parser.add_argument('--lidar-freq', type=float, default=10.0, 
                       help='LiDAR frequency in Hz (default: 10)')
    parser.add_argument('--imu-freq', type=float, default=100.0, 
                       help='IMU frequency in Hz (default: 100)')
    parser.add_argument('--gnss-freq', type=float, default=10.0, 
                       help='GNSS frequency in Hz (default: 10)')
    
    args = parser.parse_args()
    
    # Create generator instance
    generator = LivoxTestDataGenerator(output_dir=args.output)
    
    # Set parameters from command line
    generator.duration = args.duration
    generator.trajectory_type = args.trajectory
    generator.velocity = args.velocity
    generator.lidar_frequency = args.lidar_freq
    generator.imu_frequency = args.imu_freq
    generator.gnss_frequency = args.gnss_freq
    
    # Regenerate timestamps with new frequencies
    generator.generate_timestamps()
    
    # Generate all data
    result = generator.generate_all_data()
    
    print(f"\nTest data generation completed successfully!")
    print(f"Use this data to test your motion compensation algorithms.")
    
    return result


class MotionCompensationTester:
    """
    Additional class to test motion compensation algorithms with the generated data
    """
    
    def __init__(self, data_dir: str):
        self.data_dir = Path(data_dir)
        self.load_generated_data()
    
    def load_generated_data(self):
        """Load all generated test data"""
        print("Loading generated test data...")
        
        # Load GNSS data
        self.gnss_data = pd.read_csv(self.data_dir / 'rtk_gnss_data.csv')
        
        # Load IMU data
        self.imu_data = pd.read_csv(self.data_dir / 'imu_data.csv')
        
        # Load point cloud data
        with open(self.data_dir / 'lidar_point_clouds.json', 'r') as f:
            self.point_clouds = json.load(f)
        
        # Load calibration data
        with open(self.data_dir / 'calibration_data.json', 'r') as f:
            self.calibration = json.load(f)
        
        print(f"Loaded {len(self.point_clouds)} point cloud frames")
        print(f"Loaded {len(self.gnss_data)} GNSS samples")
        print(f"Loaded {len(self.imu_data)} IMU samples")
    
    def apply_motion_compensation(self, method='trajectory_interpolation'):
        """
        Apply motion compensation to point clouds using trajectory data
        """
        print(f"Applying motion compensation using method: {method}")
        
        compensated_frames = []
        
        for frame in self.point_clouds:
            frame_time = pd.to_datetime(frame['timestamp'])
            
            # Find corresponding GNSS and IMU data
            gnss_idx = self.find_closest_timestamp(frame_time, self.gnss_data['timestamps'])
            imu_idx = self.find_closest_timestamp(frame_time, self.imu_data['timestamps'])
            
            if gnss_idx is not None and imu_idx is not None:
                # Get pose at frame time
                gnss_row = self.gnss_data.iloc[gnss_idx]
                imu_row = self.imu_data.iloc[imu_idx]
                
                # Apply transformation to each point
                compensated_points = []
                for point in frame['points']:
                    # Transform from sensor frame to world frame
                    compensated_point = self.transform_point(
                        point, gnss_row, imu_row
                    )
                    compensated_points.append(compensated_point)
                
                compensated_frame = frame.copy()
                compensated_frame['points'] = compensated_points
                compensated_frame['compensation_applied'] = True
                compensated_frame['gnss_timestamp'] = gnss_row['timestamps']
                compensated_frame['imu_timestamp'] = imu_row['timestamps']
                
                compensated_frames.append(compensated_frame)
        
        return compensated_frames
    
    def find_closest_timestamp(self, target_time, timestamp_series):
        """Find index of closest timestamp"""
        timestamps = pd.to_datetime(timestamp_series)
        time_diffs = abs(timestamps - target_time)
        closest_idx = time_diffs.idxmin()
        
        # Only return if within reasonable time window (1 second)
        if time_diffs.iloc[closest_idx].total_seconds() < 1.0:
            return closest_idx
        return None
    
    def transform_point(self, point, gnss_row, imu_row):
        """
        Transform point from sensor frame to world coordinate system
        """
        # Extract position and orientation
        lat, lon, alt = gnss_row['latitude'], gnss_row['longitude'], gnss_row['altitude']
        roll, pitch, yaw = imu_row['roll'], imu_row['pitch'], imu_row['yaw']
        
        # Point in sensor frame
        p_sensor = np.array([point['x'], point['y'], point['z']])
        
        # Rotation matrices
        R_roll = np.array([[1, 0, 0],
                          [0, np.cos(roll), -np.sin(roll)],
                          [0, np.sin(roll), np.cos(roll)]])
        
        R_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                           [0, 1, 0],
                           [-np.sin(pitch), 0, np.cos(pitch)]])
        
        R_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                         [np.sin(yaw), np.cos(yaw), 0],
                         [0, 0, 1]])
        
        # Combined rotation matrix
        R = R_yaw @ R_pitch @ R_roll
        
        # Transform point to world frame
        p_world = R @ p_sensor
        
        # Convert lat/lon/alt to local coordinate system (simplified)
        # In real implementation, use proper geodetic transformations
        x_world = p_world[0] + (lon - 13.404954) * 111000 * np.cos(np.radians(lat))
        y_world = p_world[1] + (lat - 52.520008) * 111000
        z_world = p_world[2] + alt - 50.0
        
        return {
            'x': x_world,
            'y': y_world, 
            'z': z_world,
            'intensity': point['intensity'],
            'timestamp': point['timestamp'],
            'original_x': point['x'],
            'original_y': point['y'],
            'original_z': point['z']
        }
    
    def save_compensated_data(self, compensated_frames, output_file='compensated_point_clouds.json'):
        """Save motion compensated point cloud data"""
        output_path = self.data_dir / output_file
        
        with open(output_path, 'w') as f:
            json.dump(compensated_frames, f, indent=2, default=str)
        
        print(f"Compensated data saved to: {output_path}")
        return compensated_frames
    
    def generate_comparison_plots(self, compensated_frames):
        """Generate before/after comparison plots"""
        fig, axes = plt.subplots(2, 2, figsize=(15, 12))
        
        # Original point clouds (first 5 frames overlaid)
        for i in range(min(5, len(self.point_clouds))):
            frame = self.point_clouds[i]
            points = frame['points']
            x_pts = [p['x'] for p in points[::10]]  # Subsample for visualization
            y_pts = [p['y'] for p in points[::10]]
            axes[0, 0].scatter(x_pts, y_pts, s=1, alpha=0.6, label=f'Frame {i}')
        
        axes[0, 0].set_title('Original Point Clouds (Overlapping)')
        axes[0, 0].set_xlabel('X (meters)')
        axes[0, 0].set_ylabel('Y (meters)')
        axes[0, 0].legend()
        axes[0, 0].grid(True)
        
        # Compensated point clouds
        for i in range(min(5, len(compensated_frames))):
            frame = compensated_frames[i]
            points = frame['points']
            x_pts = [p['x'] for p in points[::10]]  # Subsample for visualization
            y_pts = [p['y'] for p in points[::10]]
            axes[0, 1].scatter(x_pts, y_pts, s=1, alpha=0.6, label=f'Frame {i}')
        
        axes[0, 1].set_title('Motion Compensated Point Clouds')
        axes[0, 1].set_xlabel('X (meters)')
        axes[0, 1].set_ylabel('Y (meters)')
        axes[0, 1].legend()
        axes[0, 1].grid(True)
        
        # Single frame comparison (frame 0)
        if compensated_frames:
            orig_points = self.point_clouds[0]['points']
            comp_points = compensated_frames[0]['points']
            
            orig_x = [p['x'] for p in orig_points[::20]]
            orig_y = [p['y'] for p in orig_points[::20]]
            comp_x = [p['x'] for p in comp_points[::20]]
            comp_y = [p['y'] for p in comp_points[::20]]
            
            axes[1, 0].scatter(orig_x, orig_y, s=2, alpha=0.7, c='red', label='Original')
            axes[1, 0].scatter(comp_x, comp_y, s=2, alpha=0.7, c='blue', label='Compensated')
            axes[1, 0].set_title('Single Frame Comparison (Frame 0)')
            axes[1, 0].set_xlabel('X (meters)')
            axes[1, 0].set_ylabel('Y (meters)')
            axes[1, 0].legend()
            axes[1, 0].grid(True)
            
        # Displacement analysis
        if len(compensated_frames) > 1:
            displacements = []
            for i in range(1, min(10, len(compensated_frames))):
                # Calculate center of mass displacement between frames
                frame1_points = compensated_frames[i-1]['points']
                frame2_points = compensated_frames[i]['points']
                
                cm1_x = np.mean([p['x'] for p in frame1_points])
                cm1_y = np.mean([p['y'] for p in frame1_points])
                cm2_x = np.mean([p['x'] for p in frame2_points])
                cm2_y = np.mean([p['y'] for p in frame2_points])
                
                disp = np.sqrt((cm2_x - cm1_x)**2 + (cm2_y - cm1_y)**2)
                displacements.append(disp)
            
            axes[1, 1].plot(range(len(displacements)), displacements, 'g-o', linewidth=2)
            axes[1, 1].set_title('Frame-to-Frame Displacement')
            axes[1, 1].set_xlabel('Frame Index')
            axes[1, 1].set_ylabel('Displacement (meters)')
            axes[1, 1].grid(True)
        
        plt.tight_layout()
        plt.savefig(self.data_dir / 'motion_compensation_comparison.png', 
                   dpi=300, bbox_inches='tight')
        plt.close()
        
        print("Comparison plots saved to: motion_compensation_comparison.png")


def test_motion_compensation(data_dir='livox_test_data'):
    """
    Test function to demonstrate motion compensation on generated data
    """
    print("Testing motion compensation with generated data...")
    
    tester = MotionCompensationTester(data_dir)
    compensated_frames = tester.apply_motion_compensation()
    tester.save_compensated_data(compensated_frames)
    tester.generate_comparison_plots(compensated_frames)
    
    print("Motion compensation testing completed!")
    return compensated_frames


if __name__ == "__main__":
    # Generate test data
    result = main()
    
    # Optionally test motion compensation
    print("\nTesting motion compensation algorithms...")
    test_motion_compensation()
    
    print("\nAll processing completed successfully!")
        
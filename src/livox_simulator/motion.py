"""
Motion compensation algorithms for Livox Mid-70 simulation
"""

import numpy as np
from typing import List, Dict, Any
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation
from .data_structures import IMUData


class MotionCompensator:
    """Applies motion compensation using IMU data to correct for vehicle movement during scanning."""
    
    def __init__(self, imu_config: Dict[str, Any]):
        """
        Initialize motion compensator.
        
        Args:
            imu_config: IMU configuration parameters
        """
        self.imu_config = imu_config
        self.compensation_method = 'linear_interpolation'
        self.statistics = {
            'frames_processed': 0,
            'total_compensation_error': 0.0,
            'max_compensation_error': 0.0
        }
    
    def compensate_frame(self, points: np.ndarray, imu_data: List[IMUData], frame_time: float) -> np.ndarray:
        """
        Apply motion compensation to a single LiDAR frame.
        
        Args:
            points: Raw point cloud data (N×4: x,y,z,intensity)
            imu_data: IMU measurements during frame acquisition
            frame_time: Frame timestamp
            
        Returns:
            Motion-compensated points
        """
        if not imu_data or len(imu_data) < 2:
            return points  # Cannot compensate without sufficient IMU data
        
        # Extract point timestamps (assuming they're in the 4th column or generate them)
        if points.shape[1] > 4:
            point_timestamps = points[:, 4]
        else:
            # Generate timestamps assuming uniform distribution within frame
            frame_duration = 1.0 / self.imu_config.get('frame_rate', 10)  # 10Hz default
            point_timestamps = np.linspace(frame_time, frame_time + frame_duration, len(points))
        
        # Interpolate IMU data for each point timestamp
        compensated_points = []
        
        for i, point in enumerate(points):
            point_time = point_timestamps[i] if i < len(point_timestamps) else frame_time
            
            # Get interpolated IMU data at point timestamp
            imu_sample = self._interpolate_imu_data(imu_data, point_time)
            
            # Apply motion compensation
            compensated_point = self._apply_compensation(point[:3], imu_sample, frame_time, point_time)
            
            # Preserve intensity and other attributes
            if points.shape[1] > 3:
                compensated_point = np.append(compensated_point, point[3:])
            
            compensated_points.append(compensated_point)
        
        compensated_array = np.array(compensated_points)
        
        # Update statistics
        self.statistics['frames_processed'] += 1
        
        return compensated_array
    
    def _interpolate_imu_data(self, imu_data: List[IMUData], target_time: float) -> IMUData:
        """
        Interpolate IMU data to get values at target timestamp.
        
        Args:
            imu_data: List of IMU measurements
            target_time: Target timestamp for interpolation
            
        Returns:
            Interpolated IMU data
        """
        if len(imu_data) == 1:
            return imu_data[0]
        
        # Extract timestamps and values
        timestamps = np.array([imu.timestamp for imu in imu_data])
        gyro_x = np.array([imu.gyro_x for imu in imu_data])
        gyro_y = np.array([imu.gyro_y for imu in imu_data])
        gyro_z = np.array([imu.gyro_z for imu in imu_data])
        accel_x = np.array([imu.accel_x for imu in imu_data])
        accel_y = np.array([imu.accel_y for imu in imu_data])
        accel_z = np.array([imu.accel_z for imu in imu_data])
        
        # Clamp target time to available data range
        target_time = np.clip(target_time, timestamps.min(), timestamps.max())
        
        # Interpolate each component
        if len(timestamps) >= 2:
            interp_gyro_x = interp1d(timestamps, gyro_x, kind='linear', fill_value='extrapolate')(target_time)
            interp_gyro_y = interp1d(timestamps, gyro_y, kind='linear', fill_value='extrapolate')(target_time)
            interp_gyro_z = interp1d(timestamps, gyro_z, kind='linear', fill_value='extrapolate')(target_time)
            interp_accel_x = interp1d(timestamps, accel_x, kind='linear', fill_value='extrapolate')(target_time)
            interp_accel_y = interp1d(timestamps, accel_y, kind='linear', fill_value='extrapolate')(target_time)
            interp_accel_z = interp1d(timestamps, accel_z, kind='linear', fill_value='extrapolate')(target_time)
        else:
            # Use nearest neighbor if only one sample
            idx = np.argmin(np.abs(timestamps - target_time))
            interp_gyro_x = gyro_x[idx]
            interp_gyro_y = gyro_y[idx]
            interp_gyro_z = gyro_z[idx]
            interp_accel_x = accel_x[idx]
            interp_accel_y = accel_y[idx]
            interp_accel_z = accel_z[idx]
        
        return IMUData(
            timestamp=target_time,
            gyro_x=float(interp_gyro_x),
            gyro_y=float(interp_gyro_y),
            gyro_z=float(interp_gyro_z),
            accel_x=float(interp_accel_x),
            accel_y=float(interp_accel_y),
            accel_z=float(interp_accel_z)
        )
    
    def _apply_compensation(self, point: np.ndarray, imu_data: IMUData, frame_start_time: float, point_time: float) -> np.ndarray:
        """
        Apply motion compensation to a single point.
        
        Args:
            point: 3D point coordinates [x, y, z]
            imu_data: IMU data at point timestamp
            frame_start_time: Frame acquisition start time
            point_time: Point acquisition time
            
        Returns:
            Motion-compensated point coordinates
        """
        # Calculate time difference from frame start
        dt = point_time - frame_start_time
        
        if abs(dt) < 1e-6:  # No compensation needed for frame start
            return point
        
        # Calculate rotation during scan time
        angular_velocity = np.array([imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z])
        rotation_angle = angular_velocity * dt
        
        # Create rotation matrix (small angle approximation for efficiency)
        if np.linalg.norm(rotation_angle) < 0.1:  # Small angle approximation
            rotation_matrix = self._small_angle_rotation_matrix(rotation_angle)
        else:
            rotation_matrix = Rotation.from_rotvec(rotation_angle).as_matrix()
        
        # Apply rotation compensation (inverse rotation to correct for motion)
        compensated_point = rotation_matrix.T @ point
        
        # Optional: Apply translation compensation based on acceleration
        if self.compensation_method == 'full_compensation':
            acceleration = np.array([imu_data.accel_x, imu_data.accel_y, imu_data.accel_z])
            # Remove gravity (assuming Z is up)
            acceleration[2] -= 9.81
            
            # Calculate displacement during scan time
            displacement = 0.5 * acceleration * dt**2
            compensated_point -= displacement  # Subtract to compensate
        
        return compensated_point
    
    def _small_angle_rotation_matrix(self, angles: np.ndarray) -> np.ndarray:
        """
        Create rotation matrix using small angle approximation.
        
        Args:
            angles: [rx, ry, rz] rotation angles in radians
            
        Returns:
            3x3 rotation matrix
        """
        rx, ry, rz = angles
        
        # Small angle approximation: sin(θ) ≈ θ, cos(θ) ≈ 1
        R = np.array([
            [1,   -rz,  ry],
            [rz,   1,  -rx],
            [-ry, rx,   1]
        ])
        
        return R
    
    def set_compensation_method(self, method: str):
        """
        Set motion compensation method.
        
        Args:
            method: Compensation method ('linear_interpolation', 'full_compensation')
        """
        valid_methods = ['linear_interpolation', 'full_compensation', 'rotation_only']
        if method in valid_methods:
            self.compensation_method = method
        else:
            raise ValueError(f"Invalid compensation method. Choose from: {valid_methods}")
    
    def get_compensation_statistics(self) -> Dict[str, Any]:
        """
        Get motion compensation statistics.
        
        Returns:
            Dictionary containing compensation statistics
        """
        stats = self.statistics.copy()
        if stats['frames_processed'] > 0:
            stats['average_compensation_error'] = stats['total_compensation_error'] / stats['frames_processed']
        else:
            stats['average_compensation_error'] = 0.0
        
        return stats
    
    def reset_statistics(self):
        """Reset compensation statistics."""
        self.statistics = {
            'frames_processed': 0,
            'total_compensation_error': 0.0,
            'max_compensation_error': 0.0
        }
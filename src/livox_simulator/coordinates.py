"""
Coordinate system transformations for Livox Mid-70 simulation
"""

import numpy as np
import utm
from typing import Dict, Tuple, Optional
from scipy.spatial.transform import Rotation


class CoordinateTransformer:
    """Handles coordinate system transformations between sensor, vehicle, local, and global frames."""
    
    def __init__(self, calibration_params: Dict):
        """
        Initialize coordinate transformer with calibration parameters.
        
        Args:
            calibration_params: Dictionary containing transformation parameters
        """
        self.calibration_params = calibration_params
        self.transformations = {}
        self._setup_transformations()
    
    def _setup_transformations(self):
        """Initialize transformation matrices from calibration parameters."""
        # LiDAR to Vehicle transformation
        if 'lidar_to_vehicle' in self.calibration_params:
            params = self.calibration_params['lidar_to_vehicle']
            self.transformations['lidar_to_vehicle'] = self._create_transform_matrix(
                params.get('translation', [0, 0, 0]),
                params.get('rotation', [0, 0, 0])
            )
        
        # Vehicle to GNSS transformation
        if 'vehicle_to_gnss' in self.calibration_params:
            params = self.calibration_params['vehicle_to_gnss']
            self.transformations['vehicle_to_gnss'] = self._create_transform_matrix(
                params.get('translation', [0, 0, 0]),
                params.get('rotation', [0, 0, 0])
            )
    
    def _create_transform_matrix(self, translation: list, rotation: list) -> np.ndarray:
        """
        Create 4x4 transformation matrix from translation and rotation.
        
        Args:
            translation: [x, y, z] translation in meters
            rotation: [roll, pitch, yaw] rotation in radians
            
        Returns:
            4x4 transformation matrix
        """
        T = np.eye(4)
        T[:3, 3] = translation
        T[:3, :3] = self.euler_to_rotation_matrix(*rotation)
        return T
    
    def transform_points(self, points: np.ndarray, source: str, target: str) -> np.ndarray:
        """
        Transform point cloud between coordinate systems.
        
        Args:
            points: Input points (N×3 or N×4)
            source: Source coordinate system ('sensor', 'vehicle', 'local', 'global')
            target: Target coordinate system
            
        Returns:
            Transformed points
        """
        if source == target:
            return points.copy()
        
        # Ensure points are in homogeneous coordinates
        if points.shape[1] == 3:
            points_homo = np.hstack([points, np.ones((points.shape[0], 1))])
        else:
            points_homo = points.copy()
        
        # Apply transformation chain
        transformed_points = self._apply_transformation_chain(points_homo, source, target)
        
        # Return original format
        if points.shape[1] == 3:
            return transformed_points[:, :3]
        else:
            return transformed_points
    
    def _apply_transformation_chain(self, points: np.ndarray, source: str, target: str) -> np.ndarray:
        """Apply sequence of transformations to get from source to target frame."""
        current_points = points.copy()
        
        # Define transformation chain
        if source == 'sensor' and target == 'vehicle':
            if 'lidar_to_vehicle' in self.transformations:
                current_points = self._apply_transform(current_points, self.transformations['lidar_to_vehicle'])
        
        elif source == 'sensor' and target == 'global':
            # Chain: sensor -> vehicle -> global
            if 'lidar_to_vehicle' in self.transformations:
                current_points = self._apply_transform(current_points, self.transformations['lidar_to_vehicle'])
            # Apply global transformation (would need GNSS pose)
            
        elif source == 'vehicle' and target == 'global':
            # Apply global transformation based on GNSS/INS data
            pass
        
        return current_points
    
    def _apply_transform(self, points: np.ndarray, transform_matrix: np.ndarray) -> np.ndarray:
        """Apply transformation matrix to points."""
        return (transform_matrix @ points.T).T
    
    def set_transformation(self, source: str, target: str, translation: list, rotation: list):
        """
        Set transformation parameters between coordinate frames.
        
        Args:
            source: Source coordinate frame
            target: Target coordinate frame
            translation: [x, y, z] translation in meters
            rotation: [roll, pitch, yaw] rotation in radians
        """
        key = f"{source}_to_{target}"
        self.transformations[key] = self._create_transform_matrix(translation, rotation)
    
    def get_transformation_matrix(self, source: str, target: str) -> np.ndarray:
        """
        Get transformation matrix between coordinate frames.
        
        Args:
            source: Source coordinate frame
            target: Target coordinate frame
            
        Returns:
            4x4 transformation matrix
        """
        key = f"{source}_to_{target}"
        return self.transformations.get(key, np.eye(4))
    
    @staticmethod
    def euler_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
        """
        Convert Euler angles to rotation matrix.
        
        Args:
            roll: Roll angle in radians
            pitch: Pitch angle in radians
            yaw: Yaw angle in radians
            
        Returns:
            3x3 rotation matrix
        """
        return Rotation.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
    
    @staticmethod
    def rotation_matrix_to_euler(R: np.ndarray) -> Tuple[float, float, float]:
        """
        Convert rotation matrix to Euler angles.
        
        Args:
            R: 3x3 rotation matrix
            
        Returns:
            (roll, pitch, yaw) in radians
        """
        return Rotation.from_matrix(R).as_euler('xyz')
    
    @staticmethod
    def wgs84_to_utm(latitude: float, longitude: float, altitude: float) -> Tuple[float, float, float, int, str]:
        """
        Convert WGS84 coordinates to UTM.
        
        Args:
            latitude: Latitude in degrees
            longitude: Longitude in degrees
            altitude: Altitude in meters
            
        Returns:
            (easting, northing, altitude, zone_number, zone_letter)
        """
        easting, northing, zone_number, zone_letter = utm.from_latlon(latitude, longitude)
        return easting, northing, altitude, zone_number, zone_letter
    
    @staticmethod
    def utm_to_wgs84(easting: float, northing: float, zone_number: int, zone_letter: str) -> Tuple[float, float]:
        """
        Convert UTM coordinates to WGS84.
        
        Args:
            easting: UTM easting
            northing: UTM northing
            zone_number: UTM zone number
            zone_letter: UTM zone letter
            
        Returns:
            (latitude, longitude) in degrees
        """
        return utm.to_latlon(easting, northing, zone_number, zone_letter)
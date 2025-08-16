"""
Data export functionality for Livox Mid-70 simulation
"""

import os
import json
import struct
import numpy as np
from typing import Dict, List, Any, Optional
import open3d as o3d
try:
    import laspy
    LASPY_AVAILABLE = True
except ImportError:
    LASPY_AVAILABLE = False

from .data_structures import LiDARFrame


class DataExporter:
    """Handles export of simulation data to various formats."""
    
    def __init__(self, output_directory: str):
        """
        Initialize data exporter.
        
        Args:
            output_directory: Base directory for output files
        """
        self.output_directory = output_directory
        self.supported_formats = ['pcd', 'ply', 'xyz', 'lvx2']
        if LASPY_AVAILABLE:
            self.supported_formats.append('las')
        
        # Create output directory structure
        self._create_directory_structure()
    
    def _create_directory_structure(self):
        """Create output directory structure."""
        directories = [
            self.output_directory,
            os.path.join(self.output_directory, 'raw_data'),
            os.path.join(self.output_directory, 'processed_data'),
            os.path.join(self.output_directory, 'analysis'),
            os.path.join(self.output_directory, 'configuration')
        ]
        
        for directory in directories:
            os.makedirs(directory, exist_ok=True)
    
    def export_point_cloud(self, points: np.ndarray, format: str, filename: str, metadata: Optional[Dict] = None):
        """
        Export point cloud data in specified format.
        
        Args:
            points: Point cloud data (N×3 or N×4: x,y,z[,intensity])
            format: Output format ('pcd', 'las', 'ply', 'xyz', 'lvx2')
            filename: Output filename
            metadata: Optional metadata dictionary
        """
        if format not in self.supported_formats:
            raise ValueError(f"Unsupported format: {format}. Supported: {self.supported_formats}")
        
        filepath = os.path.join(self.output_directory, 'processed_data', filename)
        
        if format == 'pcd':
            self._export_pcd(points, filepath, metadata)
        elif format == 'las' and LASPY_AVAILABLE:
            self._export_las(points, filepath, metadata)
        elif format == 'ply':
            self._export_ply(points, filepath, metadata)
        elif format == 'xyz':
            self._export_xyz(points, filepath)
        elif format == 'lvx2':
            self._export_lvx2(points, filepath, metadata)
        else:
            raise NotImplementedError(f"Export for format {format} not implemented")
    
    def _export_pcd(self, points: np.ndarray, filepath: str, metadata: Optional[Dict] = None):
        """Export to PCD format."""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        
        # Add colors based on intensity if available
        if points.shape[1] >= 4:
            intensity = points[:, 3]
            # Normalize intensity to [0, 1] and create grayscale colors
            normalized_intensity = (intensity - intensity.min()) / (intensity.max() - intensity.min() + 1e-8)
            colors = np.column_stack([normalized_intensity] * 3)
            pcd.colors = o3d.utility.Vector3dVector(colors)
        
        # Write PCD file
        o3d.io.write_point_cloud(filepath, pcd, write_ascii=False)
    
    def _export_las(self, points: np.ndarray, filepath: str, metadata: Optional[Dict] = None):
        """Export to LAS format."""
        if not LASPY_AVAILABLE:
            raise ImportError("laspy package required for LAS export")
        
        # Create LAS header
        header = laspy.LasHeader(point_format=3, version="1.4")
        
        # Set coordinate system if provided in metadata
        if metadata and 'utm_zone' in metadata:
            # Set UTM coordinate reference system
            utm_zone = metadata['utm_zone']
            hemisphere = metadata.get('hemisphere', 'N')
            epsg_code = 32600 + utm_zone if hemisphere == 'N' else 32700 + utm_zone
            header.add_crs(f"EPSG:{epsg_code}")
        
        # Create LAS file
        with laspy.open(filepath, mode="w", header=header) as las_file:
            las_file.x = points[:, 0]
            las_file.y = points[:, 1]
            las_file.z = points[:, 2]
            
            if points.shape[1] >= 4:
                las_file.intensity = points[:, 3].astype(np.uint16)
            
            # Add metadata to header
            las_file.header.system_identifier = "Livox Mid-70 Simulator"
            las_file.header.generating_software = "Python Livox Simulator v1.0"
    
    def _export_ply(self, points: np.ndarray, filepath: str, metadata: Optional[Dict] = None):
        """Export to PLY format."""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        
        if points.shape[1] >= 4:
            intensity = points[:, 3]
            normalized_intensity = (intensity - intensity.min()) / (intensity.max() - intensity.min() + 1e-8)
            colors = np.column_stack([normalized_intensity] * 3)
            pcd.colors = o3d.utility.Vector3dVector(colors)
        
        o3d.io.write_point_cloud(filepath, pcd, write_ascii=False)
    
    def _export_xyz(self, points: np.ndarray, filepath: str):
        """Export to simple XYZ format."""
        if points.shape[1] >= 4:
            # Include intensity
            np.savetxt(filepath, points[:, :4], fmt='%.6f %.6f %.6f %.0f', 
                      header='X Y Z Intensity', comments='# ')
        else:
            np.savetxt(filepath, points[:, :3], fmt='%.6f %.6f %.6f',
                      header='X Y Z', comments='# ')
    
    def _export_lvx2(self, points: np.ndarray, filepath: str, metadata: Optional[Dict] = None):
        """Export to LVX2 format (simplified implementation)."""
        # This is a simplified LVX2 writer - full implementation would require
        # detailed knowledge of the Livox binary format specification
        
        with open(filepath, 'wb') as f:
            # Write LVX2 header
            f.write(b'livox_tech')  # Signature
            f.write(struct.pack('<I', 2))  # Version (LVX2)
            f.write(struct.pack('<I', 0xAC0EA767))  # Magic code
            
            # Write device information
            device_count = 1
            f.write(struct.pack('<I', device_count))
            
            # Device info for Mid-70
            device_type = 1  # Mid-70 type
            f.write(struct.pack('<B', device_type))
            
            # Write point data
            point_count = len(points)
            f.write(struct.pack('<I', point_count))
            
            for point in points:
                # Convert to millimeters and write as integers
                x_mm = int(point[0] * 1000)
                y_mm = int(point[1] * 1000)
                z_mm = int(point[2] * 1000)
                intensity = int(point[3]) if points.shape[1] >= 4 else 128
                
                f.write(struct.pack('<iii', x_mm, y_mm, z_mm))
                f.write(struct.pack('<B', intensity))
    
    def export_trajectory(self, trajectory: List[Dict], format: str, filename: str):
        """
        Export trajectory data.
        
        Args:
            trajectory: List of trajectory points
            format: Output format ('csv', 'json', 'kml')
            filename: Output filename
        """
        filepath = os.path.join(self.output_directory, 'raw_data', filename)
        
        if format == 'csv':
            self._export_trajectory_csv(trajectory, filepath)
        elif format == 'json':
            self._export_trajectory_json(trajectory, filepath)
        elif format == 'kml':
            self._export_trajectory_kml(trajectory, filepath)
        else:
            raise ValueError(f"Unsupported trajectory format: {format}")
    
    def _export_trajectory_csv(self, trajectory: List[Dict], filepath: str):
        """Export trajectory to CSV format."""
        import pandas as pd
        
        df = pd.DataFrame(trajectory)
        df.to_csv(filepath, index=False)
    
    def _export_trajectory_json(self, trajectory: List[Dict], filepath: str):
        """Export trajectory to JSON format."""
        with open(filepath, 'w') as f:
            json.dump(trajectory, f, indent=2, default=str)
    
    def _export_trajectory_kml(self, trajectory: List[Dict], filepath: str):
        """Export trajectory to KML format for Google Earth."""
        kml_content = '''<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>Vehicle Trajectory</name>
    <Placemark>
      <name>Path</name>
      <LineString>
        <coordinates>
'''
        
        for point in trajectory:
            if 'gps_lon' in point and 'gps_lat' in point:
                lon = point['gps_lon']
                lat = point['gps_lat']
                alt = point.get('gps_alt', 0)
                kml_content += f"          {lon},{lat},{alt}\n"
        
        kml_content += '''        </coordinates>
      </LineString>
    </Placemark>
  </Document>
</kml>'''
        
        with open(filepath, 'w') as f:
            f.write(kml_content)
    
    def export_all_formats(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Export data in all supported formats.
        
        Args:
            data: Dictionary containing simulation data
            
        Returns:
            Dictionary with export results
        """
        export_results = {}
        
        # Export point cloud data
        if 'point_cloud' in data:
            points = data['point_cloud']
            metadata = data.get('metadata', {})
            
            for format in self.supported_formats:
                try:
                    filename = f"point_cloud.{format}"
                    self.export_point_cloud(points, format, filename, metadata)
                    export_results[format] = {
                        'status': 'success',
                        'filename': filename,
                        'points': len(points)
                    }
                except Exception as e:
                    export_results[format] = {
                        'status': 'failed',
                        'error': str(e)
                    }
        
        # Export trajectory data
        if 'trajectory' in data:
            trajectory = data['trajectory']
            
            for format in ['csv', 'json', 'kml']:
                try:
                    filename = f"trajectory.{format}"
                    self.export_trajectory(trajectory, format, filename)
                    export_results[f"trajectory_{format}"] = {
                        'status': 'success',
                        'filename': filename
                    }
                except Exception as e:
                    export_results[f"trajectory_{format}"] = {
                        'status': 'failed',
                        'error': str(e)
                    }
        
        # Export configuration
        if 'config' in data:
            config_path = os.path.join(self.output_directory, 'configuration', 'simulation_config.json')
            with open(config_path, 'w') as f:
                json.dump(data['config'], f, indent=2, default=str)
            export_results['config'] = {'status': 'success', 'filename': 'simulation_config.json'}
        
        # Export performance statistics
        if 'performance_stats' in data:
            stats_path = os.path.join(self.output_directory, 'analysis', 'performance_stats.json')
            with open(stats_path, 'w') as f:
                json.dump(data['performance_stats'], f, indent=2, default=str)
            export_results['performance_stats'] = {'status': 'success', 'filename': 'performance_stats.json'}
        
        return export_results
"""
Comprehensive 3D Data Visualization Module for Livox Mid-70 Simulation Framework

This module provides robust visualization capabilities for various 3D data formats
commonly used in LiDAR and point cloud processing applications.
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.colors as mcolors
from typing import Dict, List, Optional, Tuple, Union, Any
import warnings

# Optional imports with fallback handling
try:
    import open3d as o3d
    OPEN3D_AVAILABLE = True
except ImportError:
    OPEN3D_AVAILABLE = False
    warnings.warn("Open3D not available. Some visualization features will be limited.")

try:
    import laspy
    LASPY_AVAILABLE = True
except ImportError:
    LASPY_AVAILABLE = False
    warnings.warn("Laspy not available. LAS file visualization will be disabled.")

try:
    import trimesh
    TRIMESH_AVAILABLE = True
except ImportError:
    TRIMESH_AVAILABLE = False
    warnings.warn("Trimesh not available. Mesh visualization will be limited.")

try:
    import plotly.graph_objects as go
    import plotly.express as px
    from plotly.subplots import make_subplots
    PLOTLY_AVAILABLE = True
except ImportError:
    PLOTLY_AVAILABLE = False
    warnings.warn("Plotly not available. Interactive visualization will be disabled.")

try:
    import pandas as pd
    PANDAS_AVAILABLE = True
except ImportError:
    PANDAS_AVAILABLE = False


class DataVisualizer3D:
    """
    Comprehensive 3D data visualization class supporting multiple formats and rendering backends.
    """
    
    def __init__(self, backend: str = 'auto'):
        """
        Initialize the 3D data visualizer.
        
        Args:
            backend: Visualization backend ('matplotlib', 'open3d', 'plotly', 'auto')
        """
        self.backend = self._select_backend(backend)
        self.supported_formats = self._get_supported_formats()
        self.color_schemes = {
            'height': 'viridis',
            'intensity': 'plasma',
            'distance': 'coolwarm',
            'classification': 'tab10',
            'default': 'viridis'
        }
        
    def _select_backend(self, backend: str) -> str:
        """Select the best available backend."""
        if backend == 'auto':
            if OPEN3D_AVAILABLE:
                return 'open3d'
            elif PLOTLY_AVAILABLE:
                return 'plotly'
            else:
                return 'matplotlib'
        
        # Validate requested backend
        if backend == 'open3d' and not OPEN3D_AVAILABLE:
            warnings.warn("Open3D not available, falling back to matplotlib")
            return 'matplotlib'
        elif backend == 'plotly' and not PLOTLY_AVAILABLE:
            warnings.warn("Plotly not available, falling back to matplotlib")
            return 'matplotlib'
        
        return backend
    
    def _get_supported_formats(self) -> List[str]:
        """Get list of supported file formats."""
        formats = ['pcd', 'ply', 'xyz', 'csv', 'npy', 'npz']
        
        if LASPY_AVAILABLE:
            formats.extend(['las', 'laz'])
        
        if TRIMESH_AVAILABLE:
            formats.extend(['obj', 'stl', 'off'])
        
        return formats
    
    def load_data(self, filepath: str, **kwargs) -> Dict[str, Any]:
        """
        Load 3D data from various file formats.
        
        Args:
            filepath: Path to the data file
            **kwargs: Additional parameters for specific loaders
            
        Returns:
            Dictionary containing loaded data and metadata
        """
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"File not found: {filepath}")
        
        file_ext = os.path.splitext(filepath)[1].lower().lstrip('.')
        
        if file_ext not in self.supported_formats:
            raise ValueError(f"Unsupported format: {file_ext}. Supported: {self.supported_formats}")
        
        # Load data based on file extension
        if file_ext == 'pcd':
            return self._load_pcd(filepath, **kwargs)
        elif file_ext == 'ply':
            return self._load_ply(filepath, **kwargs)
        elif file_ext in ['las', 'laz']:
            return self._load_las(filepath, **kwargs)
        elif file_ext == 'xyz':
            return self._load_xyz(filepath, **kwargs)
        elif file_ext == 'csv':
            return self._load_csv(filepath, **kwargs)
        elif file_ext in ['npy', 'npz']:
            return self._load_numpy(filepath, **kwargs)
        elif file_ext in ['obj', 'stl', 'off']:
            return self._load_mesh(filepath, **kwargs)
        else:
            raise NotImplementedError(f"Loader for {file_ext} not implemented")
    
    def _load_pcd(self, filepath: str, **kwargs) -> Dict[str, Any]:
        """Load PCD point cloud file."""
        if OPEN3D_AVAILABLE:
            pcd = o3d.io.read_point_cloud(filepath)
            points = np.asarray(pcd.points)
            colors = np.asarray(pcd.colors) if pcd.has_colors() else None
            normals = np.asarray(pcd.normals) if pcd.has_normals() else None
            
            return {
                'points': points,
                'colors': colors,
                'normals': normals,
                'format': 'pcd',
                'metadata': {'num_points': len(points)}
            }
        else:
            # Fallback: simple PCD parser
            return self._parse_pcd_ascii(filepath)
    
    def _load_ply(self, filepath: str, **kwargs) -> Dict[str, Any]:
        """Load PLY point cloud file."""
        if OPEN3D_AVAILABLE:
            pcd = o3d.io.read_point_cloud(filepath)
            points = np.asarray(pcd.points)
            colors = np.asarray(pcd.colors) if pcd.has_colors() else None
            
            return {
                'points': points,
                'colors': colors,
                'format': 'ply',
                'metadata': {'num_points': len(points)}
            }
        else:
            raise NotImplementedError("PLY loading requires Open3D")
    
    def _load_las(self, filepath: str, **kwargs) -> Dict[str, Any]:
        """Load LAS/LAZ point cloud file."""
        if not LASPY_AVAILABLE:
            raise ImportError("LAS loading requires laspy package")
        
        las_file = laspy.read(filepath)
        points = np.column_stack([las_file.x, las_file.y, las_file.z])
        
        # Extract additional attributes
        intensity = las_file.intensity if hasattr(las_file, 'intensity') else None
        classification = las_file.classification if hasattr(las_file, 'classification') else None
        return_number = las_file.return_number if hasattr(las_file, 'return_number') else None
        
        return {
            'points': points,
            'intensity': intensity,
            'classification': classification,
            'return_number': return_number,
            'format': 'las',
            'metadata': {
                'num_points': len(points),
                'header': las_file.header,
                'crs': getattr(las_file.header, 'crs', None)
            }
        }
    
    def _load_xyz(self, filepath: str, **kwargs) -> Dict[str, Any]:
        """Load XYZ ASCII point cloud file."""
        delimiter = kwargs.get('delimiter', None)
        
        try:
            data = np.loadtxt(filepath, delimiter=delimiter)
        except ValueError:
            # Try with different delimiters
            for delim in [' ', '\t', ',', ';']:
                try:
                    data = np.loadtxt(filepath, delimiter=delim)
                    break
                except ValueError:
                    continue
            else:
                raise ValueError("Could not parse XYZ file with any common delimiter")
        
        if data.shape[1] < 3:
            raise ValueError("XYZ file must have at least 3 columns (x, y, z)")
        
        points = data[:, :3]
        intensity = data[:, 3] if data.shape[1] > 3 else None
        
        return {
            'points': points,
            'intensity': intensity,
            'format': 'xyz',
            'metadata': {'num_points': len(points)}
        }
    
    def _load_csv(self, filepath: str, **kwargs) -> Dict[str, Any]:
        """Load CSV point cloud file."""
        if not PANDAS_AVAILABLE:
            # Fallback to numpy
            return self._load_xyz(filepath, delimiter=',')
        
        df = pd.read_csv(filepath)
        
        # Try to identify coordinate columns
        coord_cols = []
        for col_set in [['x', 'y', 'z'], ['X', 'Y', 'Z'], ['lon', 'lat', 'alt']]:
            if all(col in df.columns for col in col_set):
                coord_cols = col_set
                break
        
        if not coord_cols:
            # Use first 3 numeric columns
            numeric_cols = df.select_dtypes(include=[np.number]).columns
            if len(numeric_cols) >= 3:
                coord_cols = numeric_cols[:3].tolist()
            else:
                raise ValueError("Could not identify coordinate columns in CSV")
        
        points = df[coord_cols].values
        
        # Extract additional attributes
        other_cols = [col for col in df.columns if col not in coord_cols]
        attributes = {col: df[col].values for col in other_cols}
        
        return {
            'points': points,
            'attributes': attributes,
            'format': 'csv',
            'metadata': {'num_points': len(points), 'columns': df.columns.tolist()}
        }
    
    def _load_numpy(self, filepath: str, **kwargs) -> Dict[str, Any]:
        """Load NumPy array file."""
        if filepath.endswith('.npz'):
            data = np.load(filepath)
            # Assume 'points' key exists, or use first array
            if 'points' in data:
                points = data['points']
            else:
                points = data[list(data.keys())[0]]
            
            attributes = {key: data[key] for key in data.keys() if key != 'points'}
        else:
            points = np.load(filepath)
            attributes = {}
        
        if points.shape[1] < 3:
            raise ValueError("Point array must have at least 3 columns")
        
        return {
            'points': points[:, :3],
            'attributes': attributes,
            'format': 'numpy',
            'metadata': {'num_points': len(points)}
        }
    
    def _load_mesh(self, filepath: str, **kwargs) -> Dict[str, Any]:
        """Load mesh file (OBJ, STL, OFF)."""
        if not TRIMESH_AVAILABLE:
            raise ImportError("Mesh loading requires trimesh package")
        
        mesh = trimesh.load(filepath)
        
        return {
            'vertices': mesh.vertices,
            'faces': mesh.faces,
            'format': 'mesh',
            'metadata': {
                'num_vertices': len(mesh.vertices),
                'num_faces': len(mesh.faces),
                'is_watertight': mesh.is_watertight,
                'volume': mesh.volume if mesh.is_volume else None
            }
        }
    
    def _parse_pcd_ascii(self, filepath: str) -> Dict[str, Any]:
        """Simple ASCII PCD parser fallback."""
        with open(filepath, 'r') as f:
            lines = f.readlines()
        
        # Parse header
        data_start = 0
        num_points = 0
        fields = []
        
        for i, line in enumerate(lines):
            line = line.strip()
            if line.startswith('FIELDS'):
                fields = line.split()[1:]
            elif line.startswith('POINTS'):
                num_points = int(line.split()[1])
            elif line.startswith('DATA'):
                data_start = i + 1
                break
        
        # Parse data
        data_lines = lines[data_start:data_start + num_points]
        data = []
        for line in data_lines:
            values = [float(x) for x in line.strip().split()]
            data.append(values)
        
        data = np.array(data)
        points = data[:, :3]
        
        return {
            'points': points,
            'format': 'pcd',
            'metadata': {'num_points': len(points), 'fields': fields}
        }
    
    def visualize(self, data: Union[str, Dict[str, Any]], **kwargs) -> None:
        """
        Visualize 3D data using the selected backend.
        
        Args:
            data: File path or loaded data dictionary
            **kwargs: Visualization parameters
        """
        # Load data if filepath provided
        if isinstance(data, str):
            data = self.load_data(data)
        
        # Select visualization method based on backend
        if self.backend == 'open3d':
            self._visualize_open3d(data, **kwargs)
        elif self.backend == 'plotly':
            self._visualize_plotly(data, **kwargs)
        else:
            self._visualize_matplotlib(data, **kwargs)
    
    def _visualize_open3d(self, data: Dict[str, Any], **kwargs) -> None:
        """Visualize using Open3D."""
        if not OPEN3D_AVAILABLE:
            raise ImportError("Open3D visualization requires open3d package")
        
        if 'points' in data:
            # Point cloud visualization
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(data['points'])
            
            # Add colors if available
            if 'colors' in data and data['colors'] is not None:
                pcd.colors = o3d.utility.Vector3dVector(data['colors'])
            elif 'intensity' in data and data['intensity'] is not None:
                colors = self._intensity_to_colors(data['intensity'])
                pcd.colors = o3d.utility.Vector3dVector(colors)
            else:
                # Color by height (Z coordinate)
                colors = self._height_to_colors(data['points'][:, 2])
                pcd.colors = o3d.utility.Vector3dVector(colors)
            
            # Add normals if available
            if 'normals' in data and data['normals'] is not None:
                pcd.normals = o3d.utility.Vector3dVector(data['normals'])
            
            # Filter kwargs for Open3D compatibility
            o3d_kwargs = {}
            if 'window_name' in kwargs:
                o3d_kwargs['window_name'] = kwargs['window_name']
            if 'width' in kwargs:
                o3d_kwargs['width'] = kwargs['width']
            if 'height' in kwargs:
                o3d_kwargs['height'] = kwargs['height']
            
            # Visualize
            o3d.visualization.draw_geometries([pcd], **o3d_kwargs)
        
        elif 'vertices' in data and 'faces' in data:
            # Mesh visualization
            mesh = o3d.geometry.TriangleMesh()
            mesh.vertices = o3d.utility.Vector3dVector(data['vertices'])
            mesh.triangles = o3d.utility.Vector3iVector(data['faces'])
            mesh.compute_vertex_normals()
            
            # Filter kwargs for Open3D compatibility
            o3d_kwargs = {}
            if 'window_name' in kwargs:
                o3d_kwargs['window_name'] = kwargs['window_name']
            if 'width' in kwargs:
                o3d_kwargs['width'] = kwargs['width']
            if 'height' in kwargs:
                o3d_kwargs['height'] = kwargs['height']
            
            o3d.visualization.draw_geometries([mesh], **o3d_kwargs)
    
    def _visualize_plotly(self, data: Dict[str, Any], **kwargs) -> None:
        """Visualize using Plotly."""
        if not PLOTLY_AVAILABLE:
            raise ImportError("Plotly visualization requires plotly package")
        
        if 'points' in data:
            points = data['points']
            
            # Determine colors
            if 'colors' in data and data['colors'] is not None:
                colors = data['colors']
                if colors.shape[1] == 3:  # RGB colors
                    color_values = ['rgb({},{},{})'.format(int(r*255), int(g*255), int(b*255)) 
                                   for r, g, b in colors]
                else:
                    color_values = colors[:, 0]  # Use first channel
            elif 'intensity' in data and data['intensity'] is not None:
                color_values = data['intensity']
            else:
                color_values = points[:, 2]  # Color by height
            
            # Create 3D scatter plot
            fig = go.Figure(data=[go.Scatter3d(
                x=points[:, 0],
                y=points[:, 1],
                z=points[:, 2],
                mode='markers',
                marker=dict(
                    size=kwargs.get('point_size', 2),
                    color=color_values,
                    colorscale=kwargs.get('colorscale', 'Viridis'),
                    opacity=kwargs.get('opacity', 0.8),
                    showscale=True
                ),
                text=[f'Point {i}' for i in range(len(points))],
                hovertemplate='X: %{x}<br>Y: %{y}<br>Z: %{z}<extra></extra>'
            )])
            
            # Update layout
            fig.update_layout(
                title=kwargs.get('title', '3D Point Cloud Visualization'),
                scene=dict(
                    xaxis_title='X',
                    yaxis_title='Y',
                    zaxis_title='Z',
                    aspectmode='data'
                ),
                width=kwargs.get('width', 800),
                height=kwargs.get('height', 600)
            )
            
            fig.show()
        
        elif 'vertices' in data and 'faces' in data:
            # Mesh visualization
            vertices = data['vertices']
            faces = data['faces']
            
            fig = go.Figure(data=[go.Mesh3d(
                x=vertices[:, 0],
                y=vertices[:, 1],
                z=vertices[:, 2],
                i=faces[:, 0],
                j=faces[:, 1],
                k=faces[:, 2],
                opacity=kwargs.get('opacity', 0.8),
                color=kwargs.get('color', 'lightblue')
            )])
            
            fig.update_layout(
                title=kwargs.get('title', '3D Mesh Visualization'),
                scene=dict(aspectmode='data')
            )
            
            fig.show()
    
    def _visualize_matplotlib(self, data: Dict[str, Any], **kwargs) -> None:
        """Visualize using Matplotlib."""
        if 'points' in data:
            points = data['points']
            
            # Subsample for performance if too many points
            max_points = kwargs.get('max_points', 50000)
            if len(points) > max_points:
                indices = np.random.choice(len(points), max_points, replace=False)
                points = points[indices]
                print(f"Subsampled to {max_points} points for visualization")
            
            fig = plt.figure(figsize=kwargs.get('figsize', (12, 8)))
            ax = fig.add_subplot(111, projection='3d')
            
            # Determine colors
            if 'intensity' in data and data['intensity'] is not None:
                colors = data['intensity']
                if len(colors) > max_points:
                    colors = colors[indices]
                scatter = ax.scatter(points[:, 0], points[:, 1], points[:, 2], 
                                   c=colors, cmap='viridis', s=kwargs.get('point_size', 1))
                plt.colorbar(scatter, label='Intensity')
            else:
                # Color by height
                colors = points[:, 2]
                scatter = ax.scatter(points[:, 0], points[:, 1], points[:, 2], 
                                   c=colors, cmap='viridis', s=kwargs.get('point_size', 1))
                plt.colorbar(scatter, label='Height (Z)')
            
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.set_title(kwargs.get('title', '3D Point Cloud Visualization'))
            
            # Set equal aspect ratio
            max_range = np.array([points[:, 0].max() - points[:, 0].min(),
                                 points[:, 1].max() - points[:, 1].min(),
                                 points[:, 2].max() - points[:, 2].min()]).max() / 2.0
            mid_x = (points[:, 0].max() + points[:, 0].min()) * 0.5
            mid_y = (points[:, 1].max() + points[:, 1].min()) * 0.5
            mid_z = (points[:, 2].max() + points[:, 2].min()) * 0.5
            ax.set_xlim(mid_x - max_range, mid_x + max_range)
            ax.set_ylim(mid_y - max_range, mid_y + max_range)
            ax.set_zlim(mid_z - max_range, mid_z + max_range)
            
            plt.tight_layout()
            plt.show()
    
    def _intensity_to_colors(self, intensity: np.ndarray) -> np.ndarray:
        """Convert intensity values to RGB colors."""
        # Normalize intensity to [0, 1]
        intensity_norm = (intensity - intensity.min()) / (intensity.max() - intensity.min() + 1e-8)
        
        # Apply colormap
        cmap = plt.cm.get_cmap('plasma')
        colors = cmap(intensity_norm)[:, :3]  # Remove alpha channel
        
        return colors
    
    def _height_to_colors(self, heights: np.ndarray) -> np.ndarray:
        """Convert height values to RGB colors."""
        # Normalize heights to [0, 1]
        heights_norm = (heights - heights.min()) / (heights.max() - heights.min() + 1e-8)
        
        # Apply colormap
        cmap = plt.cm.get_cmap('viridis')
        colors = cmap(heights_norm)[:, :3]  # Remove alpha channel
        
        return colors
    
    def create_comparison_view(self, data_list: List[Union[str, Dict[str, Any]]], 
                              titles: Optional[List[str]] = None, **kwargs) -> None:
        """
        Create side-by-side comparison of multiple datasets.
        
        Args:
            data_list: List of file paths or data dictionaries
            titles: Optional titles for each dataset
            **kwargs: Visualization parameters
        """
        if self.backend == 'plotly' and PLOTLY_AVAILABLE:
            self._create_plotly_comparison(data_list, titles, **kwargs)
        else:
            self._create_matplotlib_comparison(data_list, titles, **kwargs)
    
    def _create_plotly_comparison(self, data_list: List[Union[str, Dict[str, Any]]], 
                                 titles: Optional[List[str]] = None, **kwargs) -> None:
        """Create Plotly subplot comparison."""
        from plotly.subplots import make_subplots
        
        n_datasets = len(data_list)
        cols = min(n_datasets, 2)
        rows = (n_datasets + cols - 1) // cols
        
        fig = make_subplots(
            rows=rows, cols=cols,
            specs=[[{'type': 'scatter3d'} for _ in range(cols)] for _ in range(rows)],
            subplot_titles=titles or [f'Dataset {i+1}' for i in range(n_datasets)]
        )
        
        for i, data in enumerate(data_list):
            if isinstance(data, str):
                data = self.load_data(data)
            
            if 'points' in data:
                points = data['points']
                
                # Subsample for performance
                max_points = kwargs.get('max_points', 10000)
                if len(points) > max_points:
                    indices = np.random.choice(len(points), max_points, replace=False)
                    points = points[indices]
                
                row = i // cols + 1
                col = i % cols + 1
                
                fig.add_trace(
                    go.Scatter3d(
                        x=points[:, 0],
                        y=points[:, 1],
                        z=points[:, 2],
                        mode='markers',
                        marker=dict(
                            size=2,
                            color=points[:, 2],
                            colorscale='Viridis',
                            opacity=0.6
                        ),
                        showlegend=False
                    ),
                    row=row, col=col
                )
        
        fig.update_layout(
            title_text="3D Data Comparison",
            height=400 * rows,
            width=800
        )
        
        fig.show()
    
    def _create_matplotlib_comparison(self, data_list: List[Union[str, Dict[str, Any]]], 
                                     titles: Optional[List[str]] = None, **kwargs) -> None:
        """Create Matplotlib subplot comparison."""
        n_datasets = len(data_list)
        cols = min(n_datasets, 2)
        rows = (n_datasets + cols - 1) // cols
        
        fig = plt.figure(figsize=(8 * cols, 6 * rows))
        
        for i, data in enumerate(data_list):
            if isinstance(data, str):
                data = self.load_data(data)
            
            if 'points' in data:
                points = data['points']
                
                # Subsample for performance
                max_points = kwargs.get('max_points', 10000)
                if len(points) > max_points:
                    indices = np.random.choice(len(points), max_points, replace=False)
                    points = points[indices]
                
                ax = fig.add_subplot(rows, cols, i + 1, projection='3d')
                
                scatter = ax.scatter(points[:, 0], points[:, 1], points[:, 2], 
                                   c=points[:, 2], cmap='viridis', s=1)
                
                ax.set_xlabel('X')
                ax.set_ylabel('Y')
                ax.set_zlabel('Z')
                ax.set_title(titles[i] if titles else f'Dataset {i+1}')
        
        plt.tight_layout()
        plt.show()
    
    def generate_statistics(self, data: Union[str, Dict[str, Any]]) -> Dict[str, Any]:
        """
        Generate comprehensive statistics for 3D data.
        
        Args:
            data: File path or loaded data dictionary
            
        Returns:
            Dictionary containing statistics
        """
        if isinstance(data, str):
            data = self.load_data(data)
        
        if 'points' not in data:
            return {'error': 'No point data found'}
        
        points = data['points']
        
        stats = {
            'num_points': len(points),
            'bounds': {
                'x_min': float(points[:, 0].min()),
                'x_max': float(points[:, 0].max()),
                'y_min': float(points[:, 1].min()),
                'y_max': float(points[:, 1].max()),
                'z_min': float(points[:, 2].min()),
                'z_max': float(points[:, 2].max())
            },
            'centroid': {
                'x': float(points[:, 0].mean()),
                'y': float(points[:, 1].mean()),
                'z': float(points[:, 2].mean())
            },
            'std_dev': {
                'x': float(points[:, 0].std()),
                'y': float(points[:, 1].std()),
                'z': float(points[:, 2].std())
            },
            'dimensions': {
                'width': float(points[:, 0].max() - points[:, 0].min()),
                'depth': float(points[:, 1].max() - points[:, 1].min()),
                'height': float(points[:, 2].max() - points[:, 2].min())
            }
        }
        
        # Add intensity statistics if available
        if 'intensity' in data and data['intensity'] is not None:
            intensity = data['intensity']
            stats['intensity'] = {
                'min': float(intensity.min()),
                'max': float(intensity.max()),
                'mean': float(intensity.mean()),
                'std': float(intensity.std())
            }
        
        return stats


def main():
    """
    Command-line interface for the 3D data visualizer.
    """
    import argparse
    
    parser = argparse.ArgumentParser(description='3D Data Visualization Tool')
    parser.add_argument('file', help='Path to 3D data file')
    parser.add_argument('--backend', choices=['matplotlib', 'open3d', 'plotly', 'auto'], 
                       default='auto', help='Visualization backend')
    parser.add_argument('--max-points', type=int, default=50000, 
                       help='Maximum points to display (for performance)')
    parser.add_argument('--point-size', type=float, default=1.0, 
                       help='Point size for visualization')
    parser.add_argument('--stats', action='store_true', 
                       help='Print data statistics')
    parser.add_argument('--title', type=str, help='Visualization title')
    
    args = parser.parse_args()
    
    # Create visualizer
    visualizer = DataVisualizer3D(backend=args.backend)
    
    try:
        # Load and visualize data
        print(f"Loading {args.file}...")
        data = visualizer.load_data(args.file)
        
        if args.stats:
            stats = visualizer.generate_statistics(data)
            print("\nData Statistics:")
            print(f"Number of points: {stats['num_points']:,}")
            print(f"Bounds: X[{stats['bounds']['x_min']:.2f}, {stats['bounds']['x_max']:.2f}], "
                  f"Y[{stats['bounds']['y_min']:.2f}, {stats['bounds']['y_max']:.2f}], "
                  f"Z[{stats['bounds']['z_min']:.2f}, {stats['bounds']['z_max']:.2f}]")
            print(f"Dimensions: {stats['dimensions']['width']:.2f} x "
                  f"{stats['dimensions']['depth']:.2f} x {stats['dimensions']['height']:.2f}")
        
        print(f"Visualizing with {visualizer.backend} backend...")
        visualizer.visualize(data, 
                           max_points=args.max_points,
                           point_size=args.point_size,
                           title=args.title or f"3D Visualization - {os.path.basename(args.file)}")
        
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
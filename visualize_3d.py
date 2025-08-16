#!/usr/bin/env python3
"""
Standalone 3D Data Visualization Script

A comprehensive Python script for visualizing various 3D data formats
commonly supported within the Python ecosystem.

Usage:
    python visualize_3d.py <file_path> [options]
    python visualize_3d.py --help

Supported formats:
    - Point clouds: PCD, PLY, LAS/LAZ, XYZ, CSV
    - Meshes: OBJ, STL, OFF
    - Arrays: NPY, NPZ

Examples:
    python visualize_3d.py data.pcd
    python visualize_3d.py data.las --backend plotly --stats
    python visualize_3d.py data.ply --backend open3d --point-size 2
    python visualize_3d.py *.pcd --compare  # Compare multiple files
"""

import sys
import os
import glob
import argparse
from pathlib import Path

# Add src directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

try:
    from livox_simulator.visualizer import DataVisualizer3D
except ImportError:
    print("Error: Could not import visualizer module.")
    print("Make sure you're running from the project root directory.")
    sys.exit(1)


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description='Comprehensive 3D Data Visualization Tool',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Supported File Formats:
  Point Clouds:
    .pcd    - Point Cloud Data (PCL format)
    .ply    - Polygon File Format
    .las    - LAS point cloud format
    .laz    - Compressed LAS format
    .xyz    - ASCII point cloud
    .csv    - Comma-separated values
    
  Meshes:
    .obj    - Wavefront OBJ
    .stl    - STL mesh format
    .off    - Object File Format
    
  Arrays:
    .npy    - NumPy array
    .npz    - NumPy compressed archive

Examples:
  # Basic visualization
  python visualize_3d.py point_cloud.pcd
  
  # Use specific backend
  python visualize_3d.py data.las --backend plotly
  
  # Show statistics
  python visualize_3d.py mesh.ply --stats
  
  # Compare multiple files
  python visualize_3d.py file1.pcd file2.pcd --compare
  
  # Use wildcards
  python visualize_3d.py *.las --compare --max-points 10000
        """
    )
    
    parser.add_argument('files', nargs='+', help='Path(s) to 3D data file(s)')
    parser.add_argument('--backend', choices=['matplotlib', 'open3d', 'plotly', 'auto'], 
                       default='auto', help='Visualization backend (default: auto)')
    parser.add_argument('--max-points', type=int, default=50000, 
                       help='Maximum points to display for performance (default: 50000)')
    parser.add_argument('--point-size', type=float, default=1.0, 
                       help='Point size for visualization (default: 1.0)')
    parser.add_argument('--opacity', type=float, default=0.8, 
                       help='Point opacity (0.0-1.0, default: 0.8)')
    parser.add_argument('--colorscale', type=str, default='Viridis', 
                       help='Color scale for visualization (default: Viridis)')
    parser.add_argument('--stats', action='store_true', 
                       help='Print detailed data statistics')
    parser.add_argument('--compare', action='store_true', 
                       help='Compare multiple files side-by-side')
    parser.add_argument('--title', type=str, help='Custom visualization title')
    parser.add_argument('--output', type=str, help='Save visualization to file (if supported)')
    parser.add_argument('--width', type=int, default=800, help='Visualization width')
    parser.add_argument('--height', type=int, default=600, help='Visualization height')
    parser.add_argument('--verbose', '-v', action='store_true', help='Verbose output')
    
    return parser.parse_args()


def expand_file_patterns(file_patterns):
    """Expand file patterns and wildcards."""
    files = []
    for pattern in file_patterns:
        if '*' in pattern or '?' in pattern:
            # Expand wildcards
            expanded = glob.glob(pattern)
            if expanded:
                files.extend(expanded)
            else:
                print(f"Warning: No files found matching pattern '{pattern}'")
        else:
            # Regular file path
            if os.path.exists(pattern):
                files.append(pattern)
            else:
                print(f"Warning: File not found '{pattern}'")
    
    return files


def print_file_info(filepath, data, verbose=False):
    """Print information about a loaded file."""
    file_size = os.path.getsize(filepath) / (1024 * 1024)  # MB
    
    print(f"\nFile: {os.path.basename(filepath)}")
    print(f"  Size: {file_size:.2f} MB")
    print(f"  Format: {data.get('format', 'unknown').upper()}")
    
    if 'points' in data:
        print(f"  Points: {len(data['points']):,}")
    elif 'vertices' in data:
        print(f"  Vertices: {len(data['vertices']):,}")
        print(f"  Faces: {len(data['faces']):,}")
    
    if verbose and 'metadata' in data:
        metadata = data['metadata']
        print(f"  Metadata: {metadata}")


def print_statistics(stats):
    """Print formatted statistics."""
    print("\n" + "="*50)
    print("DATA STATISTICS")
    print("="*50)
    
    if 'error' in stats:
        print(f"Error: {stats['error']}")
        return
    
    print(f"Number of points: {stats['num_points']:,}")
    
    bounds = stats['bounds']
    print(f"\nBounding Box:")
    print(f"  X: [{bounds['x_min']:.3f}, {bounds['x_max']:.3f}]")
    print(f"  Y: [{bounds['y_min']:.3f}, {bounds['y_max']:.3f}]")
    print(f"  Z: [{bounds['z_min']:.3f}, {bounds['z_max']:.3f}]")
    
    dims = stats['dimensions']
    print(f"\nDimensions:")
    print(f"  Width (X):  {dims['width']:.3f}")
    print(f"  Depth (Y):  {dims['depth']:.3f}")
    print(f"  Height (Z): {dims['height']:.3f}")
    
    centroid = stats['centroid']
    print(f"\nCentroid:")
    print(f"  X: {centroid['x']:.3f}")
    print(f"  Y: {centroid['y']:.3f}")
    print(f"  Z: {centroid['z']:.3f}")
    
    std_dev = stats['std_dev']
    print(f"\nStandard Deviation:")
    print(f"  X: {std_dev['x']:.3f}")
    print(f"  Y: {std_dev['y']:.3f}")
    print(f"  Z: {std_dev['z']:.3f}")
    
    if 'intensity' in stats:
        intensity = stats['intensity']
        print(f"\nIntensity Statistics:")
        print(f"  Min: {intensity['min']:.3f}")
        print(f"  Max: {intensity['max']:.3f}")
        print(f"  Mean: {intensity['mean']:.3f}")
        print(f"  Std: {intensity['std']:.3f}")


def main():
    """Main function."""
    args = parse_arguments()
    
    # Expand file patterns
    files = expand_file_patterns(args.files)
    
    if not files:
        print("Error: No valid files found.")
        sys.exit(1)
    
    if args.verbose:
        print(f"Found {len(files)} file(s) to process")
        for f in files:
            print(f"  - {f}")
    
    # Create visualizer
    try:
        visualizer = DataVisualizer3D(backend=args.backend)
        if args.verbose:
            print(f"\nUsing {visualizer.backend} backend")
            print(f"Supported formats: {', '.join(visualizer.supported_formats)}")
    except Exception as e:
        print(f"Error creating visualizer: {e}")
        sys.exit(1)
    
    # Process files
    if len(files) == 1 and not args.compare:
        # Single file visualization
        filepath = files[0]
        
        try:
            if args.verbose:
                print(f"\nLoading {filepath}...")
            
            data = visualizer.load_data(filepath)
            print_file_info(filepath, data, args.verbose)
            
            if args.stats:
                stats = visualizer.generate_statistics(data)
                print_statistics(stats)
            
            # Visualization parameters
            viz_params = {
                'max_points': args.max_points,
                'point_size': args.point_size,
                'opacity': args.opacity,
                'colorscale': args.colorscale,
                'width': args.width,
                'height': args.height,
                'title': args.title or f"3D Visualization - {os.path.basename(filepath)}"
            }
            
            if args.verbose:
                print(f"\nVisualizing with parameters: {viz_params}")
            
            visualizer.visualize(data, **viz_params)
            
        except Exception as e:
            print(f"Error processing {filepath}: {e}")
            if args.verbose:
                import traceback
                traceback.print_exc()
            sys.exit(1)
    
    else:
        # Multiple file comparison or single file with --compare flag
        if args.verbose:
            print(f"\nLoading {len(files)} files for comparison...")
        
        data_list = []
        titles = []
        
        for filepath in files:
            try:
                data = visualizer.load_data(filepath)
                data_list.append(data)
                titles.append(os.path.basename(filepath))
                
                print_file_info(filepath, data, args.verbose)
                
                if args.stats:
                    stats = visualizer.generate_statistics(data)
                    print_statistics(stats)
                
            except Exception as e:
                print(f"Error loading {filepath}: {e}")
                if args.verbose:
                    import traceback
                    traceback.print_exc()
                continue
        
        if not data_list:
            print("Error: No files could be loaded successfully.")
            sys.exit(1)
        
        # Create comparison visualization
        try:
            viz_params = {
                'max_points': args.max_points,
                'width': args.width,
                'height': args.height
            }
            
            if args.verbose:
                print(f"\nCreating comparison visualization...")
            
            visualizer.create_comparison_view(data_list, titles, **viz_params)
            
        except Exception as e:
            print(f"Error creating comparison visualization: {e}")
            if args.verbose:
                import traceback
                traceback.print_exc()
            sys.exit(1)
    
    print("\nVisualization complete!")


if __name__ == '__main__':
    main()
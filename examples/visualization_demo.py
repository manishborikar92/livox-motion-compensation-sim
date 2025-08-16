#!/usr/bin/env python3
"""
3D Visualization Demo Script

This script demonstrates the comprehensive 3D visualization capabilities
of the Livox Mid-70 simulation framework.
"""

import sys
import os
import numpy as np

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from livox_simulator import DataVisualizer3D


def create_sample_data():
    """Create sample 3D data for demonstration."""
    print("Creating sample 3D datasets...")
    
    # Create sample point cloud 1: Sphere
    n_points = 5000
    phi = np.random.uniform(0, 2*np.pi, n_points)
    costheta = np.random.uniform(-1, 1, n_points)
    u = np.random.uniform(0, 1, n_points)
    
    theta = np.arccos(costheta)
    r = 10 * np.cbrt(u)
    
    x = r * np.sin(theta) * np.cos(phi)
    y = r * np.sin(theta) * np.sin(phi)
    z = r * np.cos(theta)
    
    # Add some noise and intensity
    noise = np.random.normal(0, 0.1, (n_points, 3))
    sphere_points = np.column_stack([x, y, z]) + noise
    sphere_intensity = np.random.randint(50, 255, n_points)
    
    sphere_data = {
        'points': sphere_points,
        'intensity': sphere_intensity,
        'format': 'synthetic',
        'metadata': {'name': 'sphere', 'num_points': n_points}
    }
    
    # Create sample point cloud 2: Cylinder
    n_points = 4000
    theta = np.random.uniform(0, 2*np.pi, n_points)
    height = np.random.uniform(-5, 5, n_points)
    radius = 5 + np.random.normal(0, 0.5, n_points)
    
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    z = height
    
    cylinder_points = np.column_stack([x, y, z])
    cylinder_intensity = np.random.randint(100, 200, n_points)
    
    cylinder_data = {
        'points': cylinder_points,
        'intensity': cylinder_intensity,
        'format': 'synthetic',
        'metadata': {'name': 'cylinder', 'num_points': n_points}
    }
    
    # Create sample point cloud 3: Plane with objects
    n_points = 6000
    
    # Ground plane
    x_ground = np.random.uniform(-15, 15, n_points//2)
    y_ground = np.random.uniform(-15, 15, n_points//2)
    z_ground = np.random.normal(0, 0.1, n_points//2)
    
    # Objects on the plane
    x_objects = np.random.uniform(-10, 10, n_points//2)
    y_objects = np.random.uniform(-10, 10, n_points//2)
    z_objects = np.random.uniform(0, 8, n_points//2)
    
    x = np.concatenate([x_ground, x_objects])
    y = np.concatenate([y_ground, y_objects])
    z = np.concatenate([z_ground, z_objects])
    
    scene_points = np.column_stack([x, y, z])
    scene_intensity = np.random.randint(30, 255, n_points)
    
    scene_data = {
        'points': scene_points,
        'intensity': scene_intensity,
        'format': 'synthetic',
        'metadata': {'name': 'scene', 'num_points': n_points}
    }
    
    return sphere_data, cylinder_data, scene_data


def demo_single_visualization():
    """Demonstrate single dataset visualization with different backends."""
    print("\n" + "="*60)
    print("SINGLE DATASET VISUALIZATION DEMO")
    print("="*60)
    
    # Create sample data
    sphere_data, _, _ = create_sample_data()
    
    # Test different backends
    backends = ['matplotlib', 'plotly', 'open3d']
    
    for backend in backends:
        try:
            print(f"\nTesting {backend} backend...")
            visualizer = DataVisualizer3D(backend=backend)
            
            if visualizer.backend == backend:
                print(f"✅ {backend} backend available")
                
                # Generate statistics
                stats = visualizer.generate_statistics(sphere_data)
                print(f"   Dataset: {stats['num_points']:,} points")
                print(f"   Bounds: X[{stats['bounds']['x_min']:.1f}, {stats['bounds']['x_max']:.1f}]")
                
                # Visualize (comment out to avoid opening multiple windows)
                # visualizer.visualize(sphere_data, 
                #                     title=f"Sphere Dataset - {backend.title()} Backend",
                #                     point_size=2,
                #                     opacity=0.8)
                
            else:
                print(f"⚠️  {backend} backend not available, using {visualizer.backend}")
                
        except Exception as e:
            print(f"❌ {backend} backend failed: {e}")


def demo_comparison_visualization():
    """Demonstrate multi-dataset comparison visualization."""
    print("\n" + "="*60)
    print("MULTI-DATASET COMPARISON DEMO")
    print("="*60)
    
    # Create sample datasets
    sphere_data, cylinder_data, scene_data = create_sample_data()
    datasets = [sphere_data, cylinder_data, scene_data]
    titles = ['Sphere', 'Cylinder', 'Scene']
    
    # Test comparison with different backends
    for backend in ['matplotlib', 'plotly']:
        try:
            print(f"\nTesting {backend} comparison...")
            visualizer = DataVisualizer3D(backend=backend)
            
            if visualizer.backend == backend:
                print(f"✅ {backend} comparison available")
                
                # Create comparison (comment out to avoid opening windows)
                # visualizer.create_comparison_view(datasets, titles, max_points=2000)
                
            else:
                print(f"⚠️  {backend} not available, using {visualizer.backend}")
                
        except Exception as e:
            print(f"❌ {backend} comparison failed: {e}")


def demo_statistics_generation():
    """Demonstrate comprehensive statistics generation."""
    print("\n" + "="*60)
    print("STATISTICS GENERATION DEMO")
    print("="*60)
    
    # Create sample data
    sphere_data, cylinder_data, scene_data = create_sample_data()
    datasets = [
        ('Sphere', sphere_data),
        ('Cylinder', cylinder_data),
        ('Scene', scene_data)
    ]
    
    visualizer = DataVisualizer3D()
    
    for name, data in datasets:
        print(f"\n{name} Dataset Statistics:")
        print("-" * 30)
        
        stats = visualizer.generate_statistics(data)
        
        print(f"Points: {stats['num_points']:,}")
        print(f"Bounds:")
        print(f"  X: [{stats['bounds']['x_min']:.2f}, {stats['bounds']['x_max']:.2f}]")
        print(f"  Y: [{stats['bounds']['y_min']:.2f}, {stats['bounds']['y_max']:.2f}]")
        print(f"  Z: [{stats['bounds']['z_min']:.2f}, {stats['bounds']['z_max']:.2f}]")
        
        print(f"Dimensions:")
        print(f"  Width:  {stats['dimensions']['width']:.2f}")
        print(f"  Depth:  {stats['dimensions']['depth']:.2f}")
        print(f"  Height: {stats['dimensions']['height']:.2f}")
        
        print(f"Centroid: ({stats['centroid']['x']:.2f}, {stats['centroid']['y']:.2f}, {stats['centroid']['z']:.2f})")
        
        if 'intensity' in stats:
            intensity = stats['intensity']
            print(f"Intensity: [{intensity['min']:.0f}, {intensity['max']:.0f}] (mean: {intensity['mean']:.1f})")


def demo_format_support():
    """Demonstrate supported file format detection."""
    print("\n" + "="*60)
    print("SUPPORTED FORMATS DEMO")
    print("="*60)
    
    visualizer = DataVisualizer3D()
    
    print("Supported 3D data formats:")
    for i, fmt in enumerate(visualizer.supported_formats, 1):
        print(f"  {i:2d}. {fmt.upper()}")
    
    print(f"\nTotal formats supported: {len(visualizer.supported_formats)}")
    print(f"Selected backend: {visualizer.backend}")


def main():
    """Main demonstration function."""
    print("3D Data Visualization Framework Demo")
    print("=" * 60)
    
    try:
        # Run all demonstrations
        demo_format_support()
        demo_statistics_generation()
        demo_single_visualization()
        demo_comparison_visualization()
        
        print("\n" + "="*60)
        print("DEMO COMPLETED SUCCESSFULLY")
        print("="*60)
        print("\nTo see actual visualizations, uncomment the visualization")
        print("calls in the demo functions and run the script again.")
        print("\nFor interactive visualization, try:")
        print("  python visualize_3d.py <your_data_file>")
        
    except Exception as e:
        print(f"\nDemo failed with error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
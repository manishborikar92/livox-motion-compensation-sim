#!/usr/bin/env python3
"""
Simple Optimized Livox Mid-70 LiDAR Simulator
Focused on speed and reliability for quick execution
"""

import numpy as np
import os
import time
from scipy.spatial import cKDTree
import struct

class SimpleLivoxSimulator:
    def __init__(self):
        # Drastically reduced parameters for speed
        self.fov_horizontal = 70.4  # degrees
        self.fov_vertical = 77.2    # degrees
        self.max_range = 50.0       # meters (reduced from 260m)
        self.min_range = 0.5        # meters
        
        # Very low resolution for speed
        self.points_per_frame = 500   # reduced from 100k+
        self.fps = 5                  # reduced from 10
        self.duration = 2.0           # seconds (very short)
        
        # Simple environment
        self.num_objects = 10         # reduced from 100+
        
    def generate_simple_trajectory(self):
        """Generate a simple straight-line trajectory"""
        num_frames = int(self.duration * self.fps)
        trajectory = []
        
        for i in range(num_frames):
            t = i / self.fps
            # Simple forward motion
            position = np.array([t * 2.0, 0.0, 1.5])  # 2 m/s forward
            rotation = np.array([0.0, 0.0, 0.0])      # no rotation
            trajectory.append((position, rotation))
            
        return trajectory
    
    def create_simple_environment(self):
        """Create a very simple environment with basic objects"""
        points = []
        
        # Ground plane
        x_ground = np.random.uniform(-20, 20, 200)
        y_ground = np.random.uniform(-20, 20, 200)
        z_ground = np.zeros(200)
        ground_points = np.column_stack([x_ground, y_ground, z_ground])
        points.extend(ground_points)
        
        # Simple objects (boxes)
        for i in range(self.num_objects):
            # Random box position
            center = np.random.uniform(-15, 15, 3)
            center[2] = np.random.uniform(0.5, 3.0)  # height above ground
            
            # Simple box (8 corners)
            size = np.random.uniform(0.5, 2.0)
            corners = np.array([
                [-size/2, -size/2, -size/2],
                [size/2, -size/2, -size/2],
                [size/2, size/2, -size/2],
                [-size/2, size/2, -size/2],
                [-size/2, -size/2, size/2],
                [size/2, -size/2, size/2],
                [size/2, size/2, size/2],
                [-size/2, size/2, size/2]
            ]) + center
            
            points.extend(corners)
        
        return np.array(points)
    
    def simple_lidar_scan(self, sensor_pos, environment_tree):
        """Perform a simple LiDAR scan"""
        # Generate scan pattern (simple grid)
        h_angles = np.linspace(-self.fov_horizontal/2, self.fov_horizontal/2, 25)
        v_angles = np.linspace(-self.fov_vertical/2, self.fov_vertical/2, 20)
        
        detected_points = []
        
        for h_angle in h_angles:
            for v_angle in v_angles:
                # Convert to radians
                h_rad = np.radians(h_angle)
                v_rad = np.radians(v_angle)
                
                # Ray direction
                direction = np.array([
                    np.cos(v_rad) * np.cos(h_rad),
                    np.cos(v_rad) * np.sin(h_rad),
                    np.sin(v_rad)
                ])
                
                # Simple ray casting - check points along ray
                for distance in np.linspace(self.min_range, self.max_range, 20):
                    ray_point = sensor_pos + direction * distance
                    
                    # Find nearest environment point
                    distances, indices = environment_tree.query(ray_point, k=1)
                    
                    if distances < 0.5:  # Hit threshold
                        # Add some noise
                        noise = np.random.normal(0, 0.02, 3)
                        hit_point = ray_point + noise
                        intensity = max(0, 255 - int(distance * 2))  # Simple intensity
                        
                        detected_points.append({
                            'point': hit_point,
                            'intensity': intensity,
                            'distance': distance
                        })
                        break
        
        return detected_points
    
    def save_simple_pcd(self, points, filename):
        """Save points as simple PCD file"""
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        
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
            
            for point_data in points:
                p = point_data['point']
                i = point_data['intensity']
                f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {i}\n")
    
    def run_simple_simulation(self):
        """Run the simplified simulation"""
        print("Starting Simple Optimized Livox Simulation...")
        start_time = time.time()
        
        # Create output directory
        output_dir = "simple_lidar_output"
        os.makedirs(output_dir, exist_ok=True)
        
        # Generate trajectory
        print("Generating trajectory...")
        trajectory = self.generate_simple_trajectory()
        
        # Create environment
        print("Creating environment...")
        environment_points = self.create_simple_environment()
        environment_tree = cKDTree(environment_points)
        print(f"Environment created with {len(environment_points)} points")
        
        # Simulate frames
        print("Simulating LiDAR frames...")
        all_scans = []
        
        for frame_idx, (position, rotation) in enumerate(trajectory):
            print(f"Processing frame {frame_idx + 1}/{len(trajectory)}")
            
            # Perform scan
            scan_points = self.simple_lidar_scan(position, environment_tree)
            
            if scan_points:
                # Save individual frame
                frame_file = os.path.join(output_dir, f"frame_{frame_idx:04d}.pcd")
                self.save_simple_pcd(scan_points, frame_file)
                
                # Add to merged scan
                all_scans.extend(scan_points)
        
        # Save merged scan
        if all_scans:
            merged_file = os.path.join(output_dir, "merged_scan.pcd")
            self.save_simple_pcd(all_scans, merged_file)
            print(f"Merged scan saved: {merged_file}")
        
        # Performance summary
        total_time = time.time() - start_time
        print(f"\nSimulation completed successfully!")
        print(f"Total time: {total_time:.2f} seconds")
        print(f"Frames generated: {len(trajectory)}")
        print(f"Total points: {len(all_scans)}")
        print(f"Output directory: {output_dir}")
        
        return output_dir

def main():
    """Main execution function"""
    simulator = SimpleLivoxSimulator()
    result = simulator.run_simple_simulation()
    
    if result:
        print(f"\nSuccess! Results saved to: {result}")
        print("\nNext steps:")
        print(f"- View PCD files in CloudCompare")
        print(f"- Check individual frames in {result}/")
    else:
        print("Simulation failed")

if __name__ == "__main__":
    main()
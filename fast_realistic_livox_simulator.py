#!/usr/bin/env python3
"""
Fast Realistic Livox Mid-70 LiDAR Simulator
Optimized version of realistic_livox_simulator.py for quick execution
"""

import numpy as np
import os
import time
from scipy.spatial import cKDTree
import struct
import csv

class FastLVXWriter:
    """Simplified LVX writer for basic compatibility"""
    
    def write_simple_lvx(self, filename, frames_data, version="v1.0"):
        """Write a simplified LVX file"""
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        
        with open(filename, 'wb') as f:
            # Simple LVX header
            f.write(b'LIVOX')
            f.write(struct.pack('<H', 1))  # version
            f.write(struct.pack('<I', len(frames_data)))  # frame count
            
            # Write frame data
            for frame_idx, frame in enumerate(frames_data):
                # Frame header
                f.write(struct.pack('<I', frame_idx))  # frame number
                f.write(struct.pack('<I', len(frame)))  # point count
                
                # Point data
                for point in frame:
                    x, y, z = point['point']
                    intensity = point.get('intensity', 128)
                    
                    f.write(struct.pack('<fff', x, y, z))
                    f.write(struct.pack('<B', min(255, max(0, int(intensity)))))
        
        print(f"Fast LVX file created: {filename} ({os.path.getsize(filename):,} bytes)")
        return filename

class FastRealisticLivoxMid70Simulator:
    """Fast version of the realistic Livox Mid-70 simulator"""
    
    def __init__(self):
        # Optimized parameters for speed
        self.fov_horizontal = 70.4
        self.fov_vertical = 77.2
        self.max_range = 100.0      # Reduced from 260m
        self.min_range = 0.5
        
        # Reduced resolution for speed
        self.points_per_frame = 2000  # Reduced from 100k+
        self.fps = 5                  # Reduced from 10
        self.duration = 3.0           # Short duration
        
        # Environment parameters
        self.num_objects = 25         # Reduced complexity
        
        # Pre-computed scan pattern
        self._scan_pattern = self._generate_scan_pattern()
        
    def _generate_scan_pattern(self):
        """Pre-generate scan pattern for efficiency"""
        # Rosette-like pattern but simplified
        angles = []
        
        # Horizontal sweep
        h_steps = 40  # Reduced from higher resolution
        v_steps = 25  # Reduced from higher resolution
        
        for i in range(h_steps):
            for j in range(v_steps):
                h_angle = (i / h_steps - 0.5) * self.fov_horizontal
                v_angle = (j / v_steps - 0.5) * self.fov_vertical
                
                # Convert to direction vector
                h_rad = np.radians(h_angle)
                v_rad = np.radians(v_angle)
                
                direction = np.array([
                    np.cos(v_rad) * np.cos(h_rad),
                    np.cos(v_rad) * np.sin(h_rad),
                    np.sin(v_rad)
                ])
                
                angles.append(direction)
        
        return np.array(angles)
    
    def generate_trajectory(self):
        """Generate vehicle trajectory"""
        num_frames = int(self.duration * self.fps)
        trajectory = []
        
        for i in range(num_frames):
            t = i / self.fps
            
            # Simple curved path
            speed = 5.0  # m/s
            curve_radius = 50.0
            
            angle = (speed * t) / curve_radius
            position = np.array([
                curve_radius * np.sin(angle),
                curve_radius * (1 - np.cos(angle)),
                1.8  # Vehicle height
            ])
            
            rotation = np.array([0.0, 0.0, angle])  # Yaw rotation
            
            trajectory.append({
                'position': position,
                'rotation': rotation,
                'timestamp': t
            })
        
        return trajectory
    
    def create_fast_environment(self):
        """Create a simplified but realistic environment"""
        points = []
        
        # Ground plane with some variation
        ground_size = 200
        x_ground = np.random.uniform(-50, 50, ground_size)
        y_ground = np.random.uniform(-50, 50, ground_size)
        z_ground = np.random.normal(0, 0.1, ground_size)  # Slight variation
        
        for i in range(ground_size):
            points.append({
                'point': np.array([x_ground[i], y_ground[i], z_ground[i]]),
                'material': 'asphalt'
            })
        
        # Buildings and objects
        for i in range(self.num_objects):
            obj_type = np.random.choice(['building', 'vehicle', 'pole', 'tree'])
            center = np.array([
                np.random.uniform(-40, 40),
                np.random.uniform(-40, 40),
                0
            ])
            
            if obj_type == 'building':
                # Simple building
                width = np.random.uniform(5, 15)
                height = np.random.uniform(8, 25)
                depth = np.random.uniform(5, 15)
                
                # Building corners and edges
                for x in np.linspace(-width/2, width/2, 5):
                    for y in np.linspace(-depth/2, depth/2, 5):
                        for z in np.linspace(0, height, 8):
                            if x in [-width/2, width/2] or y in [-depth/2, depth/2] or z in [0, height]:
                                points.append({
                                    'point': center + np.array([x, y, z]),
                                    'material': 'concrete'
                                })
            
            elif obj_type == 'vehicle':
                # Simple vehicle shape
                length, width, height = 4.5, 1.8, 1.5
                center[2] = height/2
                
                for x in np.linspace(-length/2, length/2, 6):
                    for y in np.linspace(-width/2, width/2, 3):
                        for z in np.linspace(-height/2, height/2, 3):
                            if abs(x) >= length/2-0.1 or abs(y) >= width/2-0.1 or abs(z) >= height/2-0.1:
                                points.append({
                                    'point': center + np.array([x, y, z]),
                                    'material': 'metal'
                                })
            
            elif obj_type == 'pole':
                # Vertical pole
                height = np.random.uniform(3, 8)
                for z in np.linspace(0, height, 10):
                    points.append({
                        'point': center + np.array([0, 0, z]),
                        'material': 'metal'
                    })
            
            elif obj_type == 'tree':
                # Simple tree
                trunk_height = np.random.uniform(2, 4)
                crown_radius = np.random.uniform(1, 3)
                
                # Trunk
                for z in np.linspace(0, trunk_height, 5):
                    points.append({
                        'point': center + np.array([0, 0, z]),
                        'material': 'wood'
                    })
                
                # Crown (simplified)
                for _ in range(20):
                    angle = np.random.uniform(0, 2*np.pi)
                    radius = np.random.uniform(0, crown_radius)
                    height = trunk_height + np.random.uniform(0, crown_radius)
                    
                    x = radius * np.cos(angle)
                    y = radius * np.sin(angle)
                    
                    points.append({
                        'point': center + np.array([x, y, height]),
                        'material': 'vegetation'
                    })
        
        return points
    
    def simulate_lidar_scan(self, sensor_pos, sensor_rot, environment_tree, environment_points):
        """Simulate LiDAR scan with optimized ray casting"""
        detected_points = []
        
        # Rotation matrix (simplified - only yaw)
        yaw = sensor_rot[2]
        cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
        rotation_matrix = np.array([
            [cos_yaw, -sin_yaw, 0],
            [sin_yaw, cos_yaw, 0],
            [0, 0, 1]
        ])
        
        # Sample subset of scan pattern for speed
        pattern_indices = np.random.choice(len(self._scan_pattern), 
                                         size=min(self.points_per_frame, len(self._scan_pattern)), 
                                         replace=False)
        
        for idx in pattern_indices:
            direction = self._scan_pattern[idx]
            
            # Apply sensor rotation
            world_direction = rotation_matrix @ direction
            
            # Ray casting with optimized steps
            hit_found = False
            for distance in np.linspace(self.min_range, self.max_range, 25):  # Reduced steps
                ray_point = sensor_pos + world_direction * distance
                
                # Query nearest points
                distances, indices = environment_tree.query(ray_point, k=1)
                
                if distances < 1.0:  # Hit threshold
                    # Get material for intensity calculation
                    hit_point_data = environment_points[indices]
                    material = hit_point_data.get('material', 'unknown')
                    
                    # Simple intensity based on distance and material
                    base_intensity = {
                        'metal': 200, 'concrete': 150, 'asphalt': 100,
                        'vegetation': 80, 'wood': 120, 'unknown': 128
                    }.get(material, 128)
                    
                    intensity = max(0, base_intensity - int(distance * 1.5))
                    
                    # Add measurement noise
                    noise = np.random.normal(0, 0.02, 3)
                    final_point = ray_point + noise
                    
                    detected_points.append({
                        'point': final_point,
                        'intensity': intensity,
                        'distance': distance,
                        'material': material
                    })
                    
                    hit_found = True
                    break
        
        return detected_points
    
    def save_pcd(self, points, filename):
        """Save points as PCD file"""
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
                i = point_data.get('intensity', 128)
                f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {i}\n")
    
    def save_trajectory_csv(self, trajectory, filename):
        """Save trajectory as CSV"""
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'x', 'y', 'z', 'roll', 'pitch', 'yaw'])
            
            for frame in trajectory:
                pos = frame['position']
                rot = frame['rotation']
                writer.writerow([
                    frame['timestamp'], pos[0], pos[1], pos[2],
                    rot[0], rot[1], rot[2]
                ])
    
    def run_simulation(self):
        """Run the fast realistic simulation"""
        print("Starting Fast Realistic Livox Mid-70 Simulation...")
        start_time = time.time()
        
        # Create output directory
        output_dir = "fast_realistic_lidar_output"
        os.makedirs(output_dir, exist_ok=True)
        
        # Generate trajectory
        print("Generating vehicle trajectory...")
        trajectory = self.generate_trajectory()
        print(f"Generated trajectory with {len(trajectory)} frames")
        
        # Create environment
        print("Creating realistic environment...")
        environment_points = self.create_fast_environment()
        
        # Convert to format suitable for KDTree
        env_coords = np.array([p['point'] for p in environment_points])
        environment_tree = cKDTree(env_coords)
        print(f"Environment created with {len(environment_points)} objects")
        
        # Simulate LiDAR frames
        print("Simulating LiDAR frames...")
        all_scans = []
        frame_data = []
        
        for frame_idx, frame_info in enumerate(trajectory):
            print(f"Processing frame {frame_idx + 1}/{len(trajectory)}")
            
            # Perform LiDAR scan
            scan_points = self.simulate_lidar_scan(
                frame_info['position'],
                frame_info['rotation'],
                environment_tree,
                environment_points
            )
            
            if scan_points:
                # Save individual frame
                frame_file = os.path.join(output_dir, f"frame_{frame_idx:04d}.pcd")
                self.save_pcd(scan_points, frame_file)
                
                # Store for merged output
                all_scans.extend(scan_points)
                frame_data.append(scan_points)
        
        # Save merged scan
        if all_scans:
            merged_file = os.path.join(output_dir, "merged_scan.pcd")
            self.save_pcd(all_scans, merged_file)
            print(f"Merged scan saved: {merged_file}")
        
        # Save trajectory
        trajectory_file = os.path.join(output_dir, "trajectory.csv")
        self.save_trajectory_csv(trajectory, trajectory_file)
        
        # Save LVX format
        if frame_data:
            lvx_writer = FastLVXWriter()
            lvx_file = os.path.join(output_dir, "fast_realistic_scan.lvx")
            lvx_writer.write_simple_lvx(lvx_file, frame_data)
        
        # Performance summary
        total_time = time.time() - start_time
        print(f"\nFast Realistic Simulation completed successfully!")
        print(f"Total time: {total_time:.2f} seconds")
        print(f"Frames generated: {len(trajectory)}")
        print(f"Total points: {len(all_scans)}")
        print(f"Average points per frame: {len(all_scans)/len(trajectory):.0f}")
        print(f"Output directory: {output_dir}")
        
        return output_dir

def main():
    """Main execution function"""
    simulator = FastRealisticLivoxMid70Simulator()
    result = simulator.run_simulation()
    
    if result:
        print(f"\nSuccess! Results saved to: {result}")
        print("\nGenerated files:")
        print(f"- Individual frames: frame_*.pcd")
        print(f"- Merged point cloud: merged_scan.pcd")
        print(f"- Vehicle trajectory: trajectory.csv")
        print(f"- LVX format: fast_realistic_scan.lvx")
        print("\nNext steps:")
        print(f"- View PCD files in CloudCompare")
        print(f"- Load LVX file in Livox Viewer")
        print(f"- Analyze trajectory data")
    else:
        print("Simulation failed")

if __name__ == "__main__":
    main()
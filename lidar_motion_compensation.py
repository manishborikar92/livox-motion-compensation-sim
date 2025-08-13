"""
LiDAR Motion Compensation Simulation for Livox Mid-70
Simulates point cloud data with realistic motion and applies transformations
for global alignment, addressing the overlapping frames problem.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
from datetime import datetime, timedelta
from scipy.spatial.transform import Rotation as R
from scipy import signal
import laspy

class LiDARMotionSimulator:
    def __init__(self, config=None):
        """
        Initialize the LiDAR Motion Simulator
        
        Args:
            config (dict): Configuration parameters
        """
        self.config = self.default_config()
        if config:
            self.config.update(config)  # Merge custom config with defaults
        np.random.seed(self.config['random_seed'])
        
    def default_config(self):
        """Default simulation configuration"""
        return {
            # Simulation parameters
            'duration': 60.0,           # Total simulation time (seconds)
            'lidar_fps': 10,            # LiDAR frame rate (Hz)
            'imu_rate': 100,            # IMU update rate (Hz)
            'gps_rate': 5,              # GPS update rate (Hz)
            'random_seed': 42,
            
            # Motion parameters
            'max_speed': 15.0,          # Maximum vehicle speed (m/s)
            'max_angular_vel': 0.5,     # Maximum angular velocity (rad/s)
            'trajectory_type': 'figure_eight',  # 'linear', 'circular', 'figure_eight'
            
            # LiDAR Mid-70 specifications
            'fov_horizontal': 70.0,     # degrees
            'fov_vertical': 77.2,       # degrees  
            'range_max': 90.0,          # meters
            'range_min': 0.05,          # meters
            'points_per_frame': 96000,  # typical for Mid-70
            'angular_resolution': 0.28, # degrees
            
            # Noise parameters
            'gps_noise_std': 0.03,      # GPS noise standard deviation (m)
            'imu_accel_noise': 0.1,     # IMU acceleration noise (m/s²)
            'imu_gyro_noise': 0.01,     # IMU gyroscope noise (rad/s)
            'lidar_range_noise': 0.02,  # LiDAR range noise (m)
            
            # Environment parameters
            'environment_complexity': 'medium',  # 'simple', 'medium', 'complex'
            'ground_height': 0.0,       # Ground plane height
            'obstacle_density': 0.1,    # Obstacle density factor
        }
    
    def generate_trajectory(self):
        """Generate vehicle trajectory based on configuration"""
        t = np.linspace(0, self.config['duration'], 
                       int(self.config['duration'] * self.config['gps_rate']))
        
        if self.config['trajectory_type'] == 'linear':
            # Linear trajectory with gentle curves
            x = self.config['max_speed'] * t
            y = 5 * np.sin(0.1 * t)  # Gentle S-curves
            z = np.zeros_like(t)
            
        elif self.config['trajectory_type'] == 'circular':
            # Circular trajectory
            radius = 50.0
            angular_freq = self.config['max_speed'] / radius
            x = radius * np.cos(angular_freq * t)
            y = radius * np.sin(angular_freq * t)
            z = np.zeros_like(t)
            
        elif self.config['trajectory_type'] == 'figure_eight':
            # Figure-eight trajectory
            scale = 30.0
            freq = 0.05
            x = scale * np.sin(2 * np.pi * freq * t)
            y = scale * np.sin(4 * np.pi * freq * t)
            z = np.zeros_like(t) + 1.5  # Vehicle height above ground
            
        # Calculate velocities
        vx = np.gradient(x) * self.config['gps_rate']
        vy = np.gradient(y) * self.config['gps_rate']
        vz = np.gradient(z) * self.config['gps_rate']
        
        # Calculate orientation (heading based on velocity direction)
        yaw = np.arctan2(vy, vx)
        roll = np.zeros_like(t)
        pitch = np.zeros_like(t)
        
        # Add realistic vehicle dynamics
        max_roll = np.radians(5)  # Max 5 degrees roll
        roll = max_roll * np.sin(0.5 * yaw) * (np.sqrt(vx**2 + vy**2) / self.config['max_speed'])
        
        return {
            'time': t,
            'position': np.column_stack([x, y, z]),
            'velocity': np.column_stack([vx, vy, vz]),
            'orientation': np.column_stack([roll, pitch, yaw])
        }
    
    def add_sensor_noise(self, trajectory):
        """Add realistic sensor noise to trajectory data"""
        # GPS noise
        gps_noise = np.random.normal(0, self.config['gps_noise_std'], 
                                   trajectory['position'].shape)
        trajectory['position_gps'] = trajectory['position'] + gps_noise
        
        # IMU noise
        accel_noise = np.random.normal(0, self.config['imu_accel_noise'],
                                     trajectory['velocity'].shape)
        gyro_noise = np.random.normal(0, self.config['imu_gyro_noise'],
                                    trajectory['orientation'].shape)
        
        # Calculate accelerations
        dt = 1.0 / self.config['gps_rate']
        acceleration = np.gradient(trajectory['velocity'], dt, axis=0)
        trajectory['acceleration'] = acceleration + accel_noise
        trajectory['orientation_imu'] = trajectory['orientation'] + gyro_noise
        
        return trajectory
    
    def generate_environment_pointcloud(self):
        """Generate a realistic 3D environment for LiDAR scanning"""
        if self.config['environment_complexity'] == 'simple':
            return self._generate_simple_environment()
        elif self.config['environment_complexity'] == 'medium':
            return self._generate_medium_environment()
        else:
            return self._generate_complex_environment()
    
    def _generate_simple_environment(self):
        """Generate simple environment with ground plane and few obstacles"""
        points = []
        
        # Ground plane
        x_ground = np.random.uniform(-100, 100, 5000)
        y_ground = np.random.uniform(-100, 100, 5000)
        z_ground = np.full_like(x_ground, self.config['ground_height'])
        intensity_ground = np.random.uniform(0.3, 0.7, 5000)
        
        points.extend(zip(x_ground, y_ground, z_ground, intensity_ground))
        
        # Simple box obstacles
        for _ in range(10):
            center = np.random.uniform(-50, 50, 3)
            size = np.random.uniform(2, 5, 3)
            box_points = self._generate_box(center, size)
            points.extend(box_points)
        
        return np.array(points)
    
    def _generate_medium_environment(self):
        """Generate medium complexity environment"""
        points = []
        
        # Ground with texture
        x_ground = np.random.uniform(-150, 150, 10000)
        y_ground = np.random.uniform(-150, 150, 10000)
        z_ground = (self.config['ground_height'] + 
                   0.2 * np.sin(x_ground * 0.1) * np.cos(y_ground * 0.1))
        intensity_ground = np.random.uniform(0.2, 0.8, 10000)
        
        points.extend(zip(x_ground, y_ground, z_ground, intensity_ground))
        
        # Buildings
        for _ in range(20):
            center = np.random.uniform(-80, 80, 2)
            center = np.append(center, np.random.uniform(5, 15))
            size = [np.random.uniform(5, 15), np.random.uniform(5, 15), 
                   np.random.uniform(10, 30)]
            building_points = self._generate_building(center, size)
            points.extend(building_points)
        
        # Trees (cylinders with foliage)
        for _ in range(50):
            center = np.random.uniform(-100, 100, 2)
            center = np.append(center, np.random.uniform(5, 12))
            tree_points = self._generate_tree(center)
            points.extend(tree_points)
        
        return np.array(points)
    
    def _generate_complex_environment(self):
        """Generate complex urban-like environment"""
        points = []
        
        # Complex ground with multiple materials
        x_ground = np.random.uniform(-200, 200, 15000)
        y_ground = np.random.uniform(-200, 200, 15000)
        z_ground = (self.config['ground_height'] + 
                   0.3 * np.sin(x_ground * 0.05) * np.cos(y_ground * 0.08) +
                   0.1 * np.random.normal(0, 1, len(x_ground)))
        intensity_ground = np.random.uniform(0.1, 0.9, 15000)
        
        points.extend(zip(x_ground, y_ground, z_ground, intensity_ground))
        
        # Urban infrastructure
        # Buildings with varying heights
        for _ in range(40):
            center = np.random.uniform(-120, 120, 2)
            center = np.append(center, np.random.uniform(8, 40))
            size = [np.random.uniform(8, 25), np.random.uniform(8, 25), 
                   np.random.uniform(15, 80)]
            building_points = self._generate_building(center, size)
            points.extend(building_points)
        
        # Power lines
        for _ in range(10):
            powerline_points = self._generate_powerline()
            points.extend(powerline_points)
        
        # Vehicles and objects
        for _ in range(30):
            obj_points = self._generate_random_object()
            points.extend(obj_points)
        
        return np.array(points)
    
    def _generate_box(self, center, size):
        """Generate points for a box obstacle"""
        points = []
        n_points = 200
        
        # Generate points on box surfaces
        for face in range(6):
            if face == 0:  # Bottom
                x = np.random.uniform(center[0] - size[0]/2, center[0] + size[0]/2, n_points//6)
                y = np.random.uniform(center[1] - size[1]/2, center[1] + size[1]/2, n_points//6)
                z = np.full_like(x, center[2] - size[2]/2)
            elif face == 1:  # Top
                x = np.random.uniform(center[0] - size[0]/2, center[0] + size[0]/2, n_points//6)
                y = np.random.uniform(center[1] - size[1]/2, center[1] + size[1]/2, n_points//6)
                z = np.full_like(x, center[2] + size[2]/2)
            # Add other faces...
            
            intensity = np.random.uniform(0.4, 0.9, len(x))
            points.extend(zip(x, y, z, intensity))
        
        return points
    
    def _generate_building(self, center, size):
        """Generate points for a building"""
        points = []
        density = 50  # points per surface unit
        
        # Walls
        for wall in range(4):
            if wall == 0:  # Front wall
                x = np.full(density, center[0] + size[0]/2)
                y = np.random.uniform(center[1] - size[1]/2, center[1] + size[1]/2, density)
                z = np.random.uniform(center[2] - size[2]/2, center[2] + size[2]/2, density)
            elif wall == 1:  # Back wall
                x = np.full(density, center[0] - size[0]/2)
                y = np.random.uniform(center[1] - size[1]/2, center[1] + size[1]/2, density)
                z = np.random.uniform(center[2] - size[2]/2, center[2] + size[2]/2, density)
            # Add other walls...
            
            intensity = np.random.uniform(0.3, 0.8, len(x))
            points.extend(zip(x, y, z, intensity))
        
        # Roof
        x = np.random.uniform(center[0] - size[0]/2, center[0] + size[0]/2, density)
        y = np.random.uniform(center[1] - size[1]/2, center[1] + size[1]/2, density)
        z = np.full_like(x, center[2] + size[2]/2)
        intensity = np.random.uniform(0.2, 0.6, len(x))
        points.extend(zip(x, y, z, intensity))
        
        return points
    
    def _generate_tree(self, center):
        """Generate points for a tree (trunk + foliage)"""
        points = []
        
        # Trunk
        trunk_height = center[2] * 0.6
        trunk_radius = 0.3
        n_trunk = 50
        
        theta = np.random.uniform(0, 2*np.pi, n_trunk)
        r = np.random.uniform(0, trunk_radius, n_trunk)
        x = center[0] + r * np.cos(theta)
        y = center[1] + r * np.sin(theta)
        z = np.random.uniform(0, trunk_height, n_trunk)
        intensity = np.random.uniform(0.2, 0.4, n_trunk)
        points.extend(zip(x, y, z, intensity))
        
        # Foliage (sphere)
        foliage_radius = np.random.uniform(2, 4)
        n_foliage = 300
        
        # Random points in sphere
        phi = np.random.uniform(0, 2*np.pi, n_foliage)
        costheta = np.random.uniform(-1, 1, n_foliage)
        u = np.random.uniform(0, 1, n_foliage)
        
        theta = np.arccos(costheta)
        r = foliage_radius * np.power(u, 1/3)
        
        x = center[0] + r * np.sin(theta) * np.cos(phi)
        y = center[1] + r * np.sin(theta) * np.sin(phi)
        z = center[2] + r * np.cos(theta)
        intensity = np.random.uniform(0.5, 0.9, n_foliage)
        points.extend(zip(x, y, z, intensity))
        
        return points
    
    def _generate_powerline(self):
        """Generate power line points"""
        points = []
        start = np.random.uniform(-100, 100, 2)
        end = np.random.uniform(-100, 100, 2)
        height = np.random.uniform(15, 25)
        
        # Line between poles with sag
        n_points = 100
        t = np.linspace(0, 1, n_points)
        x = start[0] + t * (end[0] - start[0])
        y = start[1] + t * (end[1] - start[1])
        sag = 2 * t * (1 - t)  # Parabolic sag
        z = height - sag
        
        intensity = np.random.uniform(0.1, 0.3, n_points)
        points.extend(zip(x, y, z, intensity))
        
        return points
    
    def _generate_random_object(self):
        """Generate random objects (cars, signs, etc.)"""
        points = []
        center = np.random.uniform(-80, 80, 3)
        center[2] = np.random.uniform(1, 3)
        
        # Simple rectangular object
        size = np.random.uniform(2, 6, 3)
        n_points = 100
        
        # Random points on object surface
        for _ in range(n_points):
            # Random face
            face = np.random.randint(0, 6)
            if face < 2:  # Top/bottom
                x = np.random.uniform(center[0] - size[0]/2, center[0] + size[0]/2)
                y = np.random.uniform(center[1] - size[1]/2, center[1] + size[1]/2)
                z = center[2] + (size[2]/2 if face == 1 else -size[2]/2)
            # ... other faces
            else:
                x, y, z = center  # Simplified
            
            intensity = np.random.uniform(0.3, 0.8)
            points.append((x, y, z, intensity))
        
        return points
    
    def scan_environment(self, environment, sensor_pose):
        """
        Simulate LiDAR scanning of environment from sensor pose
        
        Args:
            environment (np.array): Environment point cloud
            sensor_pose (dict): Sensor position and orientation
        """
        # Transform environment to sensor coordinate system
        sensor_pos = sensor_pose['position']
        sensor_rot = sensor_pose['orientation']
        
        # Create transformation matrix
        R_matrix = R.from_euler('xyz', sensor_rot).as_matrix()
        
        # Translate environment relative to sensor
        env_translated = environment[:, :3] - sensor_pos
        
        # Rotate to sensor frame
        env_rotated = (R_matrix.T @ env_translated.T).T
        
        # Convert to spherical coordinates
        x, y, z = env_rotated[:, 0], env_rotated[:, 1], env_rotated[:, 2]
        ranges = np.sqrt(x**2 + y**2 + z**2)
        
        # Horizontal angle (azimuth)
        azimuth = np.arctan2(y, x) * 180 / np.pi
        
        # Vertical angle (elevation)
        elevation = np.arcsin(z / ranges) * 180 / np.pi
        
        # Filter by FOV and range
        fov_h = self.config['fov_horizontal'] / 2
        fov_v = self.config['fov_vertical'] / 2
        
        mask = ((np.abs(azimuth) <= fov_h) & 
                (np.abs(elevation) <= fov_v) &
                (ranges >= self.config['range_min']) & 
                (ranges <= self.config['range_max']))
        
        visible_points = env_rotated[mask]
        visible_intensity = environment[mask, 3]
        
        # Subsample to realistic point count
        if len(visible_points) > self.config['points_per_frame']:
            indices = np.random.choice(len(visible_points), 
                                     self.config['points_per_frame'], 
                                     replace=False)
            visible_points = visible_points[indices]
            visible_intensity = visible_intensity[indices]
        
        # Add LiDAR noise
        noise = np.random.normal(0, self.config['lidar_range_noise'], 
                               visible_points.shape)
        visible_points += noise
        
        # Combine points with intensity
        scan_data = np.column_stack([visible_points, visible_intensity])
        
        return scan_data
    
    def transform_pointcloud(self, points, transformation):
        """Apply transformation to point cloud"""
        R_matrix = R.from_euler('xyz', transformation['rotation']).as_matrix()
        transformed = (R_matrix @ points[:, :3].T).T + transformation['translation']
        return np.column_stack([transformed, points[:, 3]])
    
    def run_simulation(self):
        """Run complete simulation"""
        print("Starting LiDAR Motion Compensation Simulation...")
        
        # Generate trajectory
        print("Generating vehicle trajectory...")
        trajectory = self.generate_trajectory()
        trajectory = self.add_sensor_noise(trajectory)
        
        # Generate environment
        print("Generating 3D environment...")
        environment = self.generate_environment_pointcloud()
        
        # Simulation timestamps
        lidar_times = np.linspace(0, self.config['duration'], 
                                int(self.config['duration'] * self.config['lidar_fps']))
        
        # Results storage
        all_scans = []
        aligned_pointclouds = []
        motion_data = []
        
        print(f"Simulating {len(lidar_times)} LiDAR frames...")
        
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
            
            # Simulate LiDAR scan
            scan = self.scan_environment(environment, sensor_pose)
            
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
                'gps_lat': sensor_pose['position'][1] / 111320,  # Rough conversion to lat
                'gps_lon': sensor_pose['position'][0] / 111320,  # Rough conversion to lon
                'gps_alt': sensor_pose['position'][2],
                'imu_roll': sensor_pose['orientation'][0],
                'imu_pitch': sensor_pose['orientation'][1],
                'imu_yaw': sensor_pose['orientation'][2],
                'vel_x': sensor_pose['velocity'][0],
                'vel_y': sensor_pose['velocity'][1],
                'vel_z': sensor_pose['velocity'][2]
            })
            
            if (i + 1) % 10 == 0:
                print(f"Processed frame {i + 1}/{len(lidar_times)}")
        
        return {
            'raw_scans': all_scans,
            'aligned_pointclouds': aligned_pointclouds,
            'motion_data': motion_data,
            'trajectory': trajectory,
            'environment': environment
        }
    
    def save_results(self, results, output_dir='lidar_simulation_output'):
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
            filename = os.path.join(pcd_dir, f'frame_{scan["frame_id"]:04d}.pcd')
            self.save_pcd(scan['points_local'], filename)
        
        # Save aligned point clouds
        aligned_dir = os.path.join(output_dir, 'aligned_scans_pcd')
        os.makedirs(aligned_dir, exist_ok=True)
        
        for i, aligned_pc in enumerate(results['aligned_pointclouds']):
            filename = os.path.join(aligned_dir, f'aligned_frame_{i:04d}.pcd')
            self.save_pcd(aligned_pc, filename)
        
        # Save merged aligned point cloud
        merged_aligned = np.vstack(results['aligned_pointclouds'])
        self.save_pcd(merged_aligned, os.path.join(output_dir, 'merged_aligned.pcd'))
        
        # Save merged raw point cloud (for comparison)
        merged_raw = np.vstack([scan['points_local'] for scan in results['raw_scans']])
        self.save_pcd(merged_raw, os.path.join(output_dir, 'merged_raw_overlapped.pcd'))
        
        # Save as LAS format if laspy is available
        try:
            self.save_las(merged_aligned, os.path.join(output_dir, 'merged_aligned.las'))
            print("LAS format saved successfully")
        except Exception as e:
            print(f"Could not save LAS format: {e}")
        
        # Save trajectory for visualization
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
        # Create LAS file
        header = laspy.LasHeader(point_format=3, version="1.2")
        header.add_extra_dim(laspy.ExtraBytesParams(name="intensity", type=np.float32))
        
        las = laspy.LasData(header)
        
        # Scale coordinates to fit LAS integer requirements
        las.x = points[:, 0]
        las.y = points[:, 1] 
        las.z = points[:, 2]
        las.intensity = (points[:, 3] * 65535).astype(np.uint16)  # Scale to 16-bit
        
        las.write(filename)
    
    def visualize_results(self, results, output_dir):
        """Create visualizations of simulation results"""
        print("Creating visualizations...")
        
        # 1. Trajectory plot
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 12))
        
        # 2D trajectory
        traj = results['trajectory']
        ax1.plot(traj['position'][:, 0], traj['position'][:, 1], 'b-', label='True trajectory', linewidth=2)
        ax1.plot(traj['position_gps'][:, 0], traj['position_gps'][:, 1], 'r--', label='GPS trajectory', alpha=0.7)
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title('Vehicle Trajectory (Top View)')
        ax1.legend()
        ax1.grid(True)
        ax1.axis('equal')
        
        # Velocity profile
        velocities = np.sqrt(np.sum(traj['velocity']**2, axis=1))
        ax2.plot(traj['time'], velocities, 'g-', linewidth=2)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Speed (m/s)')
        ax2.set_title('Vehicle Speed Profile')
        ax2.grid(True)
        
        # Orientation angles
        angles = traj['orientation'] * 180 / np.pi
        ax3.plot(traj['time'], angles[:, 0], 'r-', label='Roll', linewidth=2)
        ax3.plot(traj['time'], angles[:, 1], 'g-', label='Pitch', linewidth=2)
        ax3.plot(traj['time'], angles[:, 2], 'b-', label='Yaw', linewidth=2)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Angle (degrees)')
        ax3.set_title('Vehicle Orientation')
        ax3.legend()
        ax3.grid(True)
        
        # Point cloud statistics
        frame_sizes = [len(scan['points_local']) for scan in results['raw_scans']]
        ax4.bar(range(len(frame_sizes)), frame_sizes)
        ax4.set_xlabel('Frame Number')
        ax4.set_ylabel('Points per Frame')
        ax4.set_title('LiDAR Frame Point Counts')
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, 'simulation_overview.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        # 2. 3D trajectory visualization
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        ax.plot(traj['position'][:, 0], traj['position'][:, 1], traj['position'][:, 2], 
                'b-', label='True trajectory', linewidth=3)
        ax.plot(traj['position_gps'][:, 0], traj['position_gps'][:, 1], traj['position_gps'][:, 2], 
                'r--', label='GPS trajectory', linewidth=2, alpha=0.7)
        
        # Add start and end markers
        ax.scatter(traj['position'][0, 0], traj['position'][0, 1], traj['position'][0, 2], 
                  c='green', s=100, label='Start', marker='o')
        ax.scatter(traj['position'][-1, 0], traj['position'][-1, 1], traj['position'][-1, 2], 
                  c='red', s=100, label='End', marker='s')
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('3D Vehicle Trajectory')
        ax.legend()
        
        plt.savefig(os.path.join(output_dir, '3d_trajectory.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        # 3. Point cloud comparison (before/after alignment)
        if len(results['raw_scans']) > 0:
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
            
            # Sample a few frames for visualization
            sample_frames = min(5, len(results['raw_scans']))
            colors = plt.cm.Set1(np.linspace(0, 1, sample_frames))
            
            for i in range(sample_frames):
                idx = i * len(results['raw_scans']) // sample_frames
                
                # Raw overlapped scans
                raw_points = results['raw_scans'][idx]['points_local']
                ax1.scatter(raw_points[:, 0], raw_points[:, 1], c=[colors[i]], 
                           s=0.1, alpha=0.6, label=f'Frame {idx}')
                
                # Aligned scans
                aligned_points = results['aligned_pointclouds'][idx]
                ax2.scatter(aligned_points[:, 0], aligned_points[:, 1], c=[colors[i]], 
                           s=0.1, alpha=0.6, label=f'Frame {idx}')
            
            ax1.set_xlabel('X (m)')
            ax1.set_ylabel('Y (m)')
            ax1.set_title('Raw LiDAR Scans (Overlapped at Origin)')
            ax1.legend()
            ax1.grid(True)
            ax1.axis('equal')
            
            ax2.set_xlabel('X (m)')
            ax2.set_ylabel('Y (m)')
            ax2.set_title('Motion Compensated Scans (Globally Aligned)')
            ax2.legend()
            ax2.grid(True)
            ax2.axis('equal')
            
            plt.tight_layout()
            plt.savefig(os.path.join(output_dir, 'pointcloud_comparison.png'), dpi=300, bbox_inches='tight')
            plt.close()
        
        print("Visualizations saved!")
    
    def generate_report(self, results, output_dir):
        """Generate simulation report"""
        report_file = os.path.join(output_dir, 'simulation_report.txt')
        
        with open(report_file, 'w') as f:
            f.write("LiDAR Motion Compensation Simulation Report\n")
            f.write("=" * 50 + "\n\n")
            
            f.write("SIMULATION CONFIGURATION:\n")
            f.write("-" * 25 + "\n")
            for key, value in self.config.items():
                f.write(f"{key}: {value}\n")
            
            f.write("\nSIMULATION RESULTS:\n")
            f.write("-" * 19 + "\n")
            f.write(f"Total frames processed: {len(results['raw_scans'])}\n")
            f.write(f"Total simulation time: {self.config['duration']} seconds\n")
            f.write(f"Frame rate: {self.config['lidar_fps']} Hz\n")
            
            # Point cloud statistics
            total_raw_points = sum(len(scan['points_local']) for scan in results['raw_scans'])
            total_aligned_points = sum(len(pc) for pc in results['aligned_pointclouds'])
            
            f.write(f"Total points (raw): {total_raw_points:,}\n")
            f.write(f"Total points (aligned): {total_aligned_points:,}\n")
            f.write(f"Average points per frame: {total_raw_points // len(results['raw_scans']):,}\n")
            
            # Trajectory statistics
            traj = results['trajectory']
            distances = np.sqrt(np.sum(np.diff(traj['position'], axis=0)**2, axis=1))
            total_distance = np.sum(distances)
            max_speed = np.max(np.sqrt(np.sum(traj['velocity']**2, axis=1)))
            
            f.write(f"Total distance traveled: {total_distance:.1f} m\n")
            f.write(f"Maximum speed: {max_speed:.1f} m/s\n")
            
            # GPS accuracy simulation
            gps_errors = np.sqrt(np.sum((traj['position'] - traj['position_gps'])**2, axis=1))
            f.write(f"GPS RMS error: {np.sqrt(np.mean(gps_errors**2)):.3f} m\n")
            f.write(f"GPS max error: {np.max(gps_errors):.3f} m\n")
            
            f.write("\nFILES GENERATED:\n")
            f.write("-" * 16 + "\n")
            f.write("• motion_data.csv - Motion sensor data for each frame\n")
            f.write("• trajectory.csv - Complete vehicle trajectory\n")
            f.write("• raw_scans_pcd/ - Individual LiDAR frames (overlapped)\n")
            f.write("• aligned_scans_pcd/ - Motion compensated frames\n")
            f.write("• merged_aligned.pcd - Combined aligned point cloud\n")
            f.write("• merged_raw_overlapped.pcd - Combined raw point cloud\n")
            f.write("• merged_aligned.las - LAS format output\n")
            f.write("• simulation_overview.png - Analysis plots\n")
            f.write("• 3d_trajectory.png - 3D trajectory visualization\n")
            f.write("• pointcloud_comparison.png - Before/after comparison\n")
            
            f.write("\nUSAGE INSTRUCTIONS:\n")
            f.write("-" * 19 + "\n")
            f.write("1. Load motion_data.csv to see sensor synchronization format\n")
            f.write("2. Compare merged_raw_overlapped.pcd vs merged_aligned.pcd\n")
            f.write("3. Use aligned point clouds for volume estimation and analysis\n")
            f.write("4. Adapt motion compensation code for real sensor integration\n")
        
        print(f"Simulation report saved to {report_file}")


def main():
    """Main simulation execution"""
    print("LiDAR Motion Compensation Simulation")
    print("=" * 40)
    
    # Custom configuration for different scenarios
    configs = {
        'urban_complex': {
            'duration': 120.0,
            'trajectory_type': 'figure_eight',
            'environment_complexity': 'complex',
            'max_speed': 12.0,
            'lidar_fps': 10
        },
        'highway_simple': {
            'duration': 60.0,
            'trajectory_type': 'linear',
            'environment_complexity': 'simple',
            'max_speed': 25.0,
            'lidar_fps': 15
        },
        'parking_detailed': {
            'duration': 30.0,
            'trajectory_type': 'circular',
            'environment_complexity': 'medium',
            'max_speed': 5.0,
            'lidar_fps': 20
        }
    }
    
    # Select configuration
    config_name = 'urban_complex'  # Change this to test different scenarios
    print(f"Using configuration: {config_name}")
    
    # Initialize simulator
    simulator = LiDARMotionSimulator(configs[config_name])
    
    # Run simulation
    results = simulator.run_simulation()
    
    # Save results
    output_dir = simulator.save_results(results)
    
    # Generate visualizations
    simulator.visualize_results(results, output_dir)
    
    # Generate report
    simulator.generate_report(results, output_dir)
    
    print("\n" + "=" * 50)
    print("SIMULATION COMPLETE!")
    print(f"Results saved to: {output_dir}")
    print("\nKey findings:")
    print(f"• {len(results['raw_scans'])} LiDAR frames processed")
    print(f"• Motion compensation applied using GPS/IMU data")
    print(f"• Raw overlapped cloud vs aligned cloud comparison available")
    print(f"• Ready for integration with real Livox Mid-70 setup")


if __name__ == "__main__":
    main()
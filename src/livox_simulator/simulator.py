"""
Main simulation class for Livox Mid-70 LiDAR systems
"""

import time
import numpy as np
import psutil
from typing import Dict, List, Any, Optional, Callable
from tqdm import tqdm

from .data_structures import LiDARFrame, IMUData, TrajectoryPoint, SimulationConfig
from .coordinates import CoordinateTransformer
from .motion import MotionCompensator
from .export import DataExporter


class LiDARMotionSimulator:
    """Main simulation class for Livox Mid-70 LiDAR systems with motion compensation."""
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize LiDAR motion simulator.
        
        Args:
            config: Configuration dictionary containing simulation parameters
        """
        # Convert dict config to SimulationConfig object
        self.config = self._parse_config(config)
        
        # Set random seed for reproducibility
        np.random.seed(self.config.random_seed)
        
        # Initialize components
        self.coordinate_transformer = None
        self.motion_compensator = None
        self.data_exporter = DataExporter(self.config.output_directory)
        
        # Performance monitoring
        self.performance_callback = None
        self.start_time = None
        self.performance_metrics = {
            'processing_time': {},
            'memory_usage': {},
            'accuracy_metrics': {},
            'quality_metrics': {}
        }
        
        # Initialize subsystems
        self._initialize_subsystems()
    
    def _parse_config(self, config: Dict[str, Any]) -> SimulationConfig:
        """Parse configuration dictionary into SimulationConfig object."""
        # Create SimulationConfig with defaults, then update with provided values
        sim_config = SimulationConfig()
        
        # Update with provided configuration
        for key, value in config.items():
            if hasattr(sim_config, key):
                setattr(sim_config, key, value)
        
        return sim_config
    
    def _initialize_subsystems(self):
        """Initialize coordinate transformer and motion compensator."""
        # Initialize coordinate transformer with default calibration
        calibration_params = {
            'lidar_to_vehicle': {
                'translation': [0.0, 0.0, 1.5],  # 1.5m above ground
                'rotation': [0.0, 0.0, 0.0]      # No rotation
            }
        }
        self.coordinate_transformer = CoordinateTransformer(calibration_params)
        
        # Initialize motion compensator
        imu_config = {
            'update_rate': self.config.imu_update_rate,
            'gyro_noise': self.config.imu_gyro_noise,
            'accel_noise': self.config.imu_accel_noise,
            'frame_rate': self.config.frame_rate
        }
        self.motion_compensator = MotionCompensator(imu_config)
    
    def run_simulation(self) -> Dict[str, Any]:
        """
        Execute the complete simulation pipeline.
        
        Returns:
            Dictionary containing simulation results
        """
        print("Starting Livox Mid-70 motion compensation simulation...")
        self.start_time = time.time()
        
        try:
            # Generate trajectory
            print("Generating vehicle trajectory...")
            trajectory = self._generate_trajectory()
            
            # Generate environment
            print("Creating simulation environment...")
            environment = self._generate_environment()
            
            # Generate LiDAR frames with IMU data
            print("Simulating LiDAR scanning...")
            frames, imu_data = self._simulate_lidar_scanning(trajectory, environment)
            
            # Apply motion compensation
            if self.config.enable_motion_compensation:
                print("Applying motion compensation...")
                frames = self._apply_motion_compensation(frames, imu_data)
            
            # Transform to global coordinates
            print("Transforming to global coordinates...")
            global_point_cloud = self._transform_to_global_coordinates(frames, trajectory)
            
            # Calculate performance metrics
            processing_time = time.time() - self.start_time
            self._update_performance_metrics(frames, processing_time)
            
            # Prepare results
            results = {
                'frames': frames,
                'trajectory': trajectory,
                'imu_data': imu_data,
                'point_cloud': global_point_cloud,
                'processing_time': processing_time,
                'performance_metrics': self.performance_metrics,
                'config': self.config.__dict__,
                'metadata': self._generate_metadata()
            }
            
            # Export data
            print("Exporting simulation data...")
            export_results = self.data_exporter.export_all_formats(results)
            results['export_results'] = export_results
            
            print(f"Simulation completed in {processing_time:.2f} seconds")
            print(f"Generated {len(frames)} frames with {len(global_point_cloud)} total points")
            
            return results
            
        except Exception as e:
            print(f"Simulation failed: {e}")
            raise
    
    def _generate_trajectory(self) -> List[TrajectoryPoint]:
        """Generate vehicle trajectory based on configuration."""
        trajectory = []
        
        # Calculate number of trajectory points
        num_points = int(self.config.duration * 10)  # 10Hz trajectory sampling
        
        # Generate trajectory based on type
        if self.config.trajectory_type == 'linear':
            trajectory = self._generate_linear_trajectory(num_points)
        elif self.config.trajectory_type == 'circular':
            trajectory = self._generate_circular_trajectory(num_points)
        elif self.config.trajectory_type == 'figure_eight':
            trajectory = self._generate_figure_eight_trajectory(num_points)
        elif self.config.trajectory_type == 'urban_circuit':
            trajectory = self._generate_urban_circuit_trajectory(num_points)
        else:
            # Default to linear trajectory
            trajectory = self._generate_linear_trajectory(num_points)
        
        return trajectory
    
    def _generate_linear_trajectory(self, num_points: int) -> List[TrajectoryPoint]:
        """Generate linear trajectory with gentle curves."""
        trajectory = []
        
        for i in range(num_points):
            t = i / (num_points - 1) * self.config.duration
            
            # Linear motion with gentle curves
            x = self.config.max_speed * t
            y = 5 * np.sin(0.1 * x)  # Gentle sinusoidal variation
            z = 1.5  # Constant height
            
            # Calculate velocity and acceleration
            vx = self.config.max_speed
            vy = 0.5 * np.cos(0.1 * x)
            vz = 0.0
            
            # Simple orientation calculation
            yaw = np.arctan2(vy, vx)
            
            trajectory_point = TrajectoryPoint(
                timestamp=t,
                position=np.array([x, y, z]),
                orientation=np.array([0.0, 0.0, yaw]),
                velocity=np.array([vx, vy, vz]),
                acceleration=np.array([0.0, -0.05 * np.sin(0.1 * x), 0.0])
            )
            trajectory.append(trajectory_point)
        
        return trajectory
    
    def _generate_circular_trajectory(self, num_points: int) -> List[TrajectoryPoint]:
        """Generate circular trajectory."""
        trajectory = []
        radius = 50.0  # 50m radius
        
        for i in range(num_points):
            t = i / (num_points - 1) * self.config.duration
            angle = 2 * np.pi * t / self.config.duration
            
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            z = 1.5
            
            # Tangential velocity
            vx = -radius * np.sin(angle) * 2 * np.pi / self.config.duration
            vy = radius * np.cos(angle) * 2 * np.pi / self.config.duration
            vz = 0.0
            
            # Centripetal acceleration
            ax = -radius * np.cos(angle) * (2 * np.pi / self.config.duration)**2
            ay = -radius * np.sin(angle) * (2 * np.pi / self.config.duration)**2
            az = 0.0
            
            yaw = angle + np.pi/2  # Tangent direction
            
            trajectory_point = TrajectoryPoint(
                timestamp=t,
                position=np.array([x, y, z]),
                orientation=np.array([0.0, 0.0, yaw]),
                velocity=np.array([vx, vy, vz]),
                acceleration=np.array([ax, ay, az])
            )
            trajectory.append(trajectory_point)
        
        return trajectory
    
    def _generate_figure_eight_trajectory(self, num_points: int) -> List[TrajectoryPoint]:
        """Generate figure-8 trajectory."""
        trajectory = []
        scale = 30.0
        
        for i in range(num_points):
            t = i / (num_points - 1) * self.config.duration
            angle = 4 * np.pi * t / self.config.duration
            
            x = scale * np.sin(angle)
            y = scale * np.sin(angle) * np.cos(angle)
            z = 1.5
            
            # Calculate derivatives for velocity
            vx = scale * np.cos(angle) * 4 * np.pi / self.config.duration
            vy = scale * (np.cos(2*angle)) * 4 * np.pi / self.config.duration
            vz = 0.0
            
            yaw = np.arctan2(vy, vx)
            
            trajectory_point = TrajectoryPoint(
                timestamp=t,
                position=np.array([x, y, z]),
                orientation=np.array([0.0, 0.0, yaw]),
                velocity=np.array([vx, vy, vz]),
                acceleration=np.array([0.0, 0.0, 0.0])  # Simplified
            )
            trajectory.append(trajectory_point)
        
        return trajectory
    
    def _generate_urban_circuit_trajectory(self, num_points: int) -> List[TrajectoryPoint]:
        """Generate realistic urban driving trajectory."""
        # This is a simplified urban circuit - could be expanded with more complex patterns
        return self._generate_linear_trajectory(num_points)
    
    def _generate_environment(self) -> Dict[str, Any]:
        """Generate simulation environment based on complexity setting."""
        environment = {
            'complexity': self.config.environment_complexity,
            'objects': [],
            'ground_plane': {'height': 0.0, 'material': 'asphalt'}
        }
        
        # Generate objects based on complexity
        if self.config.environment_complexity == 'urban':
            # Add buildings, trees, vehicles
            num_buildings = int(20 * self.config.building_density)
            for i in range(num_buildings):
                building = {
                    'type': 'building',
                    'position': [
                        np.random.uniform(-100, 100),
                        np.random.uniform(-100, 100),
                        0
                    ],
                    'size': [
                        np.random.uniform(10, 30),
                        np.random.uniform(10, 30),
                        np.random.uniform(5, 20)
                    ]
                }
                environment['objects'].append(building)
        
        return environment
    
    def _simulate_lidar_scanning(self, trajectory: List[TrajectoryPoint], environment: Dict[str, Any]) -> tuple:
        """Simulate LiDAR scanning process."""
        frames = []
        all_imu_data = []
        
        # Calculate frame times
        frame_duration = 1.0 / self.config.frame_rate
        num_frames = int(self.config.duration * self.config.frame_rate)
        
        for frame_id in tqdm(range(num_frames), desc="Generating LiDAR frames"):
            frame_time = frame_id * frame_duration
            
            # Get trajectory point for this frame
            traj_idx = min(int(frame_time * 10), len(trajectory) - 1)  # 10Hz trajectory
            traj_point = trajectory[traj_idx]
            
            # Generate IMU data for this frame
            frame_imu_data = self._generate_imu_data(frame_time, frame_duration, traj_point)
            all_imu_data.extend(frame_imu_data)
            
            # Generate point cloud for this frame
            points = self._generate_frame_points(traj_point, environment)
            
            # Create LiDAR frame
            frame = LiDARFrame(
                timestamp=frame_time,
                points=points,
                frame_id=frame_id,
                device_info={'serial': '3GGDJ6K00200101', 'model': 'Mid-70'},
                quality_metrics={'point_count': len(points)}
            )
            frames.append(frame)
            
            # Performance callback
            if self.performance_callback:
                metrics = {
                    'frame_id': frame_id,
                    'timestamp': frame_time,
                    'points_generated': len(points),
                    'memory_mb': psutil.Process().memory_info().rss / 1024 / 1024
                }
                self.performance_callback(metrics)
        
        return frames, all_imu_data
    
    def _generate_imu_data(self, frame_time: float, frame_duration: float, traj_point: TrajectoryPoint) -> List[IMUData]:
        """Generate IMU data for a frame."""
        imu_data = []
        imu_samples_per_frame = int(self.config.imu_update_rate * frame_duration)
        
        for i in range(imu_samples_per_frame):
            sample_time = frame_time + i * frame_duration / imu_samples_per_frame
            
            # Extract angular velocity and acceleration from trajectory
            angular_vel = np.array([0.0, 0.0, 0.0])  # Simplified - could derive from trajectory
            if len(traj_point.velocity) >= 3:
                # Estimate yaw rate from velocity change
                angular_vel[2] = 0.1 * np.random.randn()  # Simplified noise
            
            # Add noise to IMU measurements
            gyro_noise = np.random.normal(0, self.config.imu_gyro_noise, 3)
            accel_noise = np.random.normal(0, self.config.imu_accel_noise, 3)
            
            imu_sample = IMUData(
                timestamp=sample_time,
                gyro_x=angular_vel[0] + gyro_noise[0],
                gyro_y=angular_vel[1] + gyro_noise[1],
                gyro_z=angular_vel[2] + gyro_noise[2],
                accel_x=traj_point.acceleration[0] + accel_noise[0],
                accel_y=traj_point.acceleration[1] + accel_noise[1],
                accel_z=traj_point.acceleration[2] + 9.81 + accel_noise[2]  # Add gravity
            )
            imu_data.append(imu_sample)
        
        return imu_data
    
    def _generate_frame_points(self, traj_point: TrajectoryPoint, environment: Dict[str, Any]) -> np.ndarray:
        """Generate point cloud for a single frame."""
        # Calculate points per frame
        points_per_frame = int(self.config.points_per_second / self.config.frame_rate)
        
        # Generate points in spherical coordinates (Livox Mid-70 pattern)
        points = []
        
        for i in range(points_per_frame):
            # Generate random point within FOV
            # Livox Mid-70 has circular FOV
            theta = np.random.uniform(0, 2 * np.pi)  # Azimuth
            phi = np.random.uniform(0, np.radians(self.config.fov_horizontal))  # Elevation from center
            
            # Random range within sensor limits
            range_val = np.random.uniform(self.config.range_min, self.config.range_max)
            
            # Convert to Cartesian coordinates (sensor frame)
            x = range_val * np.cos(phi) * np.cos(theta)
            y = range_val * np.cos(phi) * np.sin(theta)
            z = range_val * np.sin(phi)
            
            # Add measurement noise
            noise = np.random.normal(0, self.config.point_accuracy, 3)
            point = np.array([x, y, z]) + noise
            
            # Generate intensity (simplified)
            intensity = np.random.randint(50, 255)
            
            points.append([point[0], point[1], point[2], intensity])
        
        return np.array(points)
    
    def _apply_motion_compensation(self, frames: List[LiDARFrame], imu_data: List[IMUData]) -> List[LiDARFrame]:
        """Apply motion compensation to all frames."""
        compensated_frames = []
        
        for frame in tqdm(frames, desc="Applying motion compensation"):
            # Get IMU data for this frame
            frame_start = frame.timestamp
            frame_end = frame.timestamp + 1.0 / self.config.frame_rate
            
            frame_imu_data = [
                imu for imu in imu_data 
                if frame_start <= imu.timestamp <= frame_end
            ]
            
            # Apply compensation
            compensated_points = self.motion_compensator.compensate_frame(
                frame.points, frame_imu_data, frame.timestamp
            )
            
            # Create new frame with compensated points
            compensated_frame = LiDARFrame(
                timestamp=frame.timestamp,
                points=compensated_points,
                frame_id=frame.frame_id,
                device_info=frame.device_info,
                quality_metrics=frame.quality_metrics
            )
            compensated_frames.append(compensated_frame)
        
        return compensated_frames
    
    def _transform_to_global_coordinates(self, frames: List[LiDARFrame], trajectory: List[TrajectoryPoint]) -> np.ndarray:
        """Transform all points to global coordinate system."""
        all_points = []
        
        for frame in frames:
            # Find corresponding trajectory point
            traj_idx = min(int(frame.timestamp * 10), len(trajectory) - 1)
            traj_point = trajectory[traj_idx]
            
            # Transform points from sensor to global coordinates
            if self.config.coordinate_system == 'utm':
                # For simulation, use local coordinates that could be converted to UTM
                global_points = self.coordinate_transformer.transform_points(
                    frame.points[:, :3], 'sensor', 'vehicle'
                )
                
                # Add trajectory position offset
                global_points += traj_point.position
                
                # Preserve intensity
                if frame.points.shape[1] > 3:
                    global_points = np.column_stack([global_points, frame.points[:, 3:]])
                
                all_points.extend(global_points)
            else:
                # Keep in sensor coordinates
                all_points.extend(frame.points)
        
        return np.array(all_points)
    
    def _update_performance_metrics(self, frames: List[LiDARFrame], processing_time: float):
        """Update performance metrics."""
        total_points = sum(len(frame.points) for frame in frames)
        
        self.performance_metrics['processing_time'] = {
            'total_seconds': processing_time,
            'points_per_second': total_points / processing_time if processing_time > 0 else 0,
            'frames_per_second': len(frames) / processing_time if processing_time > 0 else 0
        }
        
        # Memory usage
        process = psutil.Process()
        memory_info = process.memory_info()
        self.performance_metrics['memory_usage'] = {
            'peak_mb': memory_info.rss / 1024 / 1024,
            'average_mb': memory_info.rss / 1024 / 1024  # Simplified
        }
        
        # Motion compensation statistics
        if self.motion_compensator:
            comp_stats = self.motion_compensator.get_compensation_statistics()
            self.performance_metrics['accuracy_metrics'] = {
                'motion_compensation_error': comp_stats.get('average_compensation_error', 0.0),
                'frames_processed': comp_stats.get('frames_processed', 0)
            }
    
    def _generate_metadata(self) -> Dict[str, Any]:
        """Generate simulation metadata."""
        return {
            'simulator_version': '1.0.0',
            'livox_model': 'Mid-70',
            'simulation_date': time.strftime('%Y-%m-%d %H:%M:%S'),
            'coordinate_system': self.config.coordinate_system,
            'motion_compensation_enabled': self.config.enable_motion_compensation
        }
    
    def set_performance_callback(self, callback: Callable):
        """Set callback function for real-time performance monitoring."""
        self.performance_callback = callback
    
    def get_system_info(self) -> Dict[str, Any]:
        """Get system information."""
        return {
            'python_version': f"{psutil.sys.version_info.major}.{psutil.sys.version_info.minor}",
            'cpu_count': psutil.cpu_count(),
            'memory_total_gb': psutil.virtual_memory().total / 1024**3,
            'platform': psutil.sys.platform
        }
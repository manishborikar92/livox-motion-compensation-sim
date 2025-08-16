#!/usr/bin/env python3
"""
Livox Mid-70 Motion Compensation Simulation
Main entry point for the simulation framework
"""

import argparse
import sys
import os
import json
import time
from pathlib import Path

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from livox_simulator import LiDARMotionSimulator


def load_config(config_path: str = None) -> dict:
    """Load configuration from file or return default configuration."""
    
    default_config = {
        # Simulation parameters
        'duration': 60.0,
        'random_seed': 42,
        'output_directory': './lidar_simulation_output',
        
        # Livox Mid-70 specifications
        'fov_horizontal': 70.4,
        'fov_vertical': 77.2,
        'range_max': 90.0,
        'range_min': 0.05,
        'points_per_second': 100000,
        'frame_rate': 10,
        'angular_resolution': 0.28,
        'point_accuracy': 0.02,
        
        # Motion parameters
        'trajectory_type': 'linear',
        'max_speed': 15.0,
        'max_acceleration': 2.0,
        'max_angular_velocity': 0.3,
        
        # Environment settings
        'environment_complexity': 'simple',
        'building_density': 0.4,
        'vegetation_coverage': 0.15,
        'dynamic_objects': True,
        
        # Advanced features
        'enable_motion_compensation': True,
        'coordinate_system': 'utm',
        'lvx_format': 'lvx2',
        
        # IMU configuration
        'imu_update_rate': 200,
        'imu_gyro_noise': 0.01,
        'imu_accel_noise': 0.1
    }
    
    if config_path and os.path.exists(config_path):
        try:
            with open(config_path, 'r') as f:
                if config_path.endswith('.json'):
                    user_config = json.load(f)
                else:
                    # Assume Python file with config dict
                    import importlib.util
                    spec = importlib.util.spec_from_file_location("config", config_path)
                    config_module = importlib.util.module_from_spec(spec)
                    spec.loader.exec_module(config_module)
                    user_config = config_module.config
                
                # Update default config with user config
                default_config.update(user_config)
                print(f"Loaded configuration from {config_path}")
        except Exception as e:
            print(f"Warning: Could not load config file {config_path}: {e}")
            print("Using default configuration")
    
    return default_config


def performance_monitor(metrics: dict):
    """Performance monitoring callback."""
    if metrics['frame_id'] % 10 == 0:  # Print every 10 frames
        print(f"Frame {metrics['frame_id']:4d}: "
              f"{metrics['points_generated']:6d} points, "
              f"Memory: {metrics['memory_mb']:.1f} MB")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Livox Mid-70 Motion Compensation Simulation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python lidar_motion_compensation.py
  python lidar_motion_compensation.py --duration 300 --trajectory circular
  python lidar_motion_compensation.py --config config/custom_config.json
  python lidar_motion_compensation.py --output ./my_simulation --verbose
        """
    )
    
    parser.add_argument('--config', type=str, help='Configuration file path')
    parser.add_argument('--duration', type=float, help='Simulation duration in seconds')
    parser.add_argument('--trajectory', type=str, 
                       choices=['linear', 'circular', 'figure_eight', 'urban_circuit'],
                       help='Trajectory type')
    parser.add_argument('--output', type=str, help='Output directory')
    parser.add_argument('--no-motion-compensation', action='store_true',
                       help='Disable motion compensation')
    parser.add_argument('--coordinate-system', type=str,
                       choices=['sensor', 'vehicle', 'local', 'utm'],
                       help='Output coordinate system')
    parser.add_argument('--environment', type=str,
                       choices=['simple', 'medium', 'urban', 'highway', 'industrial'],
                       help='Environment complexity')
    parser.add_argument('--verbose', action='store_true', help='Verbose output')
    parser.add_argument('--quiet', action='store_true', help='Quiet mode (minimal output)')
    
    args = parser.parse_args()
    
    # Load configuration
    config = load_config(args.config)
    
    # Override config with command line arguments
    if args.duration:
        config['duration'] = args.duration
    if args.trajectory:
        config['trajectory_type'] = args.trajectory
    if args.output:
        config['output_directory'] = args.output
    if args.no_motion_compensation:
        config['enable_motion_compensation'] = False
    if args.coordinate_system:
        config['coordinate_system'] = args.coordinate_system
    if args.environment:
        config['environment_complexity'] = args.environment
    
    # Print configuration summary
    if not args.quiet:
        print("=" * 60)
        print("Livox Mid-70 Motion Compensation Simulation")
        print("=" * 60)
        print(f"Duration: {config['duration']:.1f} seconds")
        print(f"Trajectory: {config['trajectory_type']}")
        print(f"Environment: {config['environment_complexity']}")
        print(f"Motion compensation: {'Enabled' if config['enable_motion_compensation'] else 'Disabled'}")
        print(f"Coordinate system: {config['coordinate_system']}")
        print(f"Output directory: {config['output_directory']}")
        print(f"Points per second: {config['points_per_second']:,}")
        print(f"Frame rate: {config['frame_rate']} Hz")
        print("-" * 60)
    
    try:
        # Create simulator
        simulator = LiDARMotionSimulator(config)
        
        # Set performance callback if verbose
        if args.verbose:
            simulator.set_performance_callback(performance_monitor)
        
        # Run simulation
        start_time = time.time()
        results = simulator.run_simulation()
        total_time = time.time() - start_time
        
        # Print results summary
        if not args.quiet:
            print("\n" + "=" * 60)
            print("SIMULATION RESULTS")
            print("=" * 60)
            print(f"Total processing time: {total_time:.2f} seconds")
            print(f"Frames generated: {len(results['frames'])}")
            print(f"Total points: {len(results['point_cloud']):,}")
            print(f"Points per second: {len(results['point_cloud']) / total_time:,.0f}")
            
            # Performance metrics
            perf = results['performance_metrics']
            print(f"Peak memory usage: {perf['memory_usage']['peak_mb']:.1f} MB")
            
            if config['enable_motion_compensation']:
                acc_metrics = perf.get('accuracy_metrics', {})
                print(f"Motion compensation error: {acc_metrics.get('motion_compensation_error', 0):.4f}")
            
            # Export results
            export_results = results.get('export_results', {})
            successful_exports = sum(1 for result in export_results.values() 
                                   if isinstance(result, dict) and result.get('status') == 'success')
            print(f"Successful exports: {successful_exports}/{len(export_results)}")
            
            print(f"\nOutput files saved to: {config['output_directory']}")
            print("=" * 60)
        
        return 0
        
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
        return 1
    except Exception as e:
        print(f"Simulation failed: {e}")
        if args.verbose:
            import traceback
            traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())
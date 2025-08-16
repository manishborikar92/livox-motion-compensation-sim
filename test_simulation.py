#!/usr/bin/env python3
"""
Quick test script for Livox Mid-70 simulation framework
"""

import sys
import os
import time

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

def test_dependencies():
    """Test that all required dependencies are available."""
    print("Testing Python dependencies...")
    
    try:
        import numpy
        print("✅ NumPy:", numpy.__version__)
    except ImportError:
        print("❌ NumPy not found")
        return False
    
    try:
        import scipy
        print("✅ SciPy:", scipy.__version__)
    except ImportError:
        print("❌ SciPy not found")
        return False
    
    try:
        import matplotlib
        print("✅ Matplotlib:", matplotlib.__version__)
    except ImportError:
        print("❌ Matplotlib not found")
        return False
    
    try:
        import pandas
        print("✅ Pandas:", pandas.__version__)
    except ImportError:
        print("❌ Pandas not found")
        return False
    
    try:
        import open3d
        print("✅ Open3D:", open3d.__version__)
    except ImportError:
        print("❌ Open3D not found")
        return False
    
    try:
        import laspy
        print("✅ Laspy:", laspy.__version__)
    except ImportError:
        print("⚠️  Laspy not found (LAS export will be disabled)")
    
    try:
        import utm
        print("✅ UTM: Available")
    except ImportError:
        print("❌ UTM not found")
        return False
    
    try:
        import yaml
        print("✅ PyYAML: Available")
    except ImportError:
        print("❌ PyYAML not found")
        return False
    
    try:
        import tqdm
        print("✅ tqdm: Available")
    except ImportError:
        print("❌ tqdm not found")
        return False
    
    try:
        import psutil
        print("✅ psutil:", psutil.__version__)
    except ImportError:
        print("❌ psutil not found")
        return False
    
    return True


def test_simulation():
    """Test basic simulation functionality."""
    print("\nTesting simulation framework...")
    
    try:
        from livox_simulator import LiDARMotionSimulator
        print("✅ LiDARMotionSimulator imported successfully")
    except ImportError as e:
        print(f"❌ Failed to import LiDARMotionSimulator: {e}")
        return False
    
    # Test basic simulation
    config = {
        'duration': 5.0,  # Very short test
        'trajectory_type': 'linear',
        'environment_complexity': 'simple',
        'enable_motion_compensation': True,
        'points_per_second': 10000,  # Reduced for speed
        'output_directory': './test_output'
    }
    
    try:
        print("Running 5-second test simulation...")
        start_time = time.time()
        
        simulator = LiDARMotionSimulator(config)
        results = simulator.run_simulation()
        
        end_time = time.time()
        
        print(f"✅ Simulation completed in {end_time - start_time:.2f} seconds")
        print(f"✅ Generated {len(results['frames'])} frames")
        print(f"✅ Total points: {len(results['point_cloud']):,}")
        
        return True
        
    except Exception as e:
        print(f"❌ Simulation failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Main test function."""
    print("=" * 60)
    print("Livox Mid-70 Simulation Framework Test")
    print("=" * 60)
    
    # Test dependencies
    deps_ok = test_dependencies()
    
    if not deps_ok:
        print("\n❌ Dependency test failed. Please install missing packages:")
        print("pip install -r requirements.txt")
        return 1
    
    # Test simulation
    sim_ok = test_simulation()
    
    if not sim_ok:
        print("\n❌ Simulation test failed.")
        return 1
    
    print("\n" + "=" * 60)
    print("✅ ALL TESTS PASSED")
    print("✅ Pure Python Livox Mid-70 simulation framework is ready!")
    print("=" * 60)
    print("\nNext steps:")
    print("1. Run full simulation: python lidar_motion_compensation.py")
    print("2. Try different trajectories: python lidar_motion_compensation.py --trajectory circular")
    print("3. Customize configuration: edit config/default_config.yaml")
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
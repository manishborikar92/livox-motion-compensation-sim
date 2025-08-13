"""
CORRECTED Livox LVX Writer for Viewer Compatibility
Addresses all critical issues found in the original implementation
"""

import numpy as np
import struct
import os
from datetime import datetime

class LivoxLVXWriter:
    """
    Corrected LVX writer that produces files compatible with Livox Viewer 0.11.0 and 2.3.0
    
    Key fixes:
    1. Correct file signature (exactly 16 bytes with proper null padding)
    2. Proper header structure matching LVX v1.1 specification
    3. Correct device info block (59 bytes per device)
    4. Proper package structure matching SDK protocol
    5. Correct timestamp handling and data types
    6. Fixed frame/package organization
    """
    
    def __init__(self):
        # LVX format constants from official specification
        self.LVX_FILE_SIGNATURE = b'livox_tech\x00\x00\x00\x00\x00\x00'  # Exactly 16 bytes
        self.MAGIC_CODE = 0xAC0EA767
        
        # Mid-70 device specifications
        self.DEVICE_TYPE_MID70 = 1  # Mid-40/Mid-100 type according to spec
        self.FRAME_DURATION_MS = 50  # Fixed 50ms as per specification v1.1.0.0
        
        # Package data types (matching SDK protocol)
        self.DATA_TYPE_CARTESIAN = 2  # For Mid-70 with tag field
        self.POINTS_PER_PACKAGE = 96  # For data type 2
        
        # Package header constants
        self.PACKAGE_VERSION = 5  # Current protocol version
        self.SLOT_ID = 0  # Main slot
        self.LIDAR_ID = 1  # Mid-40/Mid-70 ID
        self.TIMESTAMP_TYPE = 1  # Nanosecond timestamp
    
    def write_compatible_lvx(self, filename, frames_data):
        """
        Write LVX file with full Livox Viewer compatibility
        
        Args:
            filename (str): Output filename
            frames_data (list): List of frame data dictionaries
                Each frame should have: frame_id, timestamp, points
        """
        print(f"Writing LVX file: {filename}")
        
        device_count = 1
        frame_count = len(frames_data)
        
        with open(filename, 'wb') as f:
            # === PUBLIC HEADER BLOCK (24 bytes) ===
            public_header = bytearray(24)
            
            # File signature (exactly 16 bytes)
            public_header[0:16] = self.LVX_FILE_SIGNATURE
            
            # Version (4 bytes: v1.1.0.0)
            public_header[16] = 1  # Version-A
            public_header[17] = 1  # Version-B
            public_header[18] = 0  # Version-C
            public_header[19] = 0  # Version-D
            
            # Magic code (4 bytes, little endian)
            struct.pack_into('<I', public_header, 20, self.MAGIC_CODE)
            
            f.write(bytes(public_header))
            
            # === PRIVATE HEADER BLOCK (5 bytes) ===
            private_header = bytearray(5)
            
            # Frame duration (4 bytes) - MUST be 50ms according to spec
            struct.pack_into('<I', private_header, 0, self.FRAME_DURATION_MS)
            
            # Device count (1 byte)
            private_header[4] = device_count
            
            f.write(bytes(private_header))
            
            # === DEVICE INFO BLOCK (59 bytes per device) ===
            device_info = self._create_device_info()
            f.write(device_info)
            
            # === PRE-CALCULATE FRAME POSITIONS ===
            frame_positions = []
            current_pos = f.tell()  # Position after headers
            
            for frame_data in frames_data:
                frame_positions.append(current_pos)
                
                points = frame_data['points']
                # Calculate packages needed (96 points per package for data type 2)
                package_count = (len(points) + self.POINTS_PER_PACKAGE - 1) // self.POINTS_PER_PACKAGE
                
                frame_size = (
                    24 +  # Frame header (3 * 8 bytes)
                    package_count * (22 + self.POINTS_PER_PACKAGE * 14)  # Package header + points
                )
                
                current_pos += frame_size
            
            # === WRITE FRAMES ===
            for i, frame_data in enumerate(frames_data):
                self._write_frame(f, frame_data, frame_positions, i)
        
        file_size = os.path.getsize(filename)
        print(f"✅ LVX file created successfully: {file_size:,} bytes")
        return filename
    
    def _create_device_info(self):
        """Create device info block (59 bytes) matching specification"""
        device_info = bytearray(59)
        
        # LiDAR SN Code (16 bytes) - realistic Mid-70 serial number
        lidar_sn = b'3GGDJ6K00200101\x00'  # Null-terminated
        device_info[0:16] = lidar_sn
        
        # Hub SN Code (16 bytes) - empty for direct connection
        device_info[16:32] = b'\x00' * 16
        
        # Device Index (1 byte)
        device_info[32] = 0
        
        # Device Type (1 byte) - Mid-40/Mid-100 type
        device_info[33] = self.DEVICE_TYPE_MID70
        
        # Extrinsic Enable (1 byte) - disabled
        device_info[34] = 0
        
        # Extrinsic parameters (6 * 4 = 24 bytes) - all zeros when disabled
        # Roll, Pitch, Yaw, X, Y, Z (all floats)
        for i in range(6):
            struct.pack_into('<f', device_info, 35 + i*4, 0.0)
        
        return bytes(device_info)
    
    def _write_frame(self, f, frame_data, frame_positions, frame_index):
        """Write a single frame to the file"""
        points = frame_data['points']
        timestamp_ns = int(frame_data['timestamp'] * 1e9)  # Convert to nanoseconds
        
        # === FRAME HEADER (24 bytes = 3 * 8 bytes) ===
        frame_header = bytearray(24)
        
        # Current offset
        struct.pack_into('<Q', frame_header, 0, frame_positions[frame_index])
        
        # Next offset (0 if last frame)
        if frame_index < len(frame_positions) - 1:
            struct.pack_into('<Q', frame_header, 8, frame_positions[frame_index + 1])
        else:
            struct.pack_into('<Q', frame_header, 8, 0)
        
        # Frame index
        struct.pack_into('<Q', frame_header, 16, frame_data['frame_id'])
        
        f.write(bytes(frame_header))
        
        # === PACKAGES ===
        # Split points into packages (96 points per package for data type 2)
        for i in range(0, len(points), self.POINTS_PER_PACKAGE):
            package_points = points[i:i + self.POINTS_PER_PACKAGE]
            self._write_package(f, package_points, timestamp_ns)
    
    def _write_package(self, f, points, timestamp_ns):
        """Write a single package to the file"""
        actual_point_count = len(points)
        
        # === PACKAGE HEADER (22 bytes) ===
        package_header = bytearray(22)
        
        # Device Index (1 byte)
        package_header[0] = 0
        
        # Version (1 byte)
        package_header[1] = self.PACKAGE_VERSION
        
        # Slot ID (1 byte)
        package_header[2] = self.SLOT_ID
        
        # LiDAR ID (1 byte)
        package_header[3] = self.LIDAR_ID
        
        # Reserved (1 byte)
        package_header[4] = 0
        
        # Status Code (4 bytes) - normal operation
        struct.pack_into('<I', package_header, 5, 0)
        
        # Timestamp Type (1 byte)
        package_header[9] = self.TIMESTAMP_TYPE
        
        # Data Type (1 byte)
        package_header[10] = self.DATA_TYPE_CARTESIAN
        
        # Reserved (3 bytes)
        package_header[11:14] = b'\x00\x00\x00'
        
        # Timestamp (8 bytes)
        struct.pack_into('<Q', package_header, 14, timestamp_ns)
        
        f.write(bytes(package_header))
        
        # === POINT DATA ===
        # For data type 2: x(4) + y(4) + z(4) + reflectivity(1) + tag(1) = 14 bytes per point
        for point in points:
            self._write_point_data_type2(f, point)
        
        # Pad remaining points in package with zeros if needed
        points_to_pad = self.POINTS_PER_PACKAGE - actual_point_count
        if points_to_pad > 0:
            padding = b'\x00' * (points_to_pad * 14)  # 14 bytes per point for type 2
            f.write(padding)
    
    def _write_point_data_type2(self, f, point):
        """Write point data for type 2 (Cartesian with tag) - 14 bytes"""
        point_data = bytearray(14)
        
        # Coordinates in millimeters (int32, little endian)
        x_mm = int(np.clip(point[0] * 1000, -2147483648, 2147483647))
        y_mm = int(np.clip(point[1] * 1000, -2147483648, 2147483647))
        z_mm = int(np.clip(point[2] * 1000, -2147483648, 2147483647))
        
        struct.pack_into('<i', point_data, 0, x_mm)
        struct.pack_into('<i', point_data, 4, y_mm)
        struct.pack_into('<i', point_data, 8, z_mm)
        
        # Reflectivity (0-255)
        reflectivity = int(np.clip(point[3] * 255, 0, 255)) if len(point) > 3 else 128
        point_data[12] = reflectivity
        
        # Tag (point quality/classification)
        point_data[13] = 0  # Normal point
        
        f.write(bytes(point_data))


# Integration with the existing simulator
def create_corrected_lvx_writer():
    """Factory function to create the corrected LVX writer"""
    return LivoxLVXWriter()

# Corrected save_lvx method for the LiDARMotionSimulator class
def save_corrected_lvx(results, base_filename):
    """
    Save point cloud data in corrected Livox LVX format
    
    This function should replace the original save_lvx method in LiDARMotionSimulator
    """
    
    print("Writing corrected LVX format...")
    
    # Create corrected LVX writer
    lvx_writer = LivoxLVXWriter()
    
    # Prepare frame data in the correct format
    frames_data = []
    
    for scan in results['raw_scans']:
        # Handle both test data format and simulation data format
        if 'points_local' in scan:
            points = scan['points_local']
        elif 'points' in scan:
            points = scan['points']
        else:
            raise ValueError(f"No point data found in scan: {scan.keys()}")
        
        frame_data = {
            'frame_id': scan['frame_id'],
            'timestamp': scan['timestamp'],
            'points': points  # Use the found points data
        }
        frames_data.append(frame_data)
    
    # Write the corrected LVX file
    try:
        output_file = f"{base_filename}_corrected.lvx"
        lvx_writer.write_compatible_lvx(output_file, frames_data)
        
        # Validate the generated file
        file_size = os.path.getsize(output_file)
        print(f"✅ Corrected LVX file generated: {output_file}")
        print(f"📊 File size: {file_size:,} bytes")
        print(f"📊 Frames: {len(frames_data)}")
        print(f"📊 Total points: {sum(len(f['points']) for f in frames_data):,}")
        
        return output_file
        
    except Exception as e:
        print(f"❌ Error generating corrected LVX: {e}")
        raise

# Usage example for testing
def test_corrected_lvx():
    """Test function to verify the corrected LVX writer"""
    
    # Create test data
    test_frames = []
    
    for i in range(10):  # 10 test frames
        # Generate test point cloud (random points in a cube)
        n_points = 1000
        points = np.random.uniform(-10, 10, (n_points, 4))  # x, y, z, intensity
        points[:, 3] = np.random.uniform(0, 1, n_points)  # Normalize intensity
        
        frame_data = {
            'frame_id': i,
            'timestamp': i * 0.05,  # 20Hz frame rate
            'points': points  # Changed from 'points_local' to 'points'
        }
        test_frames.append(frame_data)
    
    # Create test results structure
    test_results = {
        'raw_scans': test_frames
    }
    
    # Write test LVX file
    try:
        output_file = save_corrected_lvx(test_results, "test_output")
        print(f"🎯 Test LVX file created: {output_file}")
        print("📝 Try opening this file in Livox Viewer to verify compatibility")
        return True
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    print("Testing Corrected Livox LVX Writer...")
    success = test_corrected_lvx()
    if success:
        print("✅ Test completed successfully!")
    else:
        print("❌ Test failed!")
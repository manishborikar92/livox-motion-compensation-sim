
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

# Try to import Open3D for advanced visualization
try:
    import open3d as o3d
    HAS_OPEN3D = True
    print("Open3D available - enhanced visualizations enabled")
except ImportError:
    HAS_OPEN3D = False
    print("Open3D not available - using matplotlib only")

def load_pcd_file(filename):
    """Load PCD file (simple ASCII format)"""
    points = []
    with open(filename, 'r') as f:
        lines = f.readlines()
        
        # Find data section
        data_start = None
        for i, line in enumerate(lines):
            if line.startswith('DATA ascii'):
                data_start = i + 1
                break
        
        if data_start:
            for line in lines[data_start:]:
                parts = line.strip().split()
                if len(parts) >= 4:
                    x, y, z, intensity = map(float, parts[:4])
                    points.append([x, y, z, intensity])
    
    return np.array(points) if points else np.empty((0, 4))

def visualize_single_scan(pcd_file):
    """Visualize a single scan with both matplotlib and Open3D"""
    
    print(f"Loading {pcd_file}")
    points = load_pcd_file(pcd_file)
    
    if len(points) == 0:
        print("No points loaded!")
        return
    
    print(f"Loaded {len(points):,} points")
    
    # Matplotlib visualization
    fig = plt.figure(figsize=(15, 5))
    
    # 3D scatter plot
    ax1 = fig.add_subplot(131, projection='3d')
    ranges = np.sqrt(np.sum(points[:, :3]**2, axis=1))
    scatter = ax1.scatter(points[:, 0], points[:, 1], points[:, 2], 
                         c=ranges, s=0.5, alpha=0.6, cmap='plasma')
    plt.colorbar(scatter, ax=ax1, shrink=0.5, label='Range (m)')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('3D Point Cloud (Colored by Range)')
    
    # Top view
    ax2 = fig.add_subplot(132)
    scatter2 = ax2.scatter(points[:, 0], points[:, 1], c=points[:, 3], 
                          s=0.5, alpha=0.7, cmap='viridis')
    plt.colorbar(scatter2, ax=ax2, label='Intensity')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('Top View (Colored by Intensity)')
    ax2.set_aspect('equal')
    
    # Range vs Intensity
    ax3 = fig.add_subplot(133)
    ax3.scatter(ranges, points[:, 3], alpha=0.5, s=1)
    ax3.set_xlabel('Range (m)')
    ax3.set_ylabel('Intensity')
    ax3.set_title('Range vs Intensity')
    ax3.grid(True)
    
    plt.tight_layout()
    plt.show()
    
    # Open3D visualization (if available)
    if HAS_OPEN3D:
        print("Creating Open3D visualization...")
        
        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        
        # Color by intensity
        intensities = points[:, 3]
        colors = plt.cm.viridis(intensities)[:, :3]  # Convert to RGB
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
        # Create coordinate frame
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=5.0)
        
        # Visualize
        print("Opening Open3D viewer...")
        print("Controls: Mouse to rotate/pan/zoom, 'Q' to quit")
        o3d.visualization.draw_geometries([pcd, coord_frame],
                                        window_name="Realistic Livox Mid-70 Simulation",
                                        width=1200, height=800)

def compare_scans():
    """Compare raw vs aligned scans"""
    output_dir = "realistic_lidar_output"
    
    # Load sample files
    raw_file = os.path.join(output_dir, "raw_scans_pcd", "frame_0000.pcd")
    aligned_file = os.path.join(output_dir, "aligned_scans_pcd", "aligned_frame_0000.pcd")
    
    if os.path.exists(raw_file) and os.path.exists(aligned_file):
        print("Comparing raw vs aligned scans...")
        
        raw_points = load_pcd_file(raw_file)
        aligned_points = load_pcd_file(aligned_file)
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
        
        # Raw scan (centered at origin)
        if len(raw_points) > 0:
            ax1.scatter(raw_points[:, 0], raw_points[:, 1], 
                       c=raw_points[:, 3], s=0.5, alpha=0.7, cmap='viridis')
            ax1.set_xlabel('X (m)')
            ax1.set_ylabel('Y (m)')
            ax1.set_title('Raw Scan (Sensor Frame)')
            ax1.set_aspect('equal')
            ax1.grid(True)
        
        # Aligned scan (world coordinates)
        if len(aligned_points) > 0:
            ax2.scatter(aligned_points[:, 0], aligned_points[:, 1], 
                       c=aligned_points[:, 3], s=0.5, alpha=0.7, cmap='viridis')
            ax2.set_xlabel('X (m)')
            ax2.set_ylabel('Y (m)')
            ax2.set_title('Aligned Scan (World Frame)')
            ax2.set_aspect('equal')
            ax2.grid(True)
        
        plt.tight_layout()
        plt.show()
        
        # Statistics comparison
        if len(raw_points) > 0 and len(aligned_points) > 0:
            print(f"\nStatistics:")
            print(f"Raw scan points: {len(raw_points):,}")
            print(f"Aligned scan points: {len(aligned_points):,}")
            
            raw_ranges = np.sqrt(np.sum(raw_points[:, :3]**2, axis=1))
            print(f"Raw scan range: {raw_ranges.min():.2f} - {raw_ranges.max():.2f} m")
            print(f"Raw scan mean intensity: {raw_points[:, 3].mean():.3f}")
            
            aligned_ranges = np.sqrt(np.sum(aligned_points[:, :3]**2, axis=1))
            print(f"Aligned scan range: {aligned_ranges.min():.2f} - {aligned_ranges.max():.2f} m")
            print(f"Aligned scan mean intensity: {aligned_points[:, 3].mean():.3f}")
    
    else:
        print("Sample scan files not found!")

def visualize_merged_cloud():
    """Visualize the complete merged point cloud"""
    output_dir = "realistic_lidar_output"
    merged_file = os.path.join(output_dir, "merged_aligned.pcd")
    
    if os.path.exists(merged_file):
        print("Loading merged aligned point cloud...")
        points = load_pcd_file(merged_file)
        
        if len(points) > 0:
            print(f"Merged cloud contains {len(points):,} points")
            
            # Subsample for visualization if too large
            if len(points) > 100000:
                indices = np.random.choice(len(points), 100000, replace=False)
                points = points[indices]
                print(f"Subsampled to {len(points):,} points for visualization")
            
            # Create comprehensive visualization
            fig = plt.figure(figsize=(20, 10))
            
            # 3D view
            ax1 = fig.add_subplot(231, projection='3d')
            ranges = np.sqrt(np.sum(points[:, :3]**2, axis=1))
            scatter1 = ax1.scatter(points[:, 0], points[:, 1], points[:, 2],
                                 c=points[:, 3], s=0.1, alpha=0.5, cmap='viridis')
            ax1.set_xlabel('X (m)')
            ax1.set_ylabel('Y (m)')
            ax1.set_zlabel('Z (m)')
            ax1.set_title('Complete Merged Point Cloud')
            
            # Top view
            ax2 = fig.add_subplot(232)
            ax2.scatter(points[:, 0], points[:, 1], c=points[:, 3], 
                       s=0.1, alpha=0.5, cmap='viridis')
            ax2.set_xlabel('X (m)')
            ax2.set_ylabel('Y (m)')
            ax2.set_title('Top View')
            ax2.set_aspect('equal')
            
            # Side view
            ax3 = fig.add_subplot(233)
            ax3.scatter(points[:, 0], points[:, 2], c=points[:, 3], 
                       s=0.1, alpha=0.5, cmap='viridis')
            ax3.set_xlabel('X (m)')
            ax3.set_ylabel('Z (m)')
            ax3.set_title('Side View')
            
            # Range histogram
            ax4 = fig.add_subplot(234)
            ax4.hist(ranges, bins=50, alpha=0.7, color='blue')
            ax4.set_xlabel('Range (m)')
            ax4.set_ylabel('Point Count')
            ax4.set_title('Range Distribution')
            ax4.grid(True)
            
            # Intensity histogram
            ax5 = fig.add_subplot(235)
            ax5.hist(points[:, 3], bins=50, alpha=0.7, color='green')
            ax5.set_xlabel('Intensity')
            ax5.set_ylabel('Point Count')
            ax5.set_title('Intensity Distribution')
            ax5.grid(True)
            
            # Height distribution
            ax6 = fig.add_subplot(236)
            ax6.hist(points[:, 2], bins=50, alpha=0.7, color='red')
            ax6.set_xlabel('Height (m)')
            ax6.set_ylabel('Point Count')
            ax6.set_title('Height Distribution')
            ax6.grid(True)
            
            plt.tight_layout()
            plt.show()
            
            # Open3D visualization of merged cloud
            if HAS_OPEN3D:
                print("Creating Open3D visualization of merged cloud...")
                
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(points[:, :3])
                
                # Color by height for better visualization
                heights = points[:, 2]
                normalized_heights = (heights - heights.min()) / (heights.max() - heights.min())
                colors = plt.cm.terrain(normalized_heights)[:, :3]
                pcd.colors = o3d.utility.Vector3dVector(colors)
                
                # Add coordinate frame and ground plane
                coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10.0)
                
                # Create simple ground plane mesh
                ground_plane = o3d.geometry.TriangleMesh.create_box(width=200, height=0.1, depth=200)
                ground_plane.translate([-100, -100, heights.min()])
                ground_plane.paint_uniform_color([0.6, 0.6, 0.6])
                
                print("Opening Open3D viewer for merged cloud...")
                o3d.visualization.draw_geometries([pcd, coord_frame, ground_plane],
                                                window_name="Complete Merged Point Cloud",
                                                width=1400, height=1000)
        else:
            print("No points in merged file!")
    else:
        print("Merged file not found!")

def main():
    """Main visualization function"""
    print("Realistic Livox Mid-70 Simulation - Visualization Tool")
    print("="*60)
    
    output_dir = "realistic_lidar_output"
    
    while True:
        print("\nChoose visualization option:")
        print("1. View single scan (first raw scan)")
        print("2. Compare raw vs aligned scans")  
        print("3. View complete merged point cloud")
        print("4. Exit")
        
        try:
            choice = input("\nEnter choice (1-4): ").strip()
            
            if choice == '1':
                # Show first raw scan
                scan_file = os.path.join(output_dir, "raw_scans_pcd", "frame_0000.pcd")
                if os.path.exists(scan_file):
                    visualize_single_scan(scan_file)
                else:
                    print("Raw scan file not found!")
                    
            elif choice == '2':
                compare_scans()
                
            elif choice == '3':
                visualize_merged_cloud()
                
            elif choice == '4':
                print("Exiting...")
                break
                
            else:
                print("Invalid choice!")
                
        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except Exception as e:
            print(f"Error: {e}")

if __name__ == "__main__":
    main()

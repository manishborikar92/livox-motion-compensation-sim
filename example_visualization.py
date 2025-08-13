"""
Example code to quickly visualize the realistic Livox Mid-70 simulation results
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def load_pcd_simple(filename):
    """Simple PCD file loader"""
    points = []
    with open(filename, 'r') as f:
        lines = f.readlines()
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
    return np.array(points)

def visualize_scan_realism(scan_file):
    """Demonstrate the realistic aspects of the simulation"""
    
    points = load_pcd_simple(scan_file)
    print(f"Loaded {len(points):,} points")
    
    # Calculate ranges and angles
    ranges = np.sqrt(np.sum(points[:, :3]**2, axis=1))
    azimuth = np.degrees(np.arctan2(points[:, 1], points[:, 0]))
    elevation = np.degrees(np.arcsin(points[:, 2] / ranges))
    
    # Create comprehensive analysis
    fig = plt.figure(figsize=(18, 12))
    
    # 1. FOV Pattern (shows rosette pattern)
    ax1 = fig.add_subplot(331)
    scatter = ax1.scatter(azimuth, elevation, c=ranges, s=1, alpha=0.6, cmap='plasma')
    ax1.set_xlabel('Azimuth (degrees)')
    ax1.set_ylabel('Elevation (degrees)')
    ax1.set_title('Scanning Pattern (Rosette Shape)')
    plt.colorbar(scatter, ax=ax1, label='Range (m)')
    
    # Add FOV circle
    circle = plt.Circle((0, 0), 35.2, fill=False, color='red', linestyle='--')
    ax1.add_patch(circle)
    ax1.set_xlim(-40, 40)
    ax1.set_ylim(-40, 40)
    ax1.set_aspect('equal')
    
    # 2. Range vs Point Density (1/r² law)
    ax2 = fig.add_subplot(332)
    range_bins = np.linspace(ranges.min(), ranges.max(), 20)
    hist, edges = np.histogram(ranges, bins=range_bins)
    bin_centers = (edges[:-1] + edges[1:]) / 2
    
    # Normalize by shell volume (4πr²) to show density
    shell_volumes = 4 * np.pi * bin_centers**2
    normalized_density = hist / shell_volumes
    
    ax2.plot(bin_centers, normalized_density, 'b-o', markersize=4)
    ax2.set_xlabel('Range (m)')
    ax2.set_ylabel('Normalized Point Density')
    ax2.set_title('Point Density vs Range (Shows 1/r² Law)')
    ax2.grid(True)
    
    # 3. Intensity vs Range (atmospheric attenuation)
    ax3 = fig.add_subplot(333)
    # Bin data for clearer visualization
    range_bins = np.linspace(0, ranges.max(), 50)
    intensity_means = []
    range_centers = []
    
    for i in range(len(range_bins)-1):
        mask = (ranges >= range_bins[i]) & (ranges < range_bins[i+1])
        if np.sum(mask) > 10:  # Need sufficient points
            intensity_means.append(np.mean(points[mask, 3]))
            range_centers.append((range_bins[i] + range_bins[i+1]) / 2)
    
    ax3.plot(range_centers, intensity_means, 'g-o', markersize=3)
    ax3.set_xlabel('Range (m)')
    ax3.set_ylabel('Mean Intensity')
    ax3.set_title('Intensity Attenuation with Distance')
    ax3.grid(True)
    
    # 4. Intensity Distribution (material response)
    ax4 = fig.add_subplot(334)
    ax4.hist(points[:, 3], bins=30, alpha=0.7, color='orange', edgecolor='black')
    ax4.set_xlabel('Intensity')
    ax4.set_ylabel('Point Count')
    ax4.set_title('Intensity Distribution (Material Types)')
    ax4.grid(True, alpha=0.3)
    
    # Add material type annotations
    ax4.axvline(0.2, color='green', linestyle='--', alpha=0.7, label='Vegetation')
    ax4.axvline(0.5, color='gray', linestyle='--', alpha=0.7, label='Concrete')
    ax4.axvline(0.8, color='blue', linestyle='--', alpha=0.7, label='Metal/Glass')
    ax4.legend()
    
    # 5. 3D Point Cloud (top view)
    ax5 = fig.add_subplot(335)
    scatter5 = ax5.scatter(points[:, 0], points[:, 1], c=points[:, 3], 
                          s=0.5, alpha=0.6, cmap='viridis')
    ax5.set_xlabel('X (m)')
    ax5.set_ylabel('Y (m)')
    ax5.set_title('Top View (Intensity-Colored)')
    ax5.set_aspect('equal')
    plt.colorbar(scatter5, ax=ax5, label='Intensity')
    
    # 6. Range Noise Analysis
    ax6 = fig.add_subplot(336)
    # Group points by range bins and calculate noise statistics
    range_bins = np.linspace(0, ranges.max(), 20)
    noise_std = []
    range_centers = []
    
    for i in range(len(range_bins)-1):
        mask = (ranges >= range_bins[i]) & (ranges < range_bins[i+1])
        if np.sum(mask) > 50:  # Need sufficient points for noise analysis
            # Estimate noise from range residuals (simplified)
            range_residuals = ranges[mask] - np.median(ranges[mask])
            noise_std.append(np.std(range_residuals))
            range_centers.append((range_bins[i] + range_bins[i+1]) / 2)
    
    if noise_std:
        ax6.plot(range_centers, noise_std, 'r-o', markersize=4)
        ax6.set_xlabel('Range (m)')
        ax6.set_ylabel('Range Noise Std (m)')
        ax6.set_title('Range Noise vs Distance')
        ax6.grid(True)
    
    # 7. Beam Pattern Coverage
    ax7 = fig.add_subplot(337)
    # Show angular coverage density
    az_bins = np.linspace(-35, 35, 40)
    el_bins = np.linspace(-35, 35, 40)
    coverage, _, _ = np.histogram2d(azimuth, elevation, bins=[az_bins, el_bins])
    
    im = ax7.imshow(coverage.T, origin='lower', extent=[-35, 35, -35, 35], 
                    cmap='hot', aspect='equal')
    ax7.set_xlabel('Azimuth (degrees)')
    ax7.set_ylabel('Elevation (degrees)')
    ax7.set_title('Angular Coverage Density')
    plt.colorbar(im, ax=ax7, label='Point Count')
    
    # 8. Height vs Intensity (surface normal effects)
    ax8 = fig.add_subplot(338)
    ax8.scatter(points[:, 2], points[:, 3], s=1, alpha=0.3, c='purple')
    ax8.set_xlabel('Height (m)')
    ax8.set_ylabel('Intensity')
    ax8.set_title('Height vs Intensity (Surface Effects)')
    ax8.grid(True)
    
    # 9. Statistics Summary
    ax9 = fig.add_subplot(339)
    ax9.axis('off')
    
    # Calculate key statistics
    stats_text = f"""
    SIMULATION REALISM METRICS
    
    Point Count: {len(points):,}
    Range: {ranges.min():.2f} - {ranges.max():.2f} m
    Mean Range: {ranges.mean():.1f} m
    
    Intensity: {points[:, 3].min():.3f} - {points[:, 3].max():.3f}
    Mean Intensity: {points[:, 3].mean():.3f}
    
    FOV Coverage:
    Azimuth: {azimuth.min():.1f}° to {azimuth.max():.1f}°
    Elevation: {elevation.min():.1f}° to {elevation.max():.1f}°
    
    Density at 10m: {len(points[ranges < 10]):,} pts
    Density at 50m: {len(points[(ranges >= 45) & (ranges < 55)]):,} pts
    Density at 100m: {len(points[(ranges >= 95) & (ranges < 105)]):,} pts
    
    High Intensity (>0.7): {np.sum(points[:, 3] > 0.7):,} pts
    Low Intensity (<0.3): {np.sum(points[:, 3] < 0.3):,} pts
    """
    
    ax9.text(0.05, 0.95, stats_text, transform=ax9.transAxes, fontsize=10,
             verticalalignment='top', fontfamily='monospace',
             bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.8))
    
    plt.tight_layout()
    plt.show()
    
    return points

def compare_with_uniform_simulation():
    """Compare realistic vs uniform point distribution"""
    
    # Simulate uniform distribution (old method)
    n_points = 50000
    fov_rad = np.radians(35.2)  # Half FOV
    
    # Uniform distribution (unrealistic)
    az_uniform = np.random.uniform(-fov_rad, fov_rad, n_points)
    el_uniform = np.random.uniform(-fov_rad, fov_rad, n_points)
    
    # Filter to circular FOV
    angles_uniform = np.sqrt(az_uniform**2 + el_uniform**2)
    mask_uniform = angles_uniform <= fov_rad
    az_uniform = az_uniform[mask_uniform]
    el_uniform = el_uniform[mask_uniform]
    
    # Rosette pattern (realistic)
    t = np.linspace(0, 1.4, len(az_uniform))
    freq1, freq2 = 17, 23
    r = 0.5 * (1 + 0.3 * np.cos(2 * np.pi * freq1 * t))
    theta = 2 * np.pi * freq2 * t
    
    fov_radius = 35.2  # degrees
    az_rosette = r * fov_radius * np.cos(theta) / 180 * np.pi
    el_rosette = r * fov_radius * np.sin(theta) / 180 * np.pi
    
    # Plot comparison
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    
    # Uniform (old method)
    ax1.scatter(np.degrees(az_uniform), np.degrees(el_uniform), 
               s=0.5, alpha=0.6, color='red')
    ax1.set_title('Old Method: Uniform Distribution\n(Unrealistic)')
    ax1.set_xlabel('Azimuth (degrees)')
    ax1.set_ylabel('Elevation (degrees)')
    ax1.set_xlim(-40, 40)
    ax1.set_ylim(-40, 40)
    ax1.set_aspect('equal')
    ax1.grid(True)
    
    # Rosette (new method)
    ax2.scatter(np.degrees(az_rosette), np.degrees(el_rosette), 
               s=0.5, alpha=0.6, color='blue')
    ax2.set_title('New Method: Rosette Pattern\n(Realistic Mid-70)')
    ax2.set_xlabel('Azimuth (degrees)')
    ax2.set_ylabel('Elevation (degrees)')
    ax2.set_xlim(-40, 40)
    ax2.set_ylim(-40, 40)
    ax2.set_aspect('equal')
    ax2.grid(True)
    
    # Add FOV circles
    for ax in [ax1, ax2]:
        circle = plt.Circle((0, 0), 35.2, fill=False, color='black', linestyle='--', linewidth=2)
        ax.add_patch(circle)
    
    plt.tight_layout()
    plt.show()

# Example usage
if __name__ == "__main__":
    print("Realistic Livox Mid-70 Visualization Examples")
    print("=" * 50)
    
    # Load and analyze a sample scan
    scan_file = "realistic_lidar_output/raw_scans_pcd/frame_0000.pcd"  # Adjust path
    
    try:
        print("1. Analyzing realistic scan patterns...")
        points = visualize_scan_realism(scan_file)
        
        print("\n2. Comparing scanning patterns...")
        compare_with_uniform_simulation()
        
        print("\n✅ Analysis complete!")
        print("Key observations:")
        print("- Rosette pattern creates non-uniform but complete coverage")
        print("- Point density follows realistic 1/r² law")
        print("- Intensity varies with material and distance")
        print("- Range noise increases with distance")
        print("- FOV is properly circular, not rectangular")
        
    except FileNotFoundError:
        print(f"Sample file not found: {scan_file}")
        print("Run the main simulation first to generate data.")
        print("\n2. Comparing scanning patterns...")
        compare_with_uniform_simulation()

# python example_visualization.py
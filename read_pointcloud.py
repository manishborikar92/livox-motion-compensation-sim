
import open3d as o3d
import numpy as np
import logging
import os
import argparse
import matplotlib.pyplot as plt


def _create_grid_ground_plane(pcd: o3d.geometry.PointCloud, grid_spacing: float = 2.0, size: float = 100.0) -> o3d.geometry.LineSet:
    """
    Create a gridded ground plane under a point cloud.
    
    Args:
        pcd: Input point cloud to position the ground plane under
        grid_spacing: Spacing between grid lines
        size: Total size of the grid (square)
        
    Returns:
        LineSet: A grid representing the ground plane
    """
    if pcd is None or not pcd.has_points():
        return None
    
    aabb = pcd.get_axis_aligned_bounding_box()
    min_bound = aabb.get_min_bound()
    center = aabb.get_center()
    
    ground_level = min_bound[2] - 0.5  # Place it slightly below the lowest point
    
    points = []
    lines = []
    line_count = 0
    
    half_size = size / 2
    
    # Create grid lines
    for i in np.arange(-half_size, half_size + grid_spacing, grid_spacing):
        # Lines parallel to X-axis
        points.append([center[0] - half_size, center[1] + i, ground_level])
        points.append([center[0] + half_size, center[1] + i, ground_level])
        lines.append([line_count, line_count + 1])
        line_count += 2
        
        # Lines parallel to Y-axis
        points.append([center[0] + i, center[1] - half_size, ground_level])
        points.append([center[0] + i, center[1] + half_size, ground_level])
        lines.append([line_count, line_count + 1])
        line_count += 2

    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines),
    )
    line_set.paint_uniform_color([0.3, 0.3, 0.3])  # Dark grey color for grid
    return line_set


def read_point_cloud(file_path):
    """Reads a point cloud from various formats with metadata extraction."""
    if not os.path.exists(file_path):
        print(f"File not found: {file_path}")
        raise FileNotFoundError(f"The file {file_path} does not exist.")

    try:
        pcd = o3d.io.read_point_cloud(file_path)
        if not pcd.has_points():
            print(f"Point cloud at {file_path} is empty.")
            return None
        
        # Extract metadata
        num_points = len(pcd.points)
        bounds = pcd.get_axis_aligned_bounding_box()
        print(f"Loaded point cloud with {num_points} points.")
        print(f"Bounding box: {bounds.min_bound} to {bounds.max_bound}")
        
        if not pcd.has_colors():
            print("Point cloud has no colors, assigning a uniform gray color.")
            pcd.paint_uniform_color([0.6, 0.6, 0.6])

        if not pcd.has_normals():
            print("Point cloud has no normals, estimating them.")
            pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
            print("Normals estimated.")

        return pcd

    except Exception as e:
        print(f"Failed to read or process {file_path}")
        return None

def visualize_point_cloud(pcd, window_name="Point Cloud Visualization", color_mode="rgb", background_theme="light"):
    """Visualizes the point cloud with enhanced options and coloring.
    
    Args:
        pcd: Open3D point cloud object
        window_name: Name of the visualization window
        color_mode: Coloring mode ('rgb', 'elevation')
        background_theme: 'light' or 'dark' background
    """
    if not pcd or not pcd.has_points():
        print("Cannot visualize: Point cloud is empty or invalid")
        return
        
    try:
        print(f"Visualizing point cloud with {len(pcd.points)} points in {color_mode} mode...")

        # Create a copy to avoid modifying the original point cloud
        pcd_to_visualize = o3d.geometry.PointCloud()
        pcd_to_visualize.points = pcd.points
        if pcd.has_normals():
            pcd_to_visualize.normals = pcd.normals

        # Apply selected color mode
        if color_mode == "elevation":
            points = np.asarray(pcd.points)
            z_values = points[:, 2]
            z_range = np.max(z_values) - np.min(z_values)
            if z_range > 1e-6:  # Avoid division by zero
                z_norm = (z_values - np.min(z_values)) / z_range
                z_norm = np.power(z_norm, 0.4)  # Non-linear scaling for better color distribution
                cmap = plt.get_cmap("jet")
                colors = cmap(z_norm)[:, :3]
                pcd_to_visualize.colors = o3d.utility.Vector3dVector(colors)
                print("Applied elevation-based coloring.")
            else:
                print("Insufficient elevation range for coloring. Using default colors.")
                pcd_to_visualize.colors = pcd.colors
                
        else:  # 'rgb' or any other mode
            pcd_to_visualize.colors = pcd.colors if pcd.has_colors() else o3d.utility.Vector3dVector(np.ones((len(pcd.points), 3)) * 0.6)
            print("Using RGB colors.")

        # Create visualization window
        vis = o3d.visualization.Visualizer()
        try:
            # Initialize window
            vis.create_window(window_name=window_name, width=1600, height=1200)
            
            # Configure render options
            render_opt = vis.get_render_option()
            
            # Advanced background themes
            background_colors = {
                "light": np.array([0.95, 0.95, 0.95]),
                "dark": np.array([0.1, 0.1, 0.15]),
                "midnight": np.array([0.05, 0.08, 0.15])
            }
            
            render_opt.background_color = background_colors.get(background_theme, background_colors["dark"])
            render_opt.point_size = 2.5
            render_opt.show_coordinate_frame = True

            render_opt.light_on = False  # Disable dynamic lighting for consistent colors
            
            # Add geometry
            vis.add_geometry(pcd_to_visualize)
            
            # Add grid ground plane by default
            ground_plane = _create_grid_ground_plane(pcd_to_visualize, grid_spacing=2.0, size=100.0)
            if ground_plane:
                vis.add_geometry(ground_plane)
            
            # Configure view
            ctr = vis.get_view_control()
            ctr.set_zoom(0.8)
            
            # Run visualization
            vis.run()
            
        except Exception as e:
            print(f"Visualization error: {e}")
            raise
            
        finally:
            if vis is not None:
                vis.destroy_window()
                
    except Exception as e:
        print("An unexpected error occurred during visualization:")
        raise

def main():
    parser = argparse.ArgumentParser(description="Load, process, and visualize a point cloud.")
    parser.add_argument("input_file", help="Path to the point cloud file.")
    
    # Visualization arguments
    vis_group = parser.add_argument_group('Visualization Options')
    vis_group.add_argument(
        "--color_mode",
        type=str,
        default="rgb",
        choices=["rgb", "elevation"],
        help="Coloring mode for visualization. 'elevation' (default) colors by height."
    )

    parser.add_argument(
        "--background",
        type=str,
        default="dark",
        choices=["dark", "light", "midnight"],
        help="Set the background color theme for visualization."
    )
    args = parser.parse_args()

    point_cloud = read_point_cloud(args.input_file)

    visualize_point_cloud(
        point_cloud,
        window_name=f"Point Cloud: {os.path.basename(args.input_file)}",
        color_mode=args.color_mode,
        background_theme=args.background
    )

if __name__ == "__main__":
    main()

# RUN EXAMPLES:
# The following commands assume you are in the root directory of the project (iwlars).

# cd iwlars-core

# Example with wagon datasets: 
# python -m read_pointcloud lidar_simulation_output/merged_aligned.pcd
# python -m read_pointcloud lidar_simulation_output/merged_raw_overlapped.pcd

# python -m read_pointcloud data/raw/two-wagon-data.pcd
# python -m read_pointcloud data/raw/five-wagon-data.pcd
# python -m read_pointcloud data/raw/multi-wagon-data.pcd
# python -m read_pointcloud data/raw/filled-five-wagon-data.pcd
# python -m read_pointcloud data/raw/main_wagon.pcd
# python -m read_pointcloud data/processed/aligned/two-wagon-data_aligned.pcd
# python -m read_pointcloud data/processed/side_filtered/two-wagon-data_aligned_nonground_wagon.pcd

# python -m read_pointcloud data/processed/aligned/cleaned_wagon_unfilled_aligned.pcd
# python -m read_pointcloud data/processed/aligned/cleaned_wagon_filled_aligned.pcd

# python -m read_pointcloud data/processed/segmented/final-five-wagon_downsampled_aligned_wagon_01.pcd
# python -m read_pointcloud data/processed/segmented/final-five-wagon_downsampled_aligned_wagon_02.pcd
# python -m read_pointcloud data/processed/segmented/final-five-wagon_downsampled_aligned_wagon_03.pcd
# python -m read_pointcloud data/processed/segmented/final-five-wagon_downsampled_aligned_wagon_04.pcd
# python -m read_pointcloud data/processed/segmented/final-five-wagon_downsampled_aligned_wagon_05.pcd

# Example with main wagon dataset:

# python -m read_pointcloud data/raw/Entire_Train.pcd
# python -m read_pointcloud data/raw/main_wagon.pcd
# python -m read_pointcloud data/processed/downsampled/main_wagon_downsampled.pcd
# python -m read_pointcloud data/processed/aligned/main_wagon_downsampled_aligned.pcd
# python -m read_pointcloud data/processed/ground_filtered/main_wagon_downsampled_aligned_nonground.pcd
# python -m read_pointcloud data/processed/side_filtered/main_wagon_downsampled_aligned_nonground_train.pcd

# python -m read_pointcloud data/processed/segmented/main_wagon_downsampled_aligned_nonground_train_wagon_01.pcd
# python -m read_pointcloud data/processed/segmented/main_wagon_downsampled_aligned_nonground_train_wagon_02.pcd
# python -m read_pointcloud data/processed/segmented/main_wagon_downsampled_aligned_nonground_train_wagon_03.pcd
# python -m read_pointcloud data/processed/segmented/main_wagon_downsampled_aligned_nonground_train_wagon_04.pcd
# python -m read_pointcloud data/processed/segmented/main_wagon_downsampled_aligned_nonground_train_wagon_05.pcd
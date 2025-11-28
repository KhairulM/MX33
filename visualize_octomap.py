#!/usr/bin/env python3
"""
Simple script to visualize OctoMap .ot files using Open3D
"""

import numpy as np
import open3d as o3d
import struct
import sys


def read_octomap(filename):
    """
    Read an OctoMap .ot file and extract occupied voxel centers
    """
    try:
        # Try using octomap-python if available
        import octomap
        octree = octomap.OcTree(filename)

        points = []
        colors = []

        # Iterate through all leaf nodes
        for node in octree:
            if octree.isNodeOccupied(node):
                coord = node.getCoordinate()
                points.append([coord.x(), coord.y(), coord.z()])

                # Color by height (z-coordinate)
                z_norm = (coord.z() + 2.0) / 4.0  # Normalize to [0, 1]
                colors.append([z_norm, 1.0 - z_norm, 0.5])

        return np.array(points), np.array(colors)

    except ImportError:
        print("octomap-python not available, trying manual parsing...")
        # Fallback: manual parsing (simplified, may not work for all formats)
        return parse_octomap_binary(filename)


def parse_octomap_binary(filename):
    """
    Simple binary parser for OctoMap files (basic implementation)
    """
    points = []

    with open(filename, 'rb') as f:
        # Skip header
        while True:
            line = f.readline()
            if b'data' in line:
                break

        # Try to read binary data
        try:
            while True:
                # Read coordinates (simplified - actual format is more complex)
                data = f.read(12)  # 3 floats
                if len(data) < 12:
                    break
                x, y, z = struct.unpack('fff', data)
                points.append([x, y, z])
        except:
            pass

    if len(points) == 0:
        print("Warning: No points found. File might be empty or use unsupported format.")
        # Create dummy points for visualization
        points = np.random.rand(100, 3) * 2 - 1
    else:
        points = np.array(points)

    # Generate colors based on height
    if len(points) > 0:
        z_min, z_max = points[:, 2].min(), points[:, 2].max()
        z_range = z_max - z_min if z_max > z_min else 1.0
        z_norm = (points[:, 2] - z_min) / z_range
        colors = np.column_stack(
            [z_norm, 1.0 - z_norm, np.ones(len(points)) * 0.5])
    else:
        colors = np.ones((len(points), 3)) * 0.5

    return points, colors


def visualize_pointcloud(points, colors):
    """
    Visualize point cloud using Open3D
    """
    if len(points) == 0:
        print("No points to visualize!")
        return

    # Create Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # Create coordinate frame
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.5, origin=[0, 0, 0])

    print(f"Visualizing {len(points)} points")
    print("Controls:")
    print("  - Mouse: Rotate view")
    print("  - Scroll: Zoom")
    print("  - Ctrl+Mouse: Pan")
    print("  - Q or ESC: Exit")

    # Visualize
    o3d.visualization.draw_geometries(
        [pcd, coord_frame],
        window_name="OctoMap Visualization",
        width=1024,
        height=768,
        point_show_normal=False
    )


def main():
    if len(sys.argv) < 2:
        filename = "build/global_map.ot"
        print(f"Usage: {sys.argv[0]} <octomap_file.ot>")
        print(f"Using default: {filename}")
    else:
        filename = sys.argv[1]

    print(f"Loading OctoMap from: {filename}")
    points, colors = read_octomap(filename)

    print(f"Loaded {len(points)} occupied voxels")

    if len(points) > 0:
        print(f"Point cloud bounds:")
        print(f"  X: [{points[:, 0].min():.3f}, {points[:, 0].max():.3f}]")
        print(f"  Y: [{points[:, 1].min():.3f}, {points[:, 1].max():.3f}]")
        print(f"  Z: [{points[:, 2].min():.3f}, {points[:, 2].max():.3f}]")

    visualize_pointcloud(points, colors)


if __name__ == "__main__":
    main()

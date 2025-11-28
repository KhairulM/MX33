#!/usr/bin/env python3
"""
Visualize OctoMap .ot files using matplotlib (works without OpenGL)
"""

import numpy as np
import matplotlib.pyplot as plt
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

        # Iterate through all leaf nodes
        for node in octree:
            if octree.isNodeOccupied(node):
                coord = node.getCoordinate()
                points.append([coord.x(), coord.y(), coord.z()])

        return np.array(points) if points else np.array([]).reshape(0, 3)

    except ImportError:
        print("octomap-python not available, checking file manually...")
        return check_octomap_file(filename)


def check_octomap_file(filename):
    """
    Check OctoMap file structure
    """
    points = []

    with open(filename, 'rb') as f:
        content = f.read()

        # Check if file has actual data beyond header
        lines = content.split(b'\n')
        header_size = 0
        for i, line in enumerate(lines):
            if b'data' in line:
                header_size = sum(len(l) + 1 for l in lines[:i+1])
                break

        data_size = len(content) - header_size
        print(f"File size: {len(content)} bytes")
        print(f"Header size: {header_size} bytes")
        print(f"Data size: {data_size} bytes")

        if data_size <= 1:
            print("\nWARNING: OctoMap file appears to be empty (no voxel data)!")
            print("This likely means:")
            print("  1. The map_server did not receive any pointcloud messages")
            print("  2. The robot did not publish any data")
            print("  3. The pointcloud data was all invalid (NaN or out of bounds)")
            return np.array([]).reshape(0, 3)

    return np.array(points) if points else np.array([]).reshape(0, 3)


def visualize_pointcloud_matplotlib(points):
    """
    Visualize point cloud using matplotlib (2D views to avoid 3D import issues)
    """
    if len(points) == 0:
        print("\n" + "="*60)
        print("NO POINTS TO VISUALIZE - MAP IS EMPTY!")
        print("="*60)
        print("\nTroubleshooting steps:")
        print("1. Check that the robot is running and publishing")
        print("2. Check that map_server is receiving messages")
        print("3. Verify the broker is running")
        print("4. Look for error messages in the terminals")
        return

    # Create figure with 3 subplots (top, front, side views)
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(f'OctoMap Visualization ({len(points)} occupied voxels)',
                 fontsize=16, fontweight='bold')

    # Color by height (z-coordinate)
    z_min, z_max = points[:, 2].min(), points[:, 2].max()
    z_range = z_max - z_min if z_max > z_min else 1.0
    colors = (points[:, 2] - z_min) / z_range

    # Top view (X-Y plane)
    ax1 = axes[0, 0]
    scatter1 = ax1.scatter(points[:, 0], points[:, 1], c=colors, cmap='viridis',
                           marker='.', s=5, alpha=0.6)
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('Top View (X-Y)')
    ax1.set_aspect('equal', adjustable='box')
    ax1.grid(True, alpha=0.3)

    # Front view (X-Z plane)
    ax2 = axes[0, 1]
    scatter2 = ax2.scatter(points[:, 0], points[:, 2], c=colors, cmap='viridis',
                           marker='.', s=5, alpha=0.6)
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Z (m)')
    ax2.set_title('Front View (X-Z)')
    ax2.set_aspect('equal', adjustable='box')
    ax2.grid(True, alpha=0.3)

    # Side view (Y-Z plane)
    ax3 = axes[1, 0]
    scatter3 = ax3.scatter(points[:, 1], points[:, 2], c=colors, cmap='viridis',
                           marker='.', s=5, alpha=0.6)
    ax3.set_xlabel('Y (m)')
    ax3.set_ylabel('Z (m)')
    ax3.set_title('Side View (Y-Z)')
    ax3.set_aspect('equal', adjustable='box')
    ax3.grid(True, alpha=0.3)

    # Statistics panel
    ax4 = axes[1, 1]
    ax4.axis('off')

    stats_text = f"""
    Statistics:
    ───────────────────────────
    Total Voxels: {len(points):,}
    
    X Range: [{points[:, 0].min():.3f}, {points[:, 0].max():.3f}] m
    Y Range: [{points[:, 1].min():.3f}, {points[:, 1].max():.3f}] m
    Z Range: [{points[:, 2].min():.3f}, {points[:, 2].max():.3f}] m
    
    Centroid:
      X: {points[:, 0].mean():.3f} m
      Y: {points[:, 1].mean():.3f} m
      Z: {points[:, 2].mean():.3f} m
    """

    ax4.text(0.1, 0.5, stats_text, fontsize=11, family='monospace',
             verticalalignment='center', transform=ax4.transAxes)

    # Add colorbar
    cbar = plt.colorbar(scatter1, ax=axes.ravel().tolist(),
                        pad=0.02, shrink=0.6, aspect=20)
    cbar.set_label('Height (Z)', rotation=270, labelpad=15)

    plt.tight_layout()
    print("\nShowing visualization window...")
    print("Close the window to exit")
    plt.show()


def main():
    if len(sys.argv) < 2:
        filename = "build/global_map.ot"
        print(f"Usage: {sys.argv[0]} <octomap_file.ot>")
        print(f"Using default: {filename}")
    else:
        filename = sys.argv[1]

    print(f"\nLoading OctoMap from: {filename}")
    print("-" * 60)

    points = read_octomap(filename)

    print(f"\nLoaded {len(points)} occupied voxels")

    if len(points) > 0:
        print(f"\nPoint cloud bounds:")
        print(f"  X: [{points[:, 0].min():.3f}, {points[:, 0].max():.3f}] m")
        print(f"  Y: [{points[:, 1].min():.3f}, {points[:, 1].max():.3f}] m")
        print(f"  Z: [{points[:, 2].min():.3f}, {points[:, 2].max():.3f}] m")

        # Calculate centroid
        centroid = points.mean(axis=0)
        print(
            f"\nCentroid: [{centroid[0]:.3f}, {centroid[1]:.3f}, {centroid[2]:.3f}] m")

    visualize_pointcloud_matplotlib(points)


if __name__ == "__main__":
    main()

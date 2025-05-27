#!/usr/bin/env python3
"""
ARCore Data Visualization Script
Visualizes 6-DoF pose estimation and 3D point cloud data from ARCore Data Logger

Requirements:
- numpy
- matplotlib
- pandas (optional, for better data handling)

Usage:
python visualize_arcore_data.py [data_folder_path]
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import sys
import argparse
from pathlib import Path

class ARCoreDataVisualizer:
    def __init__(self, data_folder):
        self.data_folder = Path(data_folder)
        self.pose_data = None
        self.point_cloud_data = None
        
    def load_pose_data(self):
        """Load 6-DoF pose data from ARCore_sensor_pose.txt"""
        pose_file = self.data_folder / "ARCore_sensor_pose.txt"
        
        if not pose_file.exists():
            print(f"Error: {pose_file} not found!")
            return False
            
        try:
            # Skip header line and load data
            data = np.loadtxt(pose_file, skiprows=1)
            
            if data.size == 0:
                print("Warning: No pose data found!")
                return False
            
            # Filter out invalid data (NaN, inf, extremely large values)
            # Check for NaN values
            valid_mask = ~np.isnan(data).any(axis=1)
            
            # Check for infinite values
            valid_mask &= ~np.isinf(data).any(axis=1)
            
            # Check for reasonable position values (threshold: 1000 meters)
            position_threshold = 1000.0
            valid_mask &= (np.abs(data[:, 5]) < position_threshold)  # tx
            valid_mask &= (np.abs(data[:, 6]) < position_threshold)  # ty
            valid_mask &= (np.abs(data[:, 7]) < position_threshold)  # tz
            
            # Check for valid quaternion values (should be normalized, so each component should be <= 1)
            valid_mask &= (np.abs(data[:, 1]) <= 1.0)  # qx
            valid_mask &= (np.abs(data[:, 2]) <= 1.0)  # qy
            valid_mask &= (np.abs(data[:, 3]) <= 1.0)  # qz
            valid_mask &= (np.abs(data[:, 4]) <= 1.0)  # qw
            
            # Apply filter
            filtered_data = data[valid_mask]
            
            if len(filtered_data) == 0:
                print("Warning: No valid pose data after filtering!")
                return False
            
            print(f"Loaded {len(data)} pose samples, {len(filtered_data)} valid after filtering")
            if len(filtered_data) < len(data):
                print(f"Filtered out {len(data) - len(filtered_data)} invalid pose samples")
                
            # Data format: timestamp, qx, qy, qz, qw, tx, ty, tz
            self.pose_data = {
                'timestamp': filtered_data[:, 0],
                'quaternion': filtered_data[:, 1:5],  # qx, qy, qz, qw
                'position': filtered_data[:, 5:8]     # tx, ty, tz
            }
            
            return True
            
        except Exception as e:
            print(f"Error loading pose data: {e}")
            return False
    
    def load_point_cloud_data(self):
        """Load 3D point cloud data from ARCore_point_cloud.txt"""
        pc_file = self.data_folder / "ARCore_point_cloud.txt"
        
        if not pc_file.exists():
            print(f"Warning: {pc_file} not found!")
            return False
            
        try:
            # Skip header line and load data
            data = np.loadtxt(pc_file, skiprows=1)
            
            if data.size == 0:
                print("Warning: No point cloud data found!")
                return False
            
            # Filter out invalid data (NaN, inf, extremely large values)
            # Check for NaN values
            valid_mask = ~np.isnan(data).any(axis=1)
            
            # Check for infinite values
            valid_mask &= ~np.isinf(data).any(axis=1)
            
            # Check for extremely large values (threshold: 1000 meters)
            position_threshold = 1000.0
            valid_mask &= (np.abs(data[:, 0]) < position_threshold)
            valid_mask &= (np.abs(data[:, 1]) < position_threshold)
            valid_mask &= (np.abs(data[:, 2]) < position_threshold)
            
            # Check for valid color values (0-255)
            valid_mask &= (data[:, 3] >= 0) & (data[:, 3] <= 255)
            valid_mask &= (data[:, 4] >= 0) & (data[:, 4] <= 255)
            valid_mask &= (data[:, 5] >= 0) & (data[:, 5] <= 255)
            
            # Apply filter
            filtered_data = data[valid_mask]
            
            if len(filtered_data) == 0:
                print("Warning: No valid point cloud data after filtering!")
                return False
            
            print(f"Loaded {len(data)} point cloud points, {len(filtered_data)} valid after filtering")
            if len(filtered_data) < len(data):
                print(f"Filtered out {len(data) - len(filtered_data)} invalid points")
                
            # Data format: position_x, position_y, position_z, color_R, color_G, color_B
            self.point_cloud_data = {
                'positions': filtered_data[:, 0:3],   # x, y, z
                'colors': filtered_data[:, 3:6] / 255.0  # RGB normalized to [0,1]
            }
            
            return True
            
        except Exception as e:
            print(f"Error loading point cloud data: {e}")
            return False
    
    def quaternion_to_rotation_matrix(self, q):
        """Convert quaternion to rotation matrix"""
        qx, qy, qz, qw = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
        
        # Normalize quaternion
        norm = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
        qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
        
        # Convert to rotation matrix
        R = np.zeros((len(q), 3, 3))
        
        R[:, 0, 0] = 1 - 2*(qy**2 + qz**2)
        R[:, 0, 1] = 2*(qx*qy - qz*qw)
        R[:, 0, 2] = 2*(qx*qz + qy*qw)
        
        R[:, 1, 0] = 2*(qx*qy + qz*qw)
        R[:, 1, 1] = 1 - 2*(qx**2 + qz**2)
        R[:, 1, 2] = 2*(qy*qz - qx*qw)
        
        R[:, 2, 0] = 2*(qx*qz - qy*qw)
        R[:, 2, 1] = 2*(qy*qz + qx*qw)
        R[:, 2, 2] = 1 - 2*(qx**2 + qy**2)
        
        return R
    
    def plot_trajectory_3d(self):
        """Plot 3D trajectory of device movement"""
        if self.pose_data is None:
            print("No pose data available for trajectory plot")
            return
            
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        positions = self.pose_data['position']
        
        # Plot trajectory
        ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 
                'b-', linewidth=2, label='Device Trajectory')
        
        # Mark start and end points
        ax.scatter(positions[0, 0], positions[0, 1], positions[0, 2], 
                  c='green', s=100, label='Start', marker='o')
        ax.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2], 
                  c='red', s=100, label='End', marker='s')
        
        # Add coordinate frames at regular intervals
        step = max(1, len(positions) // 10)
        R = self.quaternion_to_rotation_matrix(self.pose_data['quaternion'])
        
        for i in range(0, len(positions), step):
            pos = positions[i]
            rot = R[i]
            
            # Draw coordinate frame (scaled down)
            scale = 0.05
            for j, color in enumerate(['red', 'green', 'blue']):
                end_pos = pos + scale * rot[:, j]
                ax.plot([pos[0], end_pos[0]], [pos[1], end_pos[1]], [pos[2], end_pos[2]], 
                       color=color, alpha=0.6, linewidth=1)
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('ARCore 6-DoF Device Trajectory')
        ax.legend()
        ax.grid(True)
        
        # Equal aspect ratio
        max_range = np.array([positions[:, 0].max() - positions[:, 0].min(),
                             positions[:, 1].max() - positions[:, 1].min(),
                             positions[:, 2].max() - positions[:, 2].min()]).max() / 2.0
        mid_x = (positions[:, 0].max() + positions[:, 0].min()) * 0.5
        mid_y = (positions[:, 1].max() + positions[:, 1].min()) * 0.5
        mid_z = (positions[:, 2].max() + positions[:, 2].min()) * 0.5
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)
        
        plt.tight_layout()
        plt.show()
    
    def plot_pose_components(self):
        """Plot pose components over time"""
        if self.pose_data is None:
            print("No pose data available for pose component plot")
            return
            
        timestamps = self.pose_data['timestamp']
        # Convert to relative time in seconds
        time_sec = (timestamps - timestamps[0]) / 1e9
        
        positions = self.pose_data['position']
        quaternions = self.pose_data['quaternion']
        
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        
        # Position plot
        axes[0, 0].plot(time_sec, positions[:, 0], 'r-', label='X')
        axes[0, 0].plot(time_sec, positions[:, 1], 'g-', label='Y')
        axes[0, 0].plot(time_sec, positions[:, 2], 'b-', label='Z')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Position (m)')
        axes[0, 0].set_title('Device Position Over Time')
        axes[0, 0].legend()
        axes[0, 0].grid(True)
        
        # Quaternion plot
        axes[0, 1].plot(time_sec, quaternions[:, 0], 'r-', label='qx')
        axes[0, 1].plot(time_sec, quaternions[:, 1], 'g-', label='qy')
        axes[0, 1].plot(time_sec, quaternions[:, 2], 'b-', label='qz')
        axes[0, 1].plot(time_sec, quaternions[:, 3], 'm-', label='qw')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Quaternion')
        axes[0, 1].set_title('Device Orientation (Quaternion) Over Time')
        axes[0, 1].legend()
        axes[0, 1].grid(True)
        
        # Velocity plot (numerical derivative)
        if len(positions) > 1:
            dt = np.diff(time_sec)
            velocity = np.diff(positions, axis=0) / dt[:, np.newaxis]
            speed = np.linalg.norm(velocity, axis=1)
            
            axes[1, 0].plot(time_sec[1:], speed, 'k-', linewidth=2)
            axes[1, 0].set_xlabel('Time (s)')
            axes[1, 0].set_ylabel('Speed (m/s)')
            axes[1, 0].set_title('Device Speed Over Time')
            axes[1, 0].grid(True)
        
        # Distance from origin
        distance = np.linalg.norm(positions, axis=1)
        axes[1, 1].plot(time_sec, distance, 'purple', linewidth=2)
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].set_ylabel('Distance (m)')
        axes[1, 1].set_title('Distance from Origin Over Time')
        axes[1, 1].grid(True)
        
        plt.tight_layout()
        plt.show()
    
    def plot_point_cloud(self):
        """Plot 3D point cloud"""
        if self.point_cloud_data is None:
            print("No point cloud data available")
            return
            
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        positions = self.point_cloud_data['positions']
        colors = self.point_cloud_data['colors']
        
        # Plot point cloud
        scatter = ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2], 
                           c=colors, s=20, alpha=0.6)
        
        # Also plot device trajectory if available
        if self.pose_data is not None:
            device_pos = self.pose_data['position']
            ax.plot(device_pos[:, 0], device_pos[:, 1], device_pos[:, 2], 
                   'r-', linewidth=3, label='Device Path')
            ax.legend()
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('ARCore 3D Point Cloud')
        ax.grid(True)
        
        plt.tight_layout()
        plt.show()
    
    def generate_summary_report(self):
        """Generate a summary report of the data"""
        print("\n" + "="*50)
        print("ARCore Data Summary Report")
        print("="*50)
        
        if self.pose_data is not None:
            timestamps = self.pose_data['timestamp']
            positions = self.pose_data['position']
            
            duration = (timestamps[-1] - timestamps[0]) / 1e9
            total_distance = np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1))
            
            print(f"Pose Data:")
            print(f"  - Number of samples: {len(timestamps)}")
            print(f"  - Duration: {duration:.2f} seconds")
            print(f"  - Average update rate: {len(timestamps)/duration:.1f} Hz")
            print(f"  - Total distance traveled: {total_distance:.3f} m")
            print(f"  - Position range:")
            print(f"    X: [{positions[:, 0].min():.3f}, {positions[:, 0].max():.3f}] m")
            print(f"    Y: [{positions[:, 1].min():.3f}, {positions[:, 1].max():.3f}] m")
            print(f"    Z: [{positions[:, 2].min():.3f}, {positions[:, 2].max():.3f}] m")
        
        if self.point_cloud_data is not None:
            positions = self.point_cloud_data['positions']
            print(f"\nPoint Cloud Data:")
            print(f"  - Number of points: {len(positions)}")
            print(f"  - Point cloud range:")
            print(f"    X: [{positions[:, 0].min():.3f}, {positions[:, 0].max():.3f}] m")
            print(f"    Y: [{positions[:, 1].min():.3f}, {positions[:, 1].max():.3f}] m")
            print(f"    Z: [{positions[:, 2].min():.3f}, {positions[:, 2].max():.3f}] m")
        
        print("="*50)
    
    def visualize_all(self):
        """Run complete visualization pipeline"""
        print("Loading ARCore data...")
        
        pose_loaded = self.load_pose_data()
        pc_loaded = self.load_point_cloud_data()
        
        if not pose_loaded and not pc_loaded:
            print("Error: No data could be loaded!")
            return
        
        self.generate_summary_report()
        
        if pose_loaded:
            print("\nGenerating trajectory visualization...")
            self.plot_trajectory_3d()
            
            print("Generating pose component plots...")
            self.plot_pose_components()
        
        if pc_loaded:
            print("Generating point cloud visualization...")
            self.plot_point_cloud()
        
        print("Visualization complete!")

def main():
    parser = argparse.ArgumentParser(description='Visualize ARCore Data Logger output')
    parser.add_argument('data_folder', nargs='?', 
                       default='downloaded_logs',
                       help='Path to folder containing ARCore data files')
    
    args = parser.parse_args()
    
    # Find the most recent data folder if a directory with multiple folders is provided
    data_path = Path(args.data_folder)
    
    if not data_path.exists():
        print(f"Error: Path {data_path} does not exist!")
        return
    
    # If it's a directory containing multiple session folders, use the most recent one
    if data_path.is_dir():
        session_folders = [f for f in data_path.iterdir() if f.is_dir() and 'R_pjinkim_ARCore' in f.name]
        if session_folders:
            # Sort by modification time and use the most recent
            data_path = max(session_folders, key=lambda x: x.stat().st_mtime)
            print(f"Using most recent session: {data_path.name}")
    
    # Create visualizer and run
    visualizer = ARCoreDataVisualizer(data_path)
    visualizer.visualize_all()

if __name__ == "__main__":
    main() 
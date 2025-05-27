#!/usr/bin/env python3
"""
ARCore Data Analysis and Results Saver
Analyzes ARCore data and saves all results to the session folder

Usage:
python save_analysis_results.py session_folder_path
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import sys
import json
from pathlib import Path
from datetime import datetime
import argparse

class ARCoreAnalysisResultSaver:
    def __init__(self, session_folder):
        self.session_folder = Path(session_folder)
        self.pose_data = None
        self.point_cloud_data = None
        self.analysis_results = {}
        
        # Create results subfolder
        self.results_folder = self.session_folder / "analysis_results"
        self.results_folder.mkdir(exist_ok=True)
        
    def load_pose_data(self):
        """Load 6-DoF pose data from ARCore_sensor_pose.txt"""
        pose_file = self.session_folder / "ARCore_sensor_pose.txt"
        
        if not pose_file.exists():
            print(f"Error: {pose_file} not found!")
            return False
            
        try:
            # Skip header line and load data
            data = np.loadtxt(pose_file, skiprows=1)
            
            if data.size == 0:
                print("Warning: No pose data found!")
                return False
            
            # Filter out invalid data
            valid_mask = ~np.isnan(data).any(axis=1)
            valid_mask &= ~np.isinf(data).any(axis=1)
            
            # Check for reasonable position values (threshold: 1000 meters)
            position_threshold = 1000.0
            valid_mask &= (np.abs(data[:, 5]) < position_threshold)  # tx
            valid_mask &= (np.abs(data[:, 6]) < position_threshold)  # ty
            valid_mask &= (np.abs(data[:, 7]) < position_threshold)  # tz
            
            # Check for valid quaternion values
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
        pc_file = self.session_folder / "ARCore_point_cloud.txt"
        
        if not pc_file.exists():
            print(f"Warning: {pc_file} not found!")
            return False
            
        try:
            # Skip header line and load data
            data = np.loadtxt(pc_file, skiprows=1)
            
            if data.size == 0:
                print("Warning: No point cloud data found!")
                return False
            
            # Filter out invalid data
            valid_mask = ~np.isnan(data).any(axis=1)
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
                
            # Data format: position_x, position_y, position_z, color_R, color_G, color_B
            self.point_cloud_data = {
                'positions': filtered_data[:, 0:3],   # x, y, z
                'colors': filtered_data[:, 3:6] / 255.0  # RGB normalized to [0,1]
            }
            
            return True
            
        except Exception as e:
            print(f"Error loading point cloud data: {e}")
            return False
    
    def quaternion_to_euler(self, q):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        qx, qy, qz, qw = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (qw * qy - qz * qx)
        pitch = np.where(np.abs(sinp) >= 1, np.copysign(np.pi / 2, sinp), np.arcsin(sinp))
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return np.column_stack([roll, pitch, yaw])
    
    def compute_basic_statistics(self):
        """Compute basic statistics from the data"""
        if self.pose_data is None:
            return
        
        timestamps = self.pose_data['timestamp']
        positions = self.pose_data['position']
        quaternions = self.pose_data['quaternion']
        
        # Time analysis
        time_sec = (timestamps - timestamps[0]) / 1e9
        duration = time_sec[-1]
        avg_rate = len(timestamps) / duration
        
        # Motion analysis
        distances = np.linalg.norm(np.diff(positions, axis=0), axis=1)
        total_distance = np.sum(distances)
        velocities = distances / np.diff(time_sec)
        avg_speed = np.mean(velocities)
        max_speed = np.max(velocities)
        
        # Rotation analysis
        euler_angles = self.quaternion_to_euler(quaternions)
        roll_range = np.ptp(euler_angles[:, 0]) * 180 / np.pi
        pitch_range = np.ptp(euler_angles[:, 1]) * 180 / np.pi
        yaw_range = np.ptp(euler_angles[:, 2]) * 180 / np.pi
        
        # Position statistics
        pos_mean = np.mean(positions, axis=0)
        pos_std = np.std(positions, axis=0)
        pos_range = np.ptp(positions, axis=0)
        
        self.analysis_results = {
            'session_info': {
                'session_name': self.session_folder.name,
                'analysis_date': datetime.now().isoformat(),
                'total_samples': len(timestamps),
                'point_cloud_points': len(self.point_cloud_data['positions']) if self.point_cloud_data else 0
            },
            'temporal_analysis': {
                'duration_seconds': float(duration),
                'average_sampling_rate_hz': float(avg_rate),
                'first_timestamp': int(timestamps[0]),
                'last_timestamp': int(timestamps[-1])
            },
            'motion_analysis': {
                'total_distance_m': float(total_distance),
                'average_speed_ms': float(avg_speed),
                'maximum_speed_ms': float(max_speed),
                'speed_std_ms': float(np.std(velocities))
            },
            'rotation_analysis': {
                'roll_range_deg': float(roll_range),
                'pitch_range_deg': float(pitch_range),
                'yaw_range_deg': float(yaw_range)
            },
            'position_statistics': {
                'mean_position': pos_mean.tolist(),
                'std_position': pos_std.tolist(),
                'position_range': pos_range.tolist(),
                'min_position': positions.min(axis=0).tolist(),
                'max_position': positions.max(axis=0).tolist()
            }
        }
        
        if self.point_cloud_data:
            pc_positions = self.point_cloud_data['positions']
            self.analysis_results['point_cloud_statistics'] = {
                'total_points': len(pc_positions),
                'spatial_range': {
                    'x_range': [float(pc_positions[:, 0].min()), float(pc_positions[:, 0].max())],
                    'y_range': [float(pc_positions[:, 1].min()), float(pc_positions[:, 1].max())],
                    'z_range': [float(pc_positions[:, 2].min()), float(pc_positions[:, 2].max())]
                }
            }
    
    def save_trajectory_plot(self):
        """Save 3D trajectory visualization"""
        if self.pose_data is None:
            return
        
        fig = plt.figure(figsize=(15, 12))
        
        # 3D trajectory plot
        ax1 = fig.add_subplot(221, projection='3d')
        positions = self.pose_data['position']
        
        # Color trajectory by time
        colors = plt.cm.viridis(np.linspace(0, 1, len(positions)))
        
        for i in range(len(positions)-1):
            ax1.plot([positions[i][0], positions[i+1][0]], 
                    [positions[i][1], positions[i+1][1]], 
                    [positions[i][2], positions[i+1][2]], 
                    color=colors[i], alpha=0.7, linewidth=2)
        
        # Mark start and end points
        ax1.scatter(positions[0][0], positions[0][1], positions[0][2], 
                   color='green', s=100, label='Start', marker='o')
        ax1.scatter(positions[-1][0], positions[-1][1], positions[-1][2], 
                   color='red', s=100, label='End', marker='s')
        
        ax1.set_xlabel('X (m) - Right/Left')
        ax1.set_ylabel('Y (m) - Up/Down (Gravity: -Y)')
        ax1.set_zlabel('Z (m) - Camera Back/Front')
        ax1.set_title('3D Device Trajectory\n(ARCore World Coordinate System)')
        ax1.legend()
        ax1.grid(True)
        
        # 2D projections
        ax2 = fig.add_subplot(222)
        ax2.plot(positions[:, 0], positions[:, 1], 'b-', alpha=0.7, linewidth=2)
        ax2.scatter(positions[0, 0], positions[0, 1], color='green', s=50, label='Start')
        ax2.scatter(positions[-1, 0], positions[-1, 1], color='red', s=50, label='End')
        ax2.set_xlabel('X (m) - Right/Left')
        ax2.set_ylabel('Y (m) - Up/Down')
        ax2.set_title('Front View (X-Y Plane)\n[Looking along Z-axis from camera direction]')
        ax2.grid(True)
        ax2.legend()
        ax2.axis('equal')
        
        ax3 = fig.add_subplot(223)
        ax3.plot(positions[:, 0], -positions[:, 2], 'r-', alpha=0.7, linewidth=2)
        ax3.scatter(positions[0, 0], -positions[0, 2], color='green', s=50, label='Start')
        ax3.scatter(positions[-1, 0], -positions[-1, 2], color='red', s=50, label='End')
        ax3.set_xlabel('X (m) - Right/Left')
        ax3.set_ylabel('-Z (m) - Camera Front/Back')
        ax3.set_title('Top-Down View (X-Z Plane)\n[Bird\'s Eye View - Looking down from above]')
        ax3.grid(True)
        ax3.legend()
        ax3.axis('equal')
        
        ax4 = fig.add_subplot(224)
        ax4.plot(positions[:, 1], positions[:, 2], 'g-', alpha=0.7, linewidth=2)
        ax4.scatter(positions[0, 1], positions[0, 2], color='green', s=50, label='Start')
        ax4.scatter(positions[-1, 1], positions[-1, 2], color='red', s=50, label='End')
        ax4.set_xlabel('Y (m) - Up/Down')
        ax4.set_ylabel('Z (m) - Camera Back/Front')
        ax4.set_title('Side View (Y-Z Plane)\n[Looking from right side, X-axis into page]')
        ax4.grid(True)
        ax4.legend()
        ax4.axis('equal')
        
        plt.tight_layout()
        plt.savefig(self.results_folder / 'trajectory_3d.png', dpi=300, bbox_inches='tight')
        plt.close()
        print(f"Saved: {self.results_folder / 'trajectory_3d.png'}")
    
    def save_time_series_plots(self):
        """Save time series analysis plots"""
        if self.pose_data is None:
            return
        
        timestamps = self.pose_data['timestamp']
        time_sec = (timestamps - timestamps[0]) / 1e9
        positions = self.pose_data['position']
        quaternions = self.pose_data['quaternion']
        euler_angles = self.quaternion_to_euler(quaternions)
        
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        
        # Position plot
        axes[0, 0].plot(time_sec, positions[:, 0], 'r-', label='X', linewidth=2)
        axes[0, 0].plot(time_sec, positions[:, 1], 'g-', label='Y', linewidth=2)
        axes[0, 0].plot(time_sec, positions[:, 2], 'b-', label='Z', linewidth=2)
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Position (m)')
        axes[0, 0].set_title('Device Position Over Time')
        axes[0, 0].legend()
        axes[0, 0].grid(True)
        
        # Euler angles plot
        axes[0, 1].plot(time_sec, euler_angles[:, 0] * 180/np.pi, 'r-', label='Roll', linewidth=2)
        axes[0, 1].plot(time_sec, euler_angles[:, 1] * 180/np.pi, 'g-', label='Pitch', linewidth=2)
        axes[0, 1].plot(time_sec, euler_angles[:, 2] * 180/np.pi, 'b-', label='Yaw', linewidth=2)
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Angle (degrees)')
        axes[0, 1].set_title('Device Orientation Over Time')
        axes[0, 1].legend()
        axes[0, 1].grid(True)
        
        # Velocity plot
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
        plt.savefig(self.results_folder / 'time_series_analysis.png', dpi=300, bbox_inches='tight')
        plt.close()
        print(f"Saved: {self.results_folder / 'time_series_analysis.png'}")
    
    def save_point_cloud_plot(self):
        """Save point cloud visualization"""
        if self.point_cloud_data is None:
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
        
        ax.set_xlabel('X (m) - Right/Left')
        ax.set_ylabel('Y (m) - Up/Down (Gravity: -Y)')
        ax.set_zlabel('Z (m) - Camera Back/Front')
        ax.set_title('ARCore 3D Point Cloud with Device Trajectory\n(ARCore World Coordinate System)')
        ax.grid(True)
        
        plt.tight_layout()
        plt.savefig(self.results_folder / 'point_cloud_3d.png', dpi=300, bbox_inches='tight')
        plt.close()
        print(f"Saved: {self.results_folder / 'point_cloud_3d.png'}")
    
    def save_analysis_report(self):
        """Save detailed analysis report as JSON and text"""
        # Save as JSON
        json_file = self.results_folder / 'analysis_report.json'
        with open(json_file, 'w') as f:
            json.dump(self.analysis_results, f, indent=2)
        print(f"Saved: {json_file}")
        
        # Save as human-readable text
        txt_file = self.results_folder / 'analysis_report.txt'
        with open(txt_file, 'w') as f:
            f.write("="*60 + "\n")
            f.write("ARCore Data Analysis Report\n")
            f.write("="*60 + "\n\n")
            
            # Session info
            f.write("SESSION INFORMATION:\n")
            f.write("-"*30 + "\n")
            session_info = self.analysis_results['session_info']
            f.write(f"Session Name: {session_info['session_name']}\n")
            f.write(f"Analysis Date: {session_info['analysis_date']}\n")
            f.write(f"Total Pose Samples: {session_info['total_samples']}\n")
            f.write(f"Point Cloud Points: {session_info['point_cloud_points']}\n\n")
            
            # Temporal analysis
            f.write("TEMPORAL ANALYSIS:\n")
            f.write("-"*30 + "\n")
            temporal = self.analysis_results['temporal_analysis']
            f.write(f"Duration: {temporal['duration_seconds']:.2f} seconds\n")
            f.write(f"Average Sampling Rate: {temporal['average_sampling_rate_hz']:.1f} Hz\n\n")
            
            # Motion analysis
            f.write("MOTION ANALYSIS:\n")
            f.write("-"*30 + "\n")
            motion = self.analysis_results['motion_analysis']
            f.write(f"Total Distance: {motion['total_distance_m']:.3f} m\n")
            f.write(f"Average Speed: {motion['average_speed_ms']:.3f} m/s\n")
            f.write(f"Maximum Speed: {motion['maximum_speed_ms']:.3f} m/s\n")
            f.write(f"Speed Std Dev: {motion['speed_std_ms']:.3f} m/s\n\n")
            
            # Rotation analysis
            f.write("ROTATION ANALYSIS:\n")
            f.write("-"*30 + "\n")
            rotation = self.analysis_results['rotation_analysis']
            f.write(f"Roll Range: {rotation['roll_range_deg']:.1f} degrees\n")
            f.write(f"Pitch Range: {rotation['pitch_range_deg']:.1f} degrees\n")
            f.write(f"Yaw Range: {rotation['yaw_range_deg']:.1f} degrees\n\n")
            
            # Position statistics
            f.write("POSITION STATISTICS:\n")
            f.write("-"*30 + "\n")
            pos_stats = self.analysis_results['position_statistics']
            f.write(f"Mean Position: [{pos_stats['mean_position'][0]:.3f}, {pos_stats['mean_position'][1]:.3f}, {pos_stats['mean_position'][2]:.3f}] m\n")
            f.write(f"Std Position: [{pos_stats['std_position'][0]:.3f}, {pos_stats['std_position'][1]:.3f}, {pos_stats['std_position'][2]:.3f}] m\n")
            f.write(f"Position Range: [{pos_stats['position_range'][0]:.3f}, {pos_stats['position_range'][1]:.3f}, {pos_stats['position_range'][2]:.3f}] m\n")
            f.write(f"Min Position: [{pos_stats['min_position'][0]:.3f}, {pos_stats['min_position'][1]:.3f}, {pos_stats['min_position'][2]:.3f}] m\n")
            f.write(f"Max Position: [{pos_stats['max_position'][0]:.3f}, {pos_stats['max_position'][1]:.3f}, {pos_stats['max_position'][2]:.3f}] m\n\n")
            
            # Point cloud statistics
            if 'point_cloud_statistics' in self.analysis_results:
                f.write("POINT CLOUD STATISTICS:\n")
                f.write("-"*30 + "\n")
                pc_stats = self.analysis_results['point_cloud_statistics']
                f.write(f"Total Points: {pc_stats['total_points']}\n")
                spatial = pc_stats['spatial_range']
                f.write(f"X Range: [{spatial['x_range'][0]:.3f}, {spatial['x_range'][1]:.3f}] m\n")
                f.write(f"Y Range: [{spatial['y_range'][0]:.3f}, {spatial['y_range'][1]:.3f}] m\n")
                f.write(f"Z Range: [{spatial['z_range'][0]:.3f}, {spatial['z_range'][1]:.3f}] m\n")
        
        print(f"Saved: {txt_file}")
    
    def run_complete_analysis(self):
        """Run complete analysis and save all results"""
        print(f"Analyzing ARCore session: {self.session_folder.name}")
        print(f"Results will be saved to: {self.results_folder}")
        
        # Load data
        pose_loaded = self.load_pose_data()
        pc_loaded = self.load_point_cloud_data()
        
        if not pose_loaded and not pc_loaded:
            print("Error: No data could be loaded!")
            return False
        
        # Compute statistics
        print("Computing analysis statistics...")
        self.compute_basic_statistics()
        
        # Save visualizations
        print("Generating and saving visualizations...")
        if pose_loaded:
            self.save_trajectory_plot()
            self.save_time_series_plots()
        
        if pc_loaded:
            self.save_point_cloud_plot()
        
        # Save analysis report
        print("Saving analysis report...")
        self.save_analysis_report()
        
        print(f"\nAnalysis complete! All results saved to: {self.results_folder}")
        return True

def main():
    parser = argparse.ArgumentParser(description='Analyze ARCore session and save results')
    parser.add_argument('session_folder', 
                       help='Path to ARCore session folder')
    
    args = parser.parse_args()
    
    session_path = Path(args.session_folder)
    
    if not session_path.exists():
        print(f"Error: Session folder {session_path} does not exist!")
        return
    
    if not session_path.is_dir():
        print(f"Error: {session_path} is not a directory!")
        return
    
    # Create analyzer and run analysis
    analyzer = ARCoreAnalysisResultSaver(session_path)
    success = analyzer.run_complete_analysis()
    
    if success:
        print("\n" + "="*50)
        print("ANALYSIS SUMMARY")
        print("="*50)
        if analyzer.analysis_results:
            session_info = analyzer.analysis_results['session_info']
            temporal = analyzer.analysis_results['temporal_analysis']
            motion = analyzer.analysis_results['motion_analysis']
            
            print(f"Session: {session_info['session_name']}")
            print(f"Duration: {temporal['duration_seconds']:.2f} seconds")
            print(f"Samples: {session_info['total_samples']}")
            print(f"Point Cloud Points: {session_info['point_cloud_points']}")
            print(f"Total Distance: {motion['total_distance_m']:.3f} m")
            print(f"Average Speed: {motion['average_speed_ms']:.3f} m/s")
    else:
        print("Analysis failed!")

if __name__ == "__main__":
    main() 
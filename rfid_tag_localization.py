#!/usr/bin/env python3
"""
RFID Tag Localization using ARCore VIO Data
============================================

This script analyzes ARCore sensor pose data with RFID tag detections
to estimate the 3D position of RFID tags using triangulation and 
weighted centroid methods.

Author: ARCore Data Logger Team
Date: 2025
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
from pathlib import Path
import argparse
from scipy.optimize import minimize
from scipy.spatial.distance import cdist
import warnings
warnings.filterwarnings('ignore')

class RFIDTagLocalizer:
    def __init__(self, session_folder):
        """
        Initialize RFID Tag Localizer
        
        Args:
            session_folder (str): Path to ARCore session folder containing pose data
        """
        self.session_folder = Path(session_folder)
        self.pose_data = None
        self.tag_detections = None
        self.estimated_tag_positions = {}
        
    def load_pose_data(self):
        """Load ARCore sensor pose data with tag detections"""
        pose_file = self.session_folder / "ARCore_sensor_pose.txt"
        
        if not pose_file.exists():
            print(f"Error: {pose_file} not found!")
            return False
            
        try:
            # Read all lines
            with open(pose_file, 'r') as f:
                lines = f.readlines()
            
            # Skip header line
            data_lines = [line.strip() for line in lines[1:] if line.strip()]
            
            if not data_lines:
                print("Warning: No pose data found!")
                return False
            
            # Parse data - handle both formats (with and without tag ID)
            pose_data = []
            tag_detections = []
            
            for line in data_lines:
                parts = line.split()
                if len(parts) >= 8:  # timestamp + 7 pose values
                    timestamp = int(parts[0])
                    qx, qy, qz, qw = map(float, parts[1:5])
                    tx, ty, tz = map(float, parts[5:8])
                    
                    pose_data.append([timestamp, qx, qy, qz, qw, tx, ty, tz])
                    
                    # Check if tag ID is present
                    if len(parts) >= 9:
                        tag_id = parts[8]
                        tag_detections.append({
                            'timestamp': timestamp,
                            'tag_id': tag_id,
                            'position': np.array([tx, ty, tz]),
                            'quaternion': np.array([qx, qy, qz, qw])
                        })
            
            if not pose_data:
                print("Warning: No valid pose data found!")
                return False
            
            # Convert to numpy array
            pose_array = np.array(pose_data)
            
            # Filter out invalid data
            valid_mask = ~np.isnan(pose_array).any(axis=1)
            valid_mask &= ~np.isinf(pose_array).any(axis=1)
            
            # Check for reasonable position values
            position_threshold = 1000.0
            valid_mask &= (np.abs(pose_array[:, 5]) < position_threshold)  # tx
            valid_mask &= (np.abs(pose_array[:, 6]) < position_threshold)  # ty
            valid_mask &= (np.abs(pose_array[:, 7]) < position_threshold)  # tz
            
            # Apply filter
            filtered_data = pose_array[valid_mask]
            
            if len(filtered_data) == 0:
                print("Warning: No valid pose data after filtering!")
                return False
            
            print(f"Loaded {len(pose_data)} pose samples, {len(filtered_data)} valid after filtering")
            print(f"Found {len(tag_detections)} tag detections")
                
            # Store data
            self.pose_data = {
                'timestamp': filtered_data[:, 0],
                'quaternion': filtered_data[:, 1:5],  # qx, qy, qz, qw
                'position': filtered_data[:, 5:8]     # tx, ty, tz
            }
            
            self.tag_detections = tag_detections
            
            return True
            
        except Exception as e:
            print(f"Error loading pose data: {e}")
            return False
    
    def quaternion_to_rotation_matrix(self, q):
        """Convert quaternion to rotation matrix"""
        if q.ndim == 1:
            q = q.reshape(1, -1)
        
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
        
        return R.squeeze() if R.shape[0] == 1 else R
    
    def get_camera_direction(self, quaternion):
        """Get camera forward direction from quaternion"""
        R = self.quaternion_to_rotation_matrix(quaternion)
        if R.ndim == 2:
            # Single rotation matrix
            camera_forward = -R[:, 2]  # Negative Z-axis in camera frame
        else:
            # Multiple rotation matrices
            camera_forward = -R[:, :, 2]  # Negative Z-axis in camera frame
        return camera_forward
    
    def estimate_tag_position_triangulation(self, tag_id, max_distance=5.0):
        """
        Estimate tag position using triangulation from multiple detection points
        
        Args:
            tag_id (str): Tag ID to localize
            max_distance (float): Maximum assumed distance to tag (meters)
        
        Returns:
            np.array: Estimated 3D position of the tag
        """
        # Get all detections for this tag
        tag_detections = [d for d in self.tag_detections if d['tag_id'] == tag_id]
        
        if len(tag_detections) < 2:
            print(f"Warning: Need at least 2 detections for triangulation, found {len(tag_detections)}")
            return None
        
        print(f"Estimating position for {tag_id} using {len(tag_detections)} detections")
        
        # Extract detection data
        positions = np.array([d['position'] for d in tag_detections])
        quaternions = np.array([d['quaternion'] for d in tag_detections])
        
        # Get camera directions
        directions = []
        for q in quaternions:
            direction = self.get_camera_direction(q)
            directions.append(direction)
        directions = np.array(directions)
        
        # Method 1: Weighted centroid based on detection geometry
        if len(tag_detections) >= 3:
            estimated_pos = self.estimate_by_weighted_centroid(positions, directions, max_distance)
        else:
            # Method 2: Simple ray intersection for 2 detections
            estimated_pos = self.estimate_by_ray_intersection(positions, directions, max_distance)
        
        return estimated_pos
    
    def estimate_by_weighted_centroid(self, positions, directions, max_distance):
        """Estimate tag position using weighted centroid method"""
        
        def objective_function(tag_pos):
            """Objective function to minimize"""
            total_error = 0
            
            for i, (pos, direction) in enumerate(zip(positions, directions)):
                # Vector from camera to estimated tag position
                to_tag = tag_pos - pos
                distance = np.linalg.norm(to_tag)
                
                if distance > 0:
                    to_tag_normalized = to_tag / distance
                    
                    # Angular error (1 - dot product)
                    angular_error = 1 - np.dot(direction, to_tag_normalized)
                    
                    # Distance penalty (prefer closer solutions)
                    distance_penalty = distance / max_distance
                    
                    total_error += angular_error + 0.1 * distance_penalty
            
            return total_error
        
        # Initial guess: centroid of detection positions projected forward
        initial_guess = np.mean(positions, axis=0)
        mean_direction = np.mean(directions, axis=0)
        mean_direction = mean_direction / np.linalg.norm(mean_direction)
        initial_guess += mean_direction * (max_distance / 2)
        
        # Optimize
        result = minimize(objective_function, initial_guess, method='BFGS')
        
        if result.success:
            return result.x
        else:
            print("Warning: Optimization failed, using initial guess")
            return initial_guess
    
    def estimate_by_ray_intersection(self, positions, directions, max_distance):
        """Estimate tag position using ray intersection for 2 detections"""
        
        if len(positions) != 2:
            print("Ray intersection method requires exactly 2 detections")
            return None
        
        p1, p2 = positions[0], positions[1]
        d1, d2 = directions[0], directions[1]
        
        # Find closest point between two rays
        # Ray 1: p1 + t1 * d1
        # Ray 2: p2 + t2 * d2
        
        # Solve for t1 and t2 that minimize distance between rays
        w = p1 - p2
        a = np.dot(d1, d1)
        b = np.dot(d1, d2)
        c = np.dot(d2, d2)
        d = np.dot(d1, w)
        e = np.dot(d2, w)
        
        denom = a * c - b * b
        
        if abs(denom) < 1e-6:
            # Rays are parallel, use midpoint
            print("Warning: Rays are parallel, using midpoint method")
            return (p1 + p2) / 2 + (d1 + d2) / 2 * (max_distance / 2)
        
        t1 = (b * e - c * d) / denom
        t2 = (a * e - b * d) / denom
        
        # Constrain to reasonable distances
        t1 = np.clip(t1, 0.1, max_distance)
        t2 = np.clip(t2, 0.1, max_distance)
        
        # Calculate points on each ray
        point1 = p1 + t1 * d1
        point2 = p2 + t2 * d2
        
        # Return midpoint
        return (point1 + point2) / 2
    
    def localize_all_tags(self, max_distance=5.0):
        """Localize all detected tags"""
        if not self.tag_detections:
            print("No tag detections found")
            return
        
        # Get unique tag IDs
        tag_ids = list(set(d['tag_id'] for d in self.tag_detections))
        
        print(f"Localizing {len(tag_ids)} unique tags: {tag_ids}")
        
        for tag_id in tag_ids:
            estimated_pos = self.estimate_tag_position_triangulation(tag_id, max_distance)
            if estimated_pos is not None:
                self.estimated_tag_positions[tag_id] = estimated_pos
                print(f"Tag {tag_id} estimated position: ({estimated_pos[0]:.3f}, {estimated_pos[1]:.3f}, {estimated_pos[2]:.3f})")
    
    def plot_results(self):
        """Plot VIO trajectory, tag detections, and estimated tag positions"""
        if self.pose_data is None:
            print("No pose data available")
            return
        
        fig = plt.figure(figsize=(15, 10))
        
        # 3D trajectory plot
        ax1 = fig.add_subplot(221, projection='3d')
        
        # Plot device trajectory
        positions = self.pose_data['position']
        ax1.plot(positions[:, 0], positions[:, 1], positions[:, 2], 
                'b-', linewidth=2, label='Device Trajectory', alpha=0.7)
        
        # Plot tag detection points
        if self.tag_detections:
            detection_positions = np.array([d['position'] for d in self.tag_detections])
            ax1.scatter(detection_positions[:, 0], detection_positions[:, 1], detection_positions[:, 2],
                       c='red', s=100, marker='o', label='Tag Detections', alpha=0.8)
            
            # Plot camera directions at detection points
            for detection in self.tag_detections:
                pos = detection['position']
                direction = self.get_camera_direction(detection['quaternion'])
                end_pos = pos + direction * 1.0  # 1 meter arrow
                ax1.plot([pos[0], end_pos[0]], [pos[1], end_pos[1]], [pos[2], end_pos[2]], 
                        'r-', alpha=0.6, linewidth=1)
        
        # Plot estimated tag positions
        for tag_id, tag_pos in self.estimated_tag_positions.items():
            ax1.scatter(tag_pos[0], tag_pos[1], tag_pos[2], 
                       c='green', s=200, marker='*', label=f'Estimated {tag_id}')
        
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.set_title('3D RFID Tag Localization')
        ax1.legend()
        ax1.grid(True)
        
        # Top view (X-Y plane)
        ax2 = fig.add_subplot(222)
        ax2.plot(positions[:, 0], positions[:, 1], 'b-', linewidth=2, label='Device Path')
        
        if self.tag_detections:
            detection_positions = np.array([d['position'] for d in self.tag_detections])
            ax2.scatter(detection_positions[:, 0], detection_positions[:, 1],
                       c='red', s=100, marker='o', label='Tag Detections')
        
        for tag_id, tag_pos in self.estimated_tag_positions.items():
            ax2.scatter(tag_pos[0], tag_pos[1], c='green', s=200, marker='*', label=f'Est. {tag_id}')
        
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        ax2.set_title('Top View (X-Y)')
        ax2.legend()
        ax2.grid(True)
        ax2.axis('equal')
        
        # Side view (X-Z plane) - Corrected to match ARCore coordinate system
        ax3 = fig.add_subplot(223)
        # Use -Z for consistency with visualize_arcore_data.py (Top-Down View perspective)
        ax3.plot(positions[:, 0], -positions[:, 2], 'b-', linewidth=2, label='Device Path')
        
        if self.tag_detections:
            detection_positions = np.array([d['position'] for d in self.tag_detections])
            ax3.scatter(detection_positions[:, 0], -detection_positions[:, 2],
                       c='red', s=100, marker='o', label='Tag Detections')
            
            # Plot camera directions at detection points in X-Z plane
            for detection in self.tag_detections:
                pos = detection['position']
                direction = self.get_camera_direction(detection['quaternion'])
                # Project to X-Z plane with corrected Z axis
                end_pos_x = pos[0] + direction[0] * 1.0  # 1 meter arrow in X direction
                end_pos_z = -pos[2] - direction[2] * 1.0  # 1 meter arrow in -Z direction (corrected)
                ax3.plot([pos[0], end_pos_x], [-pos[2], end_pos_z], 
                        'r-', alpha=0.6, linewidth=1)
        
        for tag_id, tag_pos in self.estimated_tag_positions.items():
            ax3.scatter(tag_pos[0], -tag_pos[2], c='green', s=200, marker='*', label=f'Est. {tag_id}')
        
        ax3.set_xlabel('X (m)')
        ax3.set_ylabel('-Z (m)')
        ax3.set_title('Side View (X-Z)')
        ax3.legend()
        ax3.grid(True)
        
        # Detection timeline
        ax4 = fig.add_subplot(224)
        if self.tag_detections:
            timestamps = [d['timestamp'] for d in self.tag_detections]
            tag_ids = [d['tag_id'] for d in self.tag_detections]
            
            # Convert timestamps to relative time (seconds)
            start_time = min(timestamps)
            relative_times = [(t - start_time) / 1e9 for t in timestamps]
            
            # Create scatter plot
            unique_tags = list(set(tag_ids))
            colors = plt.cm.tab10(np.linspace(0, 1, len(unique_tags)))
            
            for i, tag_id in enumerate(unique_tags):
                tag_times = [t for t, tid in zip(relative_times, tag_ids) if tid == tag_id]
                tag_y = [i] * len(tag_times)
                ax4.scatter(tag_times, tag_y, c=[colors[i]], s=100, label=tag_id)
            
            ax4.set_xlabel('Time (seconds)')
            ax4.set_ylabel('Tag ID')
            ax4.set_title('Tag Detection Timeline')
            ax4.set_yticks(range(len(unique_tags)))
            ax4.set_yticklabels(unique_tags)
            ax4.grid(True)
            ax4.legend()
        
        plt.tight_layout()
        
        # Save plot to file
        output_file = "rfid_localization_results.png"
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Plot saved to: {output_file}")
        
        # Also try to show (may not work in non-interactive environment)
        try:
            plt.show()
        except:
            print("Interactive display not available, plot saved to file instead")
    
    def generate_report(self):
        """Generate a summary report"""
        print("\n" + "="*60)
        print("RFID Tag Localization Report")
        print("="*60)
        
        if self.pose_data is not None:
            timestamps = self.pose_data['timestamp']
            positions = self.pose_data['position']
            
            duration = (timestamps[-1] - timestamps[0]) / 1e9
            total_distance = np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1))
            
            print(f"VIO Trajectory:")
            print(f"  - Duration: {duration:.2f} seconds")
            print(f"  - Total samples: {len(timestamps)}")
            print(f"  - Average update rate: {len(timestamps)/duration:.1f} Hz")
            print(f"  - Total distance: {total_distance:.3f} m")
            print(f"  - Position range:")
            print(f"    X: [{positions[:, 0].min():.3f}, {positions[:, 0].max():.3f}] m")
            print(f"    Y: [{positions[:, 1].min():.3f}, {positions[:, 1].max():.3f}] m")
            print(f"    Z: [{positions[:, 2].min():.3f}, {positions[:, 2].max():.3f}] m")
        
        if self.tag_detections:
            print(f"\nTag Detections:")
            print(f"  - Total detections: {len(self.tag_detections)}")
            
            tag_counts = {}
            for detection in self.tag_detections:
                tag_id = detection['tag_id']
                tag_counts[tag_id] = tag_counts.get(tag_id, 0) + 1
            
            for tag_id, count in tag_counts.items():
                print(f"  - {tag_id}: {count} detections")
        
        if self.estimated_tag_positions:
            print(f"\nEstimated Tag Positions:")
            for tag_id, position in self.estimated_tag_positions.items():
                print(f"  - {tag_id}: ({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}) m")
        
        print("="*60)

def main():
    parser = argparse.ArgumentParser(description='RFID Tag Localization using ARCore VIO')
    parser.add_argument('session_folder', help='Path to ARCore session folder')
    parser.add_argument('--max-distance', type=float, default=5.0, 
                       help='Maximum assumed distance to tag (meters)')
    parser.add_argument('--no-plot', action='store_true', 
                       help='Skip plotting results')
    
    args = parser.parse_args()
    
    # Initialize localizer
    localizer = RFIDTagLocalizer(args.session_folder)
    
    # Load data
    if not localizer.load_pose_data():
        print("Failed to load pose data")
        return
    
    # Localize tags
    localizer.localize_all_tags(max_distance=args.max_distance)
    
    # Generate report
    localizer.generate_report()
    
    # Plot results
    if not args.no_plot:
        localizer.plot_results()

if __name__ == "__main__":
    main() 
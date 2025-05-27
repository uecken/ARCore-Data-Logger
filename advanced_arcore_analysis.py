#!/usr/bin/env python3
"""
Advanced ARCore Data Analysis Script
Provides detailed analysis and visualization of ARCore data including:
- Rotation analysis (Euler angles)
- Acceleration and jerk analysis
- Frequency domain analysis
- Statistical analysis
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse
from pathlib import Path
from scipy import signal
from scipy.fft import fft, fftfreq

class AdvancedARCoreAnalyzer:
    def __init__(self, data_folder):
        self.data_folder = Path(data_folder)
        self.pose_data = None
        
    def load_pose_data(self):
        """Load 6-DoF pose data from ARCore_sensor_pose.txt"""
        pose_file = self.data_folder / "ARCore_sensor_pose.txt"
        
        if not pose_file.exists():
            print(f"Error: {pose_file} not found!")
            return False
            
        try:
            data = np.loadtxt(pose_file, skiprows=1)
            if data.size == 0:
                print("Warning: No pose data found!")
                return False
                
            self.pose_data = {
                'timestamp': data[:, 0],
                'quaternion': data[:, 1:5],  # qx, qy, qz, qw
                'position': data[:, 5:8]     # tx, ty, tz
            }
            
            print(f"Loaded {len(data)} pose samples")
            return True
            
        except Exception as e:
            print(f"Error loading pose data: {e}")
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
    
    def compute_derivatives(self):
        """Compute velocity, acceleration, and jerk"""
        timestamps = self.pose_data['timestamp']
        positions = self.pose_data['position']
        
        # Convert timestamps to seconds
        time_sec = (timestamps - timestamps[0]) / 1e9
        dt = np.diff(time_sec)
        
        # Velocity (first derivative)
        velocity = np.diff(positions, axis=0) / dt[:, np.newaxis]
        
        # Acceleration (second derivative)
        acceleration = np.diff(velocity, axis=0) / dt[1:, np.newaxis]
        
        # Jerk (third derivative)
        jerk = np.diff(acceleration, axis=0) / dt[2:, np.newaxis]
        
        return {
            'time_velocity': time_sec[1:],
            'velocity': velocity,
            'time_acceleration': time_sec[2:],
            'acceleration': acceleration,
            'time_jerk': time_sec[3:],
            'jerk': jerk
        }
    
    def plot_rotation_analysis(self):
        """Plot detailed rotation analysis"""
        if self.pose_data is None:
            print("No pose data available")
            return
            
        timestamps = self.pose_data['timestamp']
        time_sec = (timestamps - timestamps[0]) / 1e9
        quaternions = self.pose_data['quaternion']
        
        # Convert to Euler angles
        euler_angles = self.quaternion_to_euler(quaternions)
        
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        
        # Euler angles plot
        axes[0, 0].plot(time_sec, np.degrees(euler_angles[:, 0]), 'r-', label='Roll')
        axes[0, 0].plot(time_sec, np.degrees(euler_angles[:, 1]), 'g-', label='Pitch')
        axes[0, 0].plot(time_sec, np.degrees(euler_angles[:, 2]), 'b-', label='Yaw')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Angle (degrees)')
        axes[0, 0].set_title('Euler Angles Over Time')
        axes[0, 0].legend()
        axes[0, 0].grid(True)
        
        # Quaternion magnitude (should be ~1)
        q_magnitude = np.linalg.norm(quaternions, axis=1)
        axes[0, 1].plot(time_sec, q_magnitude, 'k-', linewidth=2)
        axes[0, 1].axhline(y=1.0, color='r', linestyle='--', label='Expected (1.0)')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Quaternion Magnitude')
        axes[0, 1].set_title('Quaternion Normalization Check')
        axes[0, 1].legend()
        axes[0, 1].grid(True)
        
        # Angular velocity (approximate)
        if len(quaternions) > 1:
            dt = np.diff(time_sec)
            # Approximate angular velocity from quaternion differences
            q_diff = np.diff(quaternions, axis=0)
            angular_vel_approx = np.linalg.norm(q_diff, axis=1) / dt
            
            axes[1, 0].plot(time_sec[1:], angular_vel_approx, 'purple', linewidth=2)
            axes[1, 0].set_xlabel('Time (s)')
            axes[1, 0].set_ylabel('Angular Velocity (approx)')
            axes[1, 0].set_title('Approximate Angular Velocity')
            axes[1, 0].grid(True)
        
        # Rotation stability analysis
        euler_std = np.std(euler_angles, axis=0)
        axes[1, 1].bar(['Roll', 'Pitch', 'Yaw'], np.degrees(euler_std), 
                      color=['red', 'green', 'blue'], alpha=0.7)
        axes[1, 1].set_ylabel('Standard Deviation (degrees)')
        axes[1, 1].set_title('Rotation Stability Analysis')
        axes[1, 1].grid(True, axis='y')
        
        plt.tight_layout()
        plt.show()
    
    def plot_motion_analysis(self):
        """Plot detailed motion analysis"""
        if self.pose_data is None:
            print("No pose data available")
            return
            
        derivatives = self.compute_derivatives()
        
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        
        # Velocity components
        velocity = derivatives['velocity']
        time_vel = derivatives['time_velocity']
        
        axes[0, 0].plot(time_vel, velocity[:, 0], 'r-', label='Vx')
        axes[0, 0].plot(time_vel, velocity[:, 1], 'g-', label='Vy')
        axes[0, 0].plot(time_vel, velocity[:, 2], 'b-', label='Vz')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Velocity (m/s)')
        axes[0, 0].set_title('Velocity Components')
        axes[0, 0].legend()
        axes[0, 0].grid(True)
        
        # Speed (velocity magnitude)
        speed = np.linalg.norm(velocity, axis=1)
        axes[0, 1].plot(time_vel, speed, 'k-', linewidth=2)
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Speed (m/s)')
        axes[0, 1].set_title('Speed Over Time')
        axes[0, 1].grid(True)
        
        # Acceleration components
        if 'acceleration' in derivatives:
            acceleration = derivatives['acceleration']
            time_acc = derivatives['time_acceleration']
            
            axes[1, 0].plot(time_acc, acceleration[:, 0], 'r-', label='Ax')
            axes[1, 0].plot(time_acc, acceleration[:, 1], 'g-', label='Ay')
            axes[1, 0].plot(time_acc, acceleration[:, 2], 'b-', label='Az')
            axes[1, 0].set_xlabel('Time (s)')
            axes[1, 0].set_ylabel('Acceleration (m/s²)')
            axes[1, 0].set_title('Acceleration Components')
            axes[1, 0].legend()
            axes[1, 0].grid(True)
        
        # Jerk (acceleration magnitude)
        if 'jerk' in derivatives:
            jerk = derivatives['jerk']
            time_jerk = derivatives['time_jerk']
            jerk_magnitude = np.linalg.norm(jerk, axis=1)
            
            axes[1, 1].plot(time_jerk, jerk_magnitude, 'purple', linewidth=2)
            axes[1, 1].set_xlabel('Time (s)')
            axes[1, 1].set_ylabel('Jerk Magnitude (m/s³)')
            axes[1, 1].set_title('Jerk (Rate of Acceleration Change)')
            axes[1, 1].grid(True)
        
        plt.tight_layout()
        plt.show()
    
    def plot_frequency_analysis(self):
        """Plot frequency domain analysis"""
        if self.pose_data is None:
            print("No pose data available")
            return
            
        timestamps = self.pose_data['timestamp']
        positions = self.pose_data['position']
        
        # Convert to uniform time sampling
        time_sec = (timestamps - timestamps[0]) / 1e9
        
        # Interpolate to uniform sampling if needed
        if len(np.unique(np.diff(time_sec))) > 1:
            # Non-uniform sampling, interpolate
            uniform_time = np.linspace(time_sec[0], time_sec[-1], len(time_sec))
            uniform_positions = np.zeros_like(positions)
            for i in range(3):
                uniform_positions[:, i] = np.interp(uniform_time, time_sec, positions[:, i])
            time_sec = uniform_time
            positions = uniform_positions
        
        # Compute sampling frequency
        fs = 1.0 / np.mean(np.diff(time_sec))
        
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        
        # FFT for each position component
        for i, (component, color) in enumerate(zip(['X', 'Y', 'Z'], ['red', 'green', 'blue'])):
            if i < 3:  # Only plot first 3 components
                # Compute FFT
                fft_vals = fft(positions[:, i])
                freqs = fftfreq(len(positions), 1/fs)
                
                # Plot only positive frequencies
                positive_freqs = freqs[:len(freqs)//2]
                magnitude = np.abs(fft_vals[:len(freqs)//2])
                
                if i == 0:
                    ax = axes[0, 0]
                elif i == 1:
                    ax = axes[0, 1]
                else:
                    ax = axes[1, 0]
                
                ax.plot(positive_freqs, magnitude, color=color, linewidth=2)
                ax.set_xlabel('Frequency (Hz)')
                ax.set_ylabel('Magnitude')
                ax.set_title(f'FFT - Position {component}')
                ax.grid(True)
                ax.set_xlim(0, min(10, fs/2))  # Limit to 10 Hz or Nyquist frequency
        
        # Combined power spectral density
        combined_signal = np.linalg.norm(positions, axis=1)
        freqs, psd = signal.welch(combined_signal, fs, nperseg=min(256, len(combined_signal)//4))
        
        axes[1, 1].semilogy(freqs, psd, 'k-', linewidth=2)
        axes[1, 1].set_xlabel('Frequency (Hz)')
        axes[1, 1].set_ylabel('Power Spectral Density')
        axes[1, 1].set_title('Combined Position PSD')
        axes[1, 1].grid(True)
        axes[1, 1].set_xlim(0, min(10, fs/2))
        
        plt.tight_layout()
        plt.show()
    
    def generate_advanced_report(self):
        """Generate advanced statistical report"""
        if self.pose_data is None:
            print("No pose data available")
            return
            
        timestamps = self.pose_data['timestamp']
        positions = self.pose_data['position']
        quaternions = self.pose_data['quaternion']
        
        time_sec = (timestamps - timestamps[0]) / 1e9
        duration = time_sec[-1]
        
        # Motion statistics
        derivatives = self.compute_derivatives()
        velocity = derivatives['velocity']
        speed = np.linalg.norm(velocity, axis=1)
        
        # Rotation statistics
        euler_angles = self.quaternion_to_euler(quaternions)
        
        print("\n" + "="*60)
        print("ADVANCED ARCORE DATA ANALYSIS REPORT")
        print("="*60)
        
        print(f"\n📊 BASIC STATISTICS:")
        print(f"  Duration: {duration:.3f} seconds")
        print(f"  Samples: {len(timestamps)}")
        print(f"  Average sampling rate: {len(timestamps)/duration:.1f} Hz")
        
        print(f"\n🚀 MOTION ANALYSIS:")
        print(f"  Total distance: {np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1)):.3f} m")
        print(f"  Average speed: {np.mean(speed):.3f} m/s")
        print(f"  Maximum speed: {np.max(speed):.3f} m/s")
        print(f"  Speed standard deviation: {np.std(speed):.3f} m/s")
        
        print(f"\n🔄 ROTATION ANALYSIS:")
        euler_deg = np.degrees(euler_angles)
        print(f"  Roll range: [{np.min(euler_deg[:, 0]):.1f}, {np.max(euler_deg[:, 0]):.1f}] degrees")
        print(f"  Pitch range: [{np.min(euler_deg[:, 1]):.1f}, {np.max(euler_deg[:, 1]):.1f}] degrees")
        print(f"  Yaw range: [{np.min(euler_deg[:, 2]):.1f}, {np.max(euler_deg[:, 2]):.1f}] degrees")
        
        print(f"\n📍 POSITION STATISTICS:")
        for i, axis in enumerate(['X', 'Y', 'Z']):
            pos_axis = positions[:, i]
            print(f"  {axis}-axis: mean={np.mean(pos_axis):.4f}m, std={np.std(pos_axis):.4f}m, range={np.ptp(pos_axis):.4f}m")
        
        print(f"\n🎯 TRACKING QUALITY:")
        # Quaternion normalization check
        q_norms = np.linalg.norm(quaternions, axis=1)
        q_norm_error = np.abs(q_norms - 1.0)
        print(f"  Quaternion normalization error: max={np.max(q_norm_error):.6f}, mean={np.mean(q_norm_error):.6f}")
        
        # Smoothness analysis
        position_smoothness = np.mean(np.std(np.diff(positions, axis=0), axis=0))
        print(f"  Position smoothness (lower is better): {position_smoothness:.6f}")
        
        print("="*60)
    
    def run_complete_analysis(self):
        """Run complete advanced analysis"""
        print("Loading ARCore data for advanced analysis...")
        
        if not self.load_pose_data():
            print("Error: Could not load pose data!")
            return
        
        self.generate_advanced_report()
        
        print("\nGenerating rotation analysis plots...")
        self.plot_rotation_analysis()
        
        print("Generating motion analysis plots...")
        self.plot_motion_analysis()
        
        print("Generating frequency analysis plots...")
        self.plot_frequency_analysis()
        
        print("Advanced analysis complete!")

def main():
    parser = argparse.ArgumentParser(description='Advanced ARCore Data Analysis')
    parser.add_argument('data_folder', nargs='?', 
                       default='downloaded_logs',
                       help='Path to folder containing ARCore data files')
    
    args = parser.parse_args()
    
    # Find the most recent data folder
    data_path = Path(args.data_folder)
    
    if not data_path.exists():
        print(f"Error: Path {data_path} does not exist!")
        return
    
    if data_path.is_dir():
        session_folders = [f for f in data_path.iterdir() if f.is_dir() and 'R_pjinkim_ARCore' in f.name]
        if session_folders:
            data_path = max(session_folders, key=lambda x: x.stat().st_mtime)
            print(f"Using most recent session: {data_path.name}")
    
    # Create analyzer and run
    analyzer = AdvancedARCoreAnalyzer(data_path)
    analyzer.run_complete_analysis()

if __name__ == "__main__":
    main() 
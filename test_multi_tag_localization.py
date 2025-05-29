#!/usr/bin/env python3
"""
Test script for multi-tag RFID localization
Creates synthetic data with multiple tags detected simultaneously
"""

import numpy as np
import os
import tempfile
import shutil
from rfid_tag_localization import RFIDTagLocalizer

def create_test_data_with_multiple_tags():
    """Create test data with multiple RFID tags detected simultaneously"""
    
    # Create temporary directory structure
    temp_dir = tempfile.mkdtemp()
    session_folder = os.path.join(temp_dir, "test_session")
    os.makedirs(session_folder)
    
    # Generate synthetic VIO trajectory
    duration = 30.0  # seconds
    sample_rate = 30  # Hz
    num_samples = int(duration * sample_rate)
    
    # Create circular trajectory
    t = np.linspace(0, duration, num_samples)
    radius = 3.0
    x = radius * np.cos(2 * np.pi * t / duration)
    y = 0.5 + 0.3 * np.sin(4 * np.pi * t / duration)  # slight vertical movement
    z = radius * np.sin(2 * np.pi * t / duration)
    
    # Generate quaternions (camera looking towards center)
    angles = 2 * np.pi * t / duration + np.pi/2  # looking inward
    qw = np.cos(angles/2)
    qx = np.zeros_like(angles)
    qy = np.sin(angles/2)
    qz = np.zeros_like(angles)
    
    # Create pose data file
    pose_file = os.path.join(session_folder, "ARCore_sensor_pose.txt")
    
    with open(pose_file, 'w') as f:
        # Write header
        f.write("# ARCore 6-DoF Sensor Pose\n")
        f.write("# timestamp(ns) qx qy qz qw tx(m) ty(m) tz(m) [tag_id]\n")
        
        # Write pose data with tag detections
        for i in range(num_samples):
            timestamp = int(i * 1e9 / sample_rate)  # nanoseconds
            
            # Basic pose line
            pose_line = f"{timestamp} {qx[i]:.6f} {qy[i]:.6f} {qz[i]:.6f} {qw[i]:.6f} {x[i]:.6f} {y[i]:.6f} {z[i]:.6f}"
            
            # Add tag detections at specific times
            if i == 150:  # ~5 seconds - single tag
                pose_line += " TAG001"
            elif i == 300:  # ~10 seconds - two tags simultaneously
                f.write(pose_line + " TAG001\n")
                pose_line += " TAG002"
            elif i == 450:  # ~15 seconds - three tags simultaneously
                f.write(pose_line + " TAG001\n")
                f.write(pose_line + " TAG002\n")
                pose_line += " TAG003"
            elif i == 600:  # ~20 seconds - two different tags
                f.write(pose_line + " TAG002\n")
                pose_line += " TAG004"
            elif i == 750:  # ~25 seconds - single tag again
                pose_line += " TAG003"
            
            f.write(pose_line + "\n")
    
    return session_folder, temp_dir

def test_multi_tag_localization():
    """Test the multi-tag localization system"""
    print("Creating test data with multiple RFID tags...")
    
    try:
        session_folder, temp_dir = create_test_data_with_multiple_tags()
        print(f"Test session folder: {session_folder}")
        
        # Check if pose file was created
        pose_file = os.path.join(session_folder, "ARCore_sensor_pose.txt")
        if os.path.exists(pose_file):
            print(f"Pose file created: {pose_file}")
            with open(pose_file, 'r') as f:
                lines = f.readlines()
                print(f"Pose file contains {len(lines)} lines")
                # Show first few lines
                for i, line in enumerate(lines[:10]):
                    print(f"  Line {i+1}: {line.strip()}")
        else:
            print("ERROR: Pose file was not created!")
            return
        
        # Initialize localizer
        print("Initializing localizer...")
        localizer = RFIDTagLocalizer(session_folder)
        
        # Load data
        print("Loading pose data...")
        if not localizer.load_pose_data():
            print("Failed to load pose data")
            return
        
        print(f"Loaded {len(localizer.pose_data['timestamp'])} pose samples")
        print(f"Found {len(localizer.tag_detections)} tag detections")
        
        # Show tag detections
        for i, detection in enumerate(localizer.tag_detections):
            print(f"  Detection {i+1}: {detection['tag_id']} at timestamp {detection['timestamp']}")
        
        # Estimate tag positions
        print("Estimating tag positions...")
        localizer.estimate_tag_positions(max_distance=4.0)
        
        # Analyze simultaneous detections
        print("Analyzing simultaneous detections...")
        simultaneous = localizer.analyze_simultaneous_detections()
        
        print(f"\nSimultaneous detections found: {len(simultaneous)}")
        for timestamp, detections in simultaneous.items():
            tag_ids = [d['tag_id'] for d in detections]
            print(f"  Timestamp {timestamp}: {tag_ids}")
        
        # Generate comprehensive report
        print("Generating report...")
        localizer.generate_report()
        
        # Plot results
        print("Plotting results...")
        localizer.plot_results()
        
        print("\nTest completed successfully!")
        print("Check 'rfid_localization_results.png' for visualization")
        
    except Exception as e:
        print(f"ERROR: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        # Clean up temporary directory
        try:
            shutil.rmtree(temp_dir)
            print(f"Cleaned up temporary directory: {temp_dir}")
        except:
            print(f"Could not clean up temporary directory: {temp_dir}")

if __name__ == "__main__":
    test_multi_tag_localization() 
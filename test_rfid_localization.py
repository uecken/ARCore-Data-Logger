#!/usr/bin/env python3
"""
Test script for RFID Tag Localization
=====================================

This script demonstrates how to use the RFID tag localization system
with existing ARCore data.

Usage:
    python test_rfid_localization.py
"""

import sys
from pathlib import Path
from rfid_tag_localization import RFIDTagLocalizer

def test_with_existing_data():
    """Test with existing ARCore data"""
    
    # Find available session folders
    session_folders = []
    
    # Check for downloaded logs
    downloaded_logs = Path("downloaded_logs")
    if downloaded_logs.exists():
        for folder in downloaded_logs.rglob("*"):
            if folder.is_dir() and (folder / "ARCore_sensor_pose.txt").exists():
                session_folders.append(folder)
    
    # Check for ARCore_Logs
    arcore_logs = Path("downloaded_logs/ARCore_Logs")
    if arcore_logs.exists():
        for folder in arcore_logs.iterdir():
            if folder.is_dir() and (folder / "ARCore_sensor_pose.txt").exists():
                session_folders.append(folder)
    
    if not session_folders:
        print("No ARCore session folders found!")
        print("Please ensure you have ARCore data in one of these locations:")
        print("  - downloaded_logs/")
        print("  - downloaded_logs/ARCore_Logs/")
        return
    
    print(f"Found {len(session_folders)} session folders:")
    for i, folder in enumerate(session_folders):
        print(f"  {i+1}. {folder}")
    
    # Use the first available session
    session_folder = session_folders[0]
    print(f"\nUsing session: {session_folder}")
    
    # Initialize localizer
    localizer = RFIDTagLocalizer(session_folder)
    
    # Load data
    print("Loading pose data...")
    if not localizer.load_pose_data():
        print("Failed to load pose data")
        return
    
    # Check if we have tag detections
    if not localizer.tag_detections:
        print("\nNo tag detections found in this session.")
        print("To test the localization system:")
        print("1. Run the Android app")
        print("2. Start recording")
        print("3. Press 'Detect RFID Tag' button multiple times from different positions")
        print("4. Stop recording")
        print("5. Run this script again with the new data")
        return
    
    # Localize tags
    print("Localizing tags...")
    localizer.localize_all_tags(max_distance=5.0)
    
    # Generate report
    localizer.generate_report()
    
    # Plot results
    print("\nDisplaying visualization...")
    localizer.plot_results()

def create_sample_data():
    """Create sample data for testing (if no real data is available)"""
    
    import numpy as np
    from pathlib import Path
    
    print("Creating sample data for testing...")
    
    # Create sample directory
    sample_dir = Path("sample_data")
    sample_dir.mkdir(exist_ok=True)
    
    # Generate sample trajectory (circular motion)
    n_samples = 100
    t = np.linspace(0, 2*np.pi, n_samples)
    radius = 2.0
    
    # Device positions (circular trajectory)
    x = radius * np.cos(t)
    y = radius * np.sin(t)
    z = np.ones_like(t) * 1.5  # 1.5m height
    
    # Generate quaternions (looking towards center)
    quaternions = []
    for i in range(n_samples):
        # Look towards center
        look_dir = np.array([-x[i], -y[i], 0])
        look_dir = look_dir / np.linalg.norm(look_dir)
        
        # Simple quaternion (just rotation around Z-axis)
        angle = np.arctan2(-x[i], -y[i])
        qw = np.cos(angle/2)
        qx = 0
        qy = 0
        qz = np.sin(angle/2)
        
        quaternions.append([qx, qy, qz, qw])
    
    quaternions = np.array(quaternions)
    
    # Generate timestamps (30 Hz)
    start_time = 1000000000000  # Some large timestamp
    timestamps = start_time + np.arange(n_samples) * (1e9 / 30)  # 30 Hz
    
    # Create pose file with some tag detections
    pose_file = sample_dir / "ARCore_sensor_pose.txt"
    
    with open(pose_file, 'w') as f:
        f.write("# Sample ARCore pose data with RFID tag detections\n")
        
        for i in range(n_samples):
            # Regular pose data
            line = f"{int(timestamps[i])} {quaternions[i,0]:.6f} {quaternions[i,1]:.6f} {quaternions[i,2]:.6f} {quaternions[i,3]:.6f} {x[i]:.6f} {y[i]:.6f} {z[i]:.6f}"
            
            # Add tag detection at certain positions
            if i == 20:  # Single tag detection
                line += " TAG001"
            elif i == 40:  # Multiple tags detected simultaneously (new format)
                line += " TAG001 TAG002"
            elif i == 60:  # Another single tag
                line += " TAG002"
            elif i == 80:  # Three tags detected simultaneously (new format)
                line += " TAG001 TAG002 TAG003"
            
            f.write(line + "\n")
    
    print(f"Sample data created in: {sample_dir}")
    return sample_dir

def main():
    """Main test function"""
    
    print("RFID Tag Localization Test")
    print("=" * 40)
    
    # Try to test with existing data first
    test_with_existing_data()
    
    # If no real data, offer to create sample data
    if input("\nWould you like to test with sample data? (y/n): ").lower().startswith('y'):
        sample_dir = create_sample_data()
        
        print(f"\nTesting with sample data from: {sample_dir}")
        localizer = RFIDTagLocalizer(sample_dir)
        
        if localizer.load_pose_data():
            localizer.localize_all_tags(max_distance=5.0)
            localizer.generate_report()
            localizer.plot_results()

if __name__ == "__main__":
    main() 
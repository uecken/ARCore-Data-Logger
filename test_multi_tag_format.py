#!/usr/bin/env python3
"""
Test script for new multi-tag format
====================================

This script tests the new format where multiple RFID tags can be recorded
in a single line with the same timestamp and pose.

New format example:
648807530441788 0.199940 0.129349 0.679716 -0.693743 1.172212 0.292080 -3.014844 TAG001 TAG002 TAG003
"""

import sys
from pathlib import Path
import numpy as np
from rfid_tag_localization import RFIDTagLocalizer

def create_test_data_with_new_format():
    """Create test data using the new multi-tag format"""
    
    print("Creating test data with new multi-tag format...")
    
    # Create test directory
    test_dir = Path("test_multi_tag_format")
    test_dir.mkdir(exist_ok=True)
    
    # Create sample pose file with new format
    pose_file = test_dir / "ARCore_sensor_pose.txt"
    
    with open(pose_file, 'w') as f:
        f.write("# ARCore pose data with new multi-tag format\n")
        f.write("# Format: timestamp qx qy qz qw tx ty tz [tag1] [tag2] [tag3] ...\n")
        
        # Sample data with various tag detection scenarios
        test_data = [
            # timestamp, qx, qy, qz, qw, tx, ty, tz, tags
            (648807530441788, 0.199940, 0.129349, 0.679716, -0.693743, 1.172212, 0.292080, -3.014844, ["TAG001"]),
            (648807530541788, 0.199940, 0.129349, 0.679716, -0.693743, 1.272212, 0.292080, -3.014844, []),
            (648807530641788, 0.199940, 0.129349, 0.679716, -0.693743, 1.372212, 0.292080, -3.014844, ["TAG002"]),
            (648807530741788, 0.199940, 0.129349, 0.679716, -0.693743, 1.472212, 0.292080, -3.014844, ["TAG001", "TAG002"]),  # Simultaneous
            (648807530841788, 0.199940, 0.129349, 0.679716, -0.693743, 1.572212, 0.292080, -3.014844, []),
            (648807530941788, 0.199940, 0.129349, 0.679716, -0.693743, 1.672212, 0.292080, -3.014844, ["TAG003"]),
            (648807531041788, 0.199940, 0.129349, 0.679716, -0.693743, 1.772212, 0.292080, -3.014844, ["TAG001", "TAG002", "TAG003"]),  # Triple simultaneous
            (648807531141788, 0.199940, 0.129349, 0.679716, -0.693743, 1.872212, 0.292080, -3.014844, []),
            (648807531241788, 0.199940, 0.129349, 0.679716, -0.693743, 1.972212, 0.292080, -3.014844, ["TAG002", "TAG003"]),  # Dual simultaneous
            (648807531341788, 0.199940, 0.129349, 0.679716, -0.693743, 2.072212, 0.292080, -3.014844, []),
        ]
        
        for timestamp, qx, qy, qz, qw, tx, ty, tz, tags in test_data:
            line = f"{timestamp} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f} {tx:.6f} {ty:.6f} {tz:.6f}"
            
            # Add tags to the line
            for tag in tags:
                line += f" {tag}"
            
            f.write(line + "\n")
    
    print(f"Test data created in: {test_dir}")
    return test_dir

def test_new_format():
    """Test the new multi-tag format"""
    
    print("Testing New Multi-Tag Format")
    print("=" * 40)
    
    # Create test data
    test_dir = create_test_data_with_new_format()
    
    # Test with RFIDTagLocalizer
    print(f"\nTesting with data from: {test_dir}")
    localizer = RFIDTagLocalizer(test_dir)
    
    # Load and analyze data
    if localizer.load_pose_data():
        print("\n✅ Data loading successful!")
        
        # Print detection summary
        print(f"\nDetection Summary:")
        print(f"  - Total detections: {len(localizer.tag_detections)}")
        
        # Count detections per tag
        tag_counts = {}
        for detection in localizer.tag_detections:
            tag_id = detection['tag_id']
            tag_counts[tag_id] = tag_counts.get(tag_id, 0) + 1
        
        for tag_id, count in sorted(tag_counts.items()):
            print(f"  - {tag_id}: {count} detections")
        
        # Analyze simultaneous detections
        if hasattr(localizer, 'simultaneous_detections'):
            simultaneous_events = [detections for detections in localizer.simultaneous_detections.values() if len(detections) > 1]
            print(f"\nSimultaneous Detection Events: {len(simultaneous_events)}")
            
            for i, detections in enumerate(simultaneous_events):
                tag_list = [d['tag_id'] for d in detections]
                timestamp = detections[0]['timestamp']
                print(f"  - Event {i+1}: {tag_list} at timestamp {timestamp}")
        
        # Test localization
        print(f"\nTesting tag localization...")
        localizer.estimate_tag_positions(max_distance=5.0)
        
        if localizer.estimated_tag_positions:
            print(f"✅ Tag position estimation successful!")
            for tag_id, position in sorted(localizer.estimated_tag_positions.items()):
                print(f"  - {tag_id}: ({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}) m")
        else:
            print("⚠️  No tag positions estimated (need more detections)")
        
        # Generate visualization
        print(f"\nGenerating visualization...")
        try:
            localizer.plot_results()
            print("✅ Visualization generated successfully!")
        except Exception as e:
            print(f"⚠️  Visualization failed: {e}")
        
    else:
        print("❌ Data loading failed!")

def verify_format_examples():
    """Verify that the format examples work correctly"""
    
    print("\nVerifying Format Examples")
    print("=" * 30)
    
    # Test the exact format from user request
    example_line = "648807530441788 0.199940 0.129349 0.679716 -0.693743 1.172212 0.292080 -3.014844 TAG001 TAG002 TAG003"
    
    print(f"Testing line: {example_line}")
    
    # Parse the line manually
    parts = example_line.split()
    timestamp = int(parts[0])
    qx, qy, qz, qw = map(float, parts[1:5])
    tx, ty, tz = map(float, parts[5:8])
    tag_ids = parts[8:]
    
    print(f"Parsed data:")
    print(f"  - Timestamp: {timestamp}")
    print(f"  - Quaternion: [{qx:.6f}, {qy:.6f}, {qz:.6f}, {qw:.6f}]")
    print(f"  - Position: [{tx:.6f}, {ty:.6f}, {tz:.6f}]")
    print(f"  - Tags: {tag_ids}")
    print(f"  - Number of simultaneous tags: {len(tag_ids)}")
    
    print("✅ Format parsing successful!")

def main():
    """Main test function"""
    
    print("Multi-Tag Format Test Suite")
    print("=" * 50)
    
    # Verify format examples
    verify_format_examples()
    
    # Test with actual data processing
    test_new_format()
    
    print("\n" + "=" * 50)
    print("Test completed!")

if __name__ == "__main__":
    main() 
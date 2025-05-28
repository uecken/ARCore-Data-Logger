#!/usr/bin/env python3
"""
Latest RFID Tag Position Estimation Comparison
=============================================

Compare RFID tag position estimation accuracy using the latest data
with the complete VIO reset implementation.

Author: ARCore Data Logger Team
Date: 2025
"""

import numpy as np
import matplotlib.pyplot as plt
from rfid_tag_localization import RFIDTagLocalizer

def compare_estimation_accuracy():
    """Compare RFID estimation accuracy between different sessions"""
    
    # Data paths
    latest_data = "downloaded_logs/ARCore_Logs/20250528224331R_pjinkim_ARCore"  # Latest with VIO reset
    previous_vio_reset = "downloaded_logs/ARCore_Logs/20250528223223R_pjinkim_ARCore"  # Previous VIO reset
    before_vio_reset = "downloaded_logs/ARCore_Logs/20250528222153R_pjinkim_ARCore"  # Before VIO reset
    
    sessions = {
        "Latest (VIO Reset)": latest_data,
        "Previous (VIO Reset)": previous_vio_reset, 
        "Before VIO Reset": before_vio_reset
    }
    
    results = {}
    
    print("RFID Tag Position Estimation Comparison")
    print("=" * 60)
    
    for session_name, data_path in sessions.items():
        print(f"\n--- Analyzing: {session_name} ---")
        
        localizer = RFIDTagLocalizer(data_path)
        
        if not localizer.load_pose_data():
            print(f"Failed to load data for {session_name}")
            continue
            
        if not localizer.tag_detections:
            print(f"No tag detections found in {session_name}")
            continue
            
        # Localize tags
        localizer.localize_all_tags(max_distance=5.0)
        
        # Store results
        tag_detections = localizer.tag_detections
        estimated_positions = localizer.estimated_tag_positions
        pose_data = localizer.pose_data
        
        # Calculate metrics
        detection_positions = np.array([d['position'] for d in tag_detections])
        detection_distances = np.linalg.norm(np.diff(detection_positions, axis=0), axis=1)
        
        results[session_name] = {
            'num_detections': len(tag_detections),
            'estimated_position': estimated_positions.get('TAG001', None),
            'detection_positions': detection_positions,
            'max_detection_distance': np.max(detection_distances) if len(detection_distances) > 0 else 0,
            'mean_detection_distance': np.mean(detection_distances) if len(detection_distances) > 0 else 0,
            'position_spread': np.std(detection_positions, axis=0) if len(detection_positions) > 1 else np.array([0,0,0]),
            'trajectory_length': calculate_trajectory_length(pose_data['position']),
            'duration': (pose_data['timestamp'][-1] - pose_data['timestamp'][0]) / 1e9
        }
        
        print(f"Detections: {results[session_name]['num_detections']}")
        if results[session_name]['estimated_position'] is not None:
            pos = results[session_name]['estimated_position']
            print(f"Estimated Position: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
        print(f"Detection Spread (std): {results[session_name]['position_spread']}")
        print(f"Max Distance Between Detections: {results[session_name]['max_detection_distance']:.3f}m")
        print(f"Trajectory Length: {results[session_name]['trajectory_length']:.3f}m")
        print(f"Duration: {results[session_name]['duration']:.1f}s")
    
    # Generate comparison visualization
    create_comparison_plot(results)
    
    # Summary comparison
    print(f"\n" + "=" * 60)
    print("SUMMARY COMPARISON")
    print("=" * 60)
    
    if "Latest (VIO Reset)" in results and "Before VIO Reset" in results:
        latest = results["Latest (VIO Reset)"]
        before = results["Before VIO Reset"]
        
        print(f"Detection Consistency Improvement:")
        if latest['estimated_position'] is not None and before['estimated_position'] is not None:
            latest_spread = np.linalg.norm(latest['position_spread'])
            before_spread = np.linalg.norm(before['position_spread'])
            improvement = ((before_spread - latest_spread) / before_spread) * 100
            print(f"  Position Spread: {before_spread:.3f}m → {latest_spread:.3f}m ({improvement:+.1f}%)")
            
            latest_max_dist = latest['max_detection_distance']
            before_max_dist = before['max_detection_distance']
            dist_improvement = ((before_max_dist - latest_max_dist) / before_max_dist) * 100 if before_max_dist > 0 else 0
            print(f"  Max Detection Distance: {before_max_dist:.3f}m → {latest_max_dist:.3f}m ({dist_improvement:+.1f}%)")
            
            # Position difference
            pos_diff = np.linalg.norm(latest['estimated_position'] - before['estimated_position'])
            print(f"  Position Difference: {pos_diff:.3f}m")
    
    return results

def calculate_trajectory_length(positions):
    """Calculate total trajectory length"""
    if len(positions) < 2:
        return 0.0
    
    distances = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    return np.sum(distances)

def create_comparison_plot(results):
    """Create visualization comparing all sessions"""
    
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    
    # Position comparison
    ax1 = axes[0, 0]
    session_names = []
    estimated_positions = []
    colors = ['red', 'blue', 'green']
    
    for i, (session_name, data) in enumerate(results.items()):
        if data['estimated_position'] is not None:
            session_names.append(session_name)
            estimated_positions.append(data['estimated_position'])
            
            pos = data['estimated_position']
            ax1.scatter(pos[0], pos[1], c=colors[i % len(colors)], s=100, 
                       label=f"{session_name}\n({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
    
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('Estimated TAG001 Positions (Top View)')
    ax1.legend()
    ax1.grid(True)
    ax1.axis('equal')
    
    # Detection spread comparison
    ax2 = axes[0, 1]
    session_labels = []
    spread_values = []
    
    for session_name, data in results.items():
        session_labels.append(session_name.replace(' ', '\n'))
        spread_values.append(np.linalg.norm(data['position_spread']))
    
    bars = ax2.bar(session_labels, spread_values, color=['lightcoral', 'lightblue', 'lightgreen'])
    ax2.set_ylabel('Position Spread (m)')
    ax2.set_title('Detection Position Consistency\n(Lower is Better)')
    ax2.grid(True, alpha=0.3)
    
    # Add value labels on bars
    for bar, value in zip(bars, spread_values):
        ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01,
                f'{value:.3f}m', ha='center', va='bottom')
    
    # Max detection distance comparison
    ax3 = axes[1, 0]
    max_distances = [data['max_detection_distance'] for data in results.values()]
    
    bars = ax3.bar(session_labels, max_distances, color=['lightcoral', 'lightblue', 'lightgreen'])
    ax3.set_ylabel('Max Detection Distance (m)')
    ax3.set_title('Maximum Distance Between Detections\n(Lower indicates more consistent detection)')
    ax3.grid(True, alpha=0.3)
    
    # Add value labels on bars
    for bar, value in zip(bars, max_distances):
        ax3.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.05,
                f'{value:.3f}m', ha='center', va='bottom')
    
    # Number of detections comparison
    ax4 = axes[1, 1]
    num_detections = [data['num_detections'] for data in results.values()]
    
    bars = ax4.bar(session_labels, num_detections, color=['lightcoral', 'lightblue', 'lightgreen'])
    ax4.set_ylabel('Number of Detections')
    ax4.set_title('Tag Detection Count')
    ax4.grid(True, alpha=0.3)
    
    # Add value labels on bars
    for bar, value in zip(bars, num_detections):
        ax4.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.05,
                f'{value}', ha='center', va='bottom')
    
    plt.tight_layout()
    plt.savefig('latest_rfid_comparison.png', dpi=300, bbox_inches='tight')
    print(f"\nComparison plot saved to: latest_rfid_comparison.png")
    
    try:
        plt.show()
    except:
        print("Interactive display not available, plot saved to file instead")

if __name__ == "__main__":
    compare_estimation_accuracy() 
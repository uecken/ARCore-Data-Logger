#!/usr/bin/env python3
"""
Multi-Tag RFID Localization Analysis
====================================

Specialized analysis for multi-tag RFID detection data.
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from rfid_tag_localization import RFIDTagLocalizer

def analyze_multi_tag_session(session_folder):
    """Analyze multi-tag detection session"""
    
    print("Multi-Tag RFID Localization Analysis")
    print("=" * 50)
    
    # Initialize localizer
    localizer = RFIDTagLocalizer(session_folder)
    
    # Load data
    if not localizer.load_pose_data():
        print("Failed to load pose data")
        return
    
    # Estimate tag positions
    localizer.estimate_tag_positions(max_distance=5.0)
    
    # Analyze simultaneous detections
    localizer.analyze_simultaneous_detections()
    
    # Generate detailed multi-tag report
    generate_multi_tag_report(localizer)
    
    # Create specialized visualization
    create_multi_tag_visualization(localizer)
    
    return localizer

def generate_multi_tag_report(localizer):
    """Generate detailed multi-tag analysis report"""
    
    print("\n" + "="*60)
    print("DETAILED MULTI-TAG ANALYSIS REPORT")
    print("="*60)
    
    # Basic statistics
    if localizer.pose_data is not None:
        timestamps = localizer.pose_data['timestamp']
        positions = localizer.pose_data['position']
        
        duration = (timestamps[-1] - timestamps[0]) / 1e9
        total_distance = np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1))
        
        print(f"Session Overview:")
        print(f"  - Duration: {duration:.2f} seconds")
        print(f"  - Total samples: {len(timestamps)}")
        print(f"  - Average update rate: {len(timestamps)/duration:.1f} Hz")
        print(f"  - Total distance: {total_distance:.3f} m")
    
    # Tag detection analysis
    if localizer.tag_detections:
        print(f"\nTag Detection Analysis:")
        print(f"  - Total detections: {len(localizer.tag_detections)}")
        
        # Count detections per tag
        tag_counts = {}
        for detection in localizer.tag_detections:
            tag_id = detection['tag_id']
            tag_counts[tag_id] = tag_counts.get(tag_id, 0) + 1
        
        for tag_id, count in sorted(tag_counts.items()):
            print(f"  - {tag_id}: {count} detections")
        
        # Simultaneous detection analysis
        if hasattr(localizer, 'simultaneous_detections'):
            simultaneous_events = [detections for detections in localizer.simultaneous_detections.values() if len(detections) > 1]
            
            print(f"\nSimultaneous Detection Analysis:")
            print(f"  - Total simultaneous events: {len(simultaneous_events)}")
            print(f"  - Simultaneous detection rate: {len(simultaneous_events)/len(set(d['timestamp'] for d in localizer.tag_detections))*100:.1f}%")
            
            if simultaneous_events:
                for i, detections in enumerate(simultaneous_events):
                    tag_list = [d['tag_id'] for d in detections]
                    timestamp = detections[0]['timestamp']
                    relative_time = (timestamp - localizer.pose_data['timestamp'][0]) / 1e9
                    print(f"  - Event {i+1} at {relative_time:.1f}s: {tag_list}")
    
    # Position estimation analysis
    if localizer.estimated_tag_positions:
        print(f"\nPosition Estimation Results:")
        
        for tag_id, position in sorted(localizer.estimated_tag_positions.items()):
            print(f"  - {tag_id}: ({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}) m")
        
        # Calculate inter-tag distances and geometry
        if len(localizer.estimated_tag_positions) > 1:
            print(f"\nSpatial Relationship Analysis:")
            tag_ids = sorted(localizer.estimated_tag_positions.keys())
            
            for i in range(len(tag_ids)):
                for j in range(i+1, len(tag_ids)):
                    tag1, tag2 = tag_ids[i], tag_ids[j]
                    pos1 = localizer.estimated_tag_positions[tag1]
                    pos2 = localizer.estimated_tag_positions[tag2]
                    distance = np.linalg.norm(pos1 - pos2)
                    print(f"  - Distance {tag1} ↔ {tag2}: {distance:.3f} m")
            
            # Calculate centroid
            positions = np.array(list(localizer.estimated_tag_positions.values()))
            centroid = np.mean(positions, axis=0)
            print(f"  - Tag cluster centroid: ({centroid[0]:.3f}, {centroid[1]:.3f}, {centroid[2]:.3f}) m")
            
            # Calculate spread
            distances_from_centroid = [np.linalg.norm(pos - centroid) for pos in positions]
            avg_spread = np.mean(distances_from_centroid)
            max_spread = np.max(distances_from_centroid)
            print(f"  - Average spread from centroid: {avg_spread:.3f} m")
            print(f"  - Maximum spread from centroid: {max_spread:.3f} m")
    
    # Detection quality analysis
    print(f"\nDetection Quality Analysis:")
    
    if localizer.tag_detections:
        # Calculate detection positions spread for each tag
        for tag_id in sorted(set(d['tag_id'] for d in localizer.tag_detections)):
            tag_detections = [d for d in localizer.tag_detections if d['tag_id'] == tag_id]
            positions = np.array([d['position'] for d in tag_detections])
            
            if len(positions) > 1:
                position_spread = np.std(positions, axis=0)
                max_distance = np.max([np.linalg.norm(p1 - p2) for i, p1 in enumerate(positions) for p2 in positions[i+1:]])
                
                print(f"  - {tag_id} detection spread (std): [{position_spread[0]:.3f}, {position_spread[1]:.3f}, {position_spread[2]:.3f}] m")
                print(f"  - {tag_id} max detection distance: {max_distance:.3f} m")
    
    print("="*60)

def create_multi_tag_visualization(localizer):
    """Create specialized multi-tag visualization"""
    
    if localizer.pose_data is None:
        print("No pose data available for visualization")
        return
    
    fig = plt.figure(figsize=(20, 15))
    
    # Get trajectory data
    positions = localizer.pose_data['position']
    timestamps = localizer.pose_data['timestamp']
    
    # Color scheme for tags
    if localizer.tag_detections:
        unique_tags = sorted(set(d['tag_id'] for d in localizer.tag_detections))
        colors = plt.cm.Set1(np.linspace(0, 1, len(unique_tags)))
        tag_colors = {tag: colors[i] for i, tag in enumerate(unique_tags)}
    
    # 1. 3D Multi-tag view
    ax1 = fig.add_subplot(231, projection='3d')
    ax1.plot(positions[:, 0], positions[:, 1], positions[:, 2], 
             'b-', linewidth=2, label='Device Trajectory', alpha=0.7)
    
    # Add Start and End points for 3D view
    if len(positions) > 0:
        # Start point (green circle)
        ax1.scatter(positions[0, 0], positions[0, 1], positions[0, 2],
                   c='green', s=250, marker='o', edgecolors='darkgreen', 
                   linewidth=3, label='START', alpha=0.9)
        
        # End point (red square)
        if len(positions) > 1:
            ax1.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2],
                       c='red', s=250, marker='s', edgecolors='darkred', 
                       linewidth=3, label='END', alpha=0.9)
    
    # Plot tag detections
    if localizer.tag_detections:
        for detection in localizer.tag_detections:
            pos = detection['position']
            tag_id = detection['tag_id']
            ax1.scatter(pos[0], pos[1], pos[2],
                       c=[tag_colors[tag_id]], s=150, marker='o', 
                       alpha=0.8, edgecolors='black', linewidth=1)
    
    # Plot estimated positions
    if localizer.estimated_tag_positions:
        for tag_id, tag_pos in localizer.estimated_tag_positions.items():
            color = tag_colors.get(tag_id, 'green')
            ax1.scatter(tag_pos[0], tag_pos[1], tag_pos[2], 
                       c=[color], s=400, marker='*', 
                       edgecolors='black', linewidth=2,
                       label=f'Estimated {tag_id}')
    
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('3D Multi-Tag Localization')
    ax1.legend()
    
    # 2. Top view with detection vectors - MOST IMPORTANT VIEW
    ax2 = fig.add_subplot(232)
    # Use X-Z plane for true top-down view (bird's eye perspective)
    ax2.plot(positions[:, 0], -positions[:, 2], 'b-', linewidth=2, alpha=0.7, label='Device Path')
    
    # Add Start and End points for Top view (X-Z)
    if len(positions) > 0:
        # Start point (green circle)
        ax2.scatter(positions[0, 0], -positions[0, 2],
                   c='green', s=250, marker='o', edgecolors='darkgreen', 
                   linewidth=3, label='START', alpha=0.9, zorder=10)
        
        # End point (red square)
        if len(positions) > 1:
            ax2.scatter(positions[-1, 0], -positions[-1, 2],
                       c='red', s=250, marker='s', edgecolors='darkred', 
                       linewidth=3, label='END', alpha=0.9, zorder=10)
    
    # Plot detection points and directions
    if localizer.tag_detections:
        for detection in localizer.tag_detections:
            pos = detection['position']
            tag_id = detection['tag_id']
            
            # Plot detection point in X-Z plane
            ax2.scatter(pos[0], -pos[2], c=[tag_colors[tag_id]], s=150, marker='o', 
                       alpha=0.8, edgecolors='black', linewidth=1)
            
            # Plot camera direction in X-Z plane
            direction = localizer.get_camera_direction(detection['quaternion'])
            # Project to X-Z plane with corrected Z axis
            end_pos_x = pos[0] + direction[0] * 1.0  # 1 meter arrow in X direction
            end_pos_z = -pos[2] - direction[2] * 1.0  # 1 meter arrow in -Z direction
            ax2.arrow(pos[0], -pos[2], direction[0], -direction[2], 
                     head_width=0.1, head_length=0.1, fc=tag_colors[tag_id], 
                     ec=tag_colors[tag_id], alpha=0.6)
    
    # Plot estimated positions
    if localizer.estimated_tag_positions:
        for tag_id, tag_pos in localizer.estimated_tag_positions.items():
            color = tag_colors.get(tag_id, 'green')
            ax2.scatter(tag_pos[0], -tag_pos[2], c=[color], s=400, marker='*', 
                       edgecolors='black', linewidth=2, label=f'Est. {tag_id}')
    
    ax2.set_xlabel('X (m) - Left/Right')
    ax2.set_ylabel('-Z (m) - Forward/Backward')
    ax2.set_title('🔥 TOP VIEW (X-Z) - MOST IMPORTANT 🔥\nBird\'s Eye View with Detection Vectors')
    ax2.legend()
    ax2.grid(True)
    ax2.axis('equal')
    
    # 3. Detection timeline with simultaneous highlighting
    ax3 = fig.add_subplot(233)
    if localizer.tag_detections:
        detection_times = [(d['timestamp'] - timestamps[0]) / 1e9 for d in localizer.tag_detections]
        tag_ids = [d['tag_id'] for d in localizer.tag_detections]
        
        unique_tags = sorted(set(tag_ids))
        for i, tag_id in enumerate(unique_tags):
            tag_times = [t for t, tid in zip(detection_times, tag_ids) if tid == tag_id]
            tag_y = [i] * len(tag_times)
            ax3.scatter(tag_times, tag_y, c=[tag_colors[tag_id]], s=150, 
                       label=tag_id, alpha=0.8, edgecolors='black')
        
        # Highlight simultaneous detections
        if hasattr(localizer, 'simultaneous_detections'):
            for timestamp, detections in localizer.simultaneous_detections.items():
                if len(detections) > 1:
                    relative_time = (timestamp - timestamps[0]) / 1e9
                    tag_indices = [unique_tags.index(d['tag_id']) for d in detections]
                    
                    # Draw connection line
                    ax3.plot([relative_time, relative_time], 
                            [min(tag_indices)-0.2, max(tag_indices)+0.2], 
                            'r-', linewidth=3, alpha=0.8)
                    
                    # Add annotation
                    ax3.annotate(f'Simultaneous\n({len(detections)} tags)', 
                               xy=(relative_time, max(tag_indices)+0.4),
                               ha='center', fontsize=10, color='red', weight='bold')
    
    ax3.set_xlabel('Time (seconds)')
    ax3.set_ylabel('Tag ID')
    ax3.set_title('Multi-Tag Detection Timeline')
    ax3.set_yticks(range(len(unique_tags)))
    ax3.set_yticklabels(unique_tags)
    ax3.grid(True)
    ax3.legend()
    
    # 4. Inter-tag distance analysis
    ax4 = fig.add_subplot(234)
    if len(localizer.estimated_tag_positions) > 1:
        tag_ids = sorted(localizer.estimated_tag_positions.keys())
        distances = []
        labels = []
        
        for i in range(len(tag_ids)):
            for j in range(i+1, len(tag_ids)):
                tag1, tag2 = tag_ids[i], tag_ids[j]
                pos1 = localizer.estimated_tag_positions[tag1]
                pos2 = localizer.estimated_tag_positions[tag2]
                distance = np.linalg.norm(pos1 - pos2)
                distances.append(distance)
                labels.append(f'{tag1}-{tag2}')
        
        bars = ax4.bar(labels, distances, color=['skyblue', 'lightcoral', 'lightgreen'][:len(distances)])
        ax4.set_ylabel('Distance (m)')
        ax4.set_title('Inter-Tag Distances')
        ax4.grid(True, alpha=0.3)
        
        # Add value labels on bars
        for bar, distance in zip(bars, distances):
            height = bar.get_height()
            ax4.text(bar.get_x() + bar.get_width()/2., height + 0.01,
                    f'{distance:.3f}m', ha='center', va='bottom', weight='bold')
    
    # 5. Detection accuracy heatmap
    ax5 = fig.add_subplot(235)
    if localizer.tag_detections and localizer.estimated_tag_positions:
        unique_tags = sorted(set(d['tag_id'] for d in localizer.tag_detections))
        accuracy_data = []
        
        for tag_id in unique_tags:
            tag_detections = [d for d in localizer.tag_detections if d['tag_id'] == tag_id]
            estimated_pos = localizer.estimated_tag_positions.get(tag_id)
            
            if estimated_pos is not None:
                distances = [np.linalg.norm(d['position'] - estimated_pos) for d in tag_detections]
                accuracy_data.append([
                    np.mean(distances),  # Average error
                    np.std(distances),   # Standard deviation
                    np.max(distances),   # Maximum error
                    len(distances)       # Number of detections
                ])
        
        if accuracy_data:
            accuracy_array = np.array(accuracy_data)
            im = ax5.imshow(accuracy_array.T, cmap='RdYlBu_r', aspect='auto')
            
            ax5.set_xticks(range(len(unique_tags)))
            ax5.set_xticklabels(unique_tags)
            ax5.set_yticks(range(4))
            ax5.set_yticklabels(['Avg Error (m)', 'Std Dev (m)', 'Max Error (m)', 'Detections'])
            ax5.set_title('Detection Accuracy Heatmap')
            
            # Add text annotations
            for i in range(len(unique_tags)):
                for j in range(4):
                    text = ax5.text(i, j, f'{accuracy_array[i, j]:.2f}',
                                   ha="center", va="center", color="black", weight='bold')
            
            plt.colorbar(im, ax=ax5)
    
    # 6. Spatial distribution
    ax6 = fig.add_subplot(236)
    if localizer.estimated_tag_positions:
        positions = np.array(list(localizer.estimated_tag_positions.values()))
        tag_ids = list(localizer.estimated_tag_positions.keys())
        
        # Create Voronoi-like visualization
        for i, (tag_id, pos) in enumerate(localizer.estimated_tag_positions.items()):
            circle = plt.Circle((pos[0], pos[1]), 0.5, 
                              color=tag_colors[tag_id], alpha=0.3, label=f'{tag_id} Zone')
            ax6.add_patch(circle)
            ax6.scatter(pos[0], pos[1], c=[tag_colors[tag_id]], s=400, marker='*', 
                       edgecolors='black', linewidth=2)
            ax6.text(pos[0], pos[1]+0.7, tag_id, ha='center', va='center', 
                    fontsize=12, weight='bold')
        
        # Calculate and show coverage area
        if len(positions) > 2:
            from scipy.spatial import ConvexHull
            try:
                hull = ConvexHull(positions[:, :2])  # Use only X-Y coordinates
                for simplex in hull.simplices:
                    ax6.plot(positions[simplex, 0], positions[simplex, 1], 'k--', alpha=0.5)
                
                area = hull.volume  # In 2D, volume is area
                ax6.text(0.02, 0.98, f'Coverage Area: {area:.2f} m²', 
                        transform=ax6.transAxes, va='top', 
                        bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
            except:
                pass
    
    ax6.set_xlabel('X (m)')
    ax6.set_ylabel('Y (m)')
    ax6.set_title('Tag Spatial Distribution')
    ax6.grid(True, alpha=0.3)
    ax6.axis('equal')
    
    plt.tight_layout()
    
    # Save plot
    output_file = "multi_tag_analysis_results.png"
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"\nMulti-tag analysis plot saved to: {output_file}")
    
    try:
        plt.show()
    except:
        print("Interactive display not available, plot saved to file instead")

def main():
    """Main analysis function"""
    
    # Use the latest multi-tag session
    session_folder = "downloaded_logs/ARCore_Logs/20250529152811R_pjinkim_ARCore"
    
    if not Path(session_folder).exists():
        print(f"Session folder not found: {session_folder}")
        return
    
    # Run analysis
    localizer = analyze_multi_tag_session(session_folder)
    
    print(f"\nAnalysis complete! Check the generated visualization files.")

if __name__ == "__main__":
    main() 
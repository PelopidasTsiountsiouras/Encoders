#!/usr/bin/env python3
"""
Advanced Encoder Data Visualization
Includes calibration, drift, health monitoring, and error detection plots
"""

import h5py
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import sys


def load_advanced_encoder_data(filename):
    """Load advanced encoder data from HDF5 file"""
    with h5py.File(filename, 'r') as f:
        encoder_data = f['encoder_data'][:]
        
        diagnostics = None
        if 'diagnostics' in f:
            diagnostics = f['diagnostics'][:]
    
    # Parse encoder data columns
    data = {
        'time': encoder_data[:, 0],
        'count_raw': encoder_data[:, 1],
        'count_corrected': encoder_data[:, 2],
        'rotations': encoder_data[:, 3],
        'distance_cm': encoder_data[:, 4],
        'speed_cmps': encoder_data[:, 5],
        'speed_index': encoder_data[:, 6],
        'index_count': encoder_data[:, 7],
        'health_overall': encoder_data[:, 8],
        'health_signal': encoder_data[:, 9],
        'health_drift': encoder_data[:, 10],
        'health_speed': encoder_data[:, 11],
        'health_index': encoder_data[:, 12]
    }
    
    # Parse diagnostics
    if diagnostics is not None:
        diag = {
            'time': diagnostics[:, 0],
            'total_transitions': diagnostics[:, 1],
            'invalid_transitions': diagnostics[:, 2],
            'error_rate': diagnostics[:, 3],
            'count_jumps': diagnostics[:, 4],
            'index_mismatches': diagnostics[:, 5],
            'cumulative_drift': diagnostics[:, 6],
            'avg_drift': diagnostics[:, 7],
            'calibrated': diagnostics[:, 8],
            'calibrated_ppr': diagnostics[:, 9]
        }
    else:
        diag = None
    
    return data, diag


def plot_comprehensive_advanced(data, diag=None, save_prefix='encoder_advanced'):
    """Create comprehensive visualization with all advanced features"""
    
    fig = plt.figure(figsize=(20, 14))
    gs = GridSpec(5, 3, figure=fig, hspace=0.35, wspace=0.35)
    
    # ========== ROW 1: Distance and Speed ==========
    
    # 1. Distance: Raw vs Corrected
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(data['time'], data['distance_cm'], 'b-', linewidth=1.5, 
             label='Corrected', alpha=0.8)
    
    # Calculate distance from raw counts for comparison
    if np.any(data['count_corrected'] != data['count_raw']):
        # Approximate mm_per_count from data
        mm_per_count = data['distance_cm'][-1] * 10 / data['count_corrected'][-1]
        dist_raw = data['count_raw'] * mm_per_count / 10
        ax1.plot(data['time'], dist_raw, 'r--', linewidth=1, 
                label='Raw', alpha=0.6)
        ax1.legend(loc='upper left')
    
    ax1.set_xlabel('Time (s)', fontsize=10)
    ax1.set_ylabel('Distance (cm)', fontsize=10)
    ax1.set_title('Distance: Raw vs Drift-Corrected', fontsize=11, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    
    total_dist = data['distance_cm'][-1]
    ax1.text(0.98, 0.02, f'Total: {total_dist:.2f} cm', 
             transform=ax1.transAxes, ha='right',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7))
    
    # 2. Speed Comparison: Count-based vs Index-based
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(data['time'], data['speed_cmps'], 'b-', linewidth=1.5, 
             label='Count-based', alpha=0.7)
    
    # Only plot index speed where it's valid (non-zero)
    valid_index = data['speed_index'] > 0.1
    if np.any(valid_index):
        ax2.plot(data['time'][valid_index], data['speed_index'][valid_index], 
                'r-', linewidth=2, label='Index-based', alpha=0.8)
    
    ax2.set_xlabel('Time (s)', fontsize=10)
    ax2.set_ylabel('Speed (cm/s)', fontsize=10)
    ax2.set_title('Speed: Count vs Index Measurement', fontsize=11, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='upper right')
    
    avg_speed = np.mean(data['speed_cmps'])
    max_speed = np.max(data['speed_cmps'])
    ax2.axhline(avg_speed, color='g', linestyle='--', alpha=0.4, linewidth=1)
    ax2.text(0.02, 0.98, f'Max: {max_speed:.1f} cm/s\nAvg: {avg_speed:.1f} cm/s', 
             transform=ax2.transAxes, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.7))
    
    # 3. Health Score Over Time
    ax3 = fig.add_subplot(gs[0, 2])
    ax3.plot(data['time'], data['health_overall'], 'g-', linewidth=2, alpha=0.8)
    ax3.fill_between(data['time'], data['health_overall'], 0, alpha=0.2, color='g')
    ax3.axhline(90, color='g', linestyle='--', alpha=0.3, linewidth=1)
    ax3.axhline(75, color='orange', linestyle='--', alpha=0.3, linewidth=1)
    ax3.axhline(60, color='r', linestyle='--', alpha=0.3, linewidth=1)
    ax3.set_xlabel('Time (s)', fontsize=10)
    ax3.set_ylabel('Health Score', fontsize=10)
    ax3.set_title('Overall Health Score', fontsize=11, fontweight='bold')
    ax3.set_ylim([0, 105])
    ax3.grid(True, alpha=0.3)
    
    avg_health = np.mean(data['health_overall'])
    min_health = np.min(data['health_overall'])
    if avg_health >= 90:
        health_status = 'EXCELLENT'
        color = 'green'
    elif avg_health >= 75:
        health_status = 'GOOD'
        color = 'lightgreen'
    elif avg_health >= 60:
        health_status = 'FAIR'
        color = 'orange'
    else:
        health_status = 'POOR'
        color = 'red'
    
    ax3.text(0.02, 0.98, f'{health_status}\nAvg: {avg_health:.1f}\nMin: {min_health:.1f}', 
             transform=ax3.transAxes, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor=color, alpha=0.6))
    
    # ========== ROW 2: Count Analysis ==========
    
    # 4. Raw Count vs Corrected Count
    ax4 = fig.add_subplot(gs[1, 0])
    ax4.plot(data['time'], data['count_raw'], 'r-', linewidth=1, 
             label='Raw', alpha=0.7)
    ax4.plot(data['time'], data['count_corrected'], 'b-', linewidth=1.5, 
             label='Corrected', alpha=0.8)
    ax4.set_xlabel('Time (s)', fontsize=10)
    ax4.set_ylabel('Encoder Count', fontsize=10)
    ax4.set_title('Count Comparison: Raw vs Corrected', fontsize=11, fontweight='bold')
    ax4.grid(True, alpha=0.3)
    ax4.legend(loc='upper left')
    
    # 5. Count Difference (Drift)
    ax5 = fig.add_subplot(gs[1, 1])
    count_diff = data['count_corrected'] - data['count_raw']
    ax5.plot(data['time'], count_diff, 'purple', linewidth=1.5)
    ax5.axhline(0, color='k', linestyle='-', linewidth=0.5)
    ax5.set_xlabel('Time (s)', fontsize=10)
    ax5.set_ylabel('Count Difference', fontsize=10)
    ax5.set_title('Drift Correction Applied (Corrected - Raw)', fontsize=11, fontweight='bold')
    ax5.grid(True, alpha=0.3)
    
    if len(count_diff) > 0:
        final_drift = count_diff[-1]
        max_drift = np.max(np.abs(count_diff))
        ax5.text(0.98, 0.98, f'Final: {final_drift:.1f}\nMax: {max_drift:.1f}', 
                transform=ax5.transAxes, ha='right', verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='plum', alpha=0.6))
    
    # 6. Rotations and Index Pulses
    ax6 = fig.add_subplot(gs[1, 2])
    ax6_twin = ax6.twinx()
    
    l1 = ax6.plot(data['time'], data['rotations'], 'b-', linewidth=1.5, 
                  label='Rotations', alpha=0.7)
    l2 = ax6_twin.plot(data['time'], data['index_count'], 'r-', linewidth=2, 
                       label='Index Pulses', alpha=0.7)
    
    ax6.set_xlabel('Time (s)', fontsize=10)
    ax6.set_ylabel('Rotations', fontsize=10, color='b')
    ax6_twin.set_ylabel('Index Count', fontsize=10, color='r')
    ax6.set_title('Rotations vs Index Pulses', fontsize=11, fontweight='bold')
    ax6.grid(True, alpha=0.3)
    ax6.tick_params(axis='y', labelcolor='b')
    ax6_twin.tick_params(axis='y', labelcolor='r')
    
    # Combined legend
    lines = l1 + l2
    labels = [l.get_label() for l in lines]
    ax6.legend(lines, labels, loc='upper left')
    
    # ========== ROW 3: Health Components ==========
    
    # 7. Health Components Breakdown
    ax7 = fig.add_subplot(gs[2, 0])
    ax7.plot(data['time'], data['health_signal'], 'b-', linewidth=1.5, 
             label='Signal Quality', alpha=0.7)
    ax7.plot(data['time'], 100 - data['health_drift'], 'r-', linewidth=1.5, 
             label='Drift Control', alpha=0.7)
    ax7.plot(data['time'], data['health_speed'], 'g-', linewidth=1.5, 
             label='Speed Consistency', alpha=0.7)
    ax7.plot(data['time'], data['health_index'], 'm-', linewidth=1.5, 
             label='Index Reliability', alpha=0.7)
    ax7.set_xlabel('Time (s)', fontsize=10)
    ax7.set_ylabel('Score (%)', fontsize=10)
    ax7.set_title('Health Components', fontsize=11, fontweight='bold')
    ax7.set_ylim([0, 105])
    ax7.grid(True, alpha=0.3)
    ax7.legend(loc='lower left', fontsize=8)
    
    # 8. Error Rate Over Time
    ax8 = fig.add_subplot(gs[2, 1])
    if diag is not None and len(diag['error_rate']) > 0:
        ax8.plot(diag['time'], diag['error_rate'], 'r-', linewidth=2)
        ax8.fill_between(diag['time'], diag['error_rate'], 0, alpha=0.3, color='r')
        ax8.axhline(5, color='orange', linestyle='--', alpha=0.5, linewidth=1)
        ax8.axhline(10, color='red', linestyle='--', alpha=0.5, linewidth=1)
        
        avg_error = np.mean(diag['error_rate'])
        max_error = np.max(diag['error_rate'])
        
        ax8.text(0.02, 0.98, f'Avg: {avg_error:.2f}%\nMax: {max_error:.2f}%', 
                transform=ax8.transAxes, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='pink', alpha=0.7))
    else:
        ax8.text(0.5, 0.5, 'No error data', ha='center', va='center',
                transform=ax8.transAxes)
    
    ax8.set_xlabel('Time (s)', fontsize=10)
    ax8.set_ylabel('Error Rate (%)', fontsize=10)
    ax8.set_title('Signal Error Rate', fontsize=11, fontweight='bold')
    ax8.grid(True, alpha=0.3)
    
    # 9. Drift History
    ax9 = fig.add_subplot(gs[2, 2])
    if diag is not None and len(diag['avg_drift']) > 0:
        ax9.plot(diag['time'], diag['avg_drift'], 'purple', linewidth=2)
        ax9.axhline(0, color='k', linestyle='-', linewidth=0.5)
        ax9.fill_between(diag['time'], diag['avg_drift'], 0, alpha=0.3, color='purple')
        
        final_drift = diag['avg_drift'][-1]
        max_drift = np.max(np.abs(diag['avg_drift']))
        
        ax9.text(0.02, 0.98, f'Final: {final_drift:.2f}\nMax: ±{max_drift:.2f}', 
                transform=ax9.transAxes, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='lavender', alpha=0.7))
    else:
        ax9.text(0.5, 0.5, 'No drift data', ha='center', va='center',
                transform=ax9.transAxes)
    
    ax9.set_xlabel('Time (s)', fontsize=10)
    ax9.set_ylabel('Avg Drift (counts)', fontsize=10)
    ax9.set_title('Average Drift per Revolution', fontsize=11, fontweight='bold')
    ax9.grid(True, alpha=0.3)
    
    # ========== ROW 4: Advanced Diagnostics ==========
    
    # 10. Acceleration Profile
    ax10 = fig.add_subplot(gs[3, 0])
    dt = np.diff(data['time'])
    dv = np.diff(data['speed_cmps'])
    acceleration = np.zeros_like(data['speed_cmps'])
    if len(dt) > 0 and np.all(dt > 0):
        acceleration[1:] = dv / dt
    
    ax10.plot(data['time'], acceleration, 'orange', linewidth=1.5, alpha=0.7)
    ax10.axhline(0, color='k', linestyle='-', linewidth=0.5)
    ax10.set_xlabel('Time (s)', fontsize=10)
    ax10.set_ylabel('Acceleration (cm/s²)', fontsize=10)
    ax10.set_title('Acceleration Profile', fontsize=11, fontweight='bold')
    ax10.grid(True, alpha=0.3)
    
    # 11. Count Rate (counts per sample)
    ax11 = fig.add_subplot(gs[3, 1])
    count_rate = np.diff(data['count_corrected'])
    time_mid = data['time'][:-1]
    ax11.plot(time_mid, count_rate, 'teal', linewidth=1, alpha=0.7)
    ax11.set_xlabel('Time (s)', fontsize=10)
    ax11.set_ylabel('Counts/Sample', fontsize=10)
    ax11.set_title('Encoder Count Rate', fontsize=11, fontweight='bold')
    ax11.grid(True, alpha=0.3)
    
    # 12. Error Events
    ax12 = fig.add_subplot(gs[3, 2])
    if diag is not None:
        categories = ['Count\nJumps', 'Index\nMismatches', 'Invalid\nTransitions']
        values = [
            diag['count_jumps'][-1] if len(diag['count_jumps']) > 0 else 0,
            diag['index_mismatches'][-1] if len(diag['index_mismatches']) > 0 else 0,
            diag['invalid_transitions'][-1] if len(diag['invalid_transitions']) > 0 else 0
        ]
        colors = ['red', 'orange', 'yellow']
        bars = ax12.bar(categories, values, color=colors, alpha=0.7)
        
        # Add value labels on bars
        for bar, val in zip(bars, values):
            height = bar.get_height()
            ax12.text(bar.get_x() + bar.get_width()/2., height,
                     f'{int(val)}', ha='center', va='bottom')
        
        ax12.set_ylabel('Count', fontsize=10)
        ax12.set_title('Error Events Summary', fontsize=11, fontweight='bold')
        ax12.grid(True, alpha=0.3, axis='y')
    else:
        ax12.text(0.5, 0.5, 'No error events', ha='center', va='center',
                 transform=ax12.transAxes)
        ax12.set_title('Error Events', fontsize=11, fontweight='bold')
    
    # ========== ROW 5: Calibration and Speed Analysis ==========
    
    # 13. Calibration Status
    ax13 = fig.add_subplot(gs[4, 0])
    if diag is not None and len(diag['calibrated_ppr']) > 0:
        # Find where calibration completed
        cal_idx = np.where(diag['calibrated'] > 0.5)[0]
        if len(cal_idx) > 0:
            cal_time = diag['time'][cal_idx[0]]
            final_ppr = diag['calibrated_ppr'][cal_idx[0]]
            
            ax13.axvline(cal_time, color='g', linestyle='--', linewidth=2, 
                        label=f'Calibration at {cal_time:.1f}s')
            ax13.plot(diag['time'], diag['calibrated_ppr'], 'b-', linewidth=2)
            ax13.set_xlabel('Time (s)', fontsize=10)
            ax13.set_ylabel('PPR', fontsize=10)
            ax13.set_title('Calibrated PPR Over Time', fontsize=11, fontweight='bold')
            ax13.grid(True, alpha=0.3)
            ax13.legend(loc='upper right')
            
            ax13.text(0.02, 0.98, f'Final PPR: {final_ppr:.2f}', 
                     transform=ax13.transAxes, verticalalignment='top',
                     bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.7))
        else:
            ax13.text(0.5, 0.5, 'Not calibrated', ha='center', va='center',
                     transform=ax13.transAxes)
            ax13.set_title('Calibration', fontsize=11, fontweight='bold')
    else:
        ax13.text(0.5, 0.5, 'No calibration data', ha='center', va='center',
                 transform=ax13.transAxes)
        ax13.set_title('Calibration', fontsize=11, fontweight='bold')
    
    # 14. Speed vs Distance
    ax14 = fig.add_subplot(gs[4, 1])
    ax14.plot(data['distance_cm'], data['speed_cmps'], 'c-', linewidth=1.5, alpha=0.7)
    ax14.set_xlabel('Distance (cm)', fontsize=10)
    ax14.set_ylabel('Speed (cm/s)', fontsize=10)
    ax14.set_title('Speed vs Distance', fontsize=11, fontweight='bold')
    ax14.grid(True, alpha=0.3)
    
    # 15. Speed Distribution (Histogram)
    ax15 = fig.add_subplot(gs[4, 2])
    # Filter out near-zero speeds
    speeds_filtered = data['speed_cmps'][data['speed_cmps'] > 1.0]
    if len(speeds_filtered) > 0:
        ax15.hist(speeds_filtered, bins=30, color='steelblue', alpha=0.7, edgecolor='black')
        ax15.axvline(np.mean(speeds_filtered), color='r', linestyle='--', 
                    linewidth=2, label=f'Mean: {np.mean(speeds_filtered):.1f}')
        ax15.set_xlabel('Speed (cm/s)', fontsize=10)
        ax15.set_ylabel('Frequency', fontsize=10)
        ax15.set_title('Speed Distribution', fontsize=11, fontweight='bold')
        ax15.grid(True, alpha=0.3, axis='y')
        ax15.legend()
    else:
        ax15.text(0.5, 0.5, 'Insufficient speed data', ha='center', va='center',
                 transform=ax15.transAxes)
        ax15.set_title('Speed Distribution', fontsize=11, fontweight='bold')
    
    plt.suptitle('Advanced Encoder Analysis Dashboard', 
                 fontsize=18, fontweight='bold', y=0.998)
    
    # Save figure
    plt.savefig(f'{save_prefix}_dashboard.png', dpi=300, bbox_inches='tight')
    print(f"Saved: {save_prefix}_dashboard.png")
    
    plt.show()


def print_advanced_summary(data, diag):
    """Print comprehensive summary statistics"""
    print("\n" + "="*70)
    print("ADVANCED ENCODER DATA SUMMARY")
    print("="*70)
    
    print(f"\n{'TIME STATISTICS':-^70}")
    print(f"  Duration: {data['time'][-1]:.2f} seconds")
    print(f"  Samples: {len(data['time'])}")
    print(f"  Sample Rate: {len(data['time'])/data['time'][-1]:.1f} Hz")
    
    print(f"\n{'DISTANCE & ROTATION':-^70}")
    print(f"  Total Distance (corrected): {data['distance_cm'][-1]:.2f} cm")
    print(f"  Total Rotations: {data['rotations'][-1]:.2f}")
    print(f"  Total Counts (raw): {int(data['count_raw'][-1])}")
    print(f"  Total Counts (corrected): {int(data['count_corrected'][-1])}")
    print(f"  Count Difference: {int(data['count_corrected'][-1] - data['count_raw'][-1])}")
    
    print(f"\n{'SPEED STATISTICS':-^70}")
    print(f"  Average Speed: {np.mean(data['speed_cmps']):.2f} cm/s")
    print(f"  Max Speed: {np.max(data['speed_cmps']):.2f} cm/s")
    print(f"  Min Speed: {np.min(data['speed_cmps']):.2f} cm/s")
    
    # Index-based speed
    valid_index_speed = data['speed_index'][data['speed_index'] > 0.1]
    if len(valid_index_speed) > 0:
        print(f"  Avg Speed (index-based): {np.mean(valid_index_speed):.2f} cm/s")
    
    print(f"\n{'HEALTH STATUS':-^70}")
    print(f"  Overall Score: {np.mean(data['health_overall']):.1f}/100")
    print(f"  Signal Quality: {np.mean(data['health_signal']):.1f}%")
    print(f"  Drift Magnitude: {np.mean(data['health_drift']):.1f}%")
    print(f"  Speed Consistency: {np.mean(data['health_speed']):.1f}%")
    print(f"  Index Reliability: {np.mean(data['health_index']):.1f}%")
    
    if diag is not None:
        print(f"\n{'CALIBRATION':-^70}")
        cal_idx = np.where(diag['calibrated'] > 0.5)[0]
        if len(cal_idx) > 0:
            print(f"  Status: COMPLETE")
            print(f"  Calibrated PPR: {diag['calibrated_ppr'][cal_idx[0]]:.2f}")
            print(f"  Calibration Time: {diag['time'][cal_idx[0]]:.1f} seconds")
        else:
            print(f"  Status: NOT PERFORMED")
        
        print(f"\n{'ERROR STATISTICS':-^70}")
        if len(diag['error_rate']) > 0:
            print(f"  Average Error Rate: {np.mean(diag['error_rate']):.2f}%")
            print(f"  Max Error Rate: {np.max(diag['error_rate']):.2f}%")
            print(f"  Count Jumps: {int(diag['count_jumps'][-1])}")
            print(f"  Index Mismatches: {int(diag['index_mismatches'][-1])}")
            print(f"  Invalid Transitions: {int(diag['invalid_transitions'][-1])}")
        
        print(f"\n{'DRIFT ANALYSIS':-^70}")
        if len(diag['avg_drift']) > 0:
            print(f"  Final Average Drift: {diag['avg_drift'][-1]:.2f} counts/rev")
            print(f"  Max Drift: {np.max(np.abs(diag['avg_drift'])):.2f} counts/rev")
            print(f"  Cumulative Drift: {diag['cumulative_drift'][-1]:.1f} counts")
    
    print("="*70 + "\n")


def main():
    """Main analysis function"""
    if len(sys.argv) < 2:
        print("Usage: python plot_advanced_encoder.py <h5_filename>")
        print("\nExample: python plot_advanced_encoder.py encoder_advanced_20241121_143022.h5")
        sys.exit(1)
    
    filename = sys.argv[1]
    
    print(f"Loading advanced encoder data from: {filename}")
    data, diag = load_advanced_encoder_data(filename)
    
    print(f"Loaded {len(data['time'])} samples")
    
    # Print summary
    print_advanced_summary(data, diag)
    
    # Create comprehensive visualization
    save_prefix = filename.replace('.h5', '')
    plot_comprehensive_advanced(data, diag, save_prefix)
    
    print("\nAdvanced analysis complete!")


if __name__ == "__main__":
    main()
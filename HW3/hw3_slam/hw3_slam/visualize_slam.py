#!/usr/bin/env python3
"""
Visualization script for HW3 SLAM results.

Plots:
1. Robot trajectory with uncertainty ellipses
2. Landmark positions with covariance ellipses
3. Comparison with ground truth (if provided)
"""

import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import argparse
import os


def load_slam_data(log_file):
    """Load SLAM log data from JSON file"""
    with open(log_file, 'r') as f:
        data = json.load(f)
    return data


def plot_covariance_ellipse(ax, x, y, cov_xx, cov_yy, cov_xy, n_std=2.0, **kwargs):
    """
    Plot covariance ellipse.
    
    Args:
        ax: matplotlib axis
        x, y: center position
        cov_xx, cov_yy, cov_xy: covariance matrix elements
        n_std: number of standard deviations for ellipse
    """
    cov = np.array([[cov_xx, cov_xy],
                    [cov_xy, cov_yy]])
    
    # Compute eigenvalues and eigenvectors
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    
    # Get angle of first eigenvector
    angle = np.degrees(np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0]))
    
    # Width and height are 2 * n_std * sqrt(eigenvalue)
    width = 2 * n_std * np.sqrt(eigenvalues[0])
    height = 2 * n_std * np.sqrt(eigenvalues[1])
    
    ellipse = Ellipse(xy=(x, y), width=width, height=height, angle=angle,
                     **kwargs)
    ax.add_patch(ellipse)


def plot_slam_results(data, ground_truth=None, save_path=None):
    """
    Plot SLAM trajectory and landmarks.
    
    Args:
        data: SLAM log data dictionary
        ground_truth: Optional dict with ground truth landmark positions
        save_path: Optional path to save figure
    """
    trajectory = data['trajectory']
    landmarks = data['landmarks']
    
    # Extract trajectory data
    times = [p['time'] for p in trajectory]
    x_traj = [p['x'] for p in trajectory]
    y_traj = [p['y'] for p in trajectory]
    theta_traj = [p['theta'] for p in trajectory]
    
    # Create figure with subplots
    fig = plt.figure(figsize=(15, 10))
    
    # Main plot: trajectory + landmarks
    ax1 = plt.subplot(2, 2, 1)
    ax1.plot(x_traj, y_traj, 'b-', linewidth=2, label='Robot Trajectory')
    ax1.plot(x_traj[0], y_traj[0], 'go', markersize=10, label='Start')
    ax1.plot(x_traj[-1], y_traj[-1], 'ro', markersize=10, label='End')
    
    # Plot landmarks with covariance
    for tag_name, lm in landmarks.items():
        ax1.plot(lm['x'], lm['y'], 'rs', markersize=10)
        ax1.text(lm['x'] + 0.1, lm['y'] + 0.1, tag_name, fontsize=9)
        
        # Plot covariance ellipse
        plot_covariance_ellipse(
            ax1, lm['x'], lm['y'], 
            lm['cov_xx'], lm['cov_yy'], lm['cov_xy'],
            n_std=2.0, facecolor='red', alpha=0.2, edgecolor='red'
        )
    
    # Plot ground truth if provided
    if ground_truth is not None:
        for tag_name, gt in ground_truth.items():
            ax1.plot(gt['x'], gt['y'], 'g^', markersize=12, 
                    markerfacecolor='none', markeredgewidth=2)
    
    ax1.set_xlabel('X (meters)')
    ax1.set_ylabel('Y (meters)')
    ax1.set_title('SLAM Trajectory and Landmarks')
    ax1.legend()
    ax1.grid(True)
    ax1.axis('equal')
    
    # Plot trajectory with periodic uncertainty ellipses
    ax2 = plt.subplot(2, 2, 2)
    ax2.plot(x_traj, y_traj, 'b-', linewidth=2, label='Trajectory')
    
    # Sample every N points to avoid clutter
    sample_interval = max(len(trajectory) // 20, 1)
    for i in range(0, len(trajectory), sample_interval):
        p = trajectory[i]
        plot_covariance_ellipse(
            ax2, p['x'], p['y'],
            p['cov_xx'], p['cov_yy'], p['cov_xy'],
            n_std=2.0, facecolor='blue', alpha=0.1, edgecolor='blue', linewidth=0.5
        )
    
    ax2.set_xlabel('X (meters)')
    ax2.set_ylabel('Y (meters)')
    ax2.set_title('Trajectory with Position Uncertainty (2σ)')
    ax2.grid(True)
    ax2.axis('equal')
    
    # Plot position uncertainty over time
    ax3 = plt.subplot(2, 2, 3)
    cov_xx = [p['cov_xx'] for p in trajectory]
    cov_yy = [p['cov_yy'] for p in trajectory]
    sigma_x = np.sqrt(cov_xx)
    sigma_y = np.sqrt(cov_yy)
    
    ax3.plot(times, sigma_x, 'r-', label='σ_x')
    ax3.plot(times, sigma_y, 'b-', label='σ_y')
    ax3.set_xlabel('Time (seconds)')
    ax3.set_ylabel('Position Uncertainty (meters)')
    ax3.set_title('Position Uncertainty Evolution')
    ax3.legend()
    ax3.grid(True)
    
    # Plot orientation uncertainty over time
    ax4 = plt.subplot(2, 2, 4)
    cov_tt = [p['cov_tt'] for p in trajectory]
    sigma_theta = np.sqrt(cov_tt)
    
    ax4.plot(times, np.degrees(sigma_theta), 'g-')
    ax4.set_xlabel('Time (seconds)')
    ax4.set_ylabel('Orientation Uncertainty (degrees)')
    ax4.set_title('Orientation Uncertainty Evolution')
    ax4.grid(True)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f'Saved plot to: {save_path}')
    
    plt.show()


def compute_landmark_errors(slam_landmarks, ground_truth):
    """
    Compute average landmark position error.
    
    Args:
        slam_landmarks: Dict of SLAM landmark estimates
        ground_truth: Dict of ground truth positions
        
    Returns:
        Dict with error statistics
    """
    errors = []
    
    for tag_name, slam_lm in slam_landmarks.items():
        if tag_name not in ground_truth:
            continue
        
        gt_lm = ground_truth[tag_name]
        dx = slam_lm['x'] - gt_lm['x']
        dy = slam_lm['y'] - gt_lm['y']
        error = np.sqrt(dx**2 + dy**2)
        errors.append({
            'tag': tag_name,
            'error': error,
            'dx': dx,
            'dy': dy
        })
    
    if not errors:
        return None
    
    error_values = [e['error'] for e in errors]
    
    return {
        'mean_error': np.mean(error_values),
        'std_error': np.std(error_values),
        'max_error': np.max(error_values),
        'min_error': np.min(error_values),
        'individual_errors': errors
    }


def print_error_report(error_stats):
    """Print landmark error statistics"""
    print("\n" + "="*60)
    print("LANDMARK LOCALIZATION ERROR REPORT")
    print("="*60)
    print(f"Mean Error:    {error_stats['mean_error']:.4f} m")
    print(f"Std Dev:       {error_stats['std_error']:.4f} m")
    print(f"Max Error:     {error_stats['max_error']:.4f} m")
    print(f"Min Error:     {error_stats['min_error']:.4f} m")
    print("\nIndividual Landmark Errors:")
    print("-" * 60)
    print(f"{'Tag':<15} {'Error (m)':<12} {'ΔX (m)':<12} {'ΔY (m)':<12}")
    print("-" * 60)
    for err in error_stats['individual_errors']:
        print(f"{err['tag']:<15} {err['error']:>11.4f} {err['dx']:>11.4f} {err['dy']:>11.4f}")
    print("="*60 + "\n")


def main():
    parser = argparse.ArgumentParser(description='Visualize HW3 SLAM results')
    parser.add_argument('log_file', help='Path to SLAM log JSON file')
    parser.add_argument('--ground-truth', help='Path to ground truth JSON file')
    parser.add_argument('--save', help='Path to save figure')
    
    args = parser.parse_args()
    
    # Load SLAM data
    print(f'Loading SLAM data from: {args.log_file}')
    slam_data = load_slam_data(args.log_file)
    
    print(f"Trajectory points: {len(slam_data['trajectory'])}")
    print(f"Landmarks: {len(slam_data['landmarks'])}")
    
    # Load ground truth if provided
    ground_truth = None
    if args.ground_truth:
        print(f'Loading ground truth from: {args.ground_truth}')
        with open(args.ground_truth, 'r') as f:
            ground_truth = json.load(f)
        
        # Compute and print errors
        error_stats = compute_landmark_errors(slam_data['landmarks'], ground_truth)
        if error_stats:
            print_error_report(error_stats)
    
    # Plot results
    plot_slam_results(slam_data, ground_truth, args.save)


if __name__ == '__main__':
    main()

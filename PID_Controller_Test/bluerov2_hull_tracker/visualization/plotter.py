#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np

def plot_results(results, hull_points, offset=0.0):
    """Plot the comparison results including offset line (if offset != 0)"""
    plt.figure(figsize=(10, 6))
    
    # Plot original hull (thicker line)
    plt.plot(hull_points[:,0], hull_points[:,1], 'k-', linewidth=3, label='Ship Hull')
    
    # Plot offset line as thin black line only if offset != 0
    if offset != 0.0:
        offset_line_y = hull_points[:,1] + offset
        plt.plot(hull_points[:,0], offset_line_y, 'k-', linewidth=1, label=f'Offset Line (y + {offset:.2f})')
    
    # Plot each controller's trajectory (dashed lines)
    for trajectory, color, label in results:
        plt.plot(trajectory[:,0], trajectory[:,1], color, linestyle='--', label=label)
    
    plt.xlabel('X Position [m]')
    plt.ylabel('Y Position [m]')
    plt.title('PID Controller Comparison - Hull Tracking with Vertical Offset')
    plt.grid(True)
    plt.legend()
    plt.axis('equal')
    plt.show()

import cv2
import numpy as np
import time
import os
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

from vio_new import VisualOdometryIMU


def parse_ground_truth(filepath):
    GT_pose = []
    try:
        with open(filepath, 'r') as file:
            for line in file:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split(",")
                if len(parts) < 7:
                    print(f"Skipping malformed ground truth line: {line}")
                    continue
                try:
                    timestamp = float(parts[1])
                    position = [float(x) for x in parts[5:8]]
                    orientation = [float(x) for x in parts[2:5]]
                    GT_pose.append((timestamp, position, orientation))
                except ValueError as e:
                    print(f"Error parsing ground truth line: {line} - {e}")
    
        return GT_pose
    except FileNotFoundError:
        print(f"Ground truth file not found: {filepath}")
        return []

def clean_and_parse_csv(filepath):
    """Clean and parse the CSV file with proper error handling"""
    rows = []
    try:
        with open(filepath, 'r') as file:
            for line in file:
                # Remove leading/trailing spaces
                line = line.strip()
                line = line.strip("/")
                if not line:
                    continue

                # Handle malformed concatenated rows
                if 'rec_' in line[1:]:
                    parts = line.split('rec_')
                    for i, part in enumerate(parts):
                        if part.strip():
                            rows.append("rec_" + part.strip() if i != 0 else part.strip())
                else:
                    rows.append(line)
    except FileNotFoundError:
        print(f"CSV file not found: {filepath}")
        return []

    # Parse CSV content
    data = []
    header = []
    for i, row in enumerate(rows):
        columns = row.split(",")
        if i == 0:
            header = [col.strip() for col in columns]  # Clean header names
        else:
            # Skip row if column count doesn't match
            if len(columns) != len(header):
                print(f"Skipping malformed row: {row}")
                continue
            
            # Clean column values and create dictionary
            clean_columns = [col.strip() for col in columns]
            data.append(dict(zip(header, clean_columns)))
    
    return data


def main():
    """
    get_imu_callback: function returning latest imu_data dict or None if no new data
    imu_data format: {'timestamp': float_sec, 'accel': np.array([ax,ay,az]), 'gyro': np.array([gx,gy,gz])}
    """
    # Configuration
    folder_path = "test-imgs/"
    imu_data_path = os.path.join(folder_path, "imu.csv")
    
    gt_path = os.path.join(folder_path, "GroundTruth.csv")
    gt = parse_ground_truth(gt_path)
    
    # Check if files exist
    if not os.path.exists(folder_path):
        print(f"Folder not found: {folder_path}")
        return
    
    if not os.path.exists(imu_data_path):
        print(f"IMU data file not found: {imu_data_path}")
        return

    # Initialize VIO system
    vo = VisualOdometryIMU()

        # Load and clean the CSV
    print("Loading IMU data...")
    data = clean_and_parse_csv(imu_data_path)
    
    if not data:
        print("No valid data found in CSV file")
        return
    
    print(f"Loaded {len(data)} data entries")
    print("CSV Headers:", list(data[0].keys()) if data else "None")

    # Initialize variables
    tello_dict = {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "vgx": 0,
        "vgy": 0,
        "vgz": 0,
        "agx": 0.0,
        "agy": 0.0,
        "agz": 0.0
    }

    # Process each frame
    for i, entry in enumerate(data):
        # Construct image path
        image_filename = f"{entry['Image']}.png"
        image_path = os.path.join(folder_path, image_filename)

        # Check if image exists
        if not os.path.isfile(image_path):
            print(f"Image not found: {image_path}")
            continue

        # Load image
        image = cv2.imread(image_path)
        if image is None:
            print(f"Failed to load image: {image_path}")
            continue

        if i == 0:
            # Initialize pose
            t = np.array([0.316238,0.05039,-0.024567])
            # Parse initial orientation
            initial_roll = float(entry.get('roll', -0.59889))
            initial_pitch = float(entry.get('pitch', 4.417175))
            initial_yaw = float(entry.get('yaw', -0.095317))
            
            tello_dict["roll"] = initial_roll
            tello_dict["pitch"] = initial_pitch
            tello_dict["yaw"] = initial_yaw
            
            # Convert Euler angles to rotation matrix (NED convention)
            rotation = R.from_euler('ZYX', [
                np.radians(initial_yaw),
                np.radians(initial_pitch), 
                np.radians(initial_roll)
            ]).as_matrix()
            
            # Set initial conditions
            vo.set_initial_pose(rotation, t)
            vo.set_initial_image(image)
            vo.set_initial_imu(tello_dict)
            
            print(f"Initialized VIO system with orientation: roll={initial_roll:.1f}°, pitch={initial_pitch:.1f}°, yaw={initial_yaw:.1f}°")
        
        else:
            # Parse current IMU data
            current_roll = float(entry.get('roll', 0))
            current_pitch = float(entry.get('pitch', 0))
            current_yaw = float(entry.get('yaw', 0))
            
            # Raw accelerometer readings (adjust units as needed)
            ax_raw = float(entry.get("ax", 0)) / 100.0  # Convert to m/s²
            ay_raw = float(entry.get("ay", 0)) / 100.0
            az_raw = float(entry.get("az", 0)) / 100.0
    
            
            # Update IMU data
            tello_dict.update({
                "roll": current_roll,
                "pitch": current_pitch, 
                "yaw": current_yaw,
                "agx": ax_raw,
                "agy": ay_raw,
                "agz": az_raw,
                "h": float(entry.get("h", 0))/100.0
            })

            vo.run(image, tello_dict)

    estimated_path = vo.get_estimated_path()
    
    if estimated_path is not None and len(estimated_path) > 0:
        print(f"Plotting trajectory with {len(estimated_path)} points...")
        
        # Create 3D plot
        fig = plt.figure(figsize=(15, 10))
        
        # Main trajectory plot
        ax1 = fig.add_subplot(121, projection='3d')
        
        # Extract position data (last 3 columns)
        positions = estimated_path[:, 3:]
        
        # Plot trajectory
        ax1.plot(positions[:, 0], positions[:, 1], positions[:, 2], 
                'b-', linewidth=2, label='Estimated Trajectory')
        ax1.scatter(positions[0, 0], positions[0, 1], positions[0, 2], 
                  c='green', s=100, label='Start', marker='o')
        ax1.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2], 
                  c='red', s=100, label='End', marker='s')


        # # Plot ground truth if available
        if gt:
            gt_positions = np.array([p[1] for p in gt])
            ax1.plot(gt_positions[:, 0], gt_positions[:, 1], gt_positions[:, 2], 
                    'r--', linewidth=1, label='Ground Truth', alpha=0.5)
  

        # Add waypoint markers every N points
        waypoint_interval = max(1, len(positions) // 10)
        for idx in range(0, len(positions), waypoint_interval):
            ax1.scatter(positions[idx, 0], positions[idx, 1], positions[idx, 2], 
                       c='orange', s=30, alpha=0.7)
        
        # Formatting
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.set_title('Visual-Inertial Odometry Trajectory')
        ax1.legend()
        ax1.grid(True)
        
        # Set equal aspect ratio
        max_range = np.array([positions[:, 0].max()-positions[:, 0].min(),
                             positions[:, 1].max()-positions[:, 1].min(),
                             positions[:, 2].max()-positions[:, 2].min()]).max() / 2.0
        mid_x = (positions[:, 0].max()+positions[:, 0].min()) * 0.5
        mid_y = (positions[:, 1].max()+positions[:, 1].min()) * 0.5
        mid_z = (positions[:, 2].max()+positions[:, 2].min()) * 0.5
        # ax1.set_xlim(mid_x - max_range, mid_x + max_range)
        # ax1.set_ylim(mid_y - max_range, mid_y + max_range)
        # ax1.set_zlim(mid_z - max_range, mid_z + max_range)
  
        # 2D top-down view
        ax2 = fig.add_subplot(122)
        ax2.plot(positions[:, 0], positions[:, 1], 'b-', linewidth=2, label='XY Trajectory')
        ax2.scatter(positions[0, 0], positions[0, 1], c='green', s=100, label='Start', marker='o')
        ax2.scatter(positions[-1, 0], positions[-1, 1], c='red', s=100, label='End', marker='s')
        ax2.plot(gt_positions[:, 0], gt_positions[:, 1], 
                    'r--', linewidth=1, label='Ground Truth', alpha=0.5)
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        ax2.set_title('Top-Down View (XY Plane)')
        ax2.legend()
        ax2.grid(True)
        ax2.axis('equal')
        
        plt.tight_layout()
        plt.show()
        
        # Print detailed statistics
        total_distance = np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1))
        max_displacement = np.max(np.linalg.norm(positions - positions[0], axis=1))
        
        print(f"\n=== Trajectory Statistics ===")
        print(f"Total points: {len(positions)}")
        print(f"Total estimated distance traveled: {total_distance:.2f} m")
        print(f"Maximum displacement from start: {max_displacement:.2f} m")
        print(f"Final position: [{positions[-1, 0]:.2f}, {positions[-1, 1]:.2f}, {positions[-1, 2]:.2f}] m")
        print(f"Position range - X: [{positions[:, 0].min():.2f}, {positions[:, 0].max():.2f}] m")
        print(f"Position range - Y: [{positions[:, 1].min():.2f}, {positions[:, 1].max():.2f}] m") 
        print(f"Position range - Z: [{positions[:, 2].min():.2f}, {positions[:, 2].max():.2f}] m")
        
    else:
        print("No trajectory data to plot. Check if images and IMU data are being processed correctly.")


if __name__ == "__main__":
    main()
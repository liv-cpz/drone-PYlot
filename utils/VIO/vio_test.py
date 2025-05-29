import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import os
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

from tello_vio import VIO

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

def convert_tello_to_ned(ax, ay, az, roll, pitch, yaw):
    """
    Convert Tello coordinate system to NED (North-East-Down)
    Tello uses a different coordinate convention than standard robotics
    """
    # Convert Tello Euler angles to rotation matrix
    # Tello angles are in degrees, convert to radians
    roll_rad = np.radians(roll)
    pitch_rad = np.radians(pitch) 
    yaw_rad = np.radians(yaw)
    
    # Create rotation matrix from Tello frame to NED frame
    # This may need adjustment based on your specific Tello orientation
    R_tello_to_ned = R.from_euler('ZYX', [yaw_rad, pitch_rad, roll_rad]).as_matrix()
    
    # Transform acceleration vector
    accel_tello = np.array([ax, ay, az])
    accel_ned = R_tello_to_ned @ accel_tello
    
    return accel_ned[0], accel_ned[1], accel_ned[2]

def main():
    # Configuration
    folder_path = "test-imgs/"
    imu_data_path = os.path.join(folder_path, "imu.csv")
    
    # Check if files exist
    if not os.path.exists(folder_path):
        print(f"Folder not found: {folder_path}")
        return
    
    if not os.path.exists(imu_data_path):
        print(f"IMU data file not found: {imu_data_path}")
        return

    # Initialize VIO system
    vio = VIO()
    
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

    processed_count = 0
    previous_timestamp = None
    
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

        try:
            # Parse timestamp
            timestamp = float(entry.get('Time', time.time())) if 'Time' in entry else time.time()
            
            # Calculate actual dt from timestamps
            if previous_timestamp is not None:
                actual_dt = timestamp - previous_timestamp
                actual_dt = np.clip(actual_dt, 0.01, 0.2)  # Sanity check
            else:
                actual_dt = 0.033
            previous_timestamp = timestamp
            
            if i == 0:
                # Initialize pose
                t = np.array([0.0, 0.0, 0.0])
                
                # Parse initial orientation
                initial_roll = float(entry.get('roll', 0))
                initial_pitch = float(entry.get('pitch', 0))
                initial_yaw = float(entry.get('yaw', 0))
                
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
                vio.set_initial_pose(rotation, t)
                vio.set_initial_image(image)
                vio.set_initial_imu(tello_dict)
                
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
                
                # Convert to NED frame if needed
                ax_ned, ay_ned, az_ned = convert_tello_to_ned(
                    ax_raw, ay_raw, az_raw, 
                    current_roll, current_pitch, current_yaw
                )
                
                # Update IMU data
                tello_dict.update({
                    "roll": current_roll,
                    "pitch": current_pitch, 
                    "yaw": current_yaw,
                    "agx": ax_ned,
                    "agy": ay_ned,
                    "agz": az_ned
                })
                
                # Run VIO processing
                vio.run(image, tello_dict, timestamp)
                
                processed_count += 1
                
                # Progress indicator with more info
                if processed_count % 20 == 0:
                    current_pose = vio.get_current_pose()
                    if current_pose is not None:
                        pos = current_pose[:3, 3]
                        print(f"Processed {processed_count} frames, current position: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")
                    else:
                        print(f"Processed {processed_count} frames...")
                    
        except (ValueError, KeyError) as e:
            print(f"Error processing entry {i}: {e}")
            continue

    print(f"Processing complete. Processed {processed_count} frames.")

    # Visualize results
    estimated_path = vio.get_estimated_path()
    
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
        ax1.set_xlim(mid_x - max_range, mid_x + max_range)
        ax1.set_ylim(mid_y - max_range, mid_y + max_range)
        ax1.set_zlim(mid_z - max_range, mid_z + max_range)
        
        # 2D top-down view
        ax2 = fig.add_subplot(122)
        ax2.plot(positions[:, 0], positions[:, 1], 'b-', linewidth=2, label='XY Trajectory')
        ax2.scatter(positions[0, 0], positions[0, 1], c='green', s=100, label='Start', marker='o')
        ax2.scatter(positions[-1, 0], positions[-1, 1], c='red', s=100, label='End', marker='s')
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
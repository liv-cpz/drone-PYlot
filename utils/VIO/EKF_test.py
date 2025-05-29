from EKFVOIMU import EKFVOIMU 
import os
import numpy as np
import cv2


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
    ekf = EKFVOIMU()
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
        "dt": 0.0,
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


if __name__ == "__main__":
    main()
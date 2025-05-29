import numpy as np

# Your current transform matrix
R_cam_to_ned = np.array([
    [0, 0, 1],
    [1, 0, 0],
    [0, -1, 0]
])

def print_transformed_vectors():
    # Define some simple unit vectors in camera frame:
    # Forward (Z), Right (X), Down (Y)
    forward_cam = np.array([0, 0, 1])
    right_cam = np.array([1, 0, 0])
    down_cam = np.array([0, 1, 0])

    # Transform to body frame
    forward_body = R_cam_to_ned @ forward_cam
    right_body = R_cam_to_ned @ right_cam
    down_body = R_cam_to_ned @ down_cam

    print("Camera frame unit vectors:")
    print(f"Forward (Z_cam): {forward_cam}")
    print(f"Right (X_cam): {right_cam}")
    print(f"Down (Y_cam): {down_cam}")

    print("\nTransformed to body (NED) frame:")
    print(f"Forward (body): {forward_body}")
    print(f"Right (body): {right_body}")
    print(f"Down (body): {down_body}")

print_transformed_vectors()

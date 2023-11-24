import numpy as np

def calc_yaw_vec(yaw_angle: float) -> np.ndarray:
    # Convert yaw angle to radians
    yaw_rad = np.radians(yaw_angle)

    # Convert yaw angle to x and y components
    yaw_vec = np.array([np.cos(yaw_rad), np.sin(yaw_rad)])

    return yaw_vec

def calc_vec(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    # Convert the input points to NumPy arrays
    a = np.array([a[0], a[1]])
    b = np.array([b[0], b[1]])

    # Calculate the vector between the two points
    target_vec = a - b

    return target_vec

def vec_ang_delta(vector1: np.ndarray, vector2: np.ndarray) -> float:
    # Calculate the angle between two vectors
    dot_product = np.dot(vector1, vector2)
    magnitudes = np.linalg.norm(vector1) * np.linalg.norm(vector2)
    
    # Ensure the denominator is not zero (i.e., vectors are not parallel)
    if magnitudes == 0:
        return None

    # Calculate the cosine of the angle
    cos_theta = dot_product / magnitudes

    # Calculate the angle in radians
    angle_rad = np.arccos(cos_theta)

    # Calculate the cross product to determine the orientation
    cross_product = np.cross(vector1, vector2)

    # Determine the sign of the angle based on the cross product
    if cross_product < 0:
        angle_rad = -angle_rad

    # Convert the angle to degrees
    angle_deg = np.degrees(angle_rad)

    # Keep angle within +/- 90 degrees range
    if angle_deg > 90:
        angle_deg = 180 - angle_deg
    elif angle_deg < -90:
        angle_deg = -180 - angle_deg

    return angle_deg

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
    # Calculate magnitudes of the vectors
    magnitude1 = np.linalg.norm(vector1)
    magnitude2 = np.linalg.norm(vector2)

    # Check if either of the vectors has zero magnitude
    if magnitude1 == 0 or magnitude2 == 0:
        return None

    # Calculate the angle between two vectors
    dot_product = np.dot(vector1, vector2)
    
    # Calculate the cosine of the angle
    cos_theta = dot_product / (magnitude1 * magnitude2)

    # Calculate the angle in radians
    angle_rad = np.arccos(cos_theta)

    # Calculate the cross product to determine the orientation (z-component for 2D vectors)
    cross_product_z = vector1[0]*vector2[1] - vector1[1]*vector2[0]

    # Determine the sign of the angle based on the cross product
    if cross_product_z < 0:
        angle_rad = -angle_rad

    # Convert the angle to degrees
    angle_deg = np.degrees(angle_rad)

    return angle_deg

def vec_to_angle(vector: np.ndarray) -> float:
    """ Convert a 2D vector to an angle in the range 0 to 360 degrees. """
    angle = np.degrees(np.arctan2(vector[1], vector[0]))
    # Normalize the angle to be within 0 to 360 degrees
    return angle % 360
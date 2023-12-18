import numpy as np

def local_to_global(local_vec: np.ndarray, rotation_matrix: np.ndarray, origin: float) -> np.ndarray:
    # Convert the origin to a NumPy array
    origin = np.array([origin[0], origin[1], origin[2]])

    # Convert the local vector to a NumPy array
    local_vec = np.array([local_vec[0], local_vec[1], local_vec[2]])

    rotation_matrix = np.array(rotation_matrix)

    # Calculate the global vector
    global_vec = np.matmul(rotation_matrix, local_vec) + origin

    return global_vec

def calc_yaw_vec(yaw_angle: float) -> np.ndarray:
    # Convert yaw angle to radians
    yaw_rad = np.radians(yaw_angle)

    # Convert yaw angle to x and y components
    yaw_vec = np.array([np.cos(yaw_rad), np.sin(yaw_rad)])

    return yaw_vec

def calc_pitch_vec(pitch_angle: float) -> np.ndarray:
    # Convert pitch angle to radians
    pitch_rad = np.radians(pitch_angle)

    # Convert pitch angle to y and z components
    pitch_vec = np.array([np.sin(pitch_rad), np.cos(pitch_rad)])

    return pitch_vec

def calc_azi_vec(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    # Convert the input points to NumPy arrays
    a = np.array([a[1], a[0]])
    b = np.array([b[1], b[0]])

    # Calculate the vector between the two points
    target_vec = a - b

    return target_vec

def calc_elv_vec(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    # Convert the input points to NumPy arrays
    a = np.array([a[1], a[2]])
    b = np.array([b[1], b[2]])

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
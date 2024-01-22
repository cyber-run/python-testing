from scipy.optimize import fsolve
from typing import Tuple
import numpy as np
import itertools
import math

def solve_for_mxyz(points: np.ndarray, angles: np.ndarray) -> np.ndarray:
    """
    Solve for mx, my, and mz given a list of points and the respective angles between successive points.

    Args:
    points (np.ndarray): The array of points' coordinates.
    angles (np.ndarray): The array of angles in degrees between successive points.

    Returns:
    np.ndarray: The solved values of mx, my, and mz.
    """
    def equations(vars):
        mx, my, mz = vars
        equations = []
        for i in range(len(angles)):
            vector_1 = points[i] - np.array([mx, my, mz])
            vector_2 = points[i + 1] - np.array([mx, my, mz])
            dot_product = np.dot(vector_1, vector_2)
            mag_1 = np.linalg.norm(vector_1)
            mag_2 = np.linalg.norm(vector_2)
            angle_rad = np.radians(angles[i])
            eq = np.cos(angle_rad) - (dot_product / (mag_1 * mag_2))
            equations.append(eq)
        return equations
    return fsolve(equations, np.zeros(3))

def def_local_coor_sys(points: np.ndarray, local_origin: np.ndarray) -> np.ndarray:
    """
    Define the local coordinate system such that the X-axis directly points to the first point,
    and the Z-axis is orthogonal to the plane defined by the vectors formed from the origin to 
    first point and origin to second point.

    Args:
    first_point (np.ndarray): The first point's coordinates.
    global_up (np.ndarray): The global upwards direction (default is [0, 0, 1]).

    Returns:
    np.ndarray: The rotation matrix representing the local coordinate system.
    """
    x_axis = np.array(points[0], dtype=float) / np.linalg.norm(points[0])

    z_axis = np.cross(points[0] - local_origin, points[1] - local_origin) / np.linalg.norm(np.cross(points[0] - local_origin, points[1] - local_origin))

    y_axis = np.cross(x_axis, z_axis) / np.linalg.norm(np.cross(x_axis, z_axis))

    rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))

    return rotation_matrix

def def_local_coor_sys2(points: np.ndarray, local_origin: np.ndarray) -> np.ndarray:
    """
    Define the local coordinate system by averaging the cross product of every combination of vectors from the local origin to the points.

    Args:
    points (np.ndarray): The points' coordinates.
    local_origin (np.ndarray): The local origin's coordinates.

    Returns:
    np.ndarray: The rotation matrix representing the local coordinate system.
    """
    # Calculate vectors from the local origin to the points
    vectors = points - local_origin

    # Calculate the cross product of every combination of vectors
    cross_products = [np.cross(v1, v2) for v1, v2 in itertools.combinations(vectors, 2)]

    # Average the cross products to get the z-axis
    z_axis = np.mean(cross_products, axis=0)
    z_axis /= np.linalg.norm(z_axis)

    # Calculate the x-axis as the normalized vector from the local origin to the first point
    x_axis = vectors[0] / np.linalg.norm(vectors[0])

    # Calculate the y-axis as the cross product of the x-axis and z-axis
    y_axis = np.cross(x_axis, z_axis)
    y_axis /= np.linalg.norm(y_axis)

    # Recalculate the z-axis as the cross product of the x-axis and y-axis
    z_axis = np.cross(x_axis, y_axis)

    # Create the rotation matrix
    rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))

    return rotation_matrix

def svd(points: np.ndarray) -> np.ndarray:
    """
    Define the local coordinate system using singular value decomposition (SVD).

    Args:
    points (np.ndarray): The points' coordinates.

    Returns:
    np.ndarray: The rotation matrix representing the local coordinate system.
    """
    # Calculate the centroid of the points
    centroid = np.mean(points, axis=0)

    # Subtract the centroid from the points to get vectors from the centroid to the points
    vectors = points - centroid

    # Calculate the SVD of the vectors
    u, s, vh = np.linalg.svd(vectors)

    # The columns of vh are the axes of the local coordinate system
    x_axis, y_axis, z_axis = vh.T

    # Create the rotation matrix
    rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))

    return rotation_matrix

def global_to_local(point_global: np.ndarray, rotation_matrix: np.ndarray) -> np.ndarray:
    """
    Transform a point from global coordinates to local coordinates using a rotation matrix.

    Args:
    point_global (np.ndarray): The point's coordinates in the global coordinate system.
    rotation_matrix (np.ndarray): The rotation matrix for the transformation.

    Returns:
    np.ndarray: The point's coordinates in the local coordinate system.
    """
    return np.dot(np.linalg.inv(rotation_matrix), point_global)

def calc_rot_comp(point_local: np.ndarray) -> Tuple[float, float]:
    """
    Calculate the pan and tilt components of rotation from the positive X-axis.

    Args:
    point_local (np.ndarray): The point's coordinates in the local coordinate system.

    Returns:
    Tuple[float, float]: The pan and tilt rotation components.
    """
    pan_angle = math.degrees(math.atan2(point_local[1], point_local[0]))
    tilt_angle = math.degrees(math.atan2(point_local[2], math.sqrt(point_local[0]**2 + point_local[2]**2)))
    return pan_angle, tilt_angle

if __name__ == "__main__":
    # TODO: Get better set of points and angles to test with; implement auto algo testing
    points_example = np.array([(30, 50, 0), (37, 41, 13), (43, 45, 17), (62, 34, 12)])
    angles_example = np.array([17.701, 2.7967, 18.842])

    # Find local origin point
    mx, my, mz = solve_for_mxyz(points_example, angles_example)
    local_origin = np.round(np.array([mx, my, mz]), 4)

    # Find rotation matrix
    rotation_matrix = np.round(def_local_coor_sys(points_example, local_origin), 4)
    # rotation_matrix = svd(points_example)

    # Convert global point to local point
    point_global_example = (43, 45, 17)
    point_local_example = global_to_local(point_global_example, rotation_matrix)

    # Calculate rotation components
    rotation_components_example = calc_rot_comp(point_local_example)

    print("Solved local origin:", local_origin)
    print("Rotation Matrix:\n", rotation_matrix)
    print("Point Global:", point_global_example)
    print("Point Local:", point_local_example)
    print("Rotation Components:", rotation_components_example)

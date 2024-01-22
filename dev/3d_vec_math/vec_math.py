from scipy.optimize import fsolve
from typing import Tuple
import numpy as np
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

def def_local_coor_sys(first_point: np.ndarray, global_up: np.ndarray = np.array([0.0, 0.0, 1.0])) -> np.ndarray:
    """
    Define the local coordinate system such that the X-axis directly points to the first point,
    and the Z-axis aligns with the global upwards direction.

    Args:
    first_point (np.ndarray): The first point's coordinates.
    global_up (np.ndarray): The global upwards direction (default is [0, 0, 1]).

    Returns:
    np.ndarray: The rotation matrix representing the local coordinate system.
    """
    x_axis = np.array(first_point, dtype=float) / np.linalg.norm(first_point)
    z_axis = global_up / np.linalg.norm(global_up)
    if np.isclose(np.dot(x_axis, z_axis), 1.0):
        z_axis = np.array([0.0, 1.0, 0.0]) if x_axis[1] == 0 else np.array([1.0, 0.0, 0.0])
    y_axis = np.cross(x_axis, z_axis) / np.linalg.norm(np.cross(x_axis, z_axis))
    z_axis = np.cross(x_axis, y_axis)
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
    return np.dot(rotation_matrix, point_global)

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
    points_example = np.array([(30, 50, 0), (37, 41, 13), (43, 45, 17), (62, 34, 12)])
    angles_example = np.array([17.701, 2.7967, 18.842])

    mx, my, mz = solve_for_mxyz(points_example, angles_example)
    rotation_matrix = def_local_coor_sys(points_example[0])
    point_global_example = (37, 41, 13)
    point_local_example = global_to_local(point_global_example, rotation_matrix)
    rotation_components_example = calc_rot_comp(point_local_example)

    print("Solved mx, my, mz:", mx, my, mz)
    print("Rotation Matrix:\n", rotation_matrix)
    print("Point Global:", point_global_example)
    print("Point Local:", point_local_example)
    print("Rotation Components:", rotation_components_example)

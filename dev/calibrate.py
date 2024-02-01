from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from typing import Tuple
import numpy as np
import math

def calibrate(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray, p4: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Calculate the rotation matrix and intersection point that calibrates a system based on four points.
    
    Args:
    p1, p2, p3, p4 (np.ndarray): Four points in the global coordinate system to calibrate against.
    
    Returns:
    Tuple[np.ndarray, np.ndarray]: A tuple containing the rotation matrix and the intersection point.
    """
    x1, x2 = find_closest_points(p1, p2, p3, p4)
    intersection = calculate_midpoint(x1, x2)

    vec1 = (p1 + p2) / 2 - intersection
    vec2 = (p3 + p4) / 2 - intersection

    vec1 = vec1 / np.linalg.norm(vec1)
    vec2 = vec2 / np.linalg.norm(vec2)

    x_axis = vec1
    z_axis = np.cross(vec1, vec2)
    y_axis = np.cross(x_axis, z_axis)

    rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))

    return rotation_matrix, intersection

def find_closest_points(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray, p4: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Find the closest points between two lines defined by pairs of points.
    
    Args:
    p1, p2 (np.ndarray): Points defining the first line.
    p3, p4 (np.ndarray): Points defining the second line.
    
    Returns:
    Tuple[np.ndarray, np.ndarray]: The closest points on each line.
    """
    d1 = p2 - p1
    d2 = p4 - p3
    n = np.cross(d1, d2)

    if np.allclose(n, 0):
        raise ValueError("Lines are parallel")

    A = np.array([d1, -d2, n]).T
    b = p3 - p1
    t, s, _ = np.linalg.solve(A, b)

    closest_point_on_line1 = p1 + t * d1
    closest_point_on_line2 = p3 + s * d2

    return closest_point_on_line1, closest_point_on_line2

def calculate_midpoint(point1: np.ndarray, point2: np.ndarray) -> np.ndarray:
    """
    Calculate the midpoint between two points.
    
    Args:
    point1, point2 (np.ndarray): Points to calculate the midpoint between.
    
    Returns:
    np.ndarray: Midpoint between point1 and point2.
    """
    return (point1 + point2) / 2

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
    tilt_angle = math.degrees(math.atan2(point_local[2], math.sqrt(point_local[0]**2 + point_local[1]**2)))
    return pan_angle, tilt_angle

def plot_calibration(p1, p2, p3, p4, rotation_matrix, intersection):
    """
    Plots the calibration setup, including lines between the initial points,
    the calculated intersection point, the axes of the calculated rotation matrix,
    and the closest line segment between the two lines.
    
    Args:
    p1, p2, p3, p4 (np.ndarray): Input points used for calibration.
    rotation_matrix (np.ndarray): The rotation matrix obtained from calibration.
    intersection (np.ndarray): The intersection point calculated during calibration.
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot lines between points
    ax.plot(*zip(*[p1, p2]), color='g', label='Line 1')
    ax.plot(*zip(*[p3, p4]), color='g', label='Line 2')

    # Plot intersection point
    ax.scatter(*intersection, color='b', s=100, label='Intersection Point')

    # Plot axes of the rotation matrix from the intersection point
    axis_length = 0.5  # Length of the axes
    for i, axis in enumerate(['X', 'Y', 'Z']):
        end_point = intersection + axis_length * rotation_matrix[:, i]
        ax.quiver(*intersection, *(end_point-intersection), arrow_length_ratio=0.1, label=f'{axis}-axis')

    # Calculate and plot the closest line segment between the two lines
    closest_p1, closest_p2 = find_closest_points(p1, p2, p3, p4)
    ax.plot(*zip(*[closest_p1, closest_p2]), color='r', linestyle='--', label='Closest Segment')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.show()

def main():
    # Test find_closest_points function
    # Sample points for the lines
    p1 = np.random.uniform(-100, 100, size=(3,))
    p2 = np.random.uniform(-100, 100, size=(3,))


    p3 = np.random.uniform(-100, 100, size=(3,))
    p4 = np.random.uniform(-100, 100, size=(3,))
    closest_p1, closest_p2 = find_closest_points(p1, p2, p3, p4)
    print(f"Closest points on lines: {closest_p1}, {closest_p2}\n")

    # Test calculate_midpoint function
    midpoint = calculate_midpoint(closest_p1, closest_p2)
    print(f"Midpoint between closest points: {midpoint}\n")

    # Test calibrate function
    rotation_matrix, intersection = calibrate(p1, p2, p3, p4)
    print(f"Rotation Matrix: {rotation_matrix}\n\nIntersection Point: {intersection}\n")

    # Test global_to_local function
    point_global = np.array([1, 2, 3])
    point_local = global_to_local(point_global, rotation_matrix)
    print(f"Local coordinates of {point_global}: {point_local}\n")

    # Test calc_rot_comp function
    pan_angle, tilt_angle = calc_rot_comp(point_local)
    print(f"Pan angle: {pan_angle}, Tilt angle: {tilt_angle}\n")

    plot_calibration(p1, p2, p3, p4, rotation_matrix, intersection)


if __name__ == "__main__":
    main()


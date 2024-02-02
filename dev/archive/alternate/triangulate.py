from typing import Tuple, List
import math
from scipy.optimize import fsolve

def solve_for_mx_my(a_coords: Tuple[float, float, float], 
                    b_coords: Tuple[float, float, float], 
                    c_coords: Tuple[float, float, float], 
                    angle_AB: float, 
                    angle_AC: float) -> Tuple[float, float]:
    """
    Solve for mx and my given the coordinates of points A, B, C and the angles AB, AC.

    Args:
    a_coords (Tuple[float, float, float]): The coordinates of point A. [Central]
    b_coords (Tuple[float, float, float]): The coordinates of point B. [Left]
    c_coords (Tuple[float, float, float]): The coordinates of point C. [Right]
    angle_AB (float): The angle between vectors AB in degrees.
    angle_AC (float): The angle between vectors AC in degrees.

    Returns:
    Tuple[float, float]: The solved values of mx and my.
    """
    def equations(vars):
        mx, my = vars
        vector_A = [a_coords[0] - mx, a_coords[1] - my, a_coords[2]]
        vector_B = [b_coords[0] - mx, b_coords[1] - my, b_coords[2]]
        vector_C = [c_coords[0] - mx, c_coords[1] - my, c_coords[2]]

        # Dot product and magnitudes for angle AB
        dot_AB = sum(a*b for a, b in zip(vector_A, vector_B))
        mag_A = math.sqrt(sum(a**2 for a in vector_A))
        mag_B = math.sqrt(sum(a**2 for a in vector_B))
        angle_AB_rad = math.radians(angle_AB)
        cos_AB = math.cos(angle_AB_rad)

        # Dot product and magnitudes for angle AC
        dot_AC = sum(a*b for a, b in zip(vector_A, vector_C))
        mag_C = math.sqrt(sum(a**2 for a in vector_C))
        angle_AC_rad = math.radians(angle_AC)
        cos_AC = math.cos(angle_AC_rad)

        # Equations based on cosine of angles
        eq1 = cos_AB - (dot_AB / (mag_A * mag_B))
        eq2 = cos_AC - (dot_AC / (mag_A * mag_C))

        return [eq1, eq2]

    # Solving the equations
    mx, my = fsolve(equations, (0, 0))  # Initial guesses for mx and my
    return mx, my

# Example usage
a_coords = (3, 5, 0)
b_coords = (2, 5, 0)
c_coords = (4, 5, 0)
angle_AB = 11.310
angle_AC = -11.310

mx, my = solve_for_mx_my(a_coords, b_coords, c_coords, angle_AB, angle_AC)


print(f"mx: {round(mx,4)}, my: {round(my,4)}")

import numpy as np
import customtkinter as ctk
import tkinter as tk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from typing import Tuple
import matplotlib.pyplot as plt  # Make sure to import pyplot


# Function to find the closest points between two lines
def find_closest_points(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray, p4: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
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

# Function to calculate the midpoint of the shortest line segment
def calculate_midpoint(point1: np.ndarray, point2: np.ndarray) -> np.ndarray:
    return (point1 + point2) / 2

# Function to plot the lines and midpoint on a canvas
def plot_on_canvas():
    plt.style.use('seaborn-v0_8-deep')  # Set the style to dark

    p1 = np.random.uniform(-100, 100, size=(3,))
    p2 = np.random.uniform(-100, 100, size=(3,))
    p3 = np.random.uniform(-100, 100, size=(3,))
    p4 = np.random.uniform(-100, 100, size=(3,))

    closest_point1, closest_point2 = find_closest_points(p1, p2, p3, p4)
    midpoint = calculate_midpoint(closest_point1, closest_point2)

    fig = Figure(figsize=(5, 4), dpi=100)
    ax = fig.add_subplot(111, projection='3d')

    ax.plot([p1[0], closest_point1[0]], [p1[1], closest_point1[1]], [p1[2], closest_point1[2]], label='Line 1')
    ax.plot([p3[0], closest_point2[0]], [p3[1], closest_point2[1]], [p3[2], closest_point2[2]], label='Line 2')
    ax.plot([closest_point1[0], closest_point2[0]], [closest_point1[1], closest_point2[1]], [closest_point1[2], closest_point2[2]], 'r-', label='Shortest Segment')
    ax.scatter(*midpoint, color='purple', s=100, label='Midpoint')
    ax.legend()

    canvas = FigureCanvasTkAgg(fig, master=window)  
    canvas_widget = canvas.get_tk_widget()
    canvas_widget.grid(row=0, column=0, columnspan=2)

# Create a CustomTkinter window
window = ctk.CTk()
window.title("Matplotlib Graph in CustomTkinter")
window.geometry("600x500")

# Sample points for the lines
p1 = np.random.uniform(-100, 100, size=(3,))
p2 = np.random.uniform(-100, 100, size=(3,))
p3 = np.random.uniform(-100, 100, size=(3,))
p4 = np.random.uniform(-100, 100, size=(3,))

# Button to plot the graph
plot_button = ctk.CTkButton(master=window, text="Plot Graph", command=plot_on_canvas)
plot_button.grid(row=1, column=0, pady=10, padx=10)

# Run the application
window.mainloop()

# List of used packages with versions
required_packages = """
customtkinter==4.5.1
matplotlib==3.4.2
numpy==1.21.2
"""

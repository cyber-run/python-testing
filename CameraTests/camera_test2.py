import cv2
import EasyPySpin
import customtkinter as ctk
import tkinter as tk
from PIL import Image, ImageTk

class CameraApp:
    def __init__(self, master):
        # Initialize the camera
        self.cap = EasyPySpin.VideoCapture(0)
        
        if not self.cap.isOpened():
            print("Camera can't be opened.")
            exit(-1)
        
        # Set a lower resolution for the camera
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Other initializations...
        # ...
        
    def __del__(self):
        # Release the camera when the object is deleted
        if self.cap:
            self.cap.release()

    # ... Rest of the class ...

# Ensure the program exits cleanly
try:
    root = tk.Tk()
    app = CameraApp(root)
    root.mainloop()
except Exception as e:
    print(e)
finally:
    # This ensures that the camera release method is called
    if app.cap.isOpened():
        app.cap.release()

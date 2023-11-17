import cv2
import EasyPySpin
import customtkinter as ctk
import tkinter as tk
from PIL import Image, ImageTk
import threading

class CameraApp:
    def __init__(self, window, window_title):
        self.window = window
        self.window.title(window_title)

        # Initialize camera
        self.cap = EasyPySpin.VideoCapture(0)
        # Set a lower resolution for the camera
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Set the appearance mode and color theme
        ctk.set_appearance_mode("Dark")
        ctk.set_default_color_theme("blue")

        # Create a button that will start/stop the live video feed
        self.toggle_video_button = ctk.CTkButton(window, text="Start Live Feed", command=self.toggle_video_feed)
        self.toggle_video_button.pack(pady=20, padx=20)

        # Create a label that will hold the video frames
        self.video_label = ctk.CTkLabel(window, text="")
        self.video_label.pack()

        # Add a slider for exposure control
        self.exposure_slider = ctk.CTkSlider(window, from_=1, to=100000, command=self.adjust_exposure)
        self.exposure_slider.set(50)  # You can set the default value to be in the middle or to a known good default for your camera
        self.exposure_slider.pack(pady=20)

        # Exposure control label
        exposure = self.cap.get(cv2.CAP_PROP_EXPOSURE)
        self.exposure_label = ctk.CTkLabel(window, text="Exposure (us): {exposure}")
        self.exposure_label.pack(pady=(10, 0))

        # Control variable for the video feed status
        self.is_live = False
        self.thread = None

        self.window.protocol("WM_DELETE_WINDOW", self.on_closing)  # To handle window close event
        self.window.mainloop()

    def adjust_exposure(self, exposure_value):
        # Here you would call the method to set the camera's exposure
        # The exact method will depend on your camera's API; you might need to do some conversion
        # from the slider value (which is a float) to whatever your camera expects.
        # This example assumes your camera's API has a 'set_exposure' method and expects milliseconds.

        try:
            exposure_ms = float(exposure_value)  # Convert the slider value to a float
            self.cap.set(cv2.CAP_PROP_EXPOSURE, exposure_ms)
            self.exposure_label.configure(text=f"Exposure (us): {exposure_ms}")
        except AttributeError:
            # Handle the case where the camera does not have the 'set_exposure' method
            print("Camera does not support setting exposure programmatically.")

    def toggle_video_feed(self):
        if self.is_live:
            self.is_live = False
            self.toggle_video_button.configure(text="Start Live Feed")  # Corrected line
        else:
            self.is_live = True
            self.toggle_video_button.configure(text="Stop Live Feed")  # Corrected line
            self.update_video_label()  # Start video feed


    def update_video_label(self):
        if self.is_live:
            ret, frame = self.cap.read()
            if ret:
                # Convert the image to tkinter compatible format
                cv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                pil_img = Image.fromarray(cv_img)
                imgtk = ImageTk.PhotoImage(image=pil_img)
                self.video_label.imgtk = imgtk
                self.video_label.configure(image=imgtk)
            # Schedule the next frame update
            self.window.after(30, self.update_video_label)  # Update every ~33 milliseconds for ~30fps


    def video_loop(self):
        while self.is_live:
            ret, frame = self.cap.read()
            if ret:
                # Convert the image to tkinter compatible format
                cv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                pil_img = Image.fromarray(cv_img)
                imgtk = ImageTk.PhotoImage(image=pil_img)
                self.video_label.configure(image=imgtk)  # Update the label with the new image
                self.video_label.image = imgtk  # Keep a reference so it's not garbage-collected
            self.window.update()

    def on_closing(self):
        self.is_live = False
        if self.thread is not None:
            self.thread.join()

        self.cap.release()  # Release the video capture object
        self.window.destroy()  # Close the window

# Create a window and pass it to the CameraApp class
root = tk.Tk()
app = CameraApp(root, "CameraApp with EasyPySpin and CustomTkinter")

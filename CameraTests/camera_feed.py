import logging
logging.basicConfig(level=logging.INFO)

from PIL import Image, ImageTk
import customtkinter as ctk
import tkinter as tk
import EasyPySpin
import time
import cv2
import os
from typing import Optional


class CameraManager:
    def __init__(self):
        self.cap = None
        self.initialize_camera()

    def initialize_camera(self):
        try:
            self.cap = EasyPySpin.VideoCapture(0)
            self.configure_camera()
        except Exception as e:
            logging.error(f"Failed to initialize camera: {e}")

    def configure_camera(self):
        if self.cap:
            desired_width, desired_height = 960, 720
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, desired_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, desired_height)
            self.center_roi_on_sensor(desired_width, desired_height)

    def center_roi_on_sensor(self, roi_width, roi_height):
        sensor_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        sensor_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        offset_x, offset_y = (sensor_width - roi_width) // 2, (sensor_height - roi_height) // 2
        self.cap.set_pyspin_value("OffsetX", offset_x)
        self.cap.set_pyspin_value("OffsetY", offset_y)
        self.cap.set_pyspin_value("Width", roi_width)
        self.cap.set_pyspin_value("Height", roi_height)

    def read_frame(self):
        if self.cap:
            return self.cap.read()
        return False, None

    def release(self):
        if self.cap:
            self.cap.release()

class CameraApp:
    def __init__(self, window: ctk.CTk, window_title: str):
        self.window = window
        self.window.title(window_title)
        self.camera_manager = CameraManager()

        self.is_live = False
        self.is_saving_images = False
        self.show_crosshair = tk.BooleanVar(value=False)
        self.threshold_flag = tk.BooleanVar(value=False)
        self.detect_flag = tk.BooleanVar(value=False)

        self.setup_gui_elements()
        self.window.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.window.mainloop()

    def setup_gui_elements(self):
        self.video_label = ctk.CTkLabel(self.window, text="")
        self.video_label.pack(fill="both", expand=True)

        # Frame to hold buttons and sliders
        control_frame = ctk.CTkFrame(self.window)
        control_frame.pack(fill="x", pady=10)

        # Button to start/stop live feed
        self.toggle_video_button = ctk.CTkButton(control_frame, text="Start Live Feed", command=self.toggle_video_feed)
        self.toggle_video_button.pack(side="left", padx=10)

        # Button to start/stop saving images
        self.record_button = ctk.CTkButton(control_frame, text="Start Saving Images", command=self.toggle_image_saving)
        self.record_button.pack(side="left", padx=10)

        # Slider to adjust exposure
        self.exposure_slider = ctk.CTkSlider(control_frame, from_=1, to=100000, command=self.adjust_exposure)
        self.exposure_slider.set(50)
        self.exposure_slider.pack(side="left", padx=10)

        # Exposure label is updated within adjust_exposure if camera is initialized
        self.exposure_label = ctk.CTkLabel(control_frame, text="Exposure (us): N/A")
        self.exposure_label.pack(side="left", padx=10)

        # Checkbox to show crosshair
        self.crosshair_checkbox = ctk.CTkCheckBox(control_frame, text="Show Crosshair", variable=self.show_crosshair, onvalue=True, offvalue=False)
        self.crosshair_checkbox.pack(side="left", padx=10)

        # Checkbox to show thresholded image
        self.threshold_checkbox = ctk.CTkCheckBox(control_frame, text="Threshold", variable=self.threshold_flag, onvalue=True, offvalue=False)
        self.threshold_checkbox.pack(side="left", padx=10)

        # Checkbox to show detected dot
        self.detect_checkbox = ctk.CTkCheckBox(control_frame, text="Detect", variable=self.detect_flag, onvalue=True, offvalue=False)
        self.detect_checkbox.pack(side="left", padx=10)

        # Frame to hold threshold sliders
        threshold_frame = ctk.CTkFrame(self.window)
        threshold_frame.pack(fill="x", pady=10)

        # Slider for lower threshold boundary
        self.lower_threshold_slider = ctk.CTkSlider(threshold_frame, from_=0, to=255, command=self.set_lower_threshold)
        self.lower_threshold_slider.set(50)  # Default value, adjust as needed
        self.lower_threshold_slider.pack(side="left", padx=10)
        self.lower_threshold_value = 50  # Initialize with the default value

        # Label for lower threshold slider
        self.lower_threshold_label = ctk.CTkLabel(threshold_frame, text="Lower Threshold:")
        self.lower_threshold_label.pack(side="left", padx=10)

        # Slider for upper threshold boundary
        self.upper_threshold_slider = ctk.CTkSlider(threshold_frame, from_=0, to=255, command=self.set_upper_threshold)
        self.upper_threshold_slider.set(200)  # Default value, adjust as needed
        self.upper_threshold_slider.pack(side="left", padx=10)
        self.upper_threshold_value = 200  # Initialize with the default value

        # Label for upper threshold slider
        self.upper_threshold_label = ctk.CTkLabel(threshold_frame, text="Upper Threshold:")
        self.upper_threshold_label.pack(side="left", padx=10)

    def toggle_video_feed(self):
        self.is_live = not self.is_live
        self.toggle_video_button.configure(text="Stop Live Feed" if self.is_live else "Start Live Feed")
        if self.is_live:
            self.update_video_label()

    def toggle_image_saving(self):
        self.is_saving_images = not self.is_saving_images
        self.record_button.configure(text="Stop Saving Images" if self.is_saving_images else "Start Saving Images")

    def adjust_exposure(self, exposure_value: float):
        if self.camera_manager.cap:
            try:
                self.camera_manager.cap.set(cv2.CAP_PROP_EXPOSURE, exposure_value)
                self.exposure_label.configure(text=f"Exposure (us): {exposure_value}")
            except AttributeError:
                logging.error("Exposure not set.")
    
    def set_lower_threshold(self, value: float):
        self.lower_threshold_value = int(value)

    def set_upper_threshold(self, value: float):
        self.upper_threshold_value = int(value)

    def update_video_label(self):
        if self.is_live:
            ret, frame = self.camera_manager.read_frame()
            if ret:

                if self.show_crosshair.get():
                    frame = self.draw_crosshair(frame)

                if self.detect_flag.get():
                    frame = self.detect_dot(frame)

                self.display_frame(frame)
                self.save_frame_if_needed(frame)
            self.window.after(30, self.update_video_label)

    def detect_dot(self, frame):
        # Convert to grayscale if frame is in color
        if len(frame.shape) == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        _, thresh = cv2.threshold(frame, self.lower_threshold_value, self.upper_threshold_value, cv2.THRESH_BINARY)

        if self.threshold_flag.get():
            frame = thresh

        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Find the biggest contour (if detected)
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            # Draw the circle around the contour
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
            cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 255), -1)

        return frame

    def display_frame(self, frame):
        frame = cv2.flip(frame, -1)
        cv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(cv_img)
        imgtk = ImageTk.PhotoImage(image=pil_img)
        self.video_label.imgtk = imgtk
        self.video_label.configure(image=imgtk)

    def draw_crosshair(self, frame):
        height, width = frame.shape[:2]
        cv2.line(frame, (width // 2, 0), (width // 2, height), (0, 255, 0), 2)
        cv2.line(frame, (0, height // 2), (width, height // 2), (0, 255, 0), 2)
        return frame

    def save_frame_if_needed(self, frame):
        if self.is_saving_images:
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            filename = os.path.join(self.image_folder, f"image_{timestamp}.png")
            cv2.imwrite(filename, frame)

    def on_closing(self):
        self.is_live = False
        self.camera_manager.release()
        self.window.destroy()

if __name__ == "__main__":
    root = ctk.CTk()
    app = CameraApp(root, "DART")

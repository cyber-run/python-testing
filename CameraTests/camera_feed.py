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

class CameraApp:
    def __init__(self, window: ctk.CTk, window_title: str):
        self.window = window
        self.window.title(window_title)

        self.is_live = False  # Initialize the control variable for video feed status
        self.is_saving_images = False  # Initialize the control variable for image saving status

        self.initialize_camera()
        self.setup_gui_elements()

        self.window.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.window.mainloop()

    def initialize_camera(self):
        logging.info("Initializing camera")
        try:
            self.cap = EasyPySpin.VideoCapture(0)
            self.configure_camera()
        except Exception as e:
            logging.error(f"Failed to initialize camera: {e}")
            self.cap = None


    def configure_camera(self):
        # Set desired resolution for the camera
        desired_width = 960
        desired_height = 720
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, desired_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, desired_height)

        # Define your ROI size (this can be different from the desired resolution)
        roi_width = 960
        roi_height = 720

        # Center the ROI on the sensor
        self.center_roi_on_sensor(roi_width, roi_height)

    def center_roi_on_sensor(self, roi_width, roi_height):
        """
        Center the ROI on the camera sensor by adjusting the offset values.

        :param roi_width: Desired width of the ROI
        :param roi_height: Desired height of the ROI
        """
        sensor_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        sensor_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

        offset_x = (sensor_width - roi_width) // 2
        offset_y = (sensor_height - roi_height) // 2

        self.cap.set_pyspin_value("OffsetX", offset_x)
        self.cap.set_pyspin_value("OffsetY", offset_y)
        self.cap.set_pyspin_value("Width", roi_width)
        self.cap.set_pyspin_value("Height", roi_height)

    def setup_gui_elements(self):
        logging.info("Setting up GUI elements")
        
        ctk.set_appearance_mode("Dark")
        ctk.set_default_color_theme("blue")

        self.toggle_video_button = ctk.CTkButton(self.window, text="Start Live Feed", command=self.toggle_video_feed)
        self.toggle_video_button.pack(pady=20, padx=20)

        self.video_label = ctk.CTkLabel(self.window, text="")
        self.video_label.pack()

        self.setup_exposure_controls()
        self.setup_image_saving_controls()

    def setup_exposure_controls(self):
        self.exposure_slider = ctk.CTkSlider(self.window, from_=1, to=100000, command=self.adjust_exposure)
        self.exposure_slider.set(50)
        self.exposure_slider.pack(pady=20)

        exposure = self.get_camera_exposure()
        self.exposure_label = ctk.CTkLabel(self.window, text=f"Exposure (us): {exposure}")
        self.exposure_label.pack(pady=(10, 0))

    def get_camera_exposure(self) -> str:
        if self.cap:
            try:
                return str(self.cap.get(cv2.CAP_PROP_EXPOSURE))
            except AttributeError:
                logging.error("Failed to get exposure.")
        return "N/A"

    def setup_image_saving_controls(self):
        self.is_saving_images = False
        self.image_folder = "images"
        if not os.path.exists(self.image_folder):
            os.makedirs(self.image_folder)

        self.record_button = ctk.CTkButton(self.window, text="Start Saving Images", command=self.toggle_image_saving)
        self.record_button.pack(pady=20)

    def adjust_exposure(self, exposure_value: float):
        if self.cap:
            try:
                self.cap.set(cv2.CAP_PROP_EXPOSURE, exposure_value)
                self.exposure_label.configure(text=f"Exposure (us): {exposure_value}")
            except AttributeError:
                logging.error("Exposure not set.")

    def toggle_video_feed(self):
        self.is_live = not self.is_live
        button_text = "Stop Live Feed" if self.is_live else "Start Live Feed"
        self.toggle_video_button.configure(text=button_text)
        if self.is_live:
            self.update_video_label()

    def toggle_image_saving(self):
        self.is_saving_images = not self.is_saving_images
        button_text = "Stop Saving Images" if self.is_saving_images else "Start Saving Images"
        self.record_button.configure(text=button_text)

    def save_image(self, frame):
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        filename = os.path.join(self.image_folder, f"image_{timestamp}.png")
        cv2.imwrite(filename, frame)

    def update_video_label(self):
        if self.is_live and self.cap:
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.flip(frame, 0)
                frame = cv2.flip(frame, 1)
                self.display_frame(frame)
                self.save_frame_if_needed(frame)
            self.window.after(30, self.update_video_label)

    def display_frame(self, frame):
        cv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(cv_img)
        imgtk = ImageTk.PhotoImage(image=pil_img)
        self.video_label.imgtk = imgtk
        self.video_label.configure(image=imgtk)

    def save_frame_if_needed(self, frame):
        if self.is_saving_images:
            self.save_image(frame)

    def on_closing(self):
        self.is_live = False
        if self.cap:
            self.cap.release()
        self.window.destroy()

# Main execution
if __name__ == "__main__":
    logging.info("Starting application")
    root = ctk.CTk()
    app = CameraApp(root, "DART")
    logging.info("Application has started")
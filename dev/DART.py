import logging
logging.basicConfig(level=logging.INFO)

from camera_manager import CameraManager
from tracking import DynaTracker
from PIL import Image, ImageTk
import customtkinter as ctk
import tkinter as tk
import numpy as np
import cProfile
import time
import cv2
import os


class DART:
    def __init__(self, window: ctk.CTk):
        self.window = window
        self.window.title("DART")
        self.camera_manager = CameraManager()
        # self.tracker = DynaTracker()

        # GUI element flags
        self.is_live = False
        self.is_saving_images = False
        self.show_crosshair = tk.BooleanVar(value=False)
        self.threshold_flag = tk.BooleanVar(value=False)
        self.detect_flag = tk.BooleanVar(value=False)

        # Threshold values
        self.threshold_value = 70
        self.strength_value = 60

        self.setup_gui_elements()
        self.window.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.window.mainloop()

    def setup_gui_elements(self):
        self.video_label = ctk.CTkLabel(self.window, text="")
        self.video_label.pack(fill="both", expand=True)

        # Frame for camera controls
        camera_control_frame = ctk.CTkFrame(self.window)
        camera_control_frame.pack(padx = 20, pady=10)  # Increase vertical padding

        # Button to start/stop live feed
        self.toggle_video_button = ctk.CTkButton(camera_control_frame, text="Start Live Feed", command=self.toggle_video_feed)
        self.toggle_video_button.pack(side="left", padx=10)

        # Button to start/stop saving images
        self.record_button = ctk.CTkButton(camera_control_frame, text="Start Saving Images", command=self.toggle_image_saving)
        self.record_button.pack(side="left", padx=10)

        # Frame for exposure slider and label
        exposure_frame = ctk.CTkFrame(camera_control_frame)
        exposure_frame.pack(side="left", padx=10, pady = 10)
        self.exposure_slider = ctk.CTkSlider(exposure_frame, from_=1, to=100000, command=self.adjust_exposure)
        self.exposure_slider.set(50)
        self.exposure_slider.pack(padx =5, pady=5)
        self.exposure_label = ctk.CTkLabel(exposure_frame, text="Exposure (us): 57500")
        self.exposure_label.pack()

        # Frame for image processing detect
        img_processing_frame = ctk.CTkFrame(self.window)
        img_processing_frame.pack(padx=20, pady=10)  # Increase vertical padding

        # Checkbox for crosshair, threshold, and detection
        self.crosshair_checkbox = ctk.CTkCheckBox(img_processing_frame, text="Crosshair", variable=self.show_crosshair, onvalue=True, offvalue=False)
        self.crosshair_checkbox.pack(side="left", padx=10)

        # self.threshold_checkbox = ctk.CTkCheckBox(img_processing_frame, text="Threshold", variable=self.threshold_flag, onvalue=True, offvalue=False)
        # self.threshold_checkbox.pack(side="left", padx=10)

        self.detect_checkbox = ctk.CTkCheckBox(img_processing_frame, text="Detect", variable=self.detect_flag, onvalue=True, offvalue=False)
        self.detect_checkbox.pack(side="left", padx=10)

        # Frame for threshold slider and label
        threshold_frame = ctk.CTkFrame(img_processing_frame)
        threshold_frame.pack(side="left", padx=10, pady=10)
        self.threshold_slider = ctk.CTkSlider(threshold_frame, from_=0, to=255, command=self.set_threshold)
        self.threshold_slider.set(70)
        self.threshold_slider.pack(padx =5, pady=5)
        self.threshold_label = ctk.CTkLabel(threshold_frame, text="Threshold: 70")
        self.threshold_label.pack()

        # Frame for strength slider and label
        strength_frame = ctk.CTkFrame(img_processing_frame)
        strength_frame.pack(side="left", padx=10, pady=10)
        self.strength_slider = ctk.CTkSlider(strength_frame, from_=30, to=100, command=self.set_strength)
        self.strength_slider.set(60)
        self.strength_slider.pack(padx =5, pady=5)
        self.strength_label = ctk.CTkLabel(strength_frame, text="Strength: 60")
        self.strength_label.pack()

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
    
    def set_threshold(self, value: float):
        self.threshold_value = int(value)
        self.threshold_label.configure(text=f"Threshold: {int(value)}")

    def set_strength(self, value: float):
        self.strength_value = int(value)
        self.strength_label.configure(text=f"Strength: {int(value)}")

    def update_video_label(self):
        if self.is_live:
            ret, frame = self.camera_manager.read_frame()
            if ret:
                # Process the frame with all the selected options
                processed_frame = self.process_frame(frame)
                self.display_frame(processed_frame)
                
                # Save the original or processed frame if needed
                self.save_frame_if_needed(processed_frame)

            self.window.after(30, self.update_video_label)

    def process_frame(self, frame):
        # Convert to grayscale if needed and if the frame is color
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) \
            if self.needs_grayscale() and len(frame.shape) > 2 and frame.shape[2] == 3 \
            else frame

        # Detect dots if needed
        detection_frame = self.detect_circle(gray_frame) if self.detect_flag.get() else gray_frame

        # Draw a crosshair if needed
        crosshair_frame = self.draw_crosshair(detection_frame) if self.show_crosshair.get() else detection_frame

        # Flip the frame both vertically and horizontally
        flipped_frame = cv2.flip(crosshair_frame, -1)

        return flipped_frame

    def needs_grayscale(self):
        return self.threshold_flag.get() or self.detect_flag.get()

    def detect_circle(self, frame):
        # Assuming frame is in grayscale
        # Blur the image to reduce noise
        blurred_frame = cv2.medianBlur(frame, 5)
        
        # Apply Hough Circle Transform
        circles = cv2.HoughCircles(blurred_frame, cv2.HOUGH_GRADIENT, dp=1.2, minDist=100,
                                param1=self.threshold_value, param2=self.strength_value, minRadius=0, maxRadius=0)
        
        # If at least one circle is detected, draw it
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                center = (i[0], i[1])  # circle center
                radius = i[2]  # circle radius
                # Draw the circle center
                cv2.circle(frame, center, 1, (255, 0, 255), 3)
                # Draw the circle outline
                cv2.circle(frame, center, radius, (255, 0, 255), 2)
        
        return frame

    def display_frame(self, frame):
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
    app = DART(root)
    # cProfile.run('app = DART(root)')

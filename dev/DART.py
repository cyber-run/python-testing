import logging
logging.basicConfig(level=logging.INFO)

from dyna_controller import DynaController
from camera_manager import CameraManager
from dart_track import dart_track
from CTkMessagebox import CTkMessagebox
from multiprocessing import Process
from PIL import Image, ImageTk
import customtkinter as ctk
from mocap_stream import *
import vec_math2 as vm2
import tkinter as tk
import numpy as np
import cProfile
import pickle
import time
import cv2
import os


class DART:
    CALIBRATION_INITIAL = 1
    CALIBRATION_MIDDLE = 2
    CALIBRATION_FINAL = 4

    def __init__(self, window: ctk.CTk):
        self.init_window(window)
        self.init_gui_flags()
        self.setup_gui_elements()
        self.init_hardware()
        self.init_calibration()

        self.window.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.window.mainloop()

    def init_window(self, window):
        self.window = window
        self.window.title("DART")

    def init_hardware(self):
        self.camera_manager = CameraManager()
        self.target = MoCap(stream_type='3d')

        self.dyna = DynaController(com_port='COM5')
        self.dyna.set_op_mode(1, 3)  # Pan to position control
        self.dyna.set_op_mode(2, 3)  # Tilt to position control
        self.set_pan(0)
        self.set_tilt(0)

    def init_gui_flags(self):
        # Camera/image functionality
        self.is_live = False
        self.is_saving_images = False
        self.threshold_value = 70
        self.strength_value = 60

        # Image processing GUI flags
        self.show_crosshair = tk.BooleanVar(value=False)
        self.threshold_flag = tk.BooleanVar(value=False)
        self.detect_flag = tk.BooleanVar(value=False)

        # Motor control values
        self.pan_value = 0
        self.tilt_value = 0

    def init_calibration(self):
        # Calibration variables
        self.calibration_button_state = "initial"
        self.calibration_positions = []
        self.calibration_angles = []
        self.calibrated = False
        self.calibration_step = 0
        self.true_angle = 0
        self.pan_angle = 0

        # Load calibration data if it exists
        if os.path.exists('calib_data.pkl'):
            with open('calib_data.pkl', 'rb') as f:
                self.local_origin, self.rotation_matrix = pickle.load(f)
                self.calibrated = True
                logging.info("Calibration data loaded successfully.")

    def setup_gui_elements(self):
        self.video_label = ctk.CTkLabel(self.window, text="")
        self.video_label.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)

        # Frame for motor controls
        dyn_control_frame = ctk.CTkFrame(self.window)
        dyn_control_frame.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)  # Increase vertical padding

        # Frame for pan slider and label
        pan_frame = ctk.CTkFrame(dyn_control_frame)
        pan_frame.pack(side="top", padx=10, pady=10)
        self.pan_slider = ctk.CTkSlider(pan_frame, from_=-45, to=45, command=self.set_pan)
        self.pan_slider.set(self.pan_value)
        self.pan_slider.pack(padx =5, pady=5)
        self.pan_label = ctk.CTkLabel(pan_frame, text="Pan angle: 0")
        self.pan_label.pack()

        # Frame for tilt slider and label
        tilt_frame = ctk.CTkFrame(dyn_control_frame)
        tilt_frame.pack(side="top", padx=10, pady=10)
        self.tilt_slider = ctk.CTkSlider(tilt_frame, from_=-45, to=45, command=self.set_tilt)
        self.tilt_slider.set(self.tilt_value)
        self.tilt_slider.pack(padx =5, pady=5)
        self.tilt_label = ctk.CTkLabel(tilt_frame, text="Tilt angle: 0")
        self.tilt_label.pack()

        # Create a calibration button
        self.calibration_button = ctk.CTkButton(dyn_control_frame, text="Calibrate", command=self.calibrate)
        self.calibration_button.pack(side="top", padx=10, pady=10)

        # Create a track button
        self.track_button = ctk.CTkButton(dyn_control_frame, text="Track", command=self.track)
        self.track_button.pack(side="top", padx=10, pady=10)

        # Frame for camera controls
        camera_control_frame = ctk.CTkFrame(self.window)
        camera_control_frame.grid(row=1, column=0, sticky="nsew", padx=10, pady=10)  # Increase vertical padding

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
        img_processing_frame.grid(row=2, column=0, sticky="nsew", padx=10, pady=10)  # Increase vertical padding

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

    def track(self):
        if self.calibrated:
            # Close QTM connections
            self.target._close()
            self.target.close()

            # Close serial port
            self.dyna.close_port()

            time.sleep(3) # HACK: Add small blocking delay to allow serial port to close

            self.track_process = Process(target=dart_track)
            self.track_process.start()
        else:
            # Add popup window to notify user that DART is not calibrated
            CTkMessagebox(title="Error", message="DART Not Calibrated", icon="cancel")
            logging.error("DART is not calibrated.")

    def set_pan(self, value: float):
        self.pan_value = int(value)
        self.pan_label.configure(text=f"Pan angle: {int(value)}")
        angle = vm2.num_to_range(self.pan_value, -45, 45, 202.5, 247.5)
        self.dyna.set_pos(1, angle)

    def set_tilt(self, value: float):
        self.tilt_value = int(value)
        self.tilt_label.configure(text=f"Tilt angle: {int(value)}")
        angle = vm2.num_to_range(self.tilt_value, -45, 45, 292.5, 337.5)
        self.dyna.set_pos(2, angle)

    def calibrate(self):
        """Handle the calibration process for the DART system."""
        if self.calibration_button_state == "initial":
            # Reset calibration lists
            self.calibration_positions = []  # Reset positions list
            self.calibration_angles = []  # Reset angles list

            # First click: set pan angle to -5
            self.pan_angle = -5
            self.set_pan(self.pan_angle)
            time.sleep(0.1) # HACK: Add small blocking delay to allow pan to move

            # Change state and button text
            self.calibration_button_state = "collecting"
            self.update_calibration_button("Next", "green")

            # Capture true current pan angle when at initial state
            self.true_angle = vm2.num_to_range(self.dyna.get_pos(1), 202.5, 247.5, -45, 45)
        else:
            # Capture current target position
            self.calibration_positions.append(self.target.position)

            # Calculate and record true angle change if not the first measurement
            if self.calibration_step > 0:
                angle_change = self.true_angle - vm2.num_to_range(self.dyna.get_pos(1), 202.5, 247.5, -45, 45)
                self.calibration_angles.append(angle_change)
                print(f"Angle change: {angle_change}")
                self.true_angle = vm2.num_to_range(self.dyna.get_pos(1), 202.5, 247.5, -45, 45)

            # Update pan angle and step
            self.pan_angle += 5
            self.set_pan(self.pan_angle)
            self.calibration_step += 1

            # Check if the calibration is at the final step
            if self.calibration_step >= self.CALIBRATION_FINAL:
                self.perform_final_calibration()
                self.calibration_button_state = "initial"
                self.update_calibration_button("Calibrate", '#3a7ebf')
                self.calibration_step = 0  # Reset for next calibration
                self.set_pan(0)  # Reset pan angle

    def perform_final_calibration(self):
        """Perform the final step of the calibration process."""
        try:
            position_array = np.array(self.calibration_positions)
            angle_array = np.array(self.calibration_angles)
            initial_guess = np.array([-461, -497, -6])
            self.local_origin, self.rotation_matrix = vm2.calibrate(position_array, angle_array, initial_guess)
            self.calibrated = True

            with open('calib_data.pkl', 'wb') as f:
                pickle.dump((self.local_origin, self.rotation_matrix), f)

            logging.info("Calibration completed successfully.")
        except Exception as e:
            logging.error(f"Error during final calibration: {e}")

    def update_calibration_button(self, text, color):
        """Update the calibration button's text and background color."""
        self.calibration_button.configure(text=text, fg_color=color, command=self.calibrate)
        self.calibration_button_state = text.lower()

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

        return crosshair_frame

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
        # Close the camera
        self.is_live = False
        self.camera_manager.release()

        # Close QTM connections
        self.target._close()
        self.target.close()

        # Close serial port
        self.dyna.close_port()

        # Terminate subprocess
        if hasattr(self, 'track_process') and self.track_process.is_alive():
            self.track_process.terminate()
            self.track_process.join()  # Wait for the process to terminate

        self.window.destroy()

if __name__ == "__main__":
    root = ctk.CTk()
    app = DART(root)
    # cProfile.run('app = DART(root)')

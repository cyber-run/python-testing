from PIL import Image, ImageTk
import customtkinter as ctk
import tkinter as tk
import EasyPySpin
import logging
import time
import cv2
import os


class CameraApp:
    def __init__(self, window, window_title):
        self.window = window
        self.window.title(window_title)

        # Initialize camera
        self.cap = EasyPySpin.VideoCapture(0)
        # Set resolution for the camera
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Set appearance mode and color theme
        ctk.set_appearance_mode("Dark")
        ctk.set_default_color_theme("blue")

        # Button to start/stop live video feed
        self.toggle_video_button = ctk.CTkButton(window, text="Start Live Feed", command=self.toggle_video_feed)
        self.toggle_video_button.pack(pady=20, padx=20)

        # Label to hold video frames
        self.video_label = ctk.CTkLabel(window, text="")
        self.video_label.pack()

        # Slider for exposure control
        self.exposure_slider = ctk.CTkSlider(window, from_=1, to=100000, command=self.adjust_exposure)
        self.exposure_slider.set(50)
        self.exposure_slider.pack(pady=20)

        # Exposure control label
        exposure = self.cap.get(cv2.CAP_PROP_EXPOSURE)
        self.exposure_label = ctk.CTkLabel(window, text=f"Exposure (us): {exposure}")
        self.exposure_label.pack(pady=(10, 0))

        # Control variable for video feed status
        self.is_live = False

        # Button for image saving
        self.record_button = ctk.CTkButton(window, text="Start Saving Images", command=self.toggle_image_saving)
        self.record_button.pack(pady=20)

        # Variables for image saving
        self.is_saving_images = False
        self.image_folder = "images"

        # Create the images directory if it doesn't exist
        if not os.path.exists(self.image_folder):
            os.makedirs(self.image_folder)

        self.window.protocol("WM_DELETE_WINDOW", self.on_closing)  # Handle window close event

        self.window.mainloop() # Start the GUI


    def adjust_exposure(self, exposure_value):
        try:
            exposure_us = float(exposure_value)  # Convert the slider value to a float

            self.cap.set(cv2.CAP_PROP_EXPOSURE, exposure_us)  # Set the exposure value for the camera   

            self.exposure_label.configure(text=f"Exposure (us): {exposure_us}") # Update the exposure label

        except AttributeError:
            logging.error("Exposure not set.")


    def toggle_video_feed(self):
        if self.is_live:
            self.is_live = False
            self.toggle_video_button.configure(text="Start Live Feed")  # Corrected line
        else:
            self.is_live = True
            self.toggle_video_button.configure(text="Stop Live Feed")  # Corrected line
            self.update_video_label()  # Start video feed

    
    def toggle_image_saving(self):
            if self.is_saving_images:
                self.is_saving_images = False
                self.record_button.configure(text="Start Saving Images")
            else:
                self.is_saving_images = True
                self.record_button.configure(text="Stop Saving Images")


    def save_image(self, frame):
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        filename = os.path.join(self.image_folder, f"image_{timestamp}.png")
        cv2.imwrite(filename, frame)


    def update_video_label(self):
        if self.is_live:
            ret, frame = self.cap.read()
            if ret:
                if self.is_saving_images:
                    self.save_image(frame)

                cv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                pil_img = Image.fromarray(cv_img)
                imgtk = ImageTk.PhotoImage(image=pil_img)
                self.video_label.imgtk = imgtk
                self.video_label.configure(image=imgtk)

            self.window.after(30, self.update_video_label)


    def on_closing(self):
        self.is_live = False
        self.cap.release()
        self.window.destroy()

# Create a window and pass it to the CameraApp class
root = tk.Tk()
app = CameraApp(root, "DART")

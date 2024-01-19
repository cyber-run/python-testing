import EasyPySpin
import logging
import cv2 

class CameraManager:
    def __init__(self):
        self.cap = None
        self.initialize_camera()

    def initialize_camera(self):
        try:
            self.cap = EasyPySpin.VideoCapture(0)
            if not self.cap.isOpened():  # Check if the camera has been opened
                self.cap.start()  # Start the camera
            self.configure_camera()
        except Exception as e:
            logging.error(f"Failed to initialize camera: {e}")
            if self.cap is not None:
                self.cap.release()

    def configure_camera(self):
        if self.cap and self.cap.isOpened():
            desired_width, desired_height = 960, 720
            self.set_camera_properties(desired_width, desired_height)
            self.center_roi_on_sensor(desired_width, desired_height)

    def set_camera_properties(self, width, height):
        try:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        except Exception as e:
            logging.error(f"Failed to set camera properties: {e}")

    def center_roi_on_sensor(self, roi_width, roi_height):
        try:
            sensor_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            sensor_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            offset_x, offset_y = (sensor_width - roi_width) // 2, (sensor_height - roi_height) // 2
            self.cap.set_pyspin_value("OffsetX", offset_x)
            self.cap.set_pyspin_value("OffsetY", offset_y)
            self.cap.set_pyspin_value("Width", roi_width)
            self.cap.set_pyspin_value("Height", roi_height)
        except Exception as e:
            logging.error(f"Failed to center ROI on sensor: {e}")

    def read_frame(self):
        try:
            if self.cap and self.cap.isOpened():
                return self.cap.read()
        except _PySpin.SpinnakerException as e:
            logging.error(f"Error reading frame: {e}")
            # Implement reconnection logic or notify the user
        return False, None

    def release(self):
        if self.cap:
            self.cap.release()
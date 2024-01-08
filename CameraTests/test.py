import EasyPySpin
import cv2

def center_roi_on_sensor(cap, roi_width, roi_height):
    """
    Center the ROI on the camera sensor by adjusting the offset values.

    :param cap: EasyPySpin VideoCapture object
    :param roi_width: Desired width of the ROI
    :param roi_height: Desired height of the ROI
    """
    sensor_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    sensor_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

    offset_x = (sensor_width - roi_width) // 2
    offset_y = (sensor_height - roi_height) // 2

    cap.set_pyspin_value("OffsetX", offset_x)
    cap.set_pyspin_value("OffsetY", offset_y)
    cap.set_pyspin_value("Width", roi_width)
    cap.set_pyspin_value("Height", roi_height)

# Initialize camera
cap = EasyPySpin.VideoCapture(0)

# Define your ROI size
roi_width = 640
roi_height = 480

# Center the ROI on the sensor
center_roi_on_sensor(cap, roi_width, roi_height)

# Capture an image
ret, frame = cap.read()

# Check if capture is successful
if ret:
    # Save the frame
    cv2.imwrite("centered_roi.png", frame)

# Release the camera
cap.release()

import cv2
import EasyPySpin

cap = EasyPySpin.VideoCapture(0)

# cap.set(cv2.CAP_PROP_EXPOSURE, 100000) # us
# cap.set(cv2.CAP_PROP_GAIN, 10) # dB

width  = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

ret, frame = cap.read()

cv2.imwrite("frame.png", frame)

cap.release()
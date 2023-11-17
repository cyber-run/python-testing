import serial
import time
import MoCapStream
import numpy as np


def setup():
    # Configure the serial port. Make sure to use the correct COM port and baud rate.
    ser = serial.Serial('COM3', 115200, timeout=1)
    # Connect to the QTM server - make sure QTM is recording data
    telemetry = MoCapStream.MoCap(stream_type='3d_unlabelled')

    return ser, telemetry


def calculate_distance(point1, point2):
    return np.linalg.norm(np.array(point2) - np.array(point1))


def calculate_direction_vector(point1, point2):
    vector = np.array(point2) - np.array(point1)
    return vector / np.linalg.norm(vector)


def map_gaze_axis(tracker_points, target_points):
    # Assuming tracker_points and target_points are lists of 3D coordinates

    # Find the pair of points on the tracker that define the longer gaze axis
    max_distance = 0
    gaze_start = None
    gaze_end = None
    for i in range(len(tracker_points)):
        for j in range(i + 1, len(tracker_points)):
            distance = calculate_distance(tracker_points[i], tracker_points[j])
            if distance > max_distance:
                max_distance = distance
                gaze_start = tracker_points[i]
                gaze_end = tracker_points[j]

    # Calculate the direction vector of the longer gaze axis
    gaze_direction = calculate_direction_vector(gaze_start, gaze_end)

    # Apply the direction vector to the target
    target_mapped = [np.array(point) + gaze_direction * max_distance for point in target_points]

    return target_mapped

# Example usage:
tracker_points = [(0, 0, 0), (1, 0, 0), (0, 1, 0)]
target_points = [(1, 1, 1), (2, 1, 1), (1, 2, 1)]


def send_pwm_value(pwm_value):
    '''
    Sends a PWM value to the micro controller.
    
    Args:
        pwm_value (int): The PWM value to send to the micro controller.
        serial (serial.Serial): The serial port to send the PWM value to.
    '''
    if 900 <= pwm_value <= 2100:
        ser.write(f"{pwm_value}\n".encode())
    else:
        print("Invalid PWM value. Please enter a value between 900 and 2100.")


def main(serial, telemetry) -> None:
    # TODO: Configure main loop logic
    '''
    Prototype pseudo code:
    1. Get current 3D unlabelled markers from QTM - use telemetry.position
        - This returns a dictionary of marker names and their positions
        - This will have 2 objects: the target and the tracker
    2. Do coordinate calculation to find coordinate transformation needed to map the tracker to the target
        - Try masters method first with timeit
        - Likely too slow so implement with core math functions
    3. Map coordinate transformation to PWM values
    4. Send PWM values to micro controller
    '''
    try:
        while True:
            print(telemetry.position)
            send_pwm_value(1800)
            mapped_target = map_gaze_axis(tracker_points, target_points)
            print("Mapped Target Points:", mapped_target)

    except KeyboardInterrupt:
        print("Exiting program.")
        ser.close()
    

if __name__ == "__main__":
    #  Call setup function - connecting serial and QTM server
    ser, telemetry = setup()
    # Call main function - main loop
    main(ser, telemetry)

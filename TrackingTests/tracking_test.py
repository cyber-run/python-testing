import serial
import time
import MoCapStream
import numpy as np
import math


def setup():
    # Configure the serial port. Make sure to use the correct COM port and baud rate.
    ser = serial.Serial('COM3', 115200, timeout=1)
    # Connect to the QTM server - make sure QTM is recording data
    target = MoCapStream.MoCap(stream_type='3d_unlabelled')
    tracker = MoCapStream.MoCap(stream_type='6d')

    return ser, target, tracker


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


def calibrate_yaw(tracker, idx=0, lim=100):
    '''
    Sets the PWM out to 1500 and captures 100 samples of the tracker's yaw.
    Calculates the mean of the yaw values and returns the mean as the yaw offset.
    
    Args:
        tracker (MoCapStream.MoCap): The tracker object.
    
    Returns:
        yaw_offset (float): The yaw offset of the tracker from global coordinates.
    '''
    yaw = np.zeros(100)
    send_pwm_value(1500)

    while idx < lim:
        yaw[idx] = tracker.rotation
        idx += 1
        time.sleep(1/100)
    
    yaw_offset = np.mean(yaw)

    return yaw_offset
        

def calculate_yaw_change(tracker_position, target_position, offset) -> int:
    # Calculate the vector from the tracker to the target
    vector_to_target = [target_position[0] - tracker_position[0],
                        target_position[1] - tracker_position[1],
                        target_position[2] - tracker_position[2]]

    # Calculate the current yaw angle of the tracker
    current_yaw = math.atan2(vector_to_target[1], vector_to_target[0])

    # Adjust for the yaw offset obtained during calibration
    current_yaw -= offset

    return current_yaw


def map_yaw_change_to_pwm(yaw_change, pwm_center=1500, pwm_range=600) -> int:
    # Map yaw change to PWM values based on your requirements
    # This is a simple linear mapping for illustration, adjust as needed
    return int(pwm_center + yaw_change * pwm_range / (2 * math.pi))


def main(ser, target, tracker, pwm = 1500, flag = 0, step = 3) -> None:
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
    offset = calibrate_yaw(tracker)

    try:
        while True:
            # Returns x,y,z,roll,pitch,yaw of tracker rigid body
            tracker_pose = tracker.state

            tracker_position = tracker_pose[:3]
            tracker_rotation = tracker_pose[3:]
            tracker_yaw = tracker_rotation[2]

            # Returns x,y,z of target point
            target_point = target.position

            # Calculate the yaw angle change needed for alignment
            yaw_change = calculate_yaw_change(tracker_position, target_point, offset)

            # Map the yaw change to PWM values
            pwm_value = map_yaw_change_to_pwm(yaw_change)

            # Send PWM values to the servo motor
            send_pwm_value(pwm_value, ser)

            time.sleep(1)


    except KeyboardInterrupt:
        print("Exiting program.")
        ser.close()
    

if __name__ == "__main__":
    #  Call setup function - connecting serial and QTM server
    ser, target, tracker = setup()

    # Call main function - main loop
    main(ser, target, tracker)


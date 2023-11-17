import serial
import time
import MoCapStream
import numpy as np


def setup():
    # Configure the serial port. Make sure to use the correct COM port and baud rate.
    ser = serial.Serial('COM3', 115200, timeout=1)
    # Connect to the QTM server - make sure QTM is recording data
    target = MoCapStream.MoCap(stream_type='3d_unlabelled')
    tracker = MoCapStream.MoCap(stream_type='6d')

    return ser, target, tracker


def calculate_distance(point1, point2):
    return np.linalg.norm(np.array(point2) - np.array(point1))

def calculate_direction_vector(point1, point2):
    vector = np.array(point2) - np.array(point1)
    return vector / np.linalg.norm(vector)

def map_gaze_axis(tracker_pose, target_point):
    # Assuming tracker_pose is a 6DOF pose represented as [x, y, z, roll, pitch, yaw]
    # and target_point is a 3D point represented as [x, y, z]

    # Extract translation and rotation components from the tracker's pose
    tracker_translation = tracker_pose[:3]
    tracker_rotation = tracker_pose[3:]

    # Convert the rotation representation to a rotation matrix
    rotation_matrix = Rotation.from_euler('xyz', tracker_rotation, degrees=True).as_matrix()

    # Apply the rotation and translation to the target point
    target_mapped = np.dot(rotation_matrix, target_point) + tracker_translation

    return target_mapped


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


def point_towards_target(tracker_position, tracker_yaw, target_point):
    # Calculate the direction vector from the current position to the target point
    direction_vector = np.subtract(target_point, tracker_position)
    
    # Calculate the yaw angle to point towards the target (arctan2 is used to handle all quadrants)
    target_yaw = np.arctan2(direction_vector[1], direction_vector[0])

    return target_yaw


def yaw_to_pwm(change_in_yaw, yaw_range=80, pwm_min=900, pwm_max=2100):
    # Calculate PWM value based on the change in yaw
    pwm_value = pwm_min + ((change_in_yaw / yaw_range) * (pwm_max - pwm_min))

    # Ensure the PWM value is within the valid range
    pwm_value = max(pwm_min, min(pwm_value, pwm_max))

    return pwm_value


def calibrate_yaw(tracker, idx=0, lim=100):
    '''
    Calculates the yaw of tracker from global coordinate access when pwm is 1500.
    
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
            # print(f"Tracker position: {tracker.rotation - offset}")
            # print(f"Target position: {target.position}\n")
            start_time = time.time()

            tracker_pose = tracker.state
            tracker_position = tracker_pose[:3]
            tracker_rotation = tracker_pose[3:]
            tracker_yaw = tracker_rotation[2] - offset

            target_point = target.position

            target_yaw = point_towards_target(tracker_position, tracker_yaw, target_point)

            change_in_yaw = target_yaw - tracker_yaw

            mapped_pwm = yaw_to_pwm(change_in_yaw)

            print(mapped_pwm)

            send_pwm_value(mapped_pwm)
            time.sleep(5)
            print(f"--- {time.time() - start_time} seconds ---")


    except KeyboardInterrupt:
        print("Exiting program.")
        ser.close()
    

if __name__ == "__main__":
    #  Call setup function - connecting serial and QTM server
    ser, target, tracker = setup()

    # Call main function - main loop
    main(ser, target, tracker)


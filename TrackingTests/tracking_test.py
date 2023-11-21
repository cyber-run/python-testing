import serial
import time
import MoCapStream
import numpy as np
import vec_math as vm
import math
from PID import PID_controller as PID


def setup():
    # Configure the serial port. Make sure to use the correct COM port and baud rate.
    ser = serial.Serial('COM3', 115200, timeout=1)
    # Connect to the QTM server - make sure QTM is recording data
    target = MoCapStream.MoCap(stream_type='3d_unlabelled')
    tracker = MoCapStream.MoCap(stream_type='6d')

    return ser, target, tracker


def calibrate_yaw(tracker, us_range=(1000, 2000), num_cycles=3, samples_per_cycle=2):
    print(f"------Calibrating yaw offset for {num_cycles} cycles------\n")

    # Send 1500 us to center the servo and wait for manual center alignment
    send_us_val(1500)
    # input("Manually align the servo and press Enter when ready...")
    time.sleep(1)
    yaw_center = tracker.rotation
    
    min_yaw = float('inf')
    max_yaw = float('-inf')

    for _ in range(num_cycles):
        for us_val in range(us_range[0], us_range[1] + 1, (us_range[1] - us_range[0]) // samples_per_cycle):
            send_us_val(us_val)
            time.sleep(1)  # Adjust sleep time as needed, small delay to be sure qualisys data pulled in correct
            yaw = tracker.rotation
            # Normalize the yaw angle to the range [0, 360]
            yaw = (yaw + 360) % 360
            min_yaw = min(min_yaw, yaw)
            max_yaw = max(max_yaw, yaw)

    yaw_range = (abs(min_yaw) + abs(max_yaw))

    print(f"Yaw center: {yaw_center} degrees\n")
    print(f"Yaw range: {yaw_range} degrees\n")
    time.sleep(3)

    return yaw_center, yaw_range


def map_yaw_2_us(target_yaw, yaw_center, yaw_range, us_center=1500, us_range=500) -> int:

    # Map the yaw angle to a us value
    us_val = us_center + target_yaw/yaw_range * us_range

    return us_val


def send_us_val(us_val):

    if 1000 <= us_val <= 2000:
        ser.write(f"{us_val}\n".encode())
    else:
        print("Invalid us val. Please enter a val between 900 and 2100.")


def main(ser, target, tracker, us = 1500, flag = 0, step = 3) -> None:

    # Grab and state and wait to ensure QTM connection is established
    tracker_pose = tracker.state
    time.sleep(1)

    pid = PID(3, 0.1, 0)

    # Calibrate to obtain the yaw offset
    yaw_centre, yaw_range = calibrate_yaw(tracker)

    send_us_val(us)
    time.sleep(1)

    tracker_basis_pos = tracker.state[:3]
    # print(f"Tracker basis position: {tracker_basis_pos}\n")
    yaw_basis = vm.calc_yaw_vec(yaw_centre, tracker_basis_pos)
    # print(f"Tracker basis yaw: {yaw_basis} degrees\n")

    try:
        while True:
            # Returns x,y,z of target point
            target_point = target.position 

            yaw = tracker.rotation
            print(f"Yaw: {yaw} degrees\n")

            # Calculate the vector from the tracker basis to the target point
            target_vec = vm.calc_vec(tracker_basis_pos, target_point)
            print(f"Target vector: {target_vec}\n")

            # Calculate the yaw angle between the target point and the tracker basis
            yaw_target = vm.vec_ang_delta(yaw_basis, target_vec)
            print(f"Yaw target: {yaw_target} degrees\n")

            # Calculate the us val to send to the servo motor
            us_val = map_yaw_2_us(yaw_target, yaw_centre, yaw_range)
            print(f"Sending us val: {us_val}\n")

            control = pid.update(us_val)

            # Send us vals to the servo motor
            send_us_val(us_val)

            time.sleep(0.01)


    except KeyboardInterrupt:
        print("Exiting program.")
        ser.close()
    

if __name__ == "__main__":
    #  Call setup function - connecting serial and QTM server
    ser, target, tracker = setup()

    # Call main function - main loop
    main(ser, target, tracker)


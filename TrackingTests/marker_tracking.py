from PID import PID_controller as PID
from MoCapStream import MoCap
from importlib import reload
from scipy.io import savemat
import numpy as np
from itertools import count
from typing import Tuple
import vec_math as vm
import logging
import timeit
import serial
import time

import os
import win32api
import win32process
import win32con


class ServoTracker:
    def __init__(self, port: str = 'COM3', baud_rate: int = 115200, timeout: int = 1) -> None:

        # I/O interface params
        self.ser = serial.Serial(port, baud_rate, timeout=timeout)
        self.target = MoCap(stream_type='3d')
        self.tracker = MoCap(stream_type='6d')
        self.pid = PID(0.9, 0.01, 0.05)

        # Servo control params
        self.us_val = 1500
        self.angle_err = 0

        # Yaw calibration/control params
        self.min_yaw = float('inf')
        self.max_yaw = float('-inf')
        self.yaw_range = 0
        self.yaw = 0

        # Lists for storing data for saving to file
        self.data_lists = {
            'yaw': [],
            'angle error': [],
            'time stamp': []
        }

    def send_us_val(self, us_val: int) -> None:
        if 1000 <= us_val <= 2000:
            self.ser.write(f"{us_val}\n".encode())
            self.us_val = us_val
        else:
            logging.error("Invalid us val %s. Please enter a val between 1000 and 2000.", us_val)

        return
    
    def send_command(self, command: str) -> None:
        '''
        Function to send general commands over serial to micro controller.
        - Sending a number 1-50 to set the delay of the target movement:
            - ie 1 = 1ms delay, 50 = 50ms delay, where less delay is faster movement
        - Sending the character s or x will start or stop the target movement
        '''
        self.ser.write(f"{command}\n".encode())
        return

    def calibrate_yaw(self, us_range: Tuple[int, int] = (1000, 2000), num_cycles: int = 1, samples_per_cycle: int = 2) -> None:
        # Grab positions of target and tracker and wait for small duration to ensure connection to DB
        target_pos = self.target.position
        tracker_pos = self.tracker.position
        time.sleep(0.3)

        print(f"\n------Calibrating servos yaw params for {num_cycles} cycles------")

        # Send 1500 us to center the servo and wait for manual center alignment
        self.send_us_val(1500)
        input("Manually align the servo and press Enter when ready...")
        time.sleep(0.3)

        # Cycle over pwm range n times and record min and max yaw values
        for _ in range(num_cycles):
            for us_val in range(us_range[0], us_range[1] + 1, (us_range[1] - us_range[0]) // samples_per_cycle):
                self.send_us_val(us_val)
                time.sleep(0.3)  # Adjust sleep time as needed, small delay to be sure qualisys data pulled in correct
                yaw = self.tracker.rotation

                # Normalize the yaw angle to the range [0, 360]
                yaw = (yaw + 360) % 360
                self.min_yaw = min(self.min_yaw, yaw)
                self.max_yaw = max(self.max_yaw, yaw)

        self.yaw_range = self.max_yaw - self.min_yaw  # Calculate the range as the difference between max and min

        logging.info("Yaw min: %s degrees\n", self.min_yaw)
        logging.info("Yaw max: %s degrees\n", self.max_yaw)
        logging.info("Yaw range: %s degrees\n", self.max_yaw)
        logging.info("------------------------\n")

        time.sleep(0.3)

        return

    def set_yaw(self, control: float, us_center: int = 1500, us_range: int = 1000) -> None:

        self.yaw = self.yaw + control
        
        # Map the yaw angle to a us value
        us_val = us_center + (( self.yaw/self.yaw_range) * us_range)
        self.send_us_val(us_val)
        
        return
    
    def track(self) -> None:
        if self.target.lost is True or self.tracker.lost is True:
            logging.info("Target or tracker body lost. Skipping iteration.\n")
            return

        # Get current tracker yaw
        tracker_yaw = self.tracker.rotation

        # Calculate current tracker yaw angle
        yaw_vec = vm.calc_yaw_vec(tracker_yaw)

        # Get current positions of target and tracker
        target_pos = self.target.position
        tracker_pos = self.tracker.position

        # Calculate the vector between the target and tracker
        target_vec = vm.calc_vec(target_pos, tracker_pos)

        # Calculate the angle between the target vector and the yaw vector
        angle_err = vm.vec_ang_delta(target_vec, yaw_vec)
        self.angle_err = angle_err

        # Calculate the control signal
        control = self.pid.update(-angle_err)

        # Set the yaw angle
        self.set_yaw(control)

        # Log results
        logging.info("Target vector: %s", target_vec)
        logging.info("Angle error: %s", angle_err)
        logging.info("Control: %s", control)

        time.sleep(0.0001)

    def store_data(self) -> None:
            self.data_lists['yaw'].append(self.yaw)
            self.data_lists['angle error'].append(self.angle_err)
            self.data_lists['time stamp'].append(time.perf_counter())

    def save_data(self, filename: str = 'data') -> None:
        first_time = self.data_lists['time stamp'][0]
        self.data_lists['time stamp'] = [t - first_time for t in self.data_lists['time stamp']]
        savemat(filename + '.mat', self.data_lists)
        np.save(filename, self.data_lists)

    def shutdown(self) -> None:
        # Close serial connection
        self.ser.close()

        # Close QTM connections
        self.target._close()
        self.tracker._close()
        self.target.close()
        self.tracker.close()

        return

def set_realtime_priority():
        handle = win32api.OpenProcess(win32con.PROCESS_ALL_ACCESS, True, os.getpid())
        win32process.SetPriorityClass(handle, win32process.REALTIME_PRIORITY_CLASS) # you could set this to REALTIME_PRIORITY_CLASS etc.

if __name__ == "__main__":

    # Set high priority for process
    set_realtime_priority()
    
    reload(logging)
    # Set up logging
    logging.basicConfig(level=logging.CRITICAL)

    # Initialize servo controller
    servo_controller = ServoTracker()

    # Calibrate yaw
    servo_controller.calibrate_yaw()

    # Start target movement
    servo_controller.send_command('s')

    servo_controller.send_command('1')
    
    try:
        for _ in count():
            servo_controller.track()
            servo_controller.store_data()
        
    except KeyboardInterrupt:
        # Stop target movement
        servo_controller.send_command('x')

        # Shutdown I/O interfaces and save recorded data
        logging.info("Exiting program.")
        servo_controller.shutdown()
        servo_controller.save_data()

    except Exception as e:
        # Stop target movement
        servo_controller.send_command('x')

        # Log error and shutdown I/O interfaces
        logging.error(f"An error occurred: {e}")
        servo_controller.shutdown()
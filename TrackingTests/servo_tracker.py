from mocap_stream import MoCap
from importlib import reload
from itertools import count
from typing import Tuple
import vec_math as vm
from pid import PID
import numpy as np
import logging
import serial
import time

# System imports
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
        self.pid = PID(0.2, 0, 0)

        # MOCAP tracking params
        self.angle_err = 0
        self.yaw = 0

        # Servo control params
        self.us_val = 1500
        self.min_yaw = float('inf')
        self.max_yaw = float('-inf')
        self.yaw_range = 0

        # Lists for storing data for saving to file
        self.data_lists = {
            'yaw': [],
            'angle_error': [],
            'time_stamp': []
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

    def calibrate_yaw(self, delay: float = 0.3, us_range: Tuple[int, int] = (1000, 2000), num_cycles: int = 2, samples_per_cycle: int = 2) -> None:
        # Grab positions of target and tracker and wait for small duration to ensure connection to DB
        target_pos = self.target.position
        tracker_pos = self.tracker.position
        time.sleep(0.3)

        print(f"\n------Calibrating servos yaw params for {num_cycles} cycles------")

        # Send 1500 us to center the servo and wait for manual center alignment
        self.send_us_val(1500)
        input("Manually align the servo and press Enter when ready...")
        time.sleep(delay)

        # Cycle over pwm range n times and record min and max yaw values
        for _ in range(num_cycles):
            for us_val in range(us_range[0], us_range[1] + 1, (us_range[1] - us_range[0]) // samples_per_cycle):
                self.send_us_val(us_val)
                time.sleep(delay)  # Adjust sleep time as needed, small delay to be sure qualisys data pulled in correct
                yaw = self.tracker.rotation

                # Normalize the yaw angle to the range [0, 360]
                if -90 < yaw < -180:
                    yaw = (yaw + 360) % 360
                logging.info("Yaw: %s degrees", yaw)
                self.min_yaw = min(self.min_yaw, yaw)
                self.max_yaw = max(self.max_yaw, yaw)

        self.yaw_range = self.max_yaw - self.min_yaw  # Calculate the range as the difference between max and min

        logging.info("Yaw min: %s degrees\n", self.min_yaw)
        logging.info("Yaw max: %s degrees\n", self.max_yaw)
        logging.info("Yaw range: %s degrees\n", self.yaw_range)
        logging.info("------------------------\n")

        time.sleep(delay)

        return

    def set_yaw(self, control: float, us_center: int = 1500, us_range: int = 1000) -> None:

        self.yaw = self.yaw + control
        
        # Map the yaw angle to a us value
        us_val = us_center + (( self.yaw/self.yaw_range) * us_range)
        self.send_us_val(us_val)
        
        return
    
    def track(self, delay: int = 0.0005, direction: int = 1) -> None:
        '''
        Main tracking function

        args:
            delay: time to wait between iterations
            direction: 1 for clockwise, -1 for counter-clockwise
                       Used to flip the sign of the control signal
                       May be necessary depending on servo marker/QTM body orientation
        '''
        if self.target.lost is True or self.tracker.lost is True:
            logging.info("Target or tracker body lost. Skipping iteration.\n")
            return

        # Get current tracker yaw
        tracker_yaw = self.tracker.rotation

        # Calculate current tracker yaw vector
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
        control = self.pid.update(direction*angle_err)

        # Set the yaw angle
        self.set_yaw(control)

        # Log results
        logging.info("Target vector: %s", target_vec)
        logging.info("Angle error: %s", angle_err)
        logging.info("Control: %s", control)

        time.sleep(delay)

    def store_data(self) -> None:
        self.data_lists['yaw'].append(self.yaw)
        self.data_lists['angle_error'].append(self.angle_err)
        self.data_lists['time_stamp'].append(time.perf_counter())

    def save_data(self, filename: str = 'data', exclude_percent: int = 20) -> None:
        # Create the "temp" directory if it doesn't exist
        temp_dir = 'temp'
        os.makedirs(temp_dir, exist_ok=True)

        # Create a directory based on the filename inside the "temp" directory
        output_dir = os.path.join(temp_dir, filename)
        os.makedirs(output_dir, exist_ok=True)

        for key in self.data_lists.keys():
            variable_name = f"{filename}{key}"
            variable_value = np.array(self.data_lists[key])

            # Exclude the first 20% of the data due to initial windup response of servo
            exclude_samples = int(len(variable_value) * exclude_percent / 100)
            variable_value = variable_value[exclude_samples:]

            if key == 'time_stamp':
                variable_value -= variable_value[0]
                variable_value = np.around(variable_value, decimals=5)

            npy_file_path = os.path.join(output_dir, f"{key}.npy")
            np.save(npy_file_path, variable_value)

    def empty_data(self) -> None:
        self.data_lists = {
            'yaw': [],
            'angle_error': [],
            'time_stamp': []
        }

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
    
    # Reload logging package so level can be declared here w/o conflict
    reload(logging)
    logging.basicConfig(level=logging.CRITICAL)

    # Initialize servo controller
    servo_controller = ServoTracker()

    # Calibrate yaw
    servo_controller.calibrate_yaw(num_cycles=1)

    # Start target movement
    servo_controller.send_command('s')

    servo_controller.send_command('13.3')
    
    try:
        for _ in count():
            servo_controller.track(0.0005, direction=-1)
            # servo_controller.store_data()
        
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
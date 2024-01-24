from dyna_controller import *
from importlib import reload
from mocap_stream import *
import dev.alternate.vec_math as vm
import numpy as np
import cProfile
import logging
import time


class DynaTracker:
    '''
    Object to track a target using a Dynamixel servo and a QTM mocap system.
    
    - Before running this script, ensure that the Dynamixel servo is connected to
    the computer via USB and that the QTM mocap system is running and streaming
    data.
    - In QTM align
    '''
    def __init__(self, com_port='COM5'):
        # Connect to QTM; init tracker and target
        self.target = MoCap(stream_type='3d')
        self.tracker = MoCap(stream_type='6d')
        time.sleep(1)

        # Create dynamixel controller object and open serial port
        self.dyna = DynaController(com_port)
        self.dyna.open_port()
        
        # Default init operating mode into position
        self.dyna.set_op_mode(self.dyna.pan_id, 3)
        self.dyna.set_op_mode(self.dyna.tilt_id, 3)

        # Marker tracking params
        self.target_angle = 0
        self.curr_angle = 0

        # Local mirror vecs
        self.local_yaw_centre = [144.55, -43.33, 51]
        self.local_pitch_centre = [144.55, 64.77, 51]

        # # Set the dynamixel to its initial 180 degree position ie centre range
        self.dyna.set_sync_pos(225, 45)

        # Get global mirror centres
        self.global_yaw_centre = vm.local_to_global(self.local_yaw_centre, self.tracker.matrix, self.tracker.position)
        self.global_pitch_centre = vm.local_to_global(self.local_pitch_centre, self.tracker.matrix, self.tracker.position)

        # Store the current pitch centre position for angle calcs
        self.yaw_servo_centre = self.dyna.get_pos(self.dyna.pan_id)
        self.pitch_servo_centre = self.dyna.get_pos(self.dyna.tilt_id)

        # Calculate the reference vectors
        x_vector = [self.tracker.matrix[0][0], self.tracker.matrix[1][0], self.tracker.matrix[2][0]]
        self.ref_yaw_vec = [x_vector[0], x_vector[1]]

        z_vector = [self.tracker.matrix[0][2], self.tracker.matrix[1][2], self.tracker.matrix[2][2]]
        self.ref_pitch_vec = [-z_vector[1], -z_vector[2]]

        # Get the current tracker position for angle calcs
        self.tracker_pos = self.tracker.position


    def track(self):
        if self.target.lost:
            logging.info("Target lost. Skipping iteration.")
            return

        # Get target and tracker positions
        target_pos = self.target.position

        # Calculate the target vectors
        target_yaw_vec = vm.calc_azi_vec(target_pos, self.global_yaw_centre)
        target_pitch_vec = vm.calc_elv_vec(target_pos, self.global_pitch_centre)

        # Calculate the angle error
        self.target_yaw_angle = vm.vec_ang_delta(target_yaw_vec, self.ref_pitch_vec)
        self.target_pitch_angle = vm.vec_ang_delta(target_pitch_vec, self.ref_pitch_vec)
        # print(f'Target yaw angle: {self.target_yaw_angle}')
        
        # Map yaw target angle to servo range
        yaw_angle = self.num_to_range(-90, 90, 180, 270, self.target_yaw_angle)
        print(f'Current yaw angle: {self.target_yaw_angle}')

        # Map pitch target angle to servo range
        pitch_angle = self.num_to_range(-90, 0, 0, 45, self.target_pitch_angle)
        print(f'Current pitch angle: {self.target_pitch_angle}\n\n')

        self.dyna.set_sync_pos(yaw_angle, pitch_angle)
        logging.info("Adjusting angle to: %s, %s", yaw_angle, pitch_angle)


    def num_to_range(self, inMin, inMax, outMin, outMax, inVal):
        return (inVal - inMin) * (outMax - outMin) / (inMax - inMin) + outMin


    def shutdown(self) -> None:
        self.target._close()
        self.tracker._close()
    
        # Close QTM connections
        self.target.close()
        self.tracker.close()

        # Close serial port
        self.dyna.close_port()

        return
    

def main() -> None:
    reload(logging)
    logging.basicConfig(level=logging.ERROR)

    set_realtime_priority()

    dyna_tracker = DynaTracker()
    
    try:

        while True:
            dyna_tracker.track()
            # time.sleep(0.1)

    except KeyboardInterrupt:
        dyna_tracker.shutdown()
        print("Port closed successfully\n")
        sys.exit(0)

    except Exception as e:
        dyna_tracker.shutdown()
        print(f"An error occurred: {e}")
        sys.exit(1)


if __name__ == '__main__':
    # cProfile.run('main()')
    main()
        
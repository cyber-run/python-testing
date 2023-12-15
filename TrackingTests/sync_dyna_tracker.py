from servo_tracker import set_realtime_priority
from sync_dyna_controller import *
from importlib import reload
from mocap_stream import *
import vec_math as vm
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

        # Create dynamixel controller object and open serial port
        self.dyna = DynaController(com_port)
        self.dyna.open_port()
        time.sleep(0.5)

        # Marker tracking params
        self.target_angle = 0
        self.curr_angle = 0

        # Local mirror vecs
        self.local_yaw_centre = [144.55, -43.33, 51]
        self.local_pitch_centre = [144.55, 64.77, 51]

        # Default init operating mode into position
        self.dyna.set_op_mode(self.dyna.pan_id, 3)
        self.dyna.set_op_mode(self.dyna.tilt_id, 3)

        # # Set the dynamixel to its initial 180 degree position ie centre range
        self.dyna.set_sync_pos(225, 45)
        input("Press enter to continue")

        # Connect to QTM; init tracker and target
        self.target = MoCap(stream_type='3d')
        self.tracker = MoCap(stream_type='6d')
        time.sleep(1)

        # Get global mirror centres
        logging.info("Getting global mirror centres")
        self.global_yaw_centre = vm.local_to_global(self.local_yaw_centre, self.tracker.matrix, self.tracker.position)
        self.global_pitch_centre = vm.local_to_global(self.local_pitch_centre, self.tracker.matrix, self.tracker.position)
        logging.info("Yaw centre: %s", self.global_yaw_centre)
        logging.info("Pitch centre: %s", self.global_pitch_centre)

        # Store the current pitch centre position for angle calcs
        self.yaw_servo_centre = self.dyna.get_pos(self.dyna.pan_id)
        self.pitch_servo_centre = self.dyna.get_pos(self.dyna.tilt_id)

        # Calculate the reference vectors
        self.ref_yaw_vec = vm.calc_yaw_vec(225)
        self.ref_pitch_vec = vm.calc_pitch_vec(45)

        # Get the current tracker position for angle calcs
        self.tracker_pos = self.tracker.position


    def track(self):
        if self.target.lost:
            logging.info("Target lost. Skipping iteration.")
            return

        # Get target and tracker positions
        target_pos = self.target.position

        # Calculate the target vectors
        target_yaw_vec = vm.calc_elv_vec(target_pos, self.global_yaw_centre)
        target_pitch_vec = vm.calc_elv_vec(target_pos, self.global_pitch_centre)

        # Calculate the angle error
        self.target_yaw_angle = vm.vec_ang_delta(target_yaw_vec, self.ref_pitch_vec)
        self.target_pitch_angle = vm.vec_ang_delta(target_pitch_vec, self.ref_pitch_vec)

        # Adjust the servo position based on the angle error
        self.curr_yaw_angle = (self.yaw_servo_centre - self.target_yaw_angle) % 360
        self.curr_pitch_angle = (self.pitch_servo_centre - self.target_pitch_angle) % 360

        self.dyna.set_sync_pos(self.curr_yaw_angle, self.curr_pitch_angle)
        logging.info("Adjusting angle to: %s, %s", self.curr_yaw_angle, self.curr_pitch_angle)

        time.sleep(0.1)


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
    logging.basicConfig(level=logging.INFO)

    set_realtime_priority()

    dyna_tracker = DynaTracker()

    try:
        dyna_tracker.dyna.set_torque(False)

        # Change dynamixel EEPROM settings here

        dyna_tracker.dyna.set_op_mode(3)

        #############################################

        dyna_tracker.dyna.set_torque(True)

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
        
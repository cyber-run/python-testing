from servo_tracker import set_realtime_priority
from importlib import reload
from dyna_controller import *
from mocap_stream import *
import vec_math as vm
from pid import PID
import logging
import time


class DynaTracker:
    def __init__(self):

        # I/O interface params
        self.dyna = DynaController(com_port='COM5')
        self.target = MoCap(stream_type='3d')
        self.tracker = MoCap(stream_type='6d')

        # Marker tracking params
        self.angle_err = 0
        self.yaw = 0
        self.pid = PID(0.2, 0, 0)

        # Set torque to false to allow for EEPROM writes
        self.dyna.set_torque(False)
        logging.info("Get torque status: %s", self.dyna.get_torque())

        # Default init operating mode into position
        self.dyna.set_op_mode(3)
        logging.info("Get operating mode: %s", self.dyna.get_op_mode())


    def track(self):
        # Get target position
        target_pos = self.target.get_position()

        # Get tracker position
        tracker_pos = self.tracker.get_position()

        # Calculate angle error
        self.angle_err = vm.get_angle_error(target_pos, tracker_pos)

        # Send yaw to servo
        self.dyna.set_position(self.yaw)

        # Print yaw and angle error
        logging.info("Yaw: %s, Angle Error: %s", self.yaw, self.angle_err)

        return
    
if __name__ == '__main__':
    reload(logging)
    logging.basicConfig(level=logging.ERROR)

    set_realtime_priority()

    dyna_tracker = DynaTracker()

    try:
        dyna_tracker.dyna.set_torque(False)

        # Change dynamixel EEPROM settings here

        dyna_tracker.dyna.set_op_mode(3)

        #############################################

        dyna_tracker.dyna.set_torque(True)

        # Calibrate position
        dyna_tracker.dyna.calibrate_position()

        start_time = time.perf_counter()

        for i in range(0, 360, 1):

            dyna_tracker.dyna.set_pos(i)

        end_time = time.perf_counter()

        time_elapsed = end_time - start_time

        print(f"Time elapsed: {time_elapsed}")

        print(f"Time per tx: {1000*(time_elapsed/360)}")

    except KeyboardInterrupt:
        dyna_tracker.target.close()
        dyna_tracker.tracker.close()
        dyna_tracker.dyna.close_port()
        print("Port closed successfully\n")
        sys.exit(0)

    except Exception as e:
        dyna_tracker.target.close()
        dyna_tracker.tracker.close()
        dyna_tracker.dyna.close_port()
        print(f"An error occurred: {e}")
        sys.exit(1)
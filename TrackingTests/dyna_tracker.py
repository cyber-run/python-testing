from servo_tracker import set_realtime_priority
from importlib import reload
from dyna_controller import *
from mocap_stream import *
import vec_math as vm
import cProfile
import logging
import time


class DynaTracker:
    def __init__(self, com_port='COM5'):

        # I/O interface params
        self.dyna = DynaController(com_port)
        time.sleep(0.5)

        # Marker tracking params
        self.target_angle = 0
        self.curr_angle = 0

        # Set torque to false to allow for EEPROM writes
        self.dyna.set_torque(False)
        logging.info("Get torque status: %s", self.dyna.get_torque())

        # Default init operating mode into position
        self.dyna.set_op_mode(3)
        logging.info("Get operating mode: %s", self.dyna.get_op_mode())

        # Set torque to true to allow for position control
        self.dyna.set_torque(True)

        # # Set the dynamixel to its initial 180 degree position ie centre range
        self.dyna.calibrate_position()

        # Connect to QTM; init tracker and target
        self.target = MoCap(stream_type='3d')
        self.tracker = MoCap(stream_type='6d')
        time.sleep(1)

        # Record the mocap pitch value at this calibrated position
        self.ref_pitch = self.tracker.pitch
        # Calculate the pitch vector of this centre reference
        self.ref_pitch_vec = vm.calc_pitch_vec(float(self.ref_pitch))

        # Store the current pitch centre position for angle calcs
        self.servo_angle_centre = self.dyna.get_pos()

        # Get the current tracker position for angle calcs
        self.tracker_pos = self.tracker.position


    def track(self):
        if self.target.lost:
            logging.info("Target lost. Skipping iteration.")
            return

        # Get target and tracker positions
        target_pos = self.target.position

        # Calculate the target vector
        target_vec = vm.calc_elv_vec(target_pos, self.tracker_pos)

        # Calculate the angle error
        self.target_angle = vm.vec_ang_delta(target_vec, self.ref_pitch_vec)
        logging.info("Target angle: %s", self.target_angle)

        # Adjust the servo position based on the angle error
        self.curr_angle = (self.servo_angle_centre - self.target_angle) % 360

        self.dyna.set_pos(self.curr_angle)
        logging.info("Adjusting angle to: %s", self.curr_angle)


    def shutdown(self) -> None:
        self.target._close()
        self.tracker._close()
    
        # Close QTM connections
        self.target.close()
        self.tracker.close()

        self.dyna.set_torque(False)
        logging.info("Get torque status: %s", self.dyna.get_torque())
        # Close serial port
        self.dyna.close_port()

        return
    

def main() -> None:
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
        
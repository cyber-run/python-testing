# Run on python venv 312
# Units is in ticks: 1 tick = 0.0439453125
# ie 1 revolution = 8192 ticks
# Params use the pint package for units
# Can also set params using a perferred unit
# ie tm.controller.position.setpoint = 200 * ureg.degrees
# or tm.controller.position.setpoint = 200 * ureg.radians

from tinymovr.tee import init_tee
from tinymovr.config import get_bus_config, create_device
import time
import can


class ServoTracker:
    def __init__(self) -> None:

        params = get_bus_config(["canine", "slcan_disco"])
        params["bitrate"] = 1000000
        init_tee(can.Bus(**params))

        self.tm = create_device(node_id=1)

        self.tm.controller.calibrate()

        while self.tm.calibrated is False:
            time.sleep(2)
            print(f"Calibration check: {self.tm.calibrated}")

        print(f"Calibration success: {self.tm.calibrated}")        


    def position_control(self) -> None:
        self.tm.controller.position_mode()

        self.tm.controller.position.setpoint = 200
        time.sleep(1)

        self.tm.controller.position.setpoint = -200
        time.sleep(1)


    def velocity_control(self) -> None:
        self.tm.controller.velocity_mode()

        self.tm.controller.velocity.setpoint = 1000
        time.sleep(2)

        self.tm.controller.velocity.setpoint = -1000
        time.sleep(2)

    
def tics_2_degs(tics: int) -> float:
        return tics * 0.0439453125
    

def degs_2_tics(degs: float) -> int:
        return int(degs / 0.0439453125)


if __name__ == "__main__":
    tracker = ServoTracker()

    tracker.tm.controller.position_mode()

    # Using the setpoint attribute of the controller object, we can get/set position from yaml
    # Data type is pint object, so we need to use the magnitude attribute to get/set the value
    # Most likely all values will be pint objects, so we will need to use the magnitude attribute
    # If intereseted the units can be accessed with the units attribute
    start = int(tracker.tm.controller.position.setpoint.magnitude)

    print(f"Start: {tics_2_degs(start)}")

    for i in range(start,start+100000,1):
        tracker.tm.controller.position.setpoint = i

        pos = tics_2_degs(int(tracker.tm.controller.position.setpoint.magnitude))

        print(f"Current position: {pos}")
        time.sleep(0.0005)

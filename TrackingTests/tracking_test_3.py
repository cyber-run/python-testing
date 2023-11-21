from PID import PID_controller as PID
from MoCapStream import MoCap
import vec_math as vm
import timeit
import serial
import time


class ServoControl:
    def __init__(self, port='COM3', baud_rate=115200, timeout=1):
        self.ser = serial.Serial(port, baud_rate, timeout=timeout)
        self.target = MoCap(stream_type='3d_unlabelled')
        self.tracker = MoCap(stream_type='6d')
        self.pid = PID(0.2, 0.01, 0.05)

        self.us_val = 1500

        self.min_yaw = float('inf')
        self.max_yaw = float('-inf')
        self.yaw_range = 0
        self.yaw = 0

    def send_us_val(self, us_val):
        if 1000 <= us_val <= 2000:
            self.ser.write(f"{us_val}\n".encode())
            self.us_val = us_val
        else:
            print(f"Invalid us val {us_val}. Please enter a val between 1000 and 2000.")

    def calibrate_yaw(self, us_range=(1000, 2000), num_cycles=2, samples_per_cycle=2):
        print(f"------Calibrating servos yaw params for {num_cycles} cycles------\n")

        # Send 1500 us to center the servo and wait for manual center alignment
        self.send_us_val(1500)
        # input("Manually align the servo and press Enter when ready...")
        time.sleep(1)

        # Cycle over pwm range n times and record min and max yaw values
        for _ in range(num_cycles):
            for us_val in range(us_range[0], us_range[1] + 1, (us_range[1] - us_range[0]) // samples_per_cycle):
                self.send_us_val(us_val)
                time.sleep(1)  # Adjust sleep time as needed, small delay to be sure qualisys data pulled in correct
                yaw = self.tracker.rotation
                # Normalize the yaw angle to the range [0, 360]
                yaw = (yaw + 360) % 360
                # print(f'Yaw: {yaw} degrees\n')
                self.min_yaw = min(self.min_yaw, yaw)
                self.max_yaw = max(self.max_yaw, yaw)

        self.yaw_range = self.max_yaw - self.min_yaw  # Calculate the range as the difference between max and min

        print(f"Yaw min: {self.min_yaw} degrees\n")
        print(f"Yaw max: {self.max_yaw} degrees\n")
        print(f"Yaw range: {self.yaw_range} degrees\n")
        print(f"------------------------\n\n")
        time.sleep(3)

        return

    def set_yaw(self, control, us_center=1500, us_range=1000):

        self.yaw = self.yaw + control
        
        # Map the yaw angle to a us value
        us_val = us_center + (( self.yaw/self.yaw_range) * us_range)

        print(f"Us val: {us_val}\n")
        print(f"------------------------------------------------\n")

        self.send_us_val(us_val)
        
        return

    def run(self):
        # Grab positions of target and tracker and wait for small duration to ensure connection to DB
        target_pos = self.target.position
        tracker_pos = self.tracker.position
        time.sleep(1)

        # Call calibrate yaw function to calibrate the yaw offset
        self.calibrate_yaw()
        time.sleep(1)

        self.us_val = 1000 # For debug purposes put arbitrary bottom pwm period

        while True:
            try:
                # Condition to skip iteration if target or tracker position is None or NaN
                # if target_pos is None or tracker_pos is None:
                #     print("Target or tracker position is None. Skipping iteration.")
                #     break
                # if target_pos is float("NaN") or tracker_pos is float("NaN"):
                #     print("Target or tracker position is NaN. Skipping iteration.")
                #     break

                if self.target.body_lost is True or self.tracker.body_lost is True:
                    print(f"Target or tracker body lost. Skipping iteration.\n")
                    continue

                # Get current tracker yaw
                tracker_yaw = self.tracker.rotation

                # Calculate current tracker yaw angle
                yaw_vec = vm.calc_yaw_vec(tracker_yaw)

                # Get current positions of target and tracker
                target_pos = self.target.position
                tracker_pos = self.tracker.position

                # Calculate the vector between the target and tracker
                target_vec = vm.calc_vec(target_pos, tracker_pos)
                print(f"Target vector: {target_vec}\n")

                # Calculate the angle between the target vector and the yaw vector
                angle_err = vm.vec_ang_delta(target_vec, yaw_vec)
                print(f"Angle error: {angle_err} degrees\n")

                control = self.pid.update(-angle_err)
                print(f"Control: {control}\n")

                self.set_yaw(control)

                time.sleep(1)


            except KeyboardInterrupt:
                print("Exiting program.")
                self.ser.close()
                return

if __name__ == "__main__":
    servo_controller = ServoControl()
    servo_controller.run()

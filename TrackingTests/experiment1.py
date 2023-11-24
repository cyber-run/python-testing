from marker_tracking import ServoTracker
from scipy.io import savemat


def setup():
    servo_tracker = ServoTracker()
    servo_tracker.run()

    # Put marker target code here => probably send a trigger to a pico

    return servo_tracker

if __name__ == "__main__":
    servo_tracker = setup()
    error = []
    time_stamp = []
    i = 0

    try:
        while True:
        
            error[i] = servo_tracker.angle_err()
            time_stamp[i] = time_stamp.perf_counter()

    except KeyboardInterrupt:
        print("Exiting program.")

        # Close serial connection
        servo_tracker.ser.close()

        # Close QTM connections
        servo_tracker.target._close()
        servo_tracker.tracker._close()
        servo_tracker.target.close()
        servo_tracker.tracker.close()

    
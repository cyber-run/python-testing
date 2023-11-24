import logging
logging.basicConfig(level=logging.CRITICAL) # CRITICAL, ERROR, WARNING, INFO, DEBUG, NOTSET
from marker_tracking import ServoTracker
from marker_tracking import set_realtime_priority
import time



if __name__ == "__main__":
    # Set high priority for process
    set_realtime_priority()

    # Initialize servo controller
    servo_tracker = ServoTracker()

    try:
        # Calibrate yaw
        servo_tracker.calibrate_yaw(delay=0.3)

        servo_tracker.send_us_val(1500)

        time.sleep(1)

        # Start target movement
        servo_tracker.send_command('s')

        # Set target delay to 1
        servo_tracker.send_command('1')

        for i in range(1, 6, 1):
            # Stop target delay
            servo_tracker.send_command(str(i))

            # Start target movement
            servo_tracker.send_command('s')

            start = time.perf_counter()

            while time.perf_counter() < start + 8 + i:
                servo_tracker.track()
                servo_tracker.store_data()

            # Stop target movement
            servo_tracker.send_command('x')

            servo_tracker.save_data('data' + str(i))

            time.sleep(2)

            # Empty data
            servo_tracker.empty_data()
        
        # Stop target movement
        servo_tracker.send_command('x')

        # Close servo controller
        servo_tracker.shutdown()
    
    except KeyboardInterrupt:
        # Stop target movement
        servo_tracker.send_command('x')

        # Shutdown I/O interfaces and save recorded data
        logging.info("Exiting program.")
        servo_tracker.shutdown()

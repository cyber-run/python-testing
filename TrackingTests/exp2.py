from importlib import reload
from servo_tracker import *
from data_analysis import *
import numpy as np
import logging
import time


def main():
    reload(logging)
    logging.basicConfig(level=logging.ERROR)

    # Set the frequency range for the Bode plot
    start_freq = 0.1
    end_freq = 1.33
    num_points = 20

    frequencies = np.round(np.logspace(np.log10(start_freq), np.log10(end_freq), num_points), 3)

    # Set high priority for process
    set_realtime_priority()

    # Initialize servo controller
    servo_tracker = ServoTracker()

    try:
        # Calibrate yaw
        servo_tracker.calibrate_yaw(delay=0.3, num_cycles=1)

        # Create lists to store data for Bode plot
        amplitude_data = []
        phase_data = []

        for frequency in frequencies:
            # Center tracking servo to begin
            servo_tracker.send_us_val(1500)
            # Starting delay
            time.sleep(1)

            # Set target frequency (multiply by 10 for Arduino code)
            servo_tracker.send_command(frequency * 10)

            # Start target movement
            servo_tracker.send_command('s')

            # Collect data for 5 periods
            duration = 8 * (1 / frequency)
            start_time = time.perf_counter()

            while time.perf_counter() - start_time < duration:
                servo_tracker.track(direction=-1)
                servo_tracker.store_data()

            # Stop target movement
            servo_tracker.send_command('x')

            # Save data for analysis
            data_filename = f'data_{frequency}Hz'  # Update the filename extension
            servo_tracker.save_data(data_filename)

            # Analyze data and extract amplitude and phase information
            tracker_yaw, target_yaw, amplitude, phase = analyze_data(data_filename, (1 / frequency) )

            # Average phase difference over the 5 periods
            phase = -abs(np.mean(phase))

            print(f'----Frequency: {frequency} Hz----')
            print(f'Amplitude: {amplitude} (gain)')
            print(f'Phase: {phase} degrees\n')

            # Append amplitude and phase to lists
            amplitude_data.append(amplitude)
            phase_data.append(phase)

            # Empty data for the next iteration
            servo_tracker.empty_data()

        # Convert amplitude to dB
        amplitude_data_dB = [20 * np.log10(a) for a in amplitude_data]

        # Generate Bode plot
        plot_bode(frequencies, amplitude_data_dB, phase_data, 'Bode Plot')

    finally:
        # Stop target movement
        servo_tracker.send_command('x')

        # Close servo controller
        servo_tracker.shutdown()


if __name__ == "__main__":
    main()

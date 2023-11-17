import serial
import time
import MoCapStream

def setup():
    # Configure the serial port. Make sure to use the correct COM port and baud rate.
    ser = serial.Serial('COM3', 115200, timeout=1)
    # Connect to the QTM server - make sure QTM is recording data
    telemetry = MoCapStream.MoCap(stream_type='3d_unlabelled')

    return ser, telemetry


def send_pwm_value(pwm_value):
    '''
    Sends a PWM value to the micro controller.
    
    Args:
        pwm_value (int): The PWM value to send to the micro controller.
        serial (serial.Serial): The serial port to send the PWM value to.
    '''
    if 900 <= pwm_value <= 2100:
        ser.write(f"{pwm_value}\n".encode())
    else:
        print("Invalid PWM value. Please enter a value between 900 and 2100.")


def main(serial, telemetry) -> None:
    # TODO: Configure main loop logic
    '''
    Prototype pseudo code:
    1. Get current 3D unlabelled markers from QTM - use telemetry.position
        - This returns a dictionary of marker names and their positions
        - This will have 2 objects: the target and the tracker
    2. Do coordinate calculation to find coordinate transformation needed to map the tracker to the target
        - Try masters method first with timeit
        - Likely too slow so implement with core math functions
    3. Map coordinate transformation to PWM values
    4. Send PWM values to micro controller
    '''
    try:
        while True:
            print(telemetry.position)
            send_pwm_value(1800)

    except KeyboardInterrupt:
        print("Exiting program.")
        ser.close()
    

if __name__ == "__main__":
    #  Call setup function - connecting serial and QTM server
    ser, telemetry = setup()
    # Call main function - main loop
    main(ser, telemetry)

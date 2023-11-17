import serial
import time

# Configure the serial port. Make sure to use the correct COM port and baud rate.
ser = serial.Serial('/dev/tty.usbmodem144101', 115200, timeout=1)

def send_pwm_value(pwm_value):
    # Convert the PWM value to a string and send it with a newline character as the end marker
    ser.write(f"{pwm_value}\n".encode())

# Example usage:
try:
    while True:
        # Replace this with your logic to generate the PWM values (e.g., reading from user input, sensor data, etc.)
        pwm_value = int(input("Enter PWM value (500-2500): "))

        if 500 <= pwm_value <= 2500:
            send_pwm_value(pwm_value)
        else:
            print("Invalid PWM value. Please enter a value between 500 and 2500.")

except KeyboardInterrupt:
    print("Exiting program.")
    ser.close()

import serial
import time

# Configure the serial port. Make sure to use the correct COM port and baud rate.
ser = serial.Serial('/dev/cu.usbmodem144101', 115200, timeout=1)

def send_pwm_value(pwm_value):
    start_time = time.time()  # Start the timer
    ser.write(f"{pwm_value}\n".encode())
    
    # Wait for the response
    response = ser.readline().decode().strip()
    
    end_time = time.time()  # Stop the timer
    round_trip_time = (end_time - start_time)*1000
    
    print(f"Round-trip time: {round_trip_time:.6f} ms")
    print(f"Response from Pico: {response}")

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

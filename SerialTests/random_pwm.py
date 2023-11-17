import serial
import time
import random

def send_pwm_value(pwm_value: int) -> None:
    start_time = time.time()  # Start the timer
    ser.write(f"{pwm_value}\n".encode())
    
    # Wait for the response
    response = ser.readline().decode().strip()
    
    end_time = time.time()  # Stop the timer
    round_trip_time = (end_time - start_time)*1000
    
    # print(f"Round-trip time: {round_trip_time:.6f} ms")
    # print(f"Response from Pico: {response}")


# Configure the serial port. Make sure to use the correct COM port and baud rate.
ser = serial.Serial('/dev/tty.usbmodem144101', 115200, timeout=1)
pwm = 1000
flag = 0
step = 5

try:
    while True:
        # Replace this with your logic to generate the PWM values (e.g., reading from user input, sensor data, etc.)
        if flag == 0:
            pwm += step
        else:
            pwm -= step
        if pwm >= 2000:
            flag = 1
            step += 5
        elif pwm <= 1000: 
            flag = 0
            step += 5
        send_pwm_value(pwm)
        time.sleep(2/1000)
        

except KeyboardInterrupt:
    print("Exiting program.")
    ser.close()

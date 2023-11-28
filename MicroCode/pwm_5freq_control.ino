// TO SEND FREQUENCY COMMANDS TO THE TARGET SERVO:
// The arduino will divide any number between 1 and 50 by 10 to get the frequency in Hz.

#include <Servo.h>

Servo trackerServo;
int trackerPwm = 0;

Servo targetServo;
int targetPwm = 1000;  // Start from 1000
int stepSize = 4;     // Step size for increasing or decreasing PWM
float stepDelay = 6;    // Delay between each step
int stepDirection = 1; // 1 for increasing, -1 for decreasing
float targetFrequency = 0.5;  // Start with a default frequency (in Hz)

boolean startTargetServo = false; // Global flag to start/stop the target servo

// an array to store the received data
const byte numChars = 32;
char receivedChars[numChars];

boolean newData = false;
int dataNumber = 0;

void setup() {
    trackerServo.attach(0);
    Serial.begin(115200);
    Serial.println("<Arduino is ready>");
}

void setup1() {
    // Initialization for the second core
    targetServo.attach(1);
}

void loop() {
    recvWithEndMarker();
    readNewNumber();
}

void loop1() {
    if (startTargetServo) {
        targetPwm = targetPwm + (stepDirection * stepSize);

        if (targetPwm >= 2000) {
            targetPwm = 2000;
            stepDirection = -1;
        } else if (targetPwm <= 1000) {
            targetPwm = 1000;
            stepDirection = 1;
        }

        targetServo.writeMicroseconds(targetPwm);
        delay(stepDelay);
    }
}

void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;

    if (Serial.available() > 0) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        } else {
            // terminate the string
            receivedChars[ndx] = '\0';
            ndx = 0;
            newData = true;
        }
    }
}

void readNewNumber() {
    if (newData == true) {
        trackerPwm = atoi(receivedChars);

//        Serial.print("This just in: ");
//        Serial.println(receivedChars);
//        Serial.print("Data as Number: ");
//        Serial.println(trackerPwm);

        if (trackerPwm >= 1000 && trackerPwm <= 2000) {
            trackerServo.writeMicroseconds(trackerPwm);
            Serial.println("PWM set successfully");
        } else {

          // Keep alternative commands in else of PWM set, 
          // so they are skipped unless it is not a PWM command
          
          targetFrequency = atof(receivedChars);

          if (targetFrequency > 0 && targetFrequency <= 50) {
            targetFrequency = targetFrequency/10;
            // Calculate the time period for one full oscillation
            float oscillationPeriod = 1.0 / targetFrequency;
  
            // Calculate the total number of steps within one oscillation
            int totalSteps = 2000 / stepSize;  // Adjust the step size as needed
  
            // Calculate the time delay between each step
            stepDelay = int(oscillationPeriod * 1000 / totalSteps);  // Convert to milliseconds
  
            Serial.println("Frequency set.");
          }
  
          // Set the global flag based on the received command
          if (strcmp(receivedChars, "s") == 0) {
              startTargetServo = true;
              Serial.println("Starting Target Servo");
          } else if (strcmp(receivedChars, "x") == 0) {
              startTargetServo = false;
              Serial.println("Stopping Target Servo");
          }
        }

        newData = false;
    }
}
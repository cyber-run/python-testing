# include <Servo.h>

Servo trackerServo;
int trackerPwm = 0;

Servo targetServo;
int targetPwm = 1000;  // Start from 1000
int stepSize = 10;     // Step size for increasing or decreasing PWM
int timeDelay = 50;    // Delay between each step
int stepDirection = 1;     // 1 for increasing, -1 for decreasing

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
  targetPwm = targetPwm + (stepDirection*stepSize);

  if (targetPwm >= 2000) {
    targetPwm = 2000;
    stepDirection = -1;
  }
  else if (targetPwm <= 1000) {
    targetPwm = 1000;
    stepDirection = 1;
  }

  targetServo.writeMicroseconds(targetPwm);
  delay(timeDelay);
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
        }
        else {
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

        Serial.print("This just in: ");
        Serial.println(receivedChars);
        Serial.print("Data as Number: ");
        Serial.println(trackerPwm);

        if (trackerPwm >= 1000 && trackerPwm <= 2000) {
            trackerServo.writeMicroseconds(trackerPwm);
            Serial.println("PWM set successfully");  // Send a response back
        } else {
            Serial.println("Invalid PWM value. Please enter a value between 500 and 2500.");
        }

        newData = false;
    }
}
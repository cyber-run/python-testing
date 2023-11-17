# include <Servo.h>

Servo servo;
int pwm = 0;

// an array to store the received data
const byte numChars = 32;
char receivedChars[numChars];

boolean newData = false;
int dataNumber = 0;

void setup() {
    servo.attach(0);
    Serial.begin(115200);
    Serial.println("<Arduino is ready>");
}

void loop() {
    recvWithEndMarker();
    readNewNumber();
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
        pwm = atoi(receivedChars);

        Serial.print("This just in: ");
        Serial.println(receivedChars);
        
        if (pwm == 0 && receivedChars[0] != '0') {
            Serial.println("Invalid input. Please enter a valid number.");
        } else {
            Serial.print("Data as Number: ");
            Serial.println(pwm);
            // Set pwm val if within range
            if (pwm >= 900 && pwm <= 2100) {
                servo.writeMicroseconds(pwm);
            } else {
                Serial.println("Invalid PWM value. Please enter a value between 500 and 2500.");
            }
        }
        newData = false;
    }
}

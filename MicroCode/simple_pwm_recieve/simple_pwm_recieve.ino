# include <Servo.h>

Servo servo;

void setup() {
  Serial.begin(115200);
  servo.attach(0);
}

void loop() {
  if(Serial.available() > 0) {
    int pwm = Serial.parseInt();
    // maybe set some guards here ?
    if(pwm >= 500 && pwm <= 2500){
      servo.writeMicroseconds(pwm);
      //setServoPWM(pwm);
    }
  }
}

// void setServoPWM(pwm) {
//   servo.writeMicroseconds(pwm);
//   // delay(1000);
// }
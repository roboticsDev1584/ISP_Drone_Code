#include <Servo.h>

Servo motor1;

void setup() {
  //create a Servo object for the ESC initialized on pin 14 that runs between 1000 and 2000 pulse width
  motor1.attach(14,1000,2000);
  motor1.writeMicroseconds(1000); //initialize the signal
}

void loop() {
  //the Servo.write() function has a range of 0 to 180 for the motor speed
  motor1.writeMicroseconds(1200);
}

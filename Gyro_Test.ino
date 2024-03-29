/*  NOTES
X Axis Angle: more positive when rotated clockwise, more negative when rotated counterclockwise
Y Axis Angle: more positive when rotated clockwise, more negative when rotated counterclockwise
Z Axis Angle: more positive when rotated counterclockwise, more negative when rotated clockwise
*/

#include <Arduino_LSM6DSOX.h>
#include <PID_v1.h>

//basic control setup code
double maxVel = 5.0;
double maxAngle = 180.0;
float xAcc = 0.0, yAcc = 0.0, zAcc = 0.0;
double xAngle = 0.0, yAngle = 0.0, zAngle = 0.0;
double xAngleN = 0.0, yAngleN = 0.0, zAngleN = 0.0;
float xAngleC = 0.0, yAngleC = 0.0, zAngleC = 0.0;
double maxXAngle = 10000000.0, maxYAngle = 10000000.0, maxZAngle = 10000000.0;
int rangedXVel = 0, rangedYVel = 0, rangedZVel = 0, rangedXAngle = 0, rangedYAngle = 0, rangedZAngle = 0;
int motor1Set = 0, motor2Set = 0, motor3Set = 0, motor4Set = 0;

//PID setup code
double xAngleOut, xAngleSet = 0.0; //note that xAngleSet will always be set to 0.0 to make sure it re-levels
double kp1 = 1.0;
double ki1 = 0.0;
double kd1 = 0.0;
//only used when x velocity is set to 0.0
PID xAnglePID(&xAngle, &xAngleOut, &xAngleSet, kp1, ki1, kd1, DIRECT);
double yAngleOut, yAngleSet = 0.0; //note that yAngleSet will always be set to 0.0 to make sure it re-levels
double kp2 = 1.0;
double ki2 = 0.0;
double kd2 = 0.0;
//only used when y velocity is set to 0.0
PID yAnglePID(&yAngle, &yAngleOut, &yAngleSet, kp2, ki2, kd2, DIRECT);
double zAngleOut, zAngleSet = 0.0;
double kp3 = 1.0;
double ki3 = 0.0;
double kd3 = 0.0;
PID zAnglePID(&zAngle, &zAngleOut, &zAngleSet, kp3, ki3, kd3, DIRECT);
double xVel, xVelOut, xVelSet = 0.0;
double kp4 = 0.0;
double ki4 = 0.0;
double kd4 = 0.0;
PID xVelPID(&xVel, &xVelOut, &xVelSet, kp4, ki4, kd4, DIRECT);
double yVel, yVelOut, yVelSet = 0.0;
double kp5 = 0.0;
double ki5 = 0.0;
double kd5 = 0.0;
PID yVelPID(&yVel, &yVelOut, &yVelSet, kp5, ki5, kd5, DIRECT);
double zVel, zVelOut, zVelSet = 0.0;
double kp6 = 0.0;
double ki6 = 0.0;
double kd6 = 0.0;
PID zVelPID(&zVel, &zVelOut, &zVelSet, kp6, ki6, kd6, DIRECT);

//basic monitoring functions
double absolute(double num) {
  if (num < 0) { num *= -1.0; }
  return num;  
}
void updateAcc() {
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(xAcc, yAcc, zAcc);
  }
}
void updateVel() {
  updateAcc();
  xVel += (double(xAcc) * double(IMU.accelerationSampleRate()));
  yVel += (double(yAcc) * double(IMU.accelerationSampleRate()));
  zVel += (double(zAcc) * double(IMU.accelerationSampleRate()));
}
void updateAngle() {
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(xAngleC, yAngleC, zAngleC);
  }
  //fix xAngleC tolerance
  if (((xAngleC > 0.6) && (xAngleC > 0)) || ((xAngleC < -0.6) && (xAngleC < 0))) {
    xAngleN += (double(xAngleC) * double(IMU.gyroscopeSampleRate()));
  }
  //fix yAngleC tolerance
  if (((yAngleC > 1.0) && (yAngleC > 0)) || ((yAngleC < -1.0) && (yAngleC < 0))) {
    yAngleN += (double(yAngleC) * double(IMU.gyroscopeSampleRate()));
  }
  //fix zAngleC tolerance
  if (((zAngleC > 0.7) && (zAngleC > 0)) || ((zAngleC < -0.7) && (zAngleC < 0))) {
    zAngleN += (double(zAngleC) * double(IMU.gyroscopeSampleRate()));
  }
  //normalize x angle value range
  if (double(absolute((xAngleN))) > maxXAngle) { 
    if (xAngleN < 0) {
      xAngleN = (maxXAngle * -1); 
    }
    else {
      xAngleN = maxXAngle; 
    }
  }
  xAngle = (xAngleN * 360.0) / maxXAngle;
  //normalize y angle value range
  if (double(absolute((yAngleN))) > maxYAngle) { 
    if (yAngleN < 0) {
      yAngleN = (maxYAngle * -1); 
    }
    else {
      yAngleN = maxYAngle; 
    }
  }
  yAngle = (yAngleN * 360.0) / maxYAngle;
  //normalize z angle value range
  if (double(absolute((zAngleN))) > maxZAngle) { 
    if (zAngleN < 0) {
      zAngleN = (maxZAngle * -1); 
    }
    else {
      zAngleN = maxZAngle; 
    }
  }
  zAngle = (zAngleN * 360.0) / maxZAngle;
}
void resetIMU() {
  Serial.println(F("Resetting IMU"));
  IMU.end();
  IMU.begin();
}

//setpoint value functions
void updateXVelSetpoint() {
  double setpoint = 0.0;
  //the drone can be set to a max velocity of 4.0
  if (setpoint > (maxVel - 1.0)) { setpoint = maxVel; }
  else if (setpoint < 0.2) { setpoint = 0.0; }
  xVelSet = setpoint;
}
void updateYVelSetpoint() {
  double setpoint = 0.0;
  if (setpoint > (maxVel - 1.0)) { setpoint = maxVel; }
  else if (setpoint < 0.2) { setpoint = 0.0; }
  yVelSet = setpoint;
}
void updateZVelSetpoint() {
  double setpoint = 0.0;
  if (setpoint > (maxVel - 1.0)) { setpoint = maxVel; }
  else if (setpoint < 0.2) { setpoint = 0.0; }
  zVelSet = setpoint;
}
void updateZAngleSetpoint() {
  double setpoint = 0.0;
  if (setpoint > (maxAngle * 2.0)) { setpoint = (maxAngle * 2.0); }
  else if (setpoint < 1.0) { setpoint = 0.0; }
  zAngleSet = setpoint;
}
void updatePIDValues() {
  updateXVelSetpoint();
  updateYVelSetpoint();
  updateZVelSetpoint();
  updateZAngleSetpoint();
}
void computePID() {
  xAnglePID.Compute();
  yAnglePID.Compute();
  zAnglePID.Compute();
  xVelPID.Compute();
  yVelPID.Compute();
  zVelPID.Compute();
}
void rangeZVel() {
  if (double(abs((zVelOut))) > maxVel) { 
    if (zVelOut < 0) {
      zVelOut = (maxVel * -1); 
    }
    else {
      zVelOut = maxVel; 
    }
  }
  //this scales the z velocity from 0 to 2*maxVel and then scales it from 0 to maxVel
  zVelOut += maxVel;
  zVelOut /= 2;
  //this ranges the z velocity PID output on a scale from 0 to 160 b/c this is essentially thrust
  double rangeCalc = (zVelOut * 160.0) / maxVel;
  if (rangeCalc < 5.0) { rangeCalc = 0.0; }
  rangedZVel = int(rangeCalc);
}
void rangeXVel() {
  //the PID can adjust the drone to have a max correction up to a velocity of 5.0
  if (double(absolute((xVelOut))) > maxVel) { 
    if (xVelOut < 0) {
      xVelOut = (maxVel * -1); 
    }
    else {
      xVelOut = maxVel; 
    }
  }
  //this ranges the x velocity PID output on a scale from -rangedZVel to rangedZVel
  double rangeCalc = (xVelOut * double(rangedZVel)) / maxVel;
  if (absolute(rangeCalc) < 5.0) { rangeCalc = 0.0; }
  rangedXVel = int(rangeCalc);
}
void rangeYVel() {
  if (double(absolute((yVelOut))) > maxVel) { 
    if (yVelOut < 0) {
      yVelOut = (maxVel * -1); 
    }
    else {
      yVelOut = maxVel; 
    }
  }
  //this ranges the y velocity PID output on a scale from -rangedZVel to rangedZVel
  double rangeCalc = (yVelOut * double(rangedZVel)) / maxVel;
  if (absolute(rangeCalc) < 5.0) { rangeCalc = 0.0; }
  rangedYVel = int(rangeCalc);
}
void rangeXAngle() {
  //this rectifies the input to a range of -180 to 180 deg
  if (double(absolute((xAngleOut))) > (maxAngle / 2.0)) { 
    if (xAngleOut < 0) {
      //xAngleOut = ((maxAngle / 2.0) * -1); 
    }
    else {
      xAngleOut = maxAngle / 2.0; 
    }
  }
  //this ranges the x angle PID output on a scale from -90 to 90
  double rangeCalc = (xAngleOut * 90.0) / (maxAngle / 2.0);
  if (absolute(rangeCalc) < 0.5) { rangeCalc = 0.0; }
  rangedXAngle = int(rangeCalc);
}
void rangeYAngle() {
  if (double(absolute((yAngleOut))) > (maxAngle / 2.0)) { 
    if (yAngleOut < 0) {
      //yAngleOut = ((maxAngle / 2.0) * -1); 
    }
    else {
      yAngleOut = maxAngle / 2.0; 
    }
  }
  //this ranges the y angle PID output on a scale from -90 to 90
  double rangeCalc = (yAngleOut * 90.0) / (maxAngle / 2.0);
  if (absolute(rangeCalc) < 0.5) { rangeCalc = 0.0; }
  rangedYAngle = int(rangeCalc);
}
void rangeZAngle() {
  //this rectifies the input to a range of -360 to 360 deg
  if (double(absolute((zAngleOut))) > (maxAngle * 2.0)) {
    if (zAngleOut < 0) {
      zAngleOut = (maxAngle * -2.0); 
    }
    else {
      zAngleOut = (maxAngle * 2.0); 
    }
  }
  //this ranges the z angle PID output on a scale from -40 to 40
  double rangeCalc = (zAngleOut * 40.0) / (maxAngle * 2.0);
  if (absolute(rangeCalc) < 0.25) { rangeCalc = 0.0; }
  rangedZAngle = int(rangeCalc);
}
void rangeValues() {
  rangeXVel();
  rangeYVel();
  rangeZVel();
  rangeXAngle();
  rangeYAngle();
  rangeZAngle();
}
void setup() {
  Serial.begin(9600);
  if (!IMU.begin()) {
    Serial.println("IMU not initialized!");
    while(1);
  }
  xAnglePID.SetMode(AUTOMATIC);
  xAnglePID.SetOutputLimits(-90,90);
  yAnglePID.SetMode(AUTOMATIC);
  yAnglePID.SetOutputLimits(-90,90);
  zAnglePID.SetMode(AUTOMATIC);
  zAnglePID.SetOutputLimits(-360,360);
  xVelPID.SetMode(AUTOMATIC);
  yVelPID.SetMode(AUTOMATIC);
  zVelPID.SetMode(AUTOMATIC);
}

void loop() {
  updateVel();
  updateAngle();
  updatePIDValues();
  computePID();
  //range the x,y,z velocities from -160 to 160, x and y angles from -90 to 90, and z angle from -40 to 40 PWM
  rangeValues();
  Serial.print(F("X Angle: "));
  Serial.print(xAngle);
  Serial.print(F("Y Angle: "));
  Serial.print(yAngle);
  Serial.print(F("Z Angle: "));
  Serial.println(zAngle);
}

//note- fix strafing, leveling correction, and z-axis rotation based on orientation of Arduino Nano on the drone

#include <Arduino_LSM6DSOX.h>
#include <PID_v1.h>

//pin setup code
#define voltageSig A1
#define currentSig A2
#define motor1 10 //motors on the drone are labeled similar to quadrants on a x-y plane
#define motor2 9
#define motor3 8
#define motor4 7

//basic control setup code
double sourceVoltage = 0.0;
double sCurrent = 0.0;
double maxVel = 5.0;
double maxAngle = 360.0;
float xAcc, yAcc, zAcc = 0.0;
double xAngle, yAngle, zAngle = 0.0;
float xAngleC, yAngleC, zAngleC = 0.0;
int rangedXVel, rangedYVel, rangedZVel, rangedXAngle, rangedYAngle, rangedZAngle = 0;
int motor1Set, motor2Set, motor3Set, motor4Set = 0;

//PID setup code
double xAngleOut, xAngleSet = 0.0; //note that xAngleSet will always be set to 0.0 to make sure it re-levels
double kp1 = 0.0;
double ki1 = 0.0;
double kd1 = 0.0;
//only used when x, y, or z velocity is set to 0.0
PID xAnglePID(&xAngle, &xAngleOut, &xAngleSet, kp1, ki1, kd1, DIRECT);
double yAngleOut, yAngleSet = 0.0; //note that yAngleSet will always be set to 0.0 to make sure it re-levels
double kp2 = 0.0;
double ki2 = 0.0;
double kd2 = 0.0;
//only used when x, y, or z velocity is set to 0.0
PID yAnglePID(&yAngle, &yAngleOut, &yAngleSet, kp2, ki2, kd2, DIRECT);
double zAngleOut, zAngleSet = 0.0;
double kp3 = 0.0;
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
double getVoltage() {
  double voltage = analogRead(voltageSig);
  voltage *= (60.0 / 1023.0);
  return voltage;
}
double getCurrent() {
  double current = analogRead(currentSig);
  current *= (264.0 / 1023.0);
  return current;
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
  xAngle += (double(xAngleC) * double(IMU.gyroscopeSampleRate()));
  yAngle += (double(yAngleC) * double(IMU.gyroscopeSampleRate()));
  zAngle += (double(zAngleC) * double(IMU.gyroscopeSampleRate()));
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
  if (setpoint > maxAngle) { setpoint = maxAngle; }
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
  if (double(abs((xVelOut))) > maxVel) { 
    if (xVelOut < 0) {
      xVelOut = (maxVel * -1); 
    }
    else {
      xVelOut = maxVel; 
    }
  }
  //this ranges the x velocity PID output on a scale from -rangedZVel to rangedZVel
  double rangeCalc = (xVelOut * double(rangedZVel)) / maxVel;
  if (abs(rangeCalc) < 5.0) { rangeCalc = 0.0; }
  rangedXVel = int(rangeCalc);
}
void rangeYVel() {
  if (double(abs((yVelOut))) > maxVel) { 
    if (yVelOut < 0) {
      yVelOut = (maxVel * -1); 
    }
    else {
      yVelOut = maxVel; 
    }
  }
  //this ranges the y velocity PID output on a scale from -rangedZVel to rangedZVel
  double rangeCalc = (yVelOut * double(rangedZVel)) / maxVel;
  if (abs(rangeCalc) < 5.0) { rangeCalc = 0.0; }
  rangedYVel = int(rangeCalc);
}
void rangeXAngle() {
  //this rectifies the input to a range of -180 to 180 deg
  if (double(abs((xAngleOut))) > (maxAngle / 2.0)) { 
    if (xAngleOut < 0) {
      xAngleOut = ((maxAngle / 2.0) * -1); 
    }
    else {
      xAngleOut = maxAngle / 2.0; 
    }
  }
  //this ranges the x angle PID output on a scale from -90 to 90
  double rangeCalc = (xAngleOut * 90.0) / (maxAngle / 2.0);
  if (abs(rangeCalc) < 2.0) { rangeCalc = 0.0; }
  rangedXAngle = int(rangeCalc);
}
void rangeYAngle() {
  if (double(abs((yAngleOut))) > (maxAngle / 2.0)) { 
    if (yAngleOut < 0) {
      yAngleOut = ((maxAngle / 2.0) * -1); 
    }
    else {
      yAngleOut = maxAngle / 2.0; 
    }
  }
  //this ranges the x angle PID output on a scale from -90 to 90
  double rangeCalc = (yAngleOut * 90.0) / (maxAngle / 2.0);
  if (abs(rangeCalc) < 2.0) { rangeCalc = 0.0; }
  rangedYAngle = int(rangeCalc);
}
void rangeZAngle() {
  //this rectifies the input to a range of -360 to 360 deg
  if (double(abs((zAngleOut))) > maxAngle) {
    if (zAngleOut < 0) {
      zAngleOut = (maxAngle * -1); 
    }
    else {
      zAngleOut = maxAngle; 
    }
  }
  //this ranges the z angle PID output on a scale from -90 to 90
  double rangeCalc = (zAngleOut * 40.0) / maxAngle;
  if (abs(rangeCalc) < 1.0) { rangeCalc = 0.0; }
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
void initialMotorSets() {
  double motor1CalcX = (double(rangedXVel) / 4.0);
  //3/4 of the ranged velocity to make it strafe forwards/ backwards
  if (rangedXVel < 0) { motor1CalcX *= -3; }
  //3/4 of the ranged velocity to make it strafe left/ right
  double motor1CalcY = (double(rangedYVel) / 4.0);
  if (rangedYVel > 0) { motor1CalcY *= 3; }
  //add in left/ right (y axis) strafing if both at same time- chooses the greater of the two values
  motor1Set = int((motor1CalcX == motor1CalcY) ? motor1CalcX : (motor1CalcX > motor1CalcY) ? motor1CalcX : motor1CalcY);
  
  double motor2CalcX = (double(rangedXVel) / 4.0);
  if (rangedXVel < 0) { motor2CalcX *= -3; }
  double motor2CalcY = (double(rangedYVel) / 4.0);
  if (rangedYVel < 0) { motor2CalcY *= -3; }
  motor2Set = int((motor2CalcX == motor2CalcY) ? motor2CalcX : (motor2CalcX > motor2CalcY) ? motor2CalcX : motor2CalcY);
  
  double motor3CalcX = (double(rangedXVel) / 4.0);
  if (rangedXVel > 0) { motor3CalcX *= 3; }
  double motor3CalcY = (double(rangedYVel) / 4.0);
  if (rangedYVel < 0) { motor3CalcY *= -3; }
  motor3Set = int((motor3CalcX == motor3CalcY) ? motor3CalcX : (motor3CalcX > motor3CalcY) ? motor3CalcX : motor3CalcY);
  
  double motor4CalcX = (double(rangedXVel) / 4.0);
  if (rangedXVel > 0) { motor4CalcX *= 3; }
  double motor4CalcY = (double(rangedYVel) / 4.0);
  if (rangedYVel > 0) { motor4CalcY *= 3; }
  motor4Set = int((motor4CalcX == motor4CalcY) ? motor4CalcX : (motor4CalcX > motor4CalcY) ? motor4CalcX : motor4CalcY);
}

void setup() {
  pinMode(voltageSig, INPUT);
  pinMode(currentSig, INPUT);
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motor3, OUTPUT);
  pinMode(motor4, OUTPUT);
  Serial.begin(9600);
  if (!IMU.begin()) {
    Serial.println("IMU not initialized!");
    while(1);
  }
  xAnglePID.SetMode(AUTOMATIC);
  yAnglePID.SetMode(AUTOMATIC);
  zAnglePID.SetMode(AUTOMATIC);
  xVelPID.SetMode(AUTOMATIC);
  yVelPID.SetMode(AUTOMATIC);
  zVelPID.SetMode(AUTOMATIC);
}

void loop() {
  sourceVoltage = getVoltage();
  sCurrent = getCurrent();
  Serial.print(F("Voltage: "));
  Serial.print(sourceVoltage);
  Serial.print(F("V"));
  Serial.print(F("\tCurrent: "));
  Serial.print(sCurrent);
  Serial.println(F("A"));
  updateVel();
  updateAngle();
  updatePIDValues();
  computePID();
  if (sourceVoltage < 19.1) {  //19.0V safe cutoff
      Serial.println("Voltage at or below 19.0 volts, powering down.");
  }
  else if (sCurrent > 176.0) { //180A final cutoff
      Serial.println("Current at or above 176 amps, powering down.");
  }
  else {
      if (sCurrent > 139.0) { //140A safe continuous cutoff
          Serial.println("Current at or above 139 amps, not safe to continue.");
      } //otherwise, continue with normal operation
      
      //range the x,y,z velocities from -160 to 160, x and y angles from -90 to 90, and z angle from -40 to 40 PWM
      rangeValues();
      //intially set the motor PWMs for strafing (with a max of 120 PWM b/c of 3/4 calc)
      initialMotorSets();
      if (((xVel < 0.2) && (xVel > 0)) || ((xVel > -0.2) && (xVel < 0))) { //level x-axis if x velocity is less than 0.2 b/c oriented sideways- with USB outlet facing the right side
        if (rangedXAngle < 0) { //raise up the front if the back is too high
          motor1Set -= rangedXAngle;
          motor2Set -= rangedXAngle;
        }
        else { //raise up the back if the front is too high
          motor1Set += rangedXAngle;
          motor2Set += rangedXAngle;
        }
      }
      if (((yVel < 0.2) && (yVel > 0)) || ((yVel > -0.2) && (yVel < 0))) { //level y-axis if y velocity is less than 0.2 b/c oriented sideways
        if (rangedYAngle > 0) { //raise up the right if the left is too high
          motor1Set += rangedYAngle;
          motor4Set += rangedYAngle;
        }
        else { //raise up the left if the right is too high
          motor2Set -= rangedYAngle;
          motor3Set -= rangedYAngle;
        }
      }
      if (rangedZAngle > 0) { //turn right for positive z angle error
        motor2Set += rangedZAngle;
        motor4Set += rangedZAngle;
      }
      else { //turn left for negative z angle error
        motor1Set -= rangedZAngle;
        motor3Set -= rangedZAngle;
      }
      analogWrite(motor1, motor1Set);
      analogWrite(motor2, motor2Set);
      analogWrite(motor3, motor3Set);
      analogWrite(motor4, motor4Set);
  }
}

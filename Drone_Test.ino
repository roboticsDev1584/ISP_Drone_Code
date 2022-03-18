/*  NOTES
X Axis Angle: more positive when rotated clockwise, more negative when rotated counterclockwise
Y Axis Angle: more positive when rotated clockwise, more negative when rotated counterclockwise
Z Axis Angle: more positive when rotated counterclockwise, more negative when rotated clockwise

X Axis Velocity: more positive when moved left (with x axis vector), more negative when moved right (against x axis vector)
Y Axis Velocity: more positive when moved back (with y axis vector), more negative when moved forwards (against y axis vector)
Z Axis Velocity: more positive when moved up, more negative when moved down

Z Velocity is the same thing as thrust

The code below allows the drone to strafe, turn, and stop abruptly (re-level the axis used after strafing)
*/

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
double maxVel = 10.0;
double maxAngle = 180.0;
float xAcc = 0.0, yAcc = 0.0, zAcc = 0.0;
float xAccOld = 0.0, yAccOld = 0.0, zAccOld = 1.0;
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
double kp4 = 1.0;
double ki4 = 0.0;
double kd4 = 0.0;
PID xVelPID(&xVel, &xVelOut, &xVelSet, kp4, ki4, kd4, DIRECT);
double yVel, yVelOut, yVelSet = 0.0;
double kp5 = 1.0;
double ki5 = 0.0;
double kd5 = 0.0;
PID yVelPID(&yVel, &yVelOut, &yVelSet, kp5, ki5, kd5, DIRECT);
double zVel, zVelOut, zVelSet = 0.0;
double kp6 = 1.0;
double ki6 = 0.0;
double kd6 = 0.0;
PID zVelPID(&zVel, &zVelOut, &zVelSet, kp6, ki6, kd6, DIRECT);

//basic monitoring functions
double absolute(double num) {
  if (num < 0) { num *= -1.0; }
  return num;  
}
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
void updateVel() {
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(xAcc, yAcc, zAcc);
  }
  if (((xAcc < 0.02) && (xAcc > 0)) || ((xAcc > -0.02) && (xAcc < 0))) { xAcc = 0.0; }
  if (((yAcc < 0.02) && (yAcc > 0)) || ((yAcc > -0.02) && (yAcc < 0))) { yAcc = 0.0; }
  //xAcc - xAccOld
  //we want the change in acceleration value divided by 9.8 since acceleration is measured in g's
  xVel += (((double(xAcc) - double(xAccOld)) * double(IMU.accelerationSampleRate())) / 9.8);
  yVel += (((double(yAcc) - double(yAccOld)) * double(IMU.accelerationSampleRate())) / 9.8);
  zVel += (((double(zAcc) - double(zAccOld)) * double(IMU.accelerationSampleRate())) / 9.8);
  xAccOld = xAcc;
  yAccOld = yAcc;
  zAccOld = zAcc;
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
void updateZVelSetpoint() {
  double setpoint = 0.0;
  //the drone can be set to a max velocity of 10.0
  if (absolute(setpoint) > maxVel) { 
    if (setpoint < 0) {
      setpoint = (maxVel * -1.0); 
    }
    else {
      setpoint = maxVel; 
    }
  }
  else if (absolute(setpoint) < 0.1) { setpoint = 0.0; }
  zVelSet = setpoint;
}
void updateXVelSetpoint() {
  double setpoint = 0.0;
  if (absolute(setpoint) > zVelSet) { 
    if (setpoint < 0) {
      setpoint = (zVelSet * -1.0); 
    }
    else {
      setpoint = zVelSet; 
    }
  }
  else if (absolute(setpoint) < 0.1) { setpoint = 0.0; }
  xVelSet = setpoint;
}
void updateYVelSetpoint() {
  double setpoint = 0.0;
  if (absolute(setpoint) > zVelSet) { 
    if (setpoint < 0) {
      setpoint = (zVelSet * -1.0); 
    }
    else {
      setpoint = zVelSet; 
    }
  }
  else if (absolute(setpoint) < 0.1) { setpoint = 0.0; }
  yVelSet = setpoint;
}
void updateZAngleSetpoint() {
  double setpoint = 0.0;
  if (setpoint > (maxAngle * 2.0)) { setpoint = (maxAngle * 2.0); }
  else if (setpoint < 1.0) { setpoint = 0.0; }
  zAngleSet = setpoint;
}
void updatePIDValues() {
  updateZVelSetpoint();
  updateXVelSetpoint();
  updateYVelSetpoint();
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
  if (double(absolute((zVelOut))) > maxVel) {
    if (zVelOut < 0) {
      zVelOut = (maxVel * -1); 
    }
    else {
      zVelOut = maxVel; 
    }
  }
  //this ranges the z velocity PID output on a scale from 0 to 160 b/c this is essentially thrust
  double rangeCalc = (zVelOut * 160.0) / maxVel;
  if (rangeCalc < 5.0) { rangeCalc = 0.0; }
  rangedZVel = int(rangeCalc);
}
void rangeXVel() {
  //the PID can adjust the drone to have a max correction up to a velocity of 10.0
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
  rangeZVel();
  rangeXVel();
  rangeYVel();
  rangeXAngle();
  rangeYAngle();
  rangeZAngle();
}
void initialMotorSets() {
  //using the rangedYVel for the x velocity calculation because rangedYVel is of the Nano, not the overall drone
  double motor1CalcX = (double(rangedYVel) / 4.0);
  //3/4 of the ranged velocity to make it strafe forwards/ backwards
  if (rangedYVel < 0) { motor1CalcX *= 3.0; }
  //3/4 of the ranged velocity to make it strafe left/ right
  double motor1CalcY = (double(rangedXVel) / 4.0);
  if (rangedXVel > 0) { motor1CalcY *= 3.0; }
  //add in left/ right (y axis) strafing if both at same time- chooses the greater of the two values
  motor1Set = int((motor1CalcX == motor1CalcY) ? motor1CalcX : (motor1CalcX > motor1CalcY) ? motor1CalcX : motor1CalcY);
  
  double motor2CalcX = (double(rangedYVel) / 4.0);
  if (rangedYVel < 0) { motor2CalcX *= 3.0; }
  double motor2CalcY = (double(rangedXVel) / 4.0);
  if (rangedXVel < 0) { motor2CalcY *= 3.0; }
  motor2Set = int((motor2CalcX == motor2CalcY) ? motor2CalcX : (motor2CalcX > motor2CalcY) ? motor2CalcX : motor2CalcY);
  
  double motor3CalcX = (double(rangedYVel) / 4.0);
  if (rangedYVel > 0) { motor3CalcX *= 3.0; }
  double motor3CalcY = (double(rangedXVel) / 4.0);
  if (rangedXVel < 0) { motor3CalcY *= 3.0; }
  motor3Set = int((motor3CalcX == motor3CalcY) ? motor3CalcX : (motor3CalcX > motor3CalcY) ? motor3CalcX : motor3CalcY);
  
  double motor4CalcX = (double(rangedYVel) / 4.0);
  if (rangedYVel > 0) { motor4CalcX *= 3.0; }
  double motor4CalcY = (double(rangedXVel) / 4.0);
  if (rangedXVel > 0) { motor4CalcY *= 3.0; }
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
  xAnglePID.SetOutputLimits(-90,90);
  yAnglePID.SetMode(AUTOMATIC);
  yAnglePID.SetOutputLimits(-90,90);
  zAnglePID.SetMode(AUTOMATIC);
  zAnglePID.SetOutputLimits(-360,360);
  xVelPID.SetMode(AUTOMATIC);
  xVelPID.SetOutputLimits(-maxVel,maxVel);
  yVelPID.SetMode(AUTOMATIC);
  yVelPID.SetOutputLimits(-maxVel,maxVel);
  zVelPID.SetMode(AUTOMATIC);
  zVelPID.SetOutputLimits(-maxVel,maxVel);
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
      if (((yVel < 0.2) && (yVel > 0)) || ((yVel > -0.2) && (yVel < 0))) { //level Nano y-axis if |x drone velocity| is less than 0.2 b/c oriented sideways- with USB outlet facing the right side
        if (rangedXAngle > 0) { //raise up the front if the back is too high
          motor1Set += rangedXAngle;
          motor2Set += rangedXAngle;
        }
        else { //raise up the back if the front is too high
          motor3Set += -rangedXAngle;
          motor4Set += -rangedXAngle;
        }
      }
      if (((xVel < 0.2) && (xVel > 0)) || ((xVel > -0.2) && (xVel < 0))) { //level Nano x-axis if |y drone velocity| is less than 0.2 b/c oriented sideways
        if (rangedYAngle < 0) { //raise up the right if the left is too high
          motor1Set += -rangedYAngle;
          motor4Set += -rangedYAngle;
        }
        else { //raise up the left if the right is too high
          motor2Set += rangedYAngle;
          motor3Set += rangedYAngle;
        }
      }
      if (rangedZAngle > 0) { //turn left for positive z angle error
        motor1Set += rangedZAngle;
        motor3Set += rangedZAngle;
      }
      else { //turn right for negative z angle error
        motor2Set += -rangedZAngle;
        motor4Set += -rangedZAngle;
      }
      analogWrite(motor1, motor1Set);
      analogWrite(motor2, motor2Set);
      analogWrite(motor3, motor3Set);
      analogWrite(motor4, motor4Set);
  }
}

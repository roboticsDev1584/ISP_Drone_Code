#include <Arduino_LSM6DSOX.h>
#include <PID_v1.h>

//pin setup code
#define voltageSig A1
#define currentSig A2
#define motor1 10
#define motor2 9
#define motor3 8
#define motor4 7

//basic control setup code
double sourceVoltage = 0.0;
double sCurrent = 0.0;
float xAcc, yAcc, zAcc = 0.0;
float xAngle, yAngle, zAngle = 0.0;
int motor1Set, motor2Set, motor3Set, motor4Set = 0;

//PID setup code
double xAngleOut, xAngleSet = 0.0; //note that xAngleSet will always be set to 0.0 to make sure it re-levels
double kp1 = 0.0;
double ki1 = 0.0;
double kd1 = 0.0;
//only used when x, y, or z velocity is set to 0.0
PID xAnglePID(double(&xAngle), &xAngleOut, &xAngleSet, kp1, ki1, kd1, DIRECT);
double yAngleOut, yAngleSet = 0.0; //note that yAngleSet will always be set to 0.0 to make sure it re-levels
double kp2 = 0.0;
double ki2 = 0.0;
double kd2 = 0.0;
//only used when x, y, or z velocity is set to 0.0
PID yAnglePID(double(&yAngle), &yAngleOut, &yAngleSet, kp2, ki2, kd2, DIRECT);
double zAngleOut, zAngleSet = 0.0;
double kp3 = 0.0;
double ki3 = 0.0;
double kd3 = 0.0;
PID zAnglePID(double(&zAngle), &zAngleOut, &zAngleSet, kp3, ki3, kd3, DIRECT);
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
    IMU.readGyroscope(xAngle, yAngle, zAngle);  
  }
}

//setpoint value functions
void updateXVelSetpoint() {
  double setpoint = 0.0;
  xVelSet = setpoint;
}
void updateYVelSetpoint() {
  double setpoint = 0.0;
  yVelSet = setpoint;
}
void updateZVelSetpoint() {
  double setpoint = 0.0;
  zVelSet = setpoint;
}
void updateZAngleSetpoint() {
  double setpoint = 0.0;
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
      }
      //operate normally
      if (xVelSet == 0.0) { //level y-axis if x velocity is 0.0
        //need to test out the range of angles on the IMU first
        
      }
      //x-axis leveling code here
      //need to write code to set the motor power so that it can turn using z axis angle
      //need to write code to set the motor power so that it can strafe and move up/ down
      analogWrite(motor1, motor1Set);
      analogWrite(motor2, motor2Set);
      analogWrite(motor3, motor3Set);
      analogWrite(motor4, motor4Set);
  }
}

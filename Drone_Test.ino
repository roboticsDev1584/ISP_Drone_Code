#include <PID_v1.h>

#define voltageSig A1
#define currentSig A2
double sourceVoltage = 0.0;
double sCurrent = 0.0;

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

//setpoint value functions
int getMotor1Setpoint() {
  int setpoint = 0;
  return setpoint;
}

//PID setup code
double motorIn1, motorOut1, motorSet1 = 0.0;
double kp1, ki1, kd1 = 0.0;
PID motor1PID(&motorIn1, &motorOut1, &motorSet1, kp1, ki1, kd1, DIRECT);

void setup() {
  pinMode(voltageSig, INPUT);
  pinMode(currentSig, INPUT);
  Serial.begin(9600);
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
      
  }
}

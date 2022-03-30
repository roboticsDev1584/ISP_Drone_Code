int output = 14;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(output,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(output,150);
}

#define PRESSURE_PIN  A0 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(analogRead(PRESSURE_PIN ));
  delay(500);
}

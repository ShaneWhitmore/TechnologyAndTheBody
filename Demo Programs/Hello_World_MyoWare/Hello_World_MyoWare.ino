int pinENV = 35;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int sensorValue = analogRead(pinENV);

  float voltage = sensorValue * (3.3 / 4095.0);

  Serial.println("Signal: ");
  Serial.println(sensorValue);



  //Serial.println("Voltage: ");
  //Serial.println(voltage);

  delay(100);
}

#include "ESP32_NOW.h"
#include "WiFi.h"
#include <ESP32Servo.h>


Servo pointerServo;
Servo middleServo;
Servo ringServo;
Servo pinkyServo;

int pointerPin = 13;
int middlePin = 18;
int ringPin = 17;
int pinkyPin = 16;



//Variables
uint16_t receivedValue;


// Callback when data is received
void onDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *data, int dataLen) {
  memcpy(&receivedValue, data, sizeof(receivedValue));
  Serial.print("Muscle State: ");
  Serial.println(receivedValue);
}

void closeHand()
{
  middleServo.write(70);
  delay(400);
  middleServo.write(90);
}


void openHand()
{
  middleServo.write(100);
  delay(400);
  middleServo.write(90);
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  WiFi.mode(WIFI_STA);


  esp_now_register_recv_cb(onDataRecv);

  pointerServo.attach(pointerPin);
  middleServo.attach(middlePin);
  ringServo.attach(ringPin);
  pinkyServo.attach(pinkyPin);
  



}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100);

  //recieve muscle state
  // check current state (false = open hand , true = closed hand)
  // if receivedState && !currentState
  // close over the hand -> make this a function
  // else if !receivedState && currentState 
  // open the hand -> make this a function


}

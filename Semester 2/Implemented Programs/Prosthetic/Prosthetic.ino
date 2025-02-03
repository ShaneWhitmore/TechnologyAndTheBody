#include "ESP32_NOW.h"
#include "WiFi.h"
#include <ESP32Servo.h>


Servo pointerServo;
Servo middleServo;
Servo ringServo;
Servo pinkyServo;

int pointerPin = 14;
int middlePin = 13;
int ringPin = 18;
int pinkyPin = 16;

// available pins for servos 
/*
ADC1_CH1 (GPIO 37)
ADC1_CH2 (GPIO 38)
ADC1_CH3 (GPIO 39)
ADC1_CH4 (GPIO 32)
ADC1_CH5 (GPIO 33)
ADC1_CH6 (GPIO 34)
ADC1_CH7 (GPIO 35)

GPIO 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27

*/


typedef struct struct_message {
  bool t;
  uint16_t d; 
} struct_message;

struct_message receivedData;


//Variables
uint16_t receivedValue;
int currentState = 0;


// Callback when data is received
void onDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *data, int dataLen) {
  memcpy(&receivedData, data, sizeof(receivedData));
  Serial.print("Muscle State: ");
  Serial.println(receivedData.d);
  Serial.println(receivedData.t);

  if(receivedData.t == 1)
  {
    Serial.println("Closing Hand");
    closeHand();
  }
  else
  {
    Serial.println("Opening Hand");
    openHand();
  }

}

void closeHand()
{
  if(receivedData.t != currentState)
  {
    Serial.println("YAYYYYYY");

    middleServo.write(50);
    delay(400);
    middleServo.write(90);
    currentState = receivedData.t;
  }  
}


void openHand()
{
  if(receivedData.t != currentState)
  {

    Serial.println("YAYYYYYY2222222");

    middleServo.write(160);
    delay(400);
    middleServo.write(90);
    currentState = receivedData.t;
  }
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  WiFi.mode(WIFI_STA);

  
  while (!WiFi.STA.started()) {
    delay(100);
  }


  if (esp_now_init() != ESP_OK)
  {
    Serial.print("ESP NOW Initialisation Error");
    return;
  }


  esp_now_register_recv_cb(onDataRecv);

  pointerServo.attach(pointerPin);
  middleServo.attach(middlePin);
  ringServo.attach(ringPin);
  pinkyServo.attach(pinkyPin);
  



}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);

  //recieve muscle state
  // check current state (false = open hand , true = closed hand)
  // if receivedState && !currentState
  // close over the hand -> make this a function
  // else if !receivedState && currentState 
  // open the hand -> make this a function


}

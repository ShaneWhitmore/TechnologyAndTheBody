#include "ESP32_NOW.h"
#include "WiFi.h"


//Global variables
uint16_t sensorValue;
int pinENV = 33;

//Prosthetic mac address
uint8_t broadcastAddress[] = {0x88, 0x13, 0xBF , 0x68 , 0xB2 , 0x8C}; //88:13:BF:68:B2:8C


//message structure
typedef struct struct_message {
    int d;
} struct_message;

struct_message myData;


void OnDataSent(const uint8_t *mac_addr , esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


void setup() {
  Serial.begin(9600);
  pinMode(pinENV, INPUT);

  WiFi.mode(WIFI_STA);
  while (!WiFi.STA.started()) {
    delay(100);
  }

  esp_now_register_send_cb(OnDataSent);


  //Register receiver mac address
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

}

void loop() {
  // put your main code here, to run repeatedly:

  //Read from sensor
  int sensorValue = analogRead(pinENV);
  myData.d = sensorValue;

  //broadcast sensor value
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

  if (result == ESP_OK) {
    Serial.println("Message sent successfully");
  } else {
    Serial.println("Error sending message");
  }

  delay(100);

  //interperate sensor data





  //broadcast muscle stimulation state 


}

#include "ESP32_NOW.h"
#include "WiFi.h"
//devices mac address = 24:0a:c4:2e:57:64

//Global variables
uint16_t sensorValue;
int pinENV = 33;
int maxVal = 4095;
bool muscleState = false;

//Prosthetic mac address
uint8_t broadcastAddress[] = { 0x88, 0x13, 0xBF, 0x68, 0xB2, 0x8C };  //88:13:BF:68:B2:8C



//message structure
typedef struct struct_message {
  bool t;
  uint16_t d;
} struct_message;

struct_message myData;


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
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

  if (esp_now_init() != ESP_OK) {
    Serial.print("Initialising ESP NOW");
    return;
  }


  esp_now_register_send_cb(OnDataSent);


  //Register receiver mac address
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  // put your main code here, to run repeatedly:

  //Read from sensor
  int sensorValue = analogRead(pinENV);
  Serial.println(sensorValue);
  // Received from: 24:0A:C4:2E:57:64 | Muscle State: 4095 , maximum value can be 4095






  //interperate sensor data


  //maxVal = 4095
  float upperLimit = maxVal * 0.60;
  float lowerLimit = maxVal * 0.25;

  if (sensorValue > upperLimit) {
    muscleState = true;
  } else if (sensorValue < lowerLimit) {
    muscleState = false;
  }

  myData.t = muscleState;
  myData.d = sensorValue;




  //broadcast muscle stimulation state
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

  if (result == ESP_OK) {
    Serial.println("Message sent successfully");
  } else {
    Serial.println("Error sending message");
  }

  delay(1000);
}

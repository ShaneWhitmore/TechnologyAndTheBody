#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>
#include "config.h" //configuration file with credentials


//devices mac address = 24:0a:c4:2e:57:64


// Definitions
//#define ESPNOW_WIFI_CHANNEL 6


//Global variables
uint16_t sensorValue;
int pinENV = 33;
int maxVal = 4095;
bool muscleState = false;

//Prosthetic mac address
//uint8_t broadcastAddress[] = { 0x88, 0x13, 0xBF, 0x6A, 0x0A, 0x6C };  //88:13:bf:6a:0a:6c


/* Classes */

// Creating a new class that inherits from the ESP_NOW_Peer class is required.

class ESP_NOW_Broadcast_Peer : public ESP_NOW_Peer {
public:
  // Constructor of the class using the broadcast address
  ESP_NOW_Broadcast_Peer(uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) : ESP_NOW_Peer(ESP_NOW.BROADCAST_ADDR, channel, iface, lmk) {}

  // Destructor of the class
  ~ESP_NOW_Broadcast_Peer() {
    remove();
  }

  // Function to properly initialize the ESP-NOW and register the broadcast peer
  bool begin() {
    if (!ESP_NOW.begin() || !add()) {
      log_e("Failed to initialize ESP-NOW or register the broadcast peer");
      return false;
    }
    return true;
  }

  // Function to send a message to all devices within the network
  bool send_message(const uint8_t *data, size_t len) {
    if (!send(data, len)) {
      log_e("Failed to broadcast message");
      return false;
    }
    Serial.println("Message broadcasted");
    return true;
  }
};

// Create a broadcast peer object
ESP_NOW_Broadcast_Peer broadcast_peer(WiFi.channel(), WIFI_IF_STA, NULL);



//message structure
typedef struct struct_message {
  bool t;
} struct_message;

struct_message myData;





//void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//  Serial.print("\r\nLast Packet Send Status:\t");
//  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
//}


void setup() {
  Serial.begin(9600);
  pinMode(pinENV, INPUT);

  WiFi.mode(WIFI_STA);
  while (!WiFi.STA.started()) {
    delay(100);
  }


  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  randomSeed(micros());
  Serial.println("\nWiFi connected\nIP address: ");
  Serial.println(WiFi.localIP());

  // Register the broadcast peer
  if (!broadcast_peer.begin()) {
    Serial.println("Failed to initialize broadcast peer");
    Serial.println("Reebooting in 5 seconds..."); 
    delay(5000);
    ESP.restart();
  }

  /*
  if (esp_now_init() != ESP_OK) {
    Serial.print("Initialising ESP NOW");
    return;
  }


  esp_now_register_send_cb(OnDataSent);
 

  //Register receiver mac address
  esp_now_peer_info_t peerInfo;
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  */
}

void loop() {
  // put your main code here, to run repeatedly:

  //Read from sensor
  int sensorValue = analogRead(pinENV);
  Serial.println(sensorValue);

  //interperate sensor data


  //maxVal = 4095
  float upperLimit = maxVal * 0.55;
  float lowerLimit = maxVal * 0.25;

  if (sensorValue > upperLimit) {
    muscleState = true;
  } else if (sensorValue < lowerLimit) {
    muscleState = false;
  }

  myData.t = muscleState;
  

    // Received from: 24:0A:C4:2E:57:64 | Muscle State: 4095 , maximum value can be 4095


  Serial.println("muscle state added to myData");

  // Broadcast data 
  if (!broadcast_peer.send_message((uint8_t *)&myData, sizeof(myData))) {
    Serial.println("Failed to broadcast message");
  }

  /*
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

  if (result == ESP_OK) {
    Serial.println("Message sent successfully");
  } else {
    Serial.println("Error sending message");
  }
  */

  delay(1250);
}
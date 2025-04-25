#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>
#include "config.h" //configuration file with credentials


//Global variables
uint16_t sensorValue;
int pinENV = 33;
int maxVal = 4095;
bool muscleState = false;



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



void setup() {
  Serial.begin(9600);
  pinMode(pinENV, INPUT);

  // configuring wifi
  WiFi.mode(WIFI_STA);
  while (!WiFi.STA.started()) {
    delay(100);
  }

  WiFi.begin(ssid, password); //connecting to wifi network

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  } //waiting for connection to wifi network

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
}

void loop() {
  // put your main code here, to run repeatedly:

  //Read from sensor
  int sensorValue = analogRead(pinENV);
  Serial.println(sensorValue);

  //interperate sensor data


  //maxVal = 4095
  float upperLimit = maxVal * 0.50;
  float lowerLimit = maxVal * 0.20;

  // binary classification of sensor value
  if (sensorValue > upperLimit) {
    muscleState = true;
  } else if (sensorValue < lowerLimit) {
    muscleState = false;
  }

  myData.t = muscleState; //setting the muscle state in structured message


  Serial.println("muscle state added to myData");

  // Broadcast data 
  if (!broadcast_peer.send_message((uint8_t *)&myData, sizeof(myData))) {
    Serial.println("Failed to broadcast message");
  }

  delay(1250); // 1.25 second delay
}
#include "ESP32_NOW.h"
#include "WiFi.h"
#include <ESP32Servo.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include "config.h" //configuration file with credentials


//https://community.hivemq.com/t/hivemq-using-esp32-and-nodered/1291  == MQTT skeleton code


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

//MQTT
WiFiClientSecure espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;

#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];


typedef struct struct_message {
  bool t;
  uint16_t d; 
} struct_message;

struct_message receivedData;


//Variables
uint16_t receivedValue;
int currentState = 0;
String gesture = "default";




void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  randomSeed(micros());
  Serial.println("\nWiFi connected\nIP address: ");
  Serial.println(WiFi.localIP());

  while (!Serial) delay(1);

  //Connect to MQTT Broker (credentials stored in config.h file for security)
  espClient.setCACert(root_ca);
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);


  //Begin ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.print("ESP NOW Initialisation Error");
    ESP.restart();
  }


  esp_now_register_recv_cb(onDataRecv);

  //connecting Servos to pins
  pointerServo.attach(pointerPin);
  middleServo.attach(middlePin);
  ringServo.attach(ringPin);
  pinkyServo.attach(pinkyPin);
  



}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();
}


//Function to reconnect to MQTT should connection loss occur
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection…");
    String clientId = "ESP32Client-"; 
    clientId += String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("connected");
    client.subscribe(topic);   
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// Callback when data is received from MQTT
void callback(char* topic, byte* payload, unsigned int length) {
  String incomingMessage = "";
  String payloadValue = "";



  for (int i = 0; i < length; i++)
  {
    incomingMessage+=(char)payload[i];
  } 


  // Payload manipulation here
  int i = 0;
  while(incomingMessage.charAt(i) != ',')
  {
    payloadValue += incomingMessage.charAt(i);
    i++;
  }
  int payloadBeginning = incomingMessage.indexOf(":") + 1;
  payloadValue = payloadValue.substring(payloadBeginning);
  gesture = payloadValue;
  Serial.println(gesture);

}


// Callback when data is received from ESP-NOW 
void onDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *data, int dataLen) {
  memcpy(&receivedData, data, sizeof(receivedData));
  Serial.print("Muscle State: ");
  Serial.println(receivedData.d);
  Serial.println(receivedData.t);


  //re design this code to accomodate for gestures
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
    middleServo.write(160);
    delay(400);
    middleServo.write(90);
    currentState = receivedData.t;
  }
}


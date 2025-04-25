#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>  // For the MAC2STR and MACSTR macros
#include <vector>

#include <ESP32Servo.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <map>
#include "config.h" //configuration file with credentials


//https://community.hivemq.com/t/hivemq-using-esp32-and-nodered/1291  == MQTT skeleton code




Servo pointerServo;
Servo middleServo;
Servo ringServo;
Servo pinkyServo;

int pointerPin = 26;
int middlePin = 4;
int ringPin = 27;
int pinkyPin = 13;



//Variables
int currentState = 0;


struct Gestures{
  int values[4][2]; //a 2D array, [4] is for each servo and [2] is for "open"/"close" where (x,0) = close speed for servo & (x,1) = open speed for servo

};

// https://www.geeksforgeeks.org/map-associative-containers-the-c-standard-template-library-stl/

// { key , { { (pinky Close , pinky Open) , (ring Close, ring Open) , (middle Close , middle Open) , (pointer Close , pointer Open} } }
std::map<std::string, Gestures> GestureMap = {
  	{"default", { {{30, 130}, {30, 130}, {30, 130}, {30, 130}}}},
    {"point", { {{30, 130}, {30, 130}, {30, 130}, {90, 90}}}},
    {"peace", { {{30, 130}, {30, 130}, {90, 90}, {90, 90}}}},
    {"rock", { {{90, 90}, {30, 130}, {30, 130}, {90, 90}}}} 
};


//Structured Messages

  //Structured message for muscle stimulation data
typedef struct struct_message {
  bool t;
} struct_message;

struct_message receivedData;


  //Structured message for gesture
typedef struct struct_gesture {
  String g;
} struct_gesture;

struct_gesture gesture;




void moveHand(int receivedState) {
  if(receivedState != currentState) //check that received state is different from the currents state
  {
    int pinky = 90 , ring = 90 , middle = 90 , pointer = 90;

    if(receivedState == 1)  // muscle is stimulated (representing close hand)
    {                       // set variables to close values from the gesture map

      pinky = GestureMap[gesture.g.c_str()].values[0][0];
      ring = GestureMap[gesture.g.c_str()].values[1][0];
      middle = GestureMap[gesture.g.c_str()].values[2][0];
      pointer = GestureMap[gesture.g.c_str()].values[3][0];
    }
    else //otherwise muscle is not stimulated (representing an open/relax hand)
    {     //set variables to open values from the gesture map

      pinky = GestureMap[gesture.g.c_str()].values[0][1];
      ring = GestureMap[gesture.g.c_str()].values[1][1];
      middle = GestureMap[gesture.g.c_str()].values[2][1];
      pointer = GestureMap[gesture.g.c_str()].values[3][1];
    }

    //write these variables to the servos to move the hand

    pinkyServo.write(pinky);
    ringServo.write(ring);
    middleServo.write(middle);
    pointerServo.write(pointer);
    delay(675);   //time delay to allow for servos to fully close over
    pinkyServo.write(90); //STOP MOVING SERVOS
    ringServo.write(90);
    middleServo.write(90);
    pointerServo.write(90);
    currentState = receivedState; //set the received value as the current state

  }
}


/* Classes */

// Creating a new class that inherits from the ESP_NOW_Peer class is required.

class ESP_NOW_Peer_Class : public ESP_NOW_Peer {
public:
  // Constructor of the class
  ESP_NOW_Peer_Class(const uint8_t *mac_addr, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) : ESP_NOW_Peer(mac_addr, channel, iface, lmk) {}

  // Destructor of the class
  ~ESP_NOW_Peer_Class() {}

  // Function to register the master peer
  bool add_peer() {
    if (!add()) {
      log_e("Failed to register the broadcast peer");
      return false;
    }
    return true;
  }

  // Function to print the received messages from the master
  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    if(len == sizeof(receivedData))
    {
      memcpy(&receivedData, data, sizeof(receivedData)); //copy data into received data structured message
      Serial.print("Muscle State: ");
      Serial.printf("  Muscle State: %s\n", receivedData.t ? "TRUE" : "FALSE"); //print the receivedData value

      moveHand(receivedData.t); //call on moveHand function and passing receivedData
    }
    else
    {
      Serial.println("Incorrect message size");
    }
    
  }
};

std::vector<ESP_NOW_Peer_Class> masters; //list of masters

/* Callbacks */

// Callback called when an unknown peer sends a message
void register_new_master(const esp_now_recv_info_t *info, const uint8_t *data, int len, void *arg) {
  if (memcmp(info->des_addr, ESP_NOW.BROADCAST_ADDR, 6) == 0) {
    Serial.printf("Unknown peer " MACSTR " sent a broadcast message\n", MAC2STR(info->src_addr));
    Serial.println("Registering the peer as a master");

    uint8_t wifi_channel = WiFi.channel();


    ESP_NOW_Peer_Class new_master(info->src_addr, wifi_channel, WIFI_IF_STA, NULL);

    masters.push_back(new_master);
    if (!masters.back().add_peer()) {
      Serial.println("Failed to register the new master");
      return;
    }
  } else {
    // The slave will only receive broadcast messages
    log_v("Received a unicast message from " MACSTR, MAC2STR(info->src_addr));
    log_v("Igorning the message");
  }
}



//MQTT
WiFiClientSecure espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;

#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  WiFi.mode(WIFI_STA);


  while (!WiFi.STA.started()) {
    delay(100);
  }
  WiFi.begin(ssid, password); //connect to wifi router

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  } // waiting for connection

  randomSeed(micros());
  Serial.println("\nWiFi connected\nIP address: ");
  Serial.println(WiFi.localIP());

  while (!Serial) delay(1);

  //Connect to MQTT Broker (credentials stored in config.h file for security)
  espClient.setCACert(root_ca);
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  delay(500);



  //Begin ESP-NOW

  // Initialize the ESP-NOW protocol
  if (!ESP_NOW.begin()) {
    Serial.println("Failed to initialize ESP-NOW");
    Serial.println("Reeboting in 5 seconds...");
    delay(5000);
    ESP.restart();
  }

  // Register the new peer callback
  uint8_t wifi_channel = WiFi.channel();
  Serial.println(wifi_channel);
  ESP_NOW.onNewPeer(register_new_master, NULL);

  //connecting Servos to pins
  pointerServo.attach(pointerPin);
  middleServo.attach(middlePin);
  ringServo.attach(ringPin);
  pinkyServo.attach(pinkyPin);
  

  gesture.g = "default"; //set the gesture to default on start up (standard open and close of hand)


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
  payloadValue = payloadValue.substring(payloadBeginning); //returns "{valueOfPayload}" i.e "default"
  payloadValue = payloadValue.substring(1, payloadValue.length() - 1); //remove quotation marks around value so "default" becomes default

  
  Serial.println(gesture.g);
  gesture.g = payloadValue;

}



#include "ESP32_NOW.h"
#include "WiFi.h"


//Variables
uint16_t receivedValue;


// Callback when data is received
void onDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *data, int dataLen) {
  memcpy(&receivedValue, data, sizeof(receivedValue));
  Serial.print("Muscle State: ");
  Serial.println(receivedValue);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  WiFi.mode(WIFI_STA);


  esp_now_register_recv_cb(onDataRecv);


}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100);

}

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH YOUR RECEIVER MAC Address
//uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
//uint8_t broadcastAddress[] = {0xA0, 0x20, 0xA6, 0x12, 0x1B, 0x9A};
uint8_t broadcastAddress[] = {0x40, 0x22, 0xD8, 0xF0, 0xAD, 0x50};

#define GPIOVX    33
#define GPIOVY    34
#define GPIOPUSH  18

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int x; 
  int y;
  char cmd;
} struct_message;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Set GPIOPUSH as input
  pinMode(GPIOPUSH, INPUT_PULLUP);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
char last_cmd = 0xff;

void loop() {
  // Set values to send
  int vx = analogRead(GPIOVX);
  int vy = analogRead(GPIOVY);
  myData.x = vx;
  myData.y = vy;
  Serial.printf("VX=%d VY=%d\r\n",vx, vy);
  if(vy<1000){
    if(vx < 1000){
      myData.cmd = 0x04;
    }else if(vx < 3000){
      myData.cmd = 0x05;
    }else{
      // vx >= 3000
      myData.cmd = 0x01;
    }  
  }else if(vy<3000){
    if(vx < 1000){
      myData.cmd = 0x06;
    }else if(vx < 3000){
      myData.cmd = 0x00; // no movement
    }else{
      // vx >= 3000
      myData.cmd = 0x09;
    }  
  }else{
    // vy >= 3000
    if(vx < 1000){
      myData.cmd = 0x02;
    }else if(vx < 3000){
      myData.cmd = 0x0A; // FWD
    }else{
      // vx >= 3000
      myData.cmd = 0x08;    }  
  }
  if(digitalRead(GPIOPUSH) == LOW)
    myData.cmd |= 0x10;
  else
    myData.cmd &= 0x0F;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
      
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  last_cmd = myData.cmd;  

  delay(100);
}

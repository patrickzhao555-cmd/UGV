/*
// actual code with micro ros
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/point.h>

#include <WiFi.h>
#include <esp_now.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define Q_SIZE 10
#define ESPNOW_WIFI_CHANNEL 1

static const uint8_t PEER_MAC[6] = {0xC8, 0x2E, 0x18, 0xFB, 0x2A, 0x00};

typedef struct __attribute__((packed)) Msg{
  int32_t seqNum;
  uint8_t status_code;
  float target_x;
  float target_y;
}Msg;

typedef struct{
  Msg packet;
} Entry;

QueueHandle_t Queue = NULL;

rcl_publisher_t target_publisher;
geometry_msgs__msg__Point target_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn){ rcl_ret_t rc = (fn); if((rc) != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn){ rcl_ret_t rc = (fn); (void)rc; }

void error_loop(){
  while (1) { delay(100); }
}

void OnDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len){
  if (data == NULL) return;
  if (len != (int)sizeof(Msg)) return;                     
  if (memcmp(info->src_addr, PEER_MAC, 6) != 0) return;

  Entry item;
  memcpy(&item.packet, data, sizeof(Msg));

  BaseType_t hpw = pdFALSE;
  xQueueSendFromISR(Queue, &item, &hpw);
  if (hpw) portYIELD_FROM_ISR();
}

void initESPNow(){
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  delay(100);

  if (esp_now_init() != ESP_OK){
    Serial.println("ESP-NOW init failed");
    error_loop();
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, PEER_MAC, 6);
  peer.channel = ESPNOW_WIFI_CHANNEL;
  peer.encrypt = false;

  if (!esp_now_is_peer_exist(PEER_MAC)){
    if (esp_now_add_peer(&peer) != ESP_OK){
      Serial.println("Failed to add ESP NOW peer");
      error_loop();
    }
  }

  esp_now_register_recv_cb(OnDataRecv);
}

void setup(){
  Serial.begin(115200);

  Queue = xQueueCreate(Q_SIZE, sizeof(Entry));
  if (!Queue){
    Serial.println("Queue init failed");
    error_loop();
  }

  set_microros_transports();
  initESPNow();

  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK){
    delay(500);
  }

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_espnow_node", "", &support));
  RCCHECK(rclc_publisher_init_default(&target_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), "drone_target"));
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
}

void loop(){
  Entry item;
  while (xQueueReceive(Queue, &item, 0) == pdTRUE){
    esp_now_send(PEER_MAC, (uint8_t*)&item.packet, sizeof(Msg)); // echo
    
    if (item.packet.status_code == 1){
      target_msg.x = item.packet.target_x;
      target_msg.y = item.packet.target_y;
      target_msg.z = 0.0f;

      RCSOFTCHECK(rcl_publish(&target_publisher, &target_msg, NULL));
    }
  }
  delay(5);
}
*/

// communication test only code
#include <WiFi.h>
#include <esp_now.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define Q_SIZE 10
#define ESPNOW_WIFI_CHANNEL 1

static const uint8_t PEER_MAC[6] = {0xC8, 0x2E, 0x18, 0xFB, 0x2A, 0x00};

typedef struct __attribute__((packed)) Msg{
  int32_t seqNum;
  uint8_t status_code;
  float target_x;
  float target_y;
} Msg;

typedef struct{
  Msg packet;
} Entry;

QueueHandle_t Queue = NULL;

void error_loop(){while (1){delay(100);}}

void OnDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len){
  if (data == NULL) return;
  if (len != (int)sizeof(Msg)) return;
  if (memcmp(info->src_addr, PEER_MAC, 6) != 0) return;

  Entry item;
  memcpy(&item.packet, data, sizeof(Msg));

  BaseType_t hpw = pdFALSE;
  xQueueSendFromISR(Queue, &item, &hpw);
  if (hpw) portYIELD_FROM_ISR();
  return;
}

void initESPNow(){
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  delay(100);

  if (esp_now_init() != ESP_OK){
    Serial.println("ESP-NOW init failed");
    error_loop();
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, PEER_MAC, 6);
  peer.channel = ESPNOW_WIFI_CHANNEL;
  peer.encrypt = false;

  if (!esp_now_is_peer_exist(PEER_MAC)){
    if (esp_now_add_peer(&peer) != ESP_OK){
      Serial.println("Failed to add ESP-NOW peer");
      error_loop();
    }
  }

  esp_now_register_recv_cb(OnDataRecv);
  return;
}

void setup(){
  Serial.begin(115200);

  Queue = xQueueCreate(Q_SIZE, sizeof(Entry));
  if (!Queue){
    Serial.println("Queue init failed");
    error_loop();
  }

  initESPNow();
  Serial.println("ESP-NOW ready");

  String mac = WiFi.macAddress();
  Serial.print("My MAC Address: ");
  Serial.println(mac);
  return;
}

void loop(){
  Entry item;

  while (xQueueReceive(Queue, &item, 0) == pdTRUE){
    Serial.print("Received -> seqNum: ");
    Serial.print(item.packet.seqNum);
    Serial.print(", status_code: ");
    Serial.print(item.packet.status_code);
    Serial.print(", target_x: ");
    Serial.print(item.packet.target_x);
    Serial.print(", target_y: ");
    Serial.println(item.packet.target_y);

    esp_err_t result = esp_now_send(PEER_MAC, (uint8_t*)&item.packet, sizeof(Msg));
    if (result == ESP_OK){
      Serial.println("Echo sent successfully");
    }else{
      Serial.print("Echo send failed: ");
      Serial.println(result);
    }
  }

  delay(5);
}


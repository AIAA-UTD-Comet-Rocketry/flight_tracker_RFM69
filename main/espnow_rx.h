#pragma once
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint8_t from[6]; // Sender's MAC Address
  int     len;
  uint8_t data[ESPNOW_RX_MAX_LEN];
} espnow_rx_frame_t;

esp_err_t espnow_rx_start(QueueHandle_t *out_queue); // creates queue+task and returns handle
// Call espnow_rx_start() from app_main and read frames with xQueueReceive().

#ifdef __cplusplus
}
#endif // espnow_rx.h

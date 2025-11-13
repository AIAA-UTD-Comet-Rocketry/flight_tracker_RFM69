#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "global_config.h"
#include "espnow_rx.h"

static const char *TAG = "espnow_rx";
static QueueHandle_t s_rx_queue = NULL;

static void on_rx_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  // Filter out data from other MAC addresses
  if (memcmp(info->src_addr, ESPNOW_SENDER_MAC, 6) != 0)
    return;
  if (!s_rx_queue || len <= 0) 
    return;
  espnow_rx_frame_t frame = {0};
  memcpy(frame.from, info->src_addr, 6);
  frame.len = len > ESPNOW_RX_MAX_LEN ? ESPNOW_RX_MAX_LEN : len;
  memcpy(frame.data, data, frame.len);
  BaseType_t xHigherWoken = pdFALSE;
  xQueueSendFromISR(s_rx_queue, &frame, &xHigherWoken);
  if (xHigherWoken) portYIELD_FROM_ISR();
}

static esp_err_t wifi_init_fixed_channel(void) {
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));                  // receiver in STA
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));                     // disable PS for reliability
  ESP_ERROR_CHECK(esp_wifi_start());
  return esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE); // must match sender
}

static void espnow_task(void *arg) {
  // nothing to do here; actual RX is via callback -> queue
  for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
}

esp_err_t espnow_rx_start(QueueHandle_t *out_queue) {
  // Ensure NVS is ready once in your app before Wi‑Fi init
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
  }

  ESP_ERROR_CHECK(wifi_init_fixed_channel());                         // init Wi‑Fi RX path

  ESP_ERROR_CHECK(esp_now_init());                                    // start ESP‑NOW
  ESP_ERROR_CHECK(esp_now_register_recv_cb(on_rx_cb));                // RX only

  // Add known sender as peer (unencrypted)
  esp_now_peer_info_t peer = {0};
  memcpy(peer.peer_addr, ESPNOW_SENDER_MAC, 6);
  peer.ifidx   = ESPNOW_IFACE;                                        // e.g., WIFI_IF_STA
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;                                               // no LMK (unencrypted)
  ESP_ERROR_CHECK(esp_now_add_peer(&peer));

  // Create RX queue and idle task
  s_rx_queue = xQueueCreate(ESPNOW_RX_QUEUE_LEN, sizeof(espnow_rx_frame_t));
  if (!s_rx_queue) return ESP_ERR_NO_MEM;
  if (out_queue) *out_queue = s_rx_queue;

  xTaskCreatePinnedToCore(espnow_task, "espnow_task", 2048, NULL, 4, NULL, tskNO_AFFINITY);
  ESP_LOGI(TAG, "ESP‑NOW RX ready on channel %d", ESPNOW_CHANNEL);
  return ESP_OK;
}

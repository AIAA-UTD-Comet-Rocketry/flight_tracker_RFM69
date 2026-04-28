// led_status.h
#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

// Event bits for each activity
#define LED_EVT_GPS_TX   (1 << 0)
#define LED_EVT_RF_TX    (1 << 1)
#define LED_EVT_CAN_RX   (1 << 2)
#define LED_EVT_CAN_TX   (1 << 3)
#define LED_EVT_WIFI_RX  (1 << 4)
#define LED_EVT_ERROR    (1 << 5)

#ifdef __cplusplus
extern "C" {
#endif

void led_status_init(void);
void led_signal(uint32_t evt_bit);  // call from any task to trigger a blink
void led_task(void *arg);

#ifdef __cplusplus
}
#endif
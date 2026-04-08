// led_status.c
#include "led_status.h"
#include "driver/gpio.h"
#include "esp_log.h"

// GPIO pin assignments - adjust to your board
#define LED_GPS_PIN    34
#define LED_RF_PIN     35
#define LED_CAN_PIN    33
#define LED_WIFI_PIN   36
#define LED_STATUS_PIN 21

#define BLINK_MS       50   // short pulse so it doesn't block visual feedback

static const char *TAG = "LED";
static EventGroupHandle_t s_led_events;

typedef struct {
    uint32_t    evt_bit;
    gpio_num_t  pin;
} led_map_t;

static const led_map_t led_table[] = {
    { LED_EVT_GPS_TX,  LED_GPS_PIN    },
    { LED_EVT_RF_TX,   LED_RF_PIN     },
    { LED_EVT_CAN_RX,  LED_CAN_PIN   },
    { LED_EVT_WIFI_RX, LED_WIFI_PIN   },
    { LED_EVT_ERROR,   LED_STATUS_PIN },
};
static const int LED_COUNT = sizeof(led_table) / sizeof(led_table[0]);

void led_signal(uint32_t evt_bit) {
    xEventGroupSetBits(s_led_events, evt_bit);
}

void led_task(void *arg) {
    const uint32_t ALL_BITS = LED_EVT_GPS_TX | LED_EVT_RF_TX |
                              LED_EVT_CAN_RX | LED_EVT_WIFI_RX | LED_EVT_ERROR;

    // Status LED solid on = system OK
    gpio_set_level(LED_STATUS_PIN, 1);

    while (1) {
        // Wait for any LED event, timeout every 1s for status heartbeat
        uint32_t bits = xEventGroupWaitBits(
            s_led_events, ALL_BITS,
            pdTRUE,    // clear bits on return
            pdFALSE,   // any bit triggers
            pdMS_TO_TICKS(1000)
        );

        if (bits == 0) {
            // No activity — toggle status LED as heartbeat
            static bool hb = false;
            hb = !hb;
            gpio_set_level(LED_STATUS_PIN, hb);
            continue;
        }

        // Turn on whichever LEDs were signaled
        for (int i = 0; i < LED_COUNT; i++) {
            if (bits & led_table[i].evt_bit) {
                gpio_set_level(led_table[i].pin, 1);
            }
        }

        // Hold blink
        vTaskDelay(pdMS_TO_TICKS(BLINK_MS));

        // Turn them back off (except status)
        for (int i = 0; i < LED_COUNT; i++) {
            if (led_table[i].pin != LED_STATUS_PIN) {
                gpio_set_level(led_table[i].pin, 0);
            }
        }

        // Restore status LED to solid on (system OK)
        gpio_set_level(LED_STATUS_PIN, 1);
    }
}

void led_status_init(void) {
    s_led_events = xEventGroupCreate();

    for (int i = 0; i < LED_COUNT; i++) {
        gpio_reset_pin(led_table[i].pin);
        gpio_set_direction(led_table[i].pin, GPIO_MODE_OUTPUT);
        gpio_set_level(led_table[i].pin, 0);
    }

    
    ESP_LOGI(TAG, "LED status task started");
}
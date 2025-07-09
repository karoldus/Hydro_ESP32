#ifndef __WIFI_H__
#define __WIFI_H__

#include "esp_check.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "string.h"

/**
 * @brief Wait for WiFi driver to connect with AP.
 *
 * @return esp_err_t
 */
esp_err_t wait_for_wifi_connection();

/**
 * @brief Initialize WiFi driver in STATION mode.
 *
 * @return
 *    - ESP_OK: succeed
 *    - ESP_ERR_WIFI_NOT_INIT: WiFi is not initialized by esp_wifi_init
 *    - ESP_ERR_INVALID_ARG: invalid argument
 *    - ESP_ERR_NO_MEM: out of memory
 *    - ESP_ERR_WIFI_CONN: WiFi internal error, station or soft-AP control block wrong
 *    - ESP_FAIL: other WiFi internal errors
 */
esp_err_t wifi_init();

#endif // __WIFI_H__
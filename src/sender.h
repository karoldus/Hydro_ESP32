#ifndef __SENDER_H__
#define __SENDER_H__

#include "esp_check.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

/**
 * @brief Send JSON data over Wi-Fi to API
 *
 * @param TAG Tag for logging
 * @param json_data JSON data to send as unformatted string
 * @param json_data_len Length of the JSON data
 * @return esp_err_t
 */
esp_err_t send_json_data(const char *TAG, const char *json_data, const size_t json_data_len);

#endif // __SENDER_H__
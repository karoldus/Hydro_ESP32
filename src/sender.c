#include "sender.h"
#include "wifi.h"

#include "esp_http_client.h"

static esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    return ESP_OK; // Możesz rozwinąć obsługę zdarzeń jeśli potrzebujesz
}

esp_err_t http_post_json(const char *TAG, const char *url, const char *json_string, const size_t json_string_len)
{
    esp_http_client_config_t config = {
        .url = url,
        .event_handler = _http_event_handler,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, json_string, json_string_len);

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %lli\n", esp_http_client_get_status_code(client),
                 esp_http_client_get_content_length(client));
    }
    else
    {
        ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
    return err;
}

esp_err_t send_json_data(const char *TAG, const char *json_data, const size_t json_data_len)
{
    ESP_LOGI(TAG, "Sending JSON data: %s", json_data);

    // Connect to Wi-Fi if not already connected
    if (wait_for_wifi_connection(0) != ESP_OK)
    {
        ESP_LOGE(TAG, "Wi-Fi not connected, cannot send data");
        return ESP_ERR_INVALID_STATE;
    }

    // Send JSON data over HTTP
    esp_err_t err = http_post_json(TAG, HYDRO_API_URL, json_data, json_data_len);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send JSON data");
        return err;
    }

    ESP_LOGI(TAG, "JSON data sent successfully");
    return ESP_OK;
}
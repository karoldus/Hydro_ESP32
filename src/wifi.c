#include "wifi.h"

static const char *TAG = "wifi";

#define WIFI_SSID     "HydroWifi"
#define WIFI_PASSWORD "xxx"

#define WIFI_TASK_CORE (0)

#define WIFI_STA_CONNECTED_BIT BIT0

/**
 * @brief   Internal WiFi event group.
 */
static EventGroupHandle_t wifi_event_group = NULL;

static void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_id;

    ESP_LOGI(TAG, "%s STA_GOT_IP", event_base);

    xEventGroupSetBits(wifi_event_group, WIFI_STA_CONNECTED_BIT);

    const ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "got ip: " IPSTR, IP2STR(&event->ip_info.ip));
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_data;

    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        ESP_LOGI(TAG, "%s %s", event_base, "STA_START");

        wifi_config_t service_wifi_config = {
            .sta =
                {
                    .ssid = WIFI_SSID,
                    .password = WIFI_PASSWORD,
                    .bssid_set = false,
                    .channel = 0,
                    .threshold = {.authmode = WIFI_AUTH_OPEN},
                },
        };

        esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &service_wifi_config);

        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Error setting service wifi config: %s", esp_err_to_name(err));
            return;
        }

        esp_wifi_connect(); // Immediately after starting wifi, connect with configuration saved in NVS
        break;

    case WIFI_EVENT_STA_CONNECTED:
        ESP_LOGI(TAG, "%s %s", event_base, "STA_CONNECTED");
        break;

    case WIFI_EVENT_STA_DISCONNECTED:
        ESP_LOGI(TAG, "%s %s", event_base, "STA_DISCONNECTED");
        xEventGroupClearBits(wifi_event_group, WIFI_STA_CONNECTED_BIT);

        esp_wifi_connect();
        ESP_LOGI(TAG, "Reconnecting...");
        return;

    default:
        ESP_LOGI(TAG, "%s -> %ld", event_base, event_id);
        break;
    }
}

esp_err_t wait_for_wifi_connection(uint16_t timeout_ms)
{
    if (wifi_event_group == NULL)
    {
        ESP_LOGE(TAG, "wifi_event_group == NULL");
        return ESP_ERR_INVALID_STATE;
    }
    xEventGroupWaitBits(wifi_event_group, WIFI_STA_CONNECTED_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(timeout_ms));

    EventBits_t bits = xEventGroupGetBits(wifi_event_group);
    if (bits & WIFI_STA_CONNECTED_BIT)
    {
        return ESP_OK;
    }
    else
    {
        return ESP_FAIL;
    }
}

esp_err_t wifi_init()
{
    // Internal freertos event group
    wifi_event_group = xEventGroupCreate();
    if (wifi_event_group == NULL)
    {
        ESP_LOGE(TAG, "Error creating wifi event group");
        return ESP_ERR_NO_MEM;
    }

    // Default esp-idf event loop
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "esp_event_loop_create_default error: %s",
                        esp_err_to_name(err_rc_));

    // TCP/IP interface
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "esp_netif_init error: %s", esp_err_to_name(err_rc_));

    // STA interface
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    if (sta_netif == NULL)
    {
        ESP_LOGE(TAG, "Error creating default wifi sta netif");
        return ESP_ERR_NO_MEM;
    }

    // Set hostname
    esp_err_t err = esp_netif_set_hostname(sta_netif, "hydroponika");
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error setting hostname: %s", esp_err_to_name(err));
        // return err;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.wifi_task_core_id = WIFI_TASK_CORE;

    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "esp_wifi_init error: %s", esp_err_to_name(err_rc_));
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "esp_wifi_set_mode error: %s", esp_err_to_name(err_rc_));

    ESP_RETURN_ON_ERROR(
        esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL, NULL), TAG,
        "esp_event_handler_instance_register error: %s", esp_err_to_name(err_rc_));

    ESP_RETURN_ON_ERROR(
        esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL), TAG,
        "esp_event_handler_instance_register error: %s", esp_err_to_name(err_rc_));

    // Start wifi
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "esp_wifi_start error: %s", esp_err_to_name(err_rc_));

    return esp_wifi_set_ps(WIFI_PS_NONE);
}
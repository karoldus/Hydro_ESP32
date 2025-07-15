/*
NOTE: grove water level sensor and bme280 are using the same I2C address, so only one of them can be used at a time
TODO:
- Pressure from BME680 is always 0, check if it's a sensor issue or code issue
*/

#include <esp_err.h>
#include <esp_log.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>

#include "hydro_pinout.h"
#include "hydro_pump.h"
#include "hydro_sensors.h"
#include "wifi.h"

// drivers
#include "driver/gpio.h"
#include "driver/ledc.h"
#include <cJSON.h>

//================= DEFINES =================

#define PUMP_ON_TIME_MS                              (10000)  // 10 seconds
#define PUMP_OFF_TIME_MS                             (600000) // 10 minutes
#define PUMP_WATER_LEVEL_ERROR_READING_RETRY_TIME_MS (5000)
#define PUMP_WATER_LEVEL_BELOW_MINIMUM_RETRY_TIME_MS (60000) // 1 minute
#define SENSORS_MEASURE_INTERVAL_MS                  (5000)

#define HYDRO_MIN_WATER_LEVEL 20 // Minimum water level to start the pump [in mm]

//================= SENSORS =================

hydro_sensors_group_t hydro_basic_sensors = {
    .water_level_sensor =
        {
            .model = SENSOR_MODEL_ULTRASONIC_WATER_LEVEL,
            .sensor_obj.ultrasonic =
                {
                    .trigger_pin = HYDRO_PINOUT_ULTRASONIC_TRIGGER,
                    .echo_pin = HYDRO_PINOUT_ULTRASONIC_ECHO,
                },
            .description = "water level",
        },
    // {
    //     .model = SENSOR_MODEL_GROVE_WATER_LEVEL,
    //     .interface.i2c =
    //         {
    //             .port = I2C_NUM_1,
    //         },
    //     .description = "water level",
    // },
    .inside_down_temp_hum_sensor =
        {
            .model = SENSOR_MODEL_AHT20,
            .sensor_obj.aht =
                {
                    .type = AHT_TYPE_AHT20,
                    .mode = AHT_MODE_NORMAL,
                },
            .interface.i2c =
                {
                    .addr = AHT_I2C_ADDRESS_GND,
                    .port = I2C_NUM_1,
                },
            .description = "inside down",
        },
    .inside_up_temp_hum_sensor =
        {
            .model = SENSOR_MODEL_AHT20,
            .sensor_obj.aht =
                {
                    .type = AHT_TYPE_AHT20,
                    .mode = AHT_MODE_NORMAL,
                },
            .interface.i2c =
                {
                    .addr = AHT_I2C_ADDRESS_GND,
                    .port = I2C_NUM_0,
                },
            .description = "inside up",
        },
    .outside_down_temp_hum_sensor =
        {
            // TODO: dlaczego nie działa?
            .model = SENSOR_MODEL_BME680,
            .interface.i2c =
                {
                    .addr = BME680_I2C_ADDR_0,
                    .port = I2C_NUM_1,
                },
            .description = "outside down",
        },
    .outside_down_lux_sensor =
        {
            .model = SENSOR_MODEL_TSL2591,
            .interface.i2c =
                {
                    .port = I2C_NUM_1,
                },
            .description = "outside down",
        },
    .outside_up_temp_hum_sensor =
        {
            .model = SENSOR_MODEL_BME280,
            .interface.i2c =
                {
                    .addr = BMP280_I2C_ADDRESS_1,
                    .port = I2C_NUM_0,
                },
            .description = "outside up",
        },
    .outside_up_lux_sensor =
        {
            .model = SENSOR_MODEL_TSL2591,
            .interface.i2c =
                {
                    .port = I2C_NUM_0,
                },
            .description = "outside up",
        },
};

hydro_sensor_t *sensors_obj_list[] = {
    &hydro_basic_sensors.water_level_sensor,        &hydro_basic_sensors.inside_down_temp_hum_sensor,
    &hydro_basic_sensors.inside_up_temp_hum_sensor, &hydro_basic_sensors.outside_down_temp_hum_sensor,
    &hydro_basic_sensors.outside_down_lux_sensor,   &hydro_basic_sensors.outside_up_temp_hum_sensor,
    &hydro_basic_sensors.outside_up_lux_sensor,
};

// event group for pump control
EventGroupHandle_t xLedEventGroup;
#define LED_EVENT_BLINK_BIT (1 << 0) // Event bit for LED blink

void led_task(void *pvParameters)
{
    static const char *TAG = "LED TASK";

    gpio_set_direction(HYDRO_PINOUT_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(HYDRO_PINOUT_LED, 1);

    // Initialize the event group
    xLedEventGroup = xEventGroupCreate();
    if (xLedEventGroup == NULL)
    {
        ESP_LOGE(TAG, "Failed to create LED event group");
        vTaskDelete(NULL);
        return;
    }

    while (1)
    {
        // Wait for the LED blink event
        EventBits_t uxBits = xEventGroupWaitBits(xLedEventGroup, LED_EVENT_BLINK_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
        if (uxBits & LED_EVENT_BLINK_BIT)
        {
            ESP_LOGI(TAG, "Blinking LED");
            gpio_set_level(HYDRO_PINOUT_LED, 0); // Turn on LED
            vTaskDelay(pdMS_TO_TICKS(1000));
            gpio_set_level(HYDRO_PINOUT_LED, 1); // Turn off LED
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

void pump_task(void *pvParameters)
{
    static const char *TAG = "PUMP TASK";

    // Configure the PWM timer
    ledc_timer_config_t ledc_timer = {.speed_mode = LEDC_LOW_SPEED_MODE,
                                      .timer_num = LEDC_TIMER_0,
                                      .duty_resolution = LEDC_TIMER_13_BIT,
                                      .freq_hz = 5000,
                                      .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Configure the PWM channel
    ledc_channel_config_t ledc_channel = {.speed_mode = LEDC_LOW_SPEED_MODE,
                                          .channel = LEDC_CHANNEL_0,
                                          .timer_sel = LEDC_TIMER_0,
                                          .intr_type = LEDC_INTR_DISABLE,
                                          .gpio_num = HYDRO_PINOUT_PUMP_PWM,
                                          .duty = 0,
                                          .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    vTaskDelay(pdMS_TO_TICKS(5000));

    hydro_data_t data;
    esp_err_t err;

    while (1)
    {
        // Read the water level sensor
        err = read_sensor(TAG, &hydro_basic_sensors.water_level_sensor, &data);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Error reading water level sensor: %d", err);
            vTaskDelay(pdMS_TO_TICKS(PUMP_WATER_LEVEL_ERROR_READING_RETRY_TIME_MS));
            continue;
        }
        ESP_LOGI(TAG, "Water level: %d%%", data.data.water_level.water_level);

        if (data.data.water_level.water_level < HYDRO_MIN_WATER_LEVEL)
        {
            ESP_LOGE(TAG, "Water level below minimum (%d%%)!", HYDRO_MIN_WATER_LEVEL);
            // Notify the LED task to blink the LED
            xEventGroupSetBits(xLedEventGroup, LED_EVENT_BLINK_BIT);
            vTaskDelay(pdMS_TO_TICKS(PUMP_WATER_LEVEL_BELOW_MINIMUM_RETRY_TIME_MS));
            continue;
        }

        // If water level is sufficient, stop blinking the LED
        xEventGroupClearBits(xLedEventGroup, LED_EVENT_BLINK_BIT);

        ESP_LOGI(TAG, "Starting pump slow start");
        pump_slow_start(&ledc_channel);
        vTaskDelay(pdMS_TO_TICKS(PUMP_ON_TIME_MS));
        ESP_LOGI(TAG, "Stopping pump slow stop");
        pump_slow_stop(&ledc_channel);
        vTaskDelay(pdMS_TO_TICKS(PUMP_OFF_TIME_MS));
    }
}

// void sensors_task(void *pvParameters)
// {
//     static const char *TAG = "SENSORS TASK";

//     hydro_data_t data;
//     esp_err_t err;

//     while (1)
//     {
//         for (size_t i = 0; i < sizeof(sensors_obj_list) / sizeof(hydro_sensor_t *); i++)
//         {
//             err = read_sensor(TAG, sensors_obj_list[i], &data);
//             if (err != ESP_OK)
//             {
//                 ESP_LOGE(TAG, "Error reading sensor %s (%s): %d", HYDRO_SENSOR_MODEL_STR[sensors_obj_list[i]->model],
//                          sensors_obj_list[i]->description, err);
//                 continue;
//             }

//             ESP_LOGI(TAG, "Sensor %s (%s) read successfully:", HYDRO_SENSOR_MODEL_STR[sensors_obj_list[i]->model],
//                      sensors_obj_list[i]->description);

//             switch (data.type)
//             {
//             case HYDRO_DATA_TYPE_TEMP_HUM:
//                 ESP_LOGI(TAG, "Temperature: %.1f°C, Humidity: %.2f%%", data.data.temp_hum.temperature_c,
//                          data.data.temp_hum.humidity);
//                 break;
//             case HYDRO_DATA_TYPE_TEMP_HUM_PRESS:
//                 ESP_LOGI(TAG, "Temperature: %.1f°C, Humidity: %.2f%%, Pressure: %.2fPa",
//                          data.data.temp_hum_press.temperature_c, data.data.temp_hum_press.humidity,
//                          data.data.temp_hum_press.pressure_pa);
//                 break;
//             case HYDRO_DATA_TYPE_TEMP_HUM_PRESS_GAS:
//                 ESP_LOGI(TAG, "Temperature: %.1f°C, Humidity: %.2f%%, Pressure: %.2fPa, Gas resistance: %.2fOhm",
//                          data.data.temp_hum_press_gas.temperature_c, data.data.temp_hum_press_gas.humidity,
//                          data.data.temp_hum_press_gas.pressure_pa, data.data.temp_hum_press_gas.gas_resistance_ohm);
//                 break;
//             case HYDRO_DATA_TYPE_LUX:
//                 ESP_LOGI(TAG, "Lux: %.2f", data.data.lux.lux);
//                 break;
//             case HYDRO_DATA_TYPE_WATER_LEVEL:
//                 ESP_LOGI(TAG, "Water level: %d mm", data.data.water_level.water_level);
//                 break;
//             default:
//                 ESP_LOGE(TAG, "Unknown sensor data type: %d", data.type);
//             }
//         }

//         vTaskDelay(pdMS_TO_TICKS(5000));
//     }
// }

// Define a macro to read sensors and handle errors
#define READ_SENSOR(sensor_ptr, data_ptr)                                                                              \
    do                                                                                                                 \
    {                                                                                                                  \
        err = read_sensor(TAG, sensor_ptr, data_ptr);                                                                  \
        if (err != ESP_OK)                                                                                             \
        {                                                                                                              \
            ESP_LOGE(TAG, "Error reading sensor %s (%s): %d", HYDRO_SENSOR_MODEL_STR[(sensor_ptr)->model],             \
                     (sensor_ptr)->description, err);                                                                  \
        }                                                                                                              \
    } while (0)

void basic_sensors_task(void *pvParameters)
{
    static const char *TAG = "BASIC SENSORS TASK";
    esp_err_t err;

    hydro_data_t water_level = {0};
    hydro_data_t inside_down_temp_hum = {0};
    hydro_data_t inside_up_temp_hum = {0};
    hydro_data_t outside_down_temp_hum = {0};
    hydro_data_t outside_down_lux = {0};
    hydro_data_t outside_up_temp_hum = {0};
    hydro_data_t outside_up_lux = {0};

    while (1)
    {

        // Now use the macro for the water level sensor
        READ_SENSOR(&hydro_basic_sensors.water_level_sensor, &water_level);
        READ_SENSOR(&hydro_basic_sensors.inside_down_temp_hum_sensor, &inside_down_temp_hum);
        READ_SENSOR(&hydro_basic_sensors.inside_up_temp_hum_sensor, &inside_up_temp_hum);
        READ_SENSOR(&hydro_basic_sensors.outside_down_temp_hum_sensor, &outside_down_temp_hum);
        READ_SENSOR(&hydro_basic_sensors.outside_down_lux_sensor, &outside_down_lux);
        READ_SENSOR(&hydro_basic_sensors.outside_up_temp_hum_sensor, &outside_up_temp_hum);
        READ_SENSOR(&hydro_basic_sensors.outside_up_lux_sensor, &outside_up_lux);

        // Create JSON structure with sensor data
        cJSON *root = cJSON_CreateObject();
        cJSON *data = cJSON_CreateObject();
        cJSON *inside = cJSON_CreateObject();
        cJSON *outside = cJSON_CreateObject();
        cJSON *inside_up = cJSON_CreateObject();
        cJSON *inside_down = cJSON_CreateObject();
        cJSON *outside_up = cJSON_CreateObject();
        cJSON *outside_down = cJSON_CreateObject();

        // Add data to the JSON structure
        if (!root || !data || !inside || !outside || !inside_up || !inside_down || !outside_up || !outside_down)
        {
            ESP_LOGE(TAG, "Failed to create JSON objects");
            if (root) cJSON_Delete(root);
            vTaskDelay(pdMS_TO_TICKS(SENSORS_MEASURE_INTERVAL_MS));
            continue;
        }

        // Add water level data
        if (water_level.type == HYDRO_DATA_TYPE_WATER_LEVEL)
        {
            cJSON_AddNumberToObject(data, "water_level", water_level.data.water_level.water_level);
        }

        // Add inside up temperature and humidity
        if (inside_up_temp_hum.type == HYDRO_DATA_TYPE_TEMP_HUM)
        {
            cJSON_AddNumberToObject(inside_up, "temperature", inside_up_temp_hum.data.temp_hum.temperature_c);
            cJSON_AddNumberToObject(inside_up, "humidity", inside_up_temp_hum.data.temp_hum.humidity);
        }
        cJSON_AddItemToObject(inside, "up", inside_up);

        // Add inside down temperature and humidity
        if (inside_down_temp_hum.type == HYDRO_DATA_TYPE_TEMP_HUM)
        {
            cJSON_AddNumberToObject(inside_down, "temperature", inside_down_temp_hum.data.temp_hum.temperature_c);
            cJSON_AddNumberToObject(inside_down, "humidity", inside_down_temp_hum.data.temp_hum.humidity);
        }
        cJSON_AddItemToObject(inside, "down", inside_down);

        // Add outside up temperature and humidity
        if (outside_up_temp_hum.type == HYDRO_DATA_TYPE_TEMP_HUM_PRESS)
        {
            cJSON_AddNumberToObject(outside_up, "temperature", outside_up_temp_hum.data.temp_hum_press.temperature_c);
            cJSON_AddNumberToObject(outside_up, "humidity", outside_up_temp_hum.data.temp_hum_press.humidity);
        }
        // Add outside up light level
        if (outside_up_lux.type == HYDRO_DATA_TYPE_LUX)
        {
            cJSON_AddNumberToObject(outside_up, "lux", outside_up_lux.data.lux.lux);
        }
        cJSON_AddItemToObject(outside, "up", outside_up);

        // Add outside down temperature and humidity
        if (outside_down_temp_hum.type == HYDRO_DATA_TYPE_TEMP_HUM_PRESS_GAS)
        {
            cJSON_AddNumberToObject(outside_down, "temperature",
                                    outside_down_temp_hum.data.temp_hum_press_gas.temperature_c);
            cJSON_AddNumberToObject(outside_down, "humidity", outside_down_temp_hum.data.temp_hum_press_gas.humidity);
        }
        // Add outside down light level
        if (outside_down_lux.type == HYDRO_DATA_TYPE_LUX)
        {
            cJSON_AddNumberToObject(outside_down, "lux", outside_down_lux.data.lux.lux);
        }
        cJSON_AddItemToObject(outside, "down", outside_down);

        // Link everything together
        cJSON_AddItemToObject(data, "inside", inside);
        cJSON_AddItemToObject(data, "outside", outside);
        cJSON_AddItemToObject(root, "data", data);

        // Convert to string and print
        char *json_string = cJSON_Print(root);
        if (json_string)
        {
            ESP_LOGI(TAG, "Sensor data: %s", json_string);
            free(json_string);
        }

        // Clean up
        cJSON_Delete(root);

        vTaskDelay(pdMS_TO_TICKS(SENSORS_MEASURE_INTERVAL_MS));
    }
}

void app_main(void)
{
    static const char *TAG = "MAIN";
    esp_err_t err;

    ESP_ERROR_CHECK(i2cdev_init());
    ESP_ERROR_CHECK(nvs_flash_init());

    printf("Hello world!\n");

    err = init_all_sensors(TAG, sensors_obj_list, sizeof(sensors_obj_list) / sizeof(hydro_sensor_t *));
    if (err == ESP_ERR_NOT_FINISHED)
    {
        ESP_LOGW(TAG, "Not all sensors initialized!");
    }
    else if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error initializing all sensors: %d", err);
        return;
    }

    xTaskCreatePinnedToCore(basic_sensors_task, "basic-sensors-task", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL,
                            APP_CPU_NUM);

    xTaskCreatePinnedToCore(pump_task, "pump-task", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);

    xTaskCreatePinnedToCore(led_task, "led-task", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL, APP_CPU_NUM);
    err = wifi_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error initializing WiFi: %d", err);
    }
}
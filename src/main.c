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

// drivers
#include "driver/gpio.h"
#include "driver/ledc.h"

hydro_sensor_t sensors[] = {
    {
        .model = SENSOR_MODEL_GROVE_WATER_LEVEL,
        .interface.i2c =
            {
                .port = I2C_NUM_1,
            },
        .description = "water level",
    },
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
    {
        .model = SENSOR_MODEL_BME280,
        .interface.i2c =
            {
                .addr = BMP280_I2C_ADDRESS_1,
                .port = I2C_NUM_0,
            },
        .description = "outside up",
    },
    // { // TODO: dlaczego nie działa?
    //     .model = SENSOR_MODEL_BME680,
    //     .interface.i2c =
    //         {
    //             .addr = BME680_I2C_ADDR_0,
    //             .port = I2C_NUM_1,
    //         },
    //     .description = "outside down",
    // },
    {
        .model = SENSOR_MODEL_TSL2591,
        .interface.i2c =
            {
                .port = I2C_NUM_0,
            },
        .description = "outside up",
    },
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
    {
        .model = SENSOR_MODEL_TSL2591,
        .interface.i2c =
            {
                .port = I2C_NUM_1,
            },
        .description = "outside down",
    },
};

#define WATER_LEVEL_SENSOR_INDEX 0
#define HYDRO_MIN_WATER_LEVEL    35 // Minimum water level to start the pump [in mm]

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
        err = read_sensor(TAG, &sensors[WATER_LEVEL_SENSOR_INDEX], &data);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Error reading water level sensor: %d", err);
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        ESP_LOGI(TAG, "Water level: %d%%", data.data.water_level.water_level);

        if (data.data.water_level.water_level < HYDRO_MIN_WATER_LEVEL)
        {
            ESP_LOGE(TAG, "Water level below minimum (%d%%)!", HYDRO_MIN_WATER_LEVEL);
            vTaskDelay(pdMS_TO_TICKS(60000));
            continue;
        }

        ESP_LOGI(TAG, "Starting pump slow start");
        pump_slow_start(&ledc_channel);
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "Stopping pump slow stop");
        pump_slow_stop(&ledc_channel);
        vTaskDelay(pdMS_TO_TICKS(120000));
    }
}

void sensors_task(void *pvParameters)
{
    static const char *TAG = "SENSORS TASK";

    hydro_data_t data;
    esp_err_t err;

    while (1)
    {
        for (size_t i = 0; i < sizeof(sensors) / sizeof(hydro_sensor_t); i++)
        {
            err = read_sensor(TAG, &sensors[i], &data);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Error reading sensor %s (%s): %d", HYDRO_SENSOR_MODEL_STR[sensors[i].model],
                         sensors[i].description, err);
                continue;
            }

            ESP_LOGI(TAG, "Sensor %s (%s) read successfully:", HYDRO_SENSOR_MODEL_STR[sensors[i].model],
                     sensors[i].description);

            switch (data.type)
            {
            case HYDRO_DATA_TYPE_TEMP_HUM:
                ESP_LOGI(TAG, "Temperature: %.1f°C, Humidity: %.2f%%", data.data.temp_hum.temperature_c,
                         data.data.temp_hum.humidity);
                break;
            case HYDRO_DATA_TYPE_TEMP_HUM_PRESS:
                ESP_LOGI(TAG, "Temperature: %.1f°C, Humidity: %.2f%%, Pressure: %.2fPa",
                         data.data.temp_hum_press.temperature_c, data.data.temp_hum_press.humidity,
                         data.data.temp_hum_press.pressure_pa);
                break;
            case HYDRO_DATA_TYPE_TEMP_HUM_PRESS_GAS:
                ESP_LOGI(TAG, "Temperature: %.1f°C, Humidity: %.2f%%, Pressure: %.2fPa, Gas resistance: %.2fOhm",
                         data.data.temp_hum_press_gas.temperature_c, data.data.temp_hum_press_gas.humidity,
                         data.data.temp_hum_press_gas.pressure_pa, data.data.temp_hum_press_gas.gas_resistance_ohm);
                break;
            case HYDRO_DATA_TYPE_LUX:
                ESP_LOGI(TAG, "Lux: %.2f", data.data.lux.lux);
                break;
            case HYDRO_DATA_TYPE_WATER_LEVEL:
                ESP_LOGI(TAG, "Water level: %d%%", data.data.water_level.water_level);
                break;
            default:
                ESP_LOGE(TAG, "Unknown sensor data type: %d", data.type);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main(void)
{
    static const char *TAG = "MAIN";
    esp_err_t err;

    ESP_ERROR_CHECK(i2cdev_init());

    printf("Hello world!\n");

    err = init_all_sensors(TAG, sensors, sizeof(sensors) / sizeof(hydro_sensor_t));
    if (err == ESP_ERR_NOT_FINISHED)
    {
        ESP_LOGW(TAG, "Not all sensors initialized!");
    }
    else if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error initializing all sensors: %d", err);
        return;
    }

    xTaskCreatePinnedToCore(sensors_task, "sensors-task", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);

    xTaskCreatePinnedToCore(pump_task, "pump-task", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}
#include <esp_err.h>
#include <esp_log.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>

// sensor drivers
#include "grove_water_level_sensor.h"
#include <aht.h>
#include <bme680.h>
#include <bmp280.h>
#include <tsl2591.h>

#include "hydro_pinout.h"

#define ADDR     AHT_I2C_ADDRESS_GND
#define AHT_TYPE AHT_TYPE_AHT20

void aht_task(void *pvParameters)
{
    static const char *TAG = "AHT";

    aht_t dev = {0};
    dev.mode = AHT_MODE_NORMAL;
    dev.type = AHT_TYPE;

    ESP_ERROR_CHECK(aht_init_desc(&dev, ADDR, 0, HYDRO_PINOUT_I2C0_SDA, HYDRO_PINOUT_I2C0_SCL));
    ESP_ERROR_CHECK(aht_init(&dev));

    bool calibrated;
    ESP_ERROR_CHECK(aht_get_status(&dev, NULL, &calibrated));
    if (calibrated)
        ESP_LOGI(TAG, "Sensor calibrated");
    else
        ESP_LOGW(TAG, "Sensor not calibrated!");

    float temperature, humidity;

    while (1)
    {
        esp_err_t res = aht_get_data(&dev, &temperature, &humidity);
        if (res == ESP_OK)
            ESP_LOGI(TAG, "Temperature: %.1f°C, Humidity: %.2f%%", temperature, humidity);
        else
            ESP_LOGE(TAG, "Error reading data: %d (%s)", res, esp_err_to_name(res));

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void bme280_task(void *pvParameters)
{
    static const char *TAG = "BME280";

    bmp280_params_t params;
    bmp280_init_default_params(&params);
    bmp280_t dev;
    memset(&dev, 0, sizeof(bmp280_t));

    ESP_ERROR_CHECK(bmp280_init_desc(&dev, BMP280_I2C_ADDRESS_1, 0, HYDRO_PINOUT_I2C0_SDA, HYDRO_PINOUT_I2C0_SCL));

    ESP_ERROR_CHECK(bmp280_init(&dev, &params));

    bool bme280p = dev.id == BME280_CHIP_ID;
    ESP_LOGI(TAG, "BMP280: found %s", bme280p ? "BME280" : "BMP280");

    float pressure, temperature, humidity;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(5000));
        if (bmp280_read_float(&dev, &temperature, &pressure, &humidity) != ESP_OK)
        {
            ESP_LOGE(TAG, "Temperature/pressure reading failed");
            continue;
        }

        ESP_LOGI(TAG, "Pressure: %.2f Pa, Temperature: %.2f C, Humidity: %.2f%%", pressure, temperature, humidity);
    }
}

void bme680_task(void *pvParameters)
{
    static const char *TAG = "BME680";

    bme680_t sensor;
    memset(&sensor, 0, sizeof(bme680_t));

    ESP_ERROR_CHECK(bme680_init_desc(&sensor, BME680_I2C_ADDR_0, 0, HYDRO_PINOUT_I2C0_SDA, HYDRO_PINOUT_I2C0_SCL));

    // init the sensor
    ESP_ERROR_CHECK(bme680_init_sensor(&sensor));

    // Changes the oversampling rates to 4x oversampling for temperature
    // and 2x oversampling for humidity. Pressure measurement is skipped.
    bme680_set_oversampling_rates(&sensor, BME680_OSR_4X, BME680_OSR_NONE, BME680_OSR_2X);

    // Change the IIR filter size for temperature and pressure to 7.
    bme680_set_filter_size(&sensor, BME680_IIR_SIZE_7);

    // Change the heater profile 0 to 200 degree Celsius for 100 ms.
    bme680_set_heater_profile(&sensor, 0, 200, 100);
    bme680_use_heater_profile(&sensor, 0);

    // Set ambient temperature to 10 degree Celsius
    bme680_set_ambient_temperature(&sensor, 10);

    // as long as sensor configuration isn't changed, duration is constant
    uint32_t duration;
    bme680_get_measurement_duration(&sensor, &duration);

    TickType_t last_wakeup = xTaskGetTickCount();

    bme680_values_float_t values;
    while (1)
    {
        // trigger the sensor to start one TPHG measurement cycle
        if (bme680_force_measurement(&sensor) == ESP_OK)
        {
            // passive waiting until measurement results are available
            vTaskDelay(duration);

            // get the results and do something with them
            if (bme680_get_results_float(&sensor, &values) == ESP_OK)
                ESP_LOGI(TAG, "Sensor: %.2f °C, %.2f %%, %.2f hPa, %.2f Ohm", values.temperature, values.humidity,
                         values.pressure, values.gas_resistance);
        }
        // passive waiting until 5 seconds is over
        vTaskDelayUntil(&last_wakeup, pdMS_TO_TICKS(5000));
    }
}

void tsl2591_task(void *pvParameters)
{
    static const char *TAG = "TSL2591";

    tsl2591_t dev = {0};

    ESP_ERROR_CHECK(tsl2591_init_desc(&dev, 0, HYDRO_PINOUT_I2C0_SDA, HYDRO_PINOUT_I2C0_SCL));
    ESP_ERROR_CHECK(tsl2591_init(&dev));

    // Turn TSL2591 on
    ESP_ERROR_CHECK(tsl2591_set_power_status(&dev, TSL2591_POWER_ON));
    // Turn ALS on
    ESP_ERROR_CHECK(tsl2591_set_als_status(&dev, TSL2591_ALS_ON));
    // Set gain
    ESP_ERROR_CHECK(tsl2591_set_gain(&dev, TSL2591_GAIN_MEDIUM));
    // Set integration time = 300ms
    ESP_ERROR_CHECK(tsl2591_set_integration_time(&dev, TSL2591_INTEGRATION_300MS));

    float lux;
    esp_err_t res;
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(5000));

        if ((res = tsl2591_get_lux(&dev, &lux)) != ESP_OK)
            ESP_LOGE(TAG, "Could not read lux value: %d", res);
        else
            ESP_LOGI(TAG, "Lux: %f", lux);
    }
}

void grove_water_level_sensor_task(void *pvParameters)
{
    static const char *TAG = "GroveWaterLevelSensor";

    grove_water_level_sensor_t sensor;
    memset(&sensor, 0, sizeof(grove_water_level_sensor_t));

    ESP_ERROR_CHECK(grove_water_level_sensor_init_desc(&sensor, 0, HYDRO_PINOUT_I2C0_SDA, HYDRO_PINOUT_I2C0_SCL));
    ESP_ERROR_CHECK(grove_water_level_sensor_init(&sensor));

    while (1)
    {
        if (grove_water_level_sensor_get_water_level(&sensor) == ESP_OK)
            ESP_LOGI(TAG, "Water level: %d", sensor.water_level);
        else
            ESP_LOGE(TAG, "Error reading water level");

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main(void)
{
    printf("Hello world!\n");
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreatePinnedToCore(aht_task, "ath-example", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    // xTaskCreatePinnedToCore(bme280_task, "bme280-example", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    // //TODO: grove water level sensor and bme280 are using the same I2C bus
    xTaskCreatePinnedToCore(bme680_task, "bme680-example", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(tsl2591_task, "tsl2591-example", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(grove_water_level_sensor_task, "grove-water-level-sensor-example",
                            configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}
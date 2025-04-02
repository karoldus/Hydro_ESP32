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
#include "hydro_sensors.h"

// drivers
#include "driver/ledc.h"
#include "driver\gpio.h"

hydro_sensor_t sensors[] = {
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
    },
    {
        .model = SENSOR_MODEL_BME280,
        .interface.i2c =
            {
                .addr = BMP280_I2C_ADDRESS_1,
                .port = I2C_NUM_0,
            },
    },
    // { // TODO: dlaczego nie działa?
    //     .model = SENSOR_MODEL_BME680,
    //     .interface.i2c =
    //         {
    //             .addr = BME680_I2C_ADDR_0,
    //             .port = I2C_NUM_1,
    //         },
    // },
    {
        .model = SENSOR_MODEL_TSL2591,
        .interface.i2c =
            {
                .port = I2C_NUM_0,
            },
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
    },
    {
        .model = SENSOR_MODEL_TSL2591,
        .interface.i2c =
            {
                .port = I2C_NUM_1,
            },
    },
    // {
    //     .model = SENSOR_MODEL_GROVE_WATER_LEVEL,
    //     .interface.i2c =
    //         {
    //             .port = I2C_NUM_0,
    //         },
    // },
};

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

    // vTaskDelay(pdMS_TO_TICKS(2000));

    // bme680_t sensor = sensors[1].sensor_obj.bme680;
    // memset(&sensor, 0, sizeof(bme680_t));

    // ESP_ERROR_CHECK(bme680_init_desc(&sensor, BME680_I2C_ADDR_0, 0, HYDRO_PINOUT_I2C0_SDA, HYDRO_PINOUT_I2C0_SCL));

    // // init the sensor
    // ESP_ERROR_CHECK(bme680_init_sensor(&sensor));

    // // Changes the oversampling rates to 4x oversampling for temperature
    // // and 2x oversampling for humidity. Pressure measurement is skipped.
    // bme680_set_oversampling_rates(&sensor, BME680_OSR_4X, BME680_OSR_NONE, BME680_OSR_2X);

    // // Change the IIR filter size for temperature and pressure to 7.
    // bme680_set_filter_size(&sensor, BME680_IIR_SIZE_7);

    // // Change the heater profile 0 to 200 degree Celsius for 100 ms.
    // bme680_set_heater_profile(&sensor, 0, 200, 100);
    // bme680_use_heater_profile(&sensor, 0);

    // // Set ambient temperature to 10 degree Celsius
    // bme680_set_ambient_temperature(&sensor, 10);

    // as long as sensor configuration isn't changed, duration is constant
    // uint32_t duration;
    // bme680_get_measurement_duration(&sensor, &duration);

    TickType_t last_wakeup = xTaskGetTickCount();

    // bme680_values_float_t values;
    hydro_data_t values;
    esp_err_t err;
    while (1)
    {
        err = read_sensor(TAG, &sensors[1], &values);
        if (err != ESP_OK)
            ESP_LOGE(TAG, "Error reading sensor: %d", err);
        else
        {
            ESP_LOGI(TAG, "Sensor: %.2f °C, %.2f %%, %.2f hPa, %.2f Ohm", values.data.temp_hum_press_gas.temperature_c,
                     values.data.temp_hum_press_gas.humidity, values.data.temp_hum_press_gas.pressure_pa,
                     values.data.temp_hum_press_gas.gas_resistance_ohm);
        }
        // // trigger the sensor to start one TPHG measurement cycle
        // if (bme680_force_measurement(&sensor) == ESP_OK)
        // {
        //     // passive waiting until measurement results are available
        //     vTaskDelay(duration);

        //     // get the results and do something with them
        //     if (bme680_get_results_float(&sensor, &values) == ESP_OK)
        //         ESP_LOGI(TAG, "Sensor: %.2f °C, %.2f %%, %.2f hPa, %.2f Ohm", values.temperature, values.humidity,
        //                  values.pressure, values.gas_resistance);
        // }
        // passive waiting until 5 seconds is over
        vTaskDelayUntil(&last_wakeup, pdMS_TO_TICKS(5000));
    }
}

// void gpio_task(void *pvParameters)
// {

//     static const char *TAG = "GPIO";
//     gpio_set_direction(HYDRO_PINOUT_PUMP, GPIO_MODE_OUTPUT);

//     ESP_LOGI(TAG, "Pump is running");

//     while (1)
//     {
//         gpio_set_level(HYDRO_PINOUT_PUMP, 1);
//         vTaskDelay(pdMS_TO_TICKS(2000));
//         gpio_set_level(HYDRO_PINOUT_PUMP, 0);
//         vTaskDelay(pdMS_TO_TICKS(2000));
//     }
// }

// void pwm_task(void *pvParameters)
// {
//     static const char *TAG = "PWM";

//     // Configure the PWM timer
//     ledc_timer_config_t ledc_timer = {.speed_mode = LEDC_LOW_SPEED_MODE,
//                                       .timer_num = LEDC_TIMER_0,
//                                       .duty_resolution = LEDC_TIMER_13_BIT,
//                                       .freq_hz = 5000,
//                                       .clk_cfg = LEDC_AUTO_CLK};
//     ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

//     // Configure the PWM channel
//     ledc_channel_config_t ledc_channel = {.speed_mode = LEDC_LOW_SPEED_MODE,
//                                           .channel = LEDC_CHANNEL_0,
//                                           .timer_sel = LEDC_TIMER_0,
//                                           .intr_type = LEDC_INTR_DISABLE,
//                                           .gpio_num = HYDRO_PINOUT_PWM,
//                                           .duty = 0,
//                                           .hpoint = 0};
//     ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

//     int duty = 0;
//     int direction = 1;

//     while (1)
//     {
//         ESP_LOGI(TAG, "Setting PWM duty to %d", duty);
//         ESP_ERROR_CHECK(ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, duty));
//         ESP_ERROR_CHECK(ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel));

//         duty += direction * 2000;
//         if (duty >= 8191)
//         {
//             direction = -direction;
//             duty = 8191;
//         }
//         else if (duty <= 0)
//         {
//             direction = -direction;
//             duty = 0;
//         }

//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }

bool pump_slow_start(ledc_channel_config_t *ledc_channel)
{
    for (int i = 0; i < 8191; i += 1000)
    {
        if (i > 8191) i = 8191;

        // TODO: lepsza obsługa błędów
        ESP_ERROR_CHECK(ledc_set_duty(ledc_channel->speed_mode, ledc_channel->channel, i));
        ESP_ERROR_CHECK(ledc_update_duty(ledc_channel->speed_mode, ledc_channel->channel));
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    return true;
}

bool pump_slow_stop(ledc_channel_config_t *ledc_channel)
{
    for (int i = 8191; i >= 0; i -= 1000)
    {
        if (i < 0) i = 0;

        // TODO: better error handling
        ESP_ERROR_CHECK(ledc_set_duty(ledc_channel->speed_mode, ledc_channel->channel, i));
        ESP_ERROR_CHECK(ledc_update_duty(ledc_channel->speed_mode, ledc_channel->channel));
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    return true;
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

    bool state = false;

    vTaskDelay(pdMS_TO_TICKS(5000));

    while (1)
    {
        ESP_LOGI(TAG, "Starting pump slow start");
        pump_slow_start(&ledc_channel);
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "Stopping pump slow stop");
        pump_slow_stop(&ledc_channel);
        vTaskDelay(pdMS_TO_TICKS(15000));
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
            if (err != ESP_OK) ESP_LOGE(TAG, "Error reading sensor %d: %d", i, err);

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

    // xTaskCreatePinnedToCore(gpio_task, "gpio-example", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    // xTaskCreatePinnedToCore(pwm_task, "pwm-example", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);

    err = init_all_sensors(TAG, sensors, sizeof(sensors) / sizeof(hydro_sensor_t));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error initializing sensors: %d", err);
        return;
    }

    xTaskCreatePinnedToCore(sensors_task, "sensors-task", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);

    xTaskCreatePinnedToCore(pump_task, "pump-task", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);

    // xTaskCreatePinnedToCore(bme680_task, "bme680-example", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}
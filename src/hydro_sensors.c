/*
TODO : change ESP_ERROR_CHECK to something else to avoid aborting the program
*/

#include "hydro_sensors.h"

// ESP libs
#include <esp_err.h>
#include <esp_log.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>

#include "hydro_pinout.h"

#define WAIT_FOR_SENSOR_AVAILABILITY_TIMEOUT_MS (5000)

#define HYDRO_ULTRASONIC_WATER_LEVEL_SENSOR_HEIGHT_CM (12.5) // height of the water level sensor in cm

//==================================
//========= SENSORS INIT ===========
//==================================

#define __CHECK_INIT_RESP_BREAKCASE(x)                                                                                 \
    do                                                                                                                 \
    {                                                                                                                  \
        err = (x);                                                                                                     \
        if (err != ESP_OK)                                                                                             \
        {                                                                                                              \
            ESP_LOGE(TAG, "Error initializing %s (%s): %s", HYDRO_SENSOR_MODEL_STR[sensor->model],                     \
                     sensor->description, esp_err_to_name(err));                                                       \
            sensor->init_status = HYDRO_SENSOR_INIT_ERROR;                                                             \
            all_sensors_initialized = false;                                                                           \
            break;                                                                                                     \
        }                                                                                                              \
    } while (0)

// TODO: change cases to functions
esp_err_t init_all_sensors(const char *TAG, hydro_sensor_t *sensors, size_t sensor_count)
{
    esp_err_t err = ESP_OK;
    bool all_sensors_initialized = true;

    for (size_t i = 0; i < sensor_count; i++)
    {
        hydro_sensor_t *sensor = &sensors[i];
        gpio_num_t sda;
        gpio_num_t scl;

        sensor->sensor_mutex = xSemaphoreCreateMutex();
        if (sensor->sensor_mutex == NULL)
        {
            ESP_LOGE(TAG, "Error creating sensor mutex");
            return ESP_ERR_NO_MEM;
        }

        // take mutex with no timeout
        if (xSemaphoreTake(sensor->sensor_mutex, (TickType_t)0) != pdTRUE)
        {
            ESP_LOGE(TAG, "Error taking sensor mutex");
            return ESP_ERR_TIMEOUT;
        }

        switch (sensor->model)
        {
        case SENSOR_MODEL_AHT20:
            sda = sensor->interface.i2c.port == 0 ? HYDRO_PINOUT_I2C0_SDA : HYDRO_PINOUT_I2C1_SDA;
            scl = sensor->interface.i2c.port == 0 ? HYDRO_PINOUT_I2C0_SCL : HYDRO_PINOUT_I2C1_SCL;
            __CHECK_INIT_RESP_BREAKCASE(aht_init_desc(&sensor->sensor_obj.aht, sensor->interface.i2c.addr,
                                                      sensor->interface.i2c.port, sda, scl));
            __CHECK_INIT_RESP_BREAKCASE(aht_init(&sensor->sensor_obj.aht));

            // bool calibrated;
            // ESP_ERROR_CHECK(aht_get_status(&dev, NULL, &calibrated));
            // if (calibrated)
            //     ESP_LOGI(TAG, "Sensor calibrated");
            // else
            //     ESP_LOGW(TAG, "Sensor not calibrated!");

            ESP_LOGI(TAG, "Initialized AHT20 sensor '%s'", sensor->description);
            sensor->init_status = HYDRO_SENSOR_INIT_SUCCESS;
            break;
        case SENSOR_MODEL_BME280:
            sda = sensor->interface.i2c.port == 0 ? HYDRO_PINOUT_I2C0_SDA : HYDRO_PINOUT_I2C1_SDA;
            scl = sensor->interface.i2c.port == 0 ? HYDRO_PINOUT_I2C0_SCL : HYDRO_PINOUT_I2C1_SCL;
            __CHECK_INIT_RESP_BREAKCASE(bmp280_init_desc(&sensor->sensor_obj.bmp280, sensor->interface.i2c.addr,
                                                         sensor->interface.i2c.port, sda, scl));
            bmp280_params_t params;
            bmp280_init_default_params(&params);
            __CHECK_INIT_RESP_BREAKCASE(bmp280_init(&sensor->sensor_obj.bmp280, &params));
            bool bme280p = sensor->sensor_obj.bmp280.id == BME280_CHIP_ID;
            ESP_LOGI(TAG, "Initialized %s sensor '%s'", bme280p ? "BME280" : "BMP280", sensor->description);
            sensor->init_status = HYDRO_SENSOR_INIT_SUCCESS;
            break;
        case SENSOR_MODEL_BME680:
            sda = sensor->interface.i2c.port == 0 ? HYDRO_PINOUT_I2C0_SDA : HYDRO_PINOUT_I2C1_SDA;
            scl = sensor->interface.i2c.port == 0 ? HYDRO_PINOUT_I2C0_SCL : HYDRO_PINOUT_I2C1_SCL;
            __CHECK_INIT_RESP_BREAKCASE(bme680_init_desc(&sensor->sensor_obj.bme680, sensor->interface.i2c.addr,
                                                         sensor->interface.i2c.port, sda, scl));
            __CHECK_INIT_RESP_BREAKCASE(bme680_init_sensor(&sensor->sensor_obj.bme680));
            __CHECK_INIT_RESP_BREAKCASE(bme680_set_oversampling_rates(&sensor->sensor_obj.bme680, BME680_OSR_4X,
                                                                      BME680_OSR_NONE, BME680_OSR_2X));
            __CHECK_INIT_RESP_BREAKCASE(bme680_set_filter_size(&sensor->sensor_obj.bme680, BME680_IIR_SIZE_7));
            // Change the heater profile 0 to 200 degree Celsius for 100 ms.
            __CHECK_INIT_RESP_BREAKCASE(bme680_set_heater_profile(&sensor->sensor_obj.bme680, 0, 200, 100));
            __CHECK_INIT_RESP_BREAKCASE(bme680_use_heater_profile(&sensor->sensor_obj.bme680, 0));
            // Set ambient temperature to 10 degree Celsius
            __CHECK_INIT_RESP_BREAKCASE(bme680_set_ambient_temperature(&sensor->sensor_obj.bme680, 10));

            ESP_LOGI(TAG, "Initialized BME680 sensor '%s'", sensor->description);
            sensor->init_status = HYDRO_SENSOR_INIT_SUCCESS;
            break;
        case SENSOR_MODEL_TSL2591:
            sda = sensor->interface.i2c.port == 0 ? HYDRO_PINOUT_I2C0_SDA : HYDRO_PINOUT_I2C1_SDA;
            scl = sensor->interface.i2c.port == 0 ? HYDRO_PINOUT_I2C0_SCL : HYDRO_PINOUT_I2C1_SCL;
            __CHECK_INIT_RESP_BREAKCASE(
                tsl2591_init_desc(&sensor->sensor_obj.tsl2591, sensor->interface.i2c.port, sda, scl));
            __CHECK_INIT_RESP_BREAKCASE(tsl2591_init(&sensor->sensor_obj.tsl2591));
            // Turn TSL2591 on
            __CHECK_INIT_RESP_BREAKCASE(tsl2591_set_power_status(&sensor->sensor_obj.tsl2591, TSL2591_POWER_ON));
            // Turn ALS on
            __CHECK_INIT_RESP_BREAKCASE(tsl2591_set_als_status(&sensor->sensor_obj.tsl2591, TSL2591_ALS_ON));
            // Set gain
            __CHECK_INIT_RESP_BREAKCASE(tsl2591_set_gain(&sensor->sensor_obj.tsl2591, TSL2591_GAIN_MEDIUM));
            // Set integration time = 300ms
            __CHECK_INIT_RESP_BREAKCASE(
                tsl2591_set_integration_time(&sensor->sensor_obj.tsl2591, TSL2591_INTEGRATION_300MS));

            ESP_LOGI(TAG, "Initialized TSL2591 sensor '%s'", sensor->description);
            sensor->init_status = HYDRO_SENSOR_INIT_SUCCESS;
            break;
        case SENSOR_MODEL_GROVE_WATER_LEVEL:
            sda = sensor->interface.i2c.port == 0 ? HYDRO_PINOUT_I2C0_SDA : HYDRO_PINOUT_I2C1_SDA;
            scl = sensor->interface.i2c.port == 0 ? HYDRO_PINOUT_I2C0_SCL : HYDRO_PINOUT_I2C1_SCL;
            __CHECK_INIT_RESP_BREAKCASE(grove_water_level_sensor_init_desc(&sensor->sensor_obj.grove_water_level,
                                                                           sensor->interface.i2c.port, sda, scl));
            __CHECK_INIT_RESP_BREAKCASE(grove_water_level_sensor_init(&sensor->sensor_obj.grove_water_level));

            ESP_LOGI(TAG, "Initialized GROVE_WATER_LEVEL sensor '%s'", sensor->description);
            sensor->init_status = HYDRO_SENSOR_INIT_SUCCESS;
            break;
        case SENSOR_MODEL_ULTRASONIC_WATER_LEVEL:
            // Ultrasonic sensor initialization
            __CHECK_INIT_RESP_BREAKCASE(ultrasonic_init(&sensor->sensor_obj.ultrasonic));

            ESP_LOGI(TAG, "Initialized ULTRASONIC_WATER_LEVEL sensor '%s'", sensor->description);
            sensor->init_status = HYDRO_SENSOR_INIT_SUCCESS;
            break;
        default:
            ESP_LOGE("init_all_sensors", "Unknown sensor model: %d", sensor->model);
            sensor->init_status = HYDRO_SENSOR_INIT_NOT_SUPPORTED;
            all_sensors_initialized = false;
            break;
        }
        // give mutex
        xSemaphoreGive(sensor->sensor_mutex);
    }

    if (!all_sensors_initialized)
    {
        ESP_LOGE(TAG, "Not all sensors initialized successfully. Some sensors may not work.");
        return ESP_ERR_NOT_FINISHED;
    }

    return ESP_OK;
}

//==================================
//========= SENSORS READ ===========
//==================================

esp_err_t read_sensor(const char *TAG, hydro_sensor_t *sensor, hydro_data_t *output_data)
{
    esp_err_t err;

    if (sensor->init_status != HYDRO_SENSOR_INIT_SUCCESS)
    {
        ESP_LOGE(TAG, "Sensor %s (%s) not initialized successfully - can't read", HYDRO_SENSOR_MODEL_STR[sensor->model],
                 sensor->description);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGD(TAG, "Reading sensor %d - taking mutex", sensor->model);

    // take mutex with timeout WAIT_FOR_SENSOR_AVAILABILITY_TIMEOUT_MS
    if (xSemaphoreTake(sensor->sensor_mutex, pdMS_TO_TICKS(WAIT_FOR_SENSOR_AVAILABILITY_TIMEOUT_MS)) != pdTRUE)
    {
        ESP_LOGE(TAG, "Error taking sensor mutex");
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGD(TAG, "Reading sensor %d", sensor->model);

    switch (sensor->model)
    {
    case SENSOR_MODEL_AHT20:
        output_data->type = HYDRO_DATA_TYPE_TEMP_HUM;
        err = aht_get_data(&sensor->sensor_obj.aht, &output_data->data.temp_hum.temperature_c,
                           &output_data->data.temp_hum.humidity);
        break;
    case SENSOR_MODEL_BME280:
        output_data->type = HYDRO_DATA_TYPE_TEMP_HUM_PRESS;
        err = bmp280_read_float(&sensor->sensor_obj.bmp280, &output_data->data.temp_hum_press.temperature_c,
                                &output_data->data.temp_hum_press.pressure_pa,
                                &output_data->data.temp_hum_press.humidity);
        break;
    case SENSOR_MODEL_BME680:
        output_data->type = HYDRO_DATA_TYPE_TEMP_HUM_PRESS_GAS;
        bme680_values_float_t values;
        err = bme680_measure_float(&sensor->sensor_obj.bme680, &values);
        if (err != ESP_OK) break;
        output_data->data.temp_hum_press_gas.temperature_c = values.temperature;
        output_data->data.temp_hum_press_gas.pressure_pa = values.pressure; // TODO: hPa?
        output_data->data.temp_hum_press_gas.humidity = values.humidity;
        output_data->data.temp_hum_press_gas.gas_resistance_ohm = values.gas_resistance;
        break;
    case SENSOR_MODEL_TSL2591:
        output_data->type = HYDRO_DATA_TYPE_LUX;
        err = tsl2591_get_lux(&sensor->sensor_obj.tsl2591, &output_data->data.lux.lux);
        break;
    case SENSOR_MODEL_GROVE_WATER_LEVEL:
        output_data->type = HYDRO_DATA_TYPE_WATER_LEVEL;
        err = grove_water_level_sensor_get_water_level(
            &sensor->sensor_obj.grove_water_level); // TODO fix this function to return value
        if (err != ESP_OK) break;
        output_data->data.water_level.water_level = sensor->sensor_obj.grove_water_level.water_level;
        break;
    case SENSOR_MODEL_ULTRASONIC_WATER_LEVEL:
        output_data->type = HYDRO_DATA_TYPE_WATER_LEVEL;
        // Ultrasonic sensor reading
        float distance_m;
        float distance_cm;
        err = ultrasonic_measure(&sensor->sensor_obj.ultrasonic, 1, &distance_m);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Error reading ultrasonic sensor: %s", esp_err_to_name(err));
            break;
        }
        distance_cm = distance_m * 100; // Convert meters to centimeters

        ESP_LOGD(TAG, "Ultrasonic sensor distance: %.2f cm", distance_cm);
        // Convert distance to water level in mm
        if (distance_cm > HYDRO_ULTRASONIC_WATER_LEVEL_SENSOR_HEIGHT_CM)
        {
            output_data->data.water_level.water_level = 0; // No water detected
        }
        else
        {
            output_data->data.water_level.water_level =
                (uint8_t)((HYDRO_ULTRASONIC_WATER_LEVEL_SENSOR_HEIGHT_CM - distance_cm) * 10.0);
        }
        break;
    default:
        ESP_LOGE("read_sensor", "Unknown sensor model: %d", sensor->model);
        err = ESP_ERR_NOT_SUPPORTED;
    }

    // give mutex
    xSemaphoreGive(sensor->sensor_mutex);

    ESP_LOGD(TAG, "Reading sensor %d - given mutex", sensor->model);

    return err;
}
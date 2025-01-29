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

//==================================
//========= SENSORS INIT ===========
//==================================

// TODO: change cases to functions
esp_err_t init_all_sensors(const char *TAG, hydro_sensor_t *sensors, size_t sensor_count)
{
    for (size_t i = 0; i < sensor_count; i++)
    {
        hydro_sensor_t *sensor = &sensors[i];
        gpio_num_t sda;
        gpio_num_t scl;
        switch (sensor->model)
        {
        case SENSOR_MODEL_AHT20:
            sda = sensor->interface.i2c.port == 0 ? HYDRO_PINOUT_I2C0_SDA : HYDRO_PINOUT_I2C1_SDA;
            scl = sensor->interface.i2c.port == 0 ? HYDRO_PINOUT_I2C0_SCL : HYDRO_PINOUT_I2C1_SCL;
            ESP_ERROR_CHECK(aht_init_desc(&sensor->sensor_obj.aht, sensor->interface.i2c.addr,
                                          sensor->interface.i2c.port, sda, scl));
            ESP_ERROR_CHECK(aht_init(&sensor->sensor_obj.aht));
            // bool calibrated;
            // ESP_ERROR_CHECK(aht_get_status(&dev, NULL, &calibrated));
            // if (calibrated)
            //     ESP_LOGI(TAG, "Sensor calibrated");
            // else
            //     ESP_LOGW(TAG, "Sensor not calibrated!");
            break;
        case SENSOR_MODEL_BME280:
            sda = sensor->interface.i2c.port == 0 ? HYDRO_PINOUT_I2C0_SDA : HYDRO_PINOUT_I2C1_SDA;
            scl = sensor->interface.i2c.port == 0 ? HYDRO_PINOUT_I2C0_SCL : HYDRO_PINOUT_I2C1_SCL;
            ESP_ERROR_CHECK(bmp280_init_desc(&sensor->sensor_obj.bmp280, sensor->interface.i2c.addr,
                                             sensor->interface.i2c.port, sda, scl));
            bmp280_params_t params;
            bmp280_init_default_params(&params);
            ESP_ERROR_CHECK(bmp280_init(&sensor->sensor_obj.bmp280, &params));
            bool bme280p = sensor->sensor_obj.bmp280.id == BME280_CHIP_ID;
            ESP_LOGI(TAG, "BMP280: found %s", bme280p ? "BME280" : "BMP280");
            break;
        case SENSOR_MODEL_BME680:
            sda = sensor->interface.i2c.port == 0 ? HYDRO_PINOUT_I2C0_SDA : HYDRO_PINOUT_I2C1_SDA;
            scl = sensor->interface.i2c.port == 0 ? HYDRO_PINOUT_I2C0_SCL : HYDRO_PINOUT_I2C1_SCL;
            ESP_ERROR_CHECK(bme680_init_desc(&sensor->sensor_obj.bme680, sensor->interface.i2c.addr,
                                             sensor->interface.i2c.port, sda, scl));
            ESP_ERROR_CHECK(bme680_init_sensor(&sensor->sensor_obj.bme680));
            ESP_ERROR_CHECK(bme680_set_oversampling_rates(&sensor->sensor_obj.bme680, BME680_OSR_4X, BME680_OSR_NONE,
                                                          BME680_OSR_2X));
            ESP_ERROR_CHECK(bme680_set_filter_size(&sensor->sensor_obj.bme680, BME680_IIR_SIZE_7));
            // Change the heater profile 0 to 200 degree Celsius for 100 ms.
            ESP_ERROR_CHECK(bme680_set_heater_profile(&sensor->sensor_obj.bme680, 0, 200, 100));
            ESP_ERROR_CHECK(bme680_use_heater_profile(&sensor->sensor_obj.bme680, 0));
            // Set ambient temperature to 10 degree Celsius
            ESP_ERROR_CHECK(bme680_set_ambient_temperature(&sensor->sensor_obj.bme680, 10));
            break;
        case SENSOR_MODEL_TSL2591:
            sda = sensor->interface.i2c.port == 0 ? HYDRO_PINOUT_I2C0_SDA : HYDRO_PINOUT_I2C1_SDA;
            scl = sensor->interface.i2c.port == 0 ? HYDRO_PINOUT_I2C0_SCL : HYDRO_PINOUT_I2C1_SCL;
            ESP_ERROR_CHECK(tsl2591_init_desc(&sensor->sensor_obj.tsl2591, sensor->interface.i2c.port, sda, scl));
            ESP_ERROR_CHECK(tsl2591_init(&sensor->sensor_obj.tsl2591));
            // Turn TSL2591 on
            ESP_ERROR_CHECK(tsl2591_set_power_status(&sensor->sensor_obj.tsl2591, TSL2591_POWER_ON));
            // Turn ALS on
            ESP_ERROR_CHECK(tsl2591_set_als_status(&sensor->sensor_obj.tsl2591, TSL2591_ALS_ON));
            // Set gain
            ESP_ERROR_CHECK(tsl2591_set_gain(&sensor->sensor_obj.tsl2591, TSL2591_GAIN_MEDIUM));
            // Set integration time = 300ms
            ESP_ERROR_CHECK(tsl2591_set_integration_time(&sensor->sensor_obj.tsl2591, TSL2591_INTEGRATION_300MS));
            break;
        case SENSOR_MODEL_GROVE_WATER_LEVEL:
            sda = sensor->interface.i2c.port == 0 ? HYDRO_PINOUT_I2C0_SDA : HYDRO_PINOUT_I2C1_SDA;
            scl = sensor->interface.i2c.port == 0 ? HYDRO_PINOUT_I2C0_SCL : HYDRO_PINOUT_I2C1_SCL;
            ESP_ERROR_CHECK(grove_water_level_sensor_init_desc(&sensor->sensor_obj.grove_water_level,
                                                               sensor->interface.i2c.port, sda, scl));
            ESP_ERROR_CHECK(grove_water_level_sensor_init(&sensor->sensor_obj.grove_water_level));
            break;
        default:
            ESP_LOGE("init_all_sensors", "Unknown sensor model: %d", sensor->model);
            return ESP_ERR_NOT_SUPPORTED;
        }
    }
    return ESP_OK;
}

//==================================
//========= SENSORS READ ===========
//==================================

esp_err_t read_sensor(const char *TAG, hydro_sensor_t *sensor, hydro_data_t *output_data)
{
    esp_err_t err;
    switch (sensor->model)
    {
    case SENSOR_MODEL_AHT20:
        output_data->type = HYDRO_DATA_TYPE_TEMP_HUM;
        return aht_get_data(&sensor->sensor_obj.aht, &output_data->data.temp_hum.temperature_c,
                            &output_data->data.temp_hum.humidity);
    case SENSOR_MODEL_BME280:
        output_data->type = HYDRO_DATA_TYPE_TEMP_HUM_PRESS;
        return bmp280_read_float(&sensor->sensor_obj.bmp280, &output_data->data.temp_hum_press.temperature_c,
                                 &output_data->data.temp_hum_press.pressure_pa,
                                 &output_data->data.temp_hum_press.humidity);
    case SENSOR_MODEL_BME680:
        output_data->type = HYDRO_DATA_TYPE_TEMP_HUM_PRESS_GAS;
        bme680_values_float_t values;
        err = bme680_measure_float(&sensor->sensor_obj.bme680, &values);
        if (err != ESP_OK) return err;
        output_data->data.temp_hum_press_gas.temperature_c = values.temperature;
        output_data->data.temp_hum_press_gas.pressure_pa = values.pressure; // TODO: hPa?
        output_data->data.temp_hum_press_gas.humidity = values.humidity;
        output_data->data.temp_hum_press_gas.gas_resistance_ohm = values.gas_resistance;
        return ESP_OK;
    case SENSOR_MODEL_TSL2591:
        output_data->type = HYDRO_DATA_TYPE_LUX;
        return tsl2591_get_lux(&sensor->sensor_obj.tsl2591, &output_data->data.lux.lux);
    case SENSOR_MODEL_GROVE_WATER_LEVEL:
        output_data->type = HYDRO_DATA_TYPE_WATER_LEVEL;
        err = grove_water_level_sensor_get_water_level(
            &sensor->sensor_obj.grove_water_level); // TODO fix this function to return value
        if (err != ESP_OK) return err;
        output_data->data.water_level.water_level = sensor->sensor_obj.grove_water_level.water_level;
        return ESP_OK;
    default:
        ESP_LOGE("read_sensor", "Unknown sensor model: %d", sensor->model);
        return ESP_ERR_NOT_SUPPORTED;
    }
}
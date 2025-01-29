#ifndef __HYDRO_SENSORS_H__
#define __HYDRO_SENSORS_H__

// sensor drivers
#include "grove_water_level_sensor.h"
#include <aht.h>
#include <bme680.h>
#include <bmp280.h>
#include <tsl2591.h>

typedef enum
{
    SENSOR_MODEL_AHT20,
    SENSOR_MODEL_BME280,
    SENSOR_MODEL_BME680,
    SENSOR_MODEL_TSL2591,
    SENSOR_MODEL_GROVE_WATER_LEVEL
} hydro_sensor_model_t;

typedef struct
{
    hydro_sensor_model_t model;
    union
    {
        aht_t aht;
        bmp280_t bmp280;
        bme680_t bme680;
        tsl2591_t tsl2591;
        grove_water_level_sensor_t grove_water_level;
    } sensor_obj;
    union
    {
        struct
        {
            uint8_t addr;
            i2c_port_t port;
        } i2c;

        // Add other interfaces here eg. SPI, analog, etc.
    } interface;
    SemaphoreHandle_t sensor_mutex;
} hydro_sensor_t;

// =========== SENSOR DATA ===========

typedef enum
{
    HYDRO_DATA_TYPE_TEMP_HUM,
    HYDRO_DATA_TYPE_TEMP_HUM_PRESS,
    HYDRO_DATA_TYPE_TEMP_HUM_PRESS_GAS,
    HYDRO_DATA_TYPE_LUX,
    HYDRO_DATA_TYPE_WATER_LEVEL
} hydro_data_type_t;

typedef struct
{
    float temperature_c;
    float humidity;
} hydro_data_temp_hum_t;

typedef struct
{
    float temperature_c;
    float humidity;
    float pressure_pa;
} hydro_data_temp_hum_press_t;

typedef struct
{
    float temperature_c;
    float humidity;
    float pressure_pa;
    float gas_resistance_ohm;
} hydro_data_temp_hum_press_gas_t;

typedef struct
{
    float lux;
} hydro_data_lux_t;

typedef struct
{
    uint8_t water_level; // 0-100 %
} hydro_data_water_level_t;

typedef struct
{
    hydro_data_type_t type;
    union
    {
        hydro_data_temp_hum_t temp_hum;
        hydro_data_temp_hum_press_t temp_hum_press;
        hydro_data_temp_hum_press_gas_t temp_hum_press_gas;
        hydro_data_lux_t lux;
        hydro_data_water_level_t water_level;
    } data;
} hydro_data_t;

//==================================
//========= PUBLIC FUNCTIONS =======
//==================================

esp_err_t init_all_sensors(const char *TAG, hydro_sensor_t *sensors, size_t sensor_count);

esp_err_t read_sensor(const char *TAG, hydro_sensor_t *sensor, hydro_data_t *output_data);

#endif // __HYDRO_SENSORS_H__
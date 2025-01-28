#ifndef __GROVE_WATER_LEVEL_SENSOR_H__
#define __GROVE_WATER_LEVEL_SENSOR_H__

#include <i2cdev.h>
#include <stdio.h>

/**
 * Device descriptor.
 */
typedef struct
{
    i2c_dev_t i2c_dev_high;
    i2c_dev_t i2c_dev_low;
    uint8_t water_level; // 0-100
} grove_water_level_sensor_t;

esp_err_t grove_water_level_sensor_init_desc(grove_water_level_sensor_t *dev, i2c_port_t port, gpio_num_t sda_gpio,
                                             gpio_num_t scl_gpio);

esp_err_t grove_water_level_sensor_free_desc(grove_water_level_sensor_t *dev);

esp_err_t grove_water_level_sensor_init(grove_water_level_sensor_t *dev);

esp_err_t grove_water_level_sensor_get_water_level(grove_water_level_sensor_t *dev);

#endif // __GROVE_WATER_LEVEL_SENSOR_H__
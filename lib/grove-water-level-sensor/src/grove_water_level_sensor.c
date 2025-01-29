#include "grove_water_level_sensor.h"

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2cdev.h"
#include <stdio.h>
#include <string.h>

#include <esp_idf_lib_helpers.h>

#define I2C_FREQ_HZ 400000 // 400kHz
static const char *TAG = "GroveWaterLevelSensor";

// ADDRESS
#define ATTINY1_HIGH_ADDR 0x78
#define ATTINY2_LOW_ADDR  0x77

#define THRESHOLD 100

// Funkcja do odczytu danych z magistrali I2C
static inline esp_err_t read_i2c_data(i2c_dev_t *dev, uint8_t *value, size_t len)
{
    esp_err_t err;
    err = i2c_dev_read_reg(dev, 0x00, value, len);
    return err;
}

static inline esp_err_t get_high_12_section_value(grove_water_level_sensor_t *dev, uint8_t *high_data)
{
    memset(high_data, 0, 12);
    if (read_i2c_data(&dev->i2c_dev_high, high_data, 12) == ESP_OK)
    {
        ESP_LOGD(TAG, "High section data read successfully");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to read high section data");
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

static inline esp_err_t get_low_8_section_value(grove_water_level_sensor_t *dev, uint8_t *low_data)
{
    memset(low_data, 0, 8);
    if (read_i2c_data(&dev->i2c_dev_low, low_data, 8) == ESP_OK)
    {
        ESP_LOGD(TAG, "Low section data read successfully");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to read low section data");
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

esp_err_t grove_water_level_sensor_init_desc(grove_water_level_sensor_t *dev, i2c_port_t port, gpio_num_t sda_gpio,
                                             gpio_num_t scl_gpio)
{
    // CHECK_ARG(dev);
    ESP_LOGD(TAG, "Initialize descriptor");

    dev->i2c_dev_high.port = port;
    dev->i2c_dev_high.addr = ATTINY1_HIGH_ADDR;
    dev->i2c_dev_high.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev_high.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev_high.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    dev->i2c_dev_low.port = port;
    dev->i2c_dev_low.addr = ATTINY2_LOW_ADDR;
    dev->i2c_dev_low.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev_low.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev_low.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    esp_err_t err = i2c_dev_create_mutex(&dev->i2c_dev_high);
    if (err != ESP_OK)
    {
        return err;
    }
    return i2c_dev_create_mutex(&dev->i2c_dev_low);
}

esp_err_t grove_water_level_sensor_free_desc(grove_water_level_sensor_t *dev)
{
    // CHECK_ARG(dev);
    ESP_LOGD(TAG, "Free descriptor.");

    esp_err_t err = i2c_dev_delete_mutex(&dev->i2c_dev_high);
    if (err != ESP_OK)
    {
        return err;
    }
    return i2c_dev_delete_mutex(&dev->i2c_dev_low);
}

esp_err_t grove_water_level_sensor_init(grove_water_level_sensor_t *dev)
{
    // CHECK_ARG(dev);
    ESP_LOGD(TAG, "Initialize sensor.");

    uint8_t high_data[12];
    uint8_t low_data[8];

    ESP_ERROR_CHECK(get_high_12_section_value(dev, high_data));
    ESP_ERROR_CHECK(get_low_8_section_value(dev, low_data));

    return ESP_OK;
}

esp_err_t grove_water_level_sensor_get_water_level(grove_water_level_sensor_t *dev)
{
    // CHECK_ARG(dev);
    ESP_LOGD(TAG, "Get water level.");

    uint8_t high_data[12];
    uint8_t low_data[8];

    uint32_t touch_val = 0;
    uint8_t trig_section = 0;

    esp_err_t err = get_high_12_section_value(dev, high_data);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get high section value");
        return err;
    }

    err = get_low_8_section_value(dev, low_data);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get low section value");
        return err;
    }

    ESP_LOG_BUFFER_HEX_LEVEL(TAG, high_data, 12, ESP_LOG_DEBUG);
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, low_data, 8, ESP_LOG_DEBUG);

    for (int i = 0; i < 8; i++)
    {
        if (low_data[i] > THRESHOLD)
        {
            touch_val |= 1 << i;
        }
    }
    for (int i = 0; i < 12; i++)
    {
        if (high_data[i] > THRESHOLD)
        {
            touch_val |= (uint32_t)1 << (8 + i);
        }
    }

    while (touch_val & 0x01)
    {
        trig_section++;
        touch_val >>= 1;
    }

    dev->water_level = trig_section * 5;

    return ESP_OK;
}
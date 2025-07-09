#include "hydro_pump.h"

#include <esp_err.h>
#include <esp_log.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>

#define PWM_MAX_DUTY 4000 // 8191

bool pump_slow_start(ledc_channel_config_t *ledc_channel)
{
    bool noerror = true;
    esp_err_t err = ESP_OK;

    for (int i = 0; i < PWM_MAX_DUTY; i += PWM_MAX_DUTY / 10)
    {
        if (i > PWM_MAX_DUTY) i = PWM_MAX_DUTY;

        err = ledc_set_duty(ledc_channel->speed_mode, ledc_channel->channel, i);
        if (err != ESP_OK)
        {
            ESP_LOGE("PUMP SLOW START", "Error setting duty: %d", err);
            noerror = false;
            break;
        }

        err = ledc_update_duty(ledc_channel->speed_mode, ledc_channel->channel);
        if (err != ESP_OK)
        {
            ESP_LOGE("PUMP SLOW START", "Error updating duty: %d", err);
            noerror = false;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    return noerror;
}

bool pump_slow_stop(ledc_channel_config_t *ledc_channel)
{
    bool noerror = true;
    esp_err_t err = ESP_OK;

    for (int i = PWM_MAX_DUTY; i >= 0; i -= PWM_MAX_DUTY / 10)
    {
        if (i < 0) i = 0;

        err = ledc_set_duty(ledc_channel->speed_mode, ledc_channel->channel, i);
        if (err != ESP_OK)
        {
            ESP_LOGE("PUMP SLOW STOP", "Error setting duty: %d", err);
            noerror = false;
            break;
        }

        err = ledc_update_duty(ledc_channel->speed_mode, ledc_channel->channel);
        if (err != ESP_OK)
        {
            ESP_LOGE("PUMP SLOW STOP", "Error updating duty: %d", err);
            noerror = false;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    return noerror;
}
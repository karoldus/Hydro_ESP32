#ifndef __HYDRO_PUMP_H__
#define __HYDRO_PUMP_H__

#include "driver/gpio.h"
#include "driver/ledc.h"

bool pump_slow_start(ledc_channel_config_t *ledc_channel);
bool pump_slow_stop(ledc_channel_config_t *ledc_channel);

#endif // __HYDRO_PUMP_H__
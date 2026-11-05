#pragma once

#include <stdint.h>

#include "esp_err.h"

namespace motor_output
{
    esp_err_t init_safe();
    esp_err_t set_pulse_us(uint32_t pulse_width_us);
    esp_err_t set_safe();
}

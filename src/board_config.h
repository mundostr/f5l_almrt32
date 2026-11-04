#pragma once

#include "driver/gpio.h"

namespace board
{
    constexpr gpio_num_t BMP280_SDA_GPIO = GPIO_NUM_6;
    constexpr gpio_num_t BMP280_SCL_GPIO = GPIO_NUM_7;
    constexpr gpio_num_t RECEIVER_SIGNAL_GPIO = GPIO_NUM_4;
    constexpr gpio_num_t ESC_SIGNAL_GPIO = GPIO_NUM_5;
}

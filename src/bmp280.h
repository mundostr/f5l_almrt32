#pragma once

#include <stdint.h>

#include "esp_err.h"

namespace bmp280
{
    constexpr uint32_t PRESSURE_READING_INTERVAL_MS = 100;
    constexpr uint32_t GROUND_REFERENCE_MAX_INIT_TIME_MS = 2000;
    constexpr uint8_t GROUND_REFERENCE_SAMPLE_COUNT = 15;
    constexpr float MOTOR_CUTOFF_ALTITUDE_M = 90.0F;

    esp_err_t start_ground_reference();
    esp_err_t start_altitude_monitoring();
    esp_err_t stop_altitude_monitoring();

    }

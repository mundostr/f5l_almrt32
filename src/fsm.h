#pragma once

#include "esp_err.h"

namespace fsm
{
    enum class Event : unsigned char
    {
        SYSTEM_STARTED,
        ARMING_DELAY_ELAPSED,
        GROUND_REFERENCE_CAPTURED,
        GROUND_REFERENCE_FAILED,
        MOTOR_START_DETECTED,
        MANUAL_MOTOR_STOP,
        ALTITUDE_LIMIT_REACHED,
        MOTOR_TIME_LIMIT_REACHED
    };

    esp_err_t start();
    esp_err_t post_event(Event event, float value = 0.0F);
    bool get_ground_reference_pressure_pa(float *pressure_pa);
}

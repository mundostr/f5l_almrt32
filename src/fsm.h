#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

namespace fsm
{
    enum class Event : unsigned char
    {
        SYSTEM_STARTED,
        ARMING_DELAY_ELAPSED,
        GROUND_REFERENCE_CAPTURED,
        GROUND_REFERENCE_FAILED,
        RECEIVER_PULSE_RECEIVED,
        ALTITUDE_LIMIT_REACHED,
        MOTOR_TIME_LIMIT_REACHED,
        ALTITUDE_MONITOR_FAILED
    };

    esp_err_t start();
    esp_err_t post_event(Event event, float value = 0.0F);
    bool post_receiver_pulse_from_isr(
        uint32_t pulse_width_us,
        int64_t pulse_start_time_us,
        BaseType_t *higher_priority_task_woken);
    bool get_ground_reference_pressure_pa(float *pressure_pa);
}

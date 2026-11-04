#include "fsm.h"

#include "bmp280.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/timers.h"

namespace fsm
{
    constexpr char TAG[] = "almrt_fsm";
    constexpr UBaseType_t EVENT_QUEUE_LENGTH = 8;
    constexpr uint32_t FSM_TASK_STACK_SIZE = 3072;
    constexpr UBaseType_t FSM_TASK_PRIORITY = 5;
    constexpr uint32_t THROTTLE_GUARD_DURATION_MS = 2000;

    enum class State : unsigned char
    {
        BOOT,
        THROTTLE_GUARD,
        GROUND_REFERENCE,
        READY,
        MOTOR_RUNNING,
        LOCKED,
        FAULT
    };

    struct EventMessage
    {
        Event event;
        float value;
    };

    static QueueHandle_t event_queue = nullptr;
    static TimerHandle_t throttle_guard_timer = nullptr;
    static State current_state = State::BOOT;
    static portMUX_TYPE ground_reference_mux = portMUX_INITIALIZER_UNLOCKED;
    static float ground_reference_pressure_pa = 0.0F;
    static bool ground_reference_valid = false;
    static bool throttle_guard_elapsed = false;

    static const char *state_name(State state)
    {
        switch (state)
        {
        case State::BOOT:
            return "BOOT";
        case State::THROTTLE_GUARD:
            return "THROTTLE_GUARD";
        case State::GROUND_REFERENCE:
            return "GROUND_REFERENCE";
        case State::READY:
            return "READY";
        case State::MOTOR_RUNNING:
            return "MOTOR_RUNNING";
        case State::LOCKED:
            return "LOCKED";
        case State::FAULT:
            return "FAULT";
        }

        return "UNKNOWN";
    }

    static void enter_state(State next_state)
    {
        current_state = next_state;
        ESP_LOGI(TAG, "Entered state %s", state_name(current_state));

        if (next_state == State::THROTTLE_GUARD)
        {
            if (xTimerStart(throttle_guard_timer, 0) != pdPASS)
            {
                current_state = State::FAULT;
                ESP_LOGE(TAG, "Failed to start throttle guard timer; entering FAULT");
                return;
            }

            const esp_err_t error = bmp280::start_ground_reference();
            if (error != ESP_OK)
            {
                current_state = State::FAULT;
                ESP_LOGE(TAG, "Failed to start BMP280 capture: %s", esp_err_to_name(error));
                return;
            }

            ESP_LOGI(TAG, "Device armed; throttle start is ignored for 2 seconds");
        }
    }

    static void process_event(const EventMessage &message)
    {
        switch (current_state)
        {
        case State::BOOT:
            if (message.event == Event::SYSTEM_STARTED)
            {
                enter_state(State::THROTTLE_GUARD);
                return;
            }
            break;

        case State::THROTTLE_GUARD:
            if (message.event == Event::ARMING_DELAY_ELAPSED)
            {
                throttle_guard_elapsed = true;
                enter_state(ground_reference_valid ? State::READY : State::GROUND_REFERENCE);
                return;
            }
            if (message.event == Event::GROUND_REFERENCE_CAPTURED)
            {
                portENTER_CRITICAL(&ground_reference_mux);
                ground_reference_pressure_pa = message.value;
                ground_reference_valid = true;
                portEXIT_CRITICAL(&ground_reference_mux);
                return;
            }
            if (message.event == Event::GROUND_REFERENCE_FAILED)
            {
                enter_state(State::FAULT);
                return;
            }
            break;

        case State::GROUND_REFERENCE:
            if (message.event == Event::GROUND_REFERENCE_CAPTURED)
            {
                portENTER_CRITICAL(&ground_reference_mux);
                ground_reference_pressure_pa = message.value;
                ground_reference_valid = true;
                portEXIT_CRITICAL(&ground_reference_mux);
                enter_state(State::READY);
                return;
            }
            if (message.event == Event::GROUND_REFERENCE_FAILED)
            {
                enter_state(State::FAULT);
                return;
            }
            break;

        case State::READY:
            if (message.event == Event::MOTOR_START_DETECTED)
            {
                enter_state(State::MOTOR_RUNNING);
                return;
            }
            break;

        case State::MOTOR_RUNNING:
            if (message.event == Event::MANUAL_MOTOR_STOP || message.event == Event::ALTITUDE_LIMIT_REACHED || message.event == Event::MOTOR_TIME_LIMIT_REACHED)
            {
                enter_state(State::LOCKED);
                return;
            }
            break;

        case State::LOCKED:
        case State::FAULT:
            break;
        }

        ESP_LOGW(TAG, "Ignoring event %u in state %s",
                 static_cast<unsigned int>(message.event), state_name(current_state));
    }

    static void fsm_task(void *)
    {
        EventMessage message = {};
        for (;;)
        {
            if (xQueueReceive(event_queue, &message, portMAX_DELAY) == pdPASS)
            {
                process_event(message);
            }
        }
    }

    static void throttle_guard_timer_callback(TimerHandle_t)
    {
        const esp_err_t error = post_event(Event::ARMING_DELAY_ELAPSED);
        if (error != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to post throttle-guard completion: %s", esp_err_to_name(error));
        }
    }

    esp_err_t start()
    {
        if (event_queue != nullptr)
        {
            return ESP_ERR_INVALID_STATE;
        }

        event_queue = xQueueCreate(EVENT_QUEUE_LENGTH, sizeof(EventMessage));
        if (event_queue == nullptr)
        {
            return ESP_ERR_NO_MEM;
        }

        throttle_guard_timer = xTimerCreate(
            "throttle_guard",
            pdMS_TO_TICKS(THROTTLE_GUARD_DURATION_MS),
            pdFALSE,
            nullptr,
            throttle_guard_timer_callback);
        if (throttle_guard_timer == nullptr)
        {
            vQueueDelete(event_queue);
            event_queue = nullptr;
            return ESP_ERR_NO_MEM;
        }

        if (xTaskCreate(fsm_task, "almrt_fsm", FSM_TASK_STACK_SIZE, nullptr,
                        FSM_TASK_PRIORITY, nullptr) != pdPASS)
        {
            xTimerDelete(throttle_guard_timer, 0);
            throttle_guard_timer = nullptr;
            vQueueDelete(event_queue);
            event_queue = nullptr;
            return ESP_ERR_NO_MEM;
        }

        const esp_err_t error = post_event(Event::SYSTEM_STARTED);
        if (error != ESP_OK)
        {
            return error;
        }

        return ESP_OK;
    }

    esp_err_t post_event(Event event, float value)
    {
        if (event_queue == nullptr)
        {
            return ESP_ERR_INVALID_STATE;
        }

        const EventMessage message = {event, value};
        return xQueueSend(event_queue, &message, 0) == pdPASS ? ESP_OK : ESP_ERR_TIMEOUT;
    }

    bool get_ground_reference_pressure_pa(float *pressure_pa)
    {
        if (pressure_pa == nullptr)
        {
            return false;
        }

        portENTER_CRITICAL(&ground_reference_mux);
        const bool valid = ground_reference_valid;
        if (valid)
        {
            *pressure_pa = ground_reference_pressure_pa;
        }
        portEXIT_CRITICAL(&ground_reference_mux);

        return valid;
    }
}

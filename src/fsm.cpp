#include "fsm.h"

#include "bmp280.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "motor_output.h"

namespace fsm
{

constexpr char TAG[] = "almrt_fsm";
constexpr UBaseType_t EVENT_QUEUE_LENGTH = 16;
constexpr uint32_t FSM_TASK_STACK_SIZE = 4096;
constexpr UBaseType_t FSM_TASK_PRIORITY = 5;
constexpr uint32_t THROTTLE_GUARD_DURATION_MS = 2000;
constexpr uint32_t MOTOR_RUN_LIMIT_MS = 30000;
constexpr uint32_t MOTOR_START_THRESHOLD_US = 1200;

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
    float pressure_pa;
    uint32_t pulse_width_us;
    int64_t pulse_start_time_us;
};

static QueueHandle_t event_queue = nullptr;
static TimerHandle_t throttle_guard_timer = nullptr;
static TimerHandle_t motor_run_timer = nullptr;
static State current_state = State::BOOT;
static portMUX_TYPE ground_reference_mux = portMUX_INITIALIZER_UNLOCKED;
static float ground_reference_pressure_pa = 0.0F;
static bool ground_reference_valid = false;
static int64_t ready_since_time_us = 0;

static const char *state_name(State state)
{
    switch (state)
    {
    case State::BOOT: return "BOOT";
    case State::THROTTLE_GUARD: return "THROTTLE_GUARD";
    case State::GROUND_REFERENCE: return "GROUND_REFERENCE";
    case State::READY: return "READY";
    case State::MOTOR_RUNNING: return "MOTOR_RUNNING";
    case State::LOCKED: return "LOCKED";
    case State::FAULT: return "FAULT";
    }

    return "UNKNOWN";
}

static void enter_state(State next_state)
{
    current_state = next_state;
    if (next_state == State::READY)
    {
        ready_since_time_us = esp_timer_get_time();
    }
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

static void stop_motor_and_lock(bool sensor_fault)
{
    const esp_err_t output_error = motor_output::set_safe();
    if (output_error != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to apply safe ESC output: %s", esp_err_to_name(output_error));
        enter_state(State::FAULT);
        return;
    }

    if (xTimerStop(motor_run_timer, 0) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to stop motor-run timer");
    }

    const esp_err_t sensor_error = bmp280::stop_altitude_monitoring();
    if (sensor_error != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to stop altitude monitoring: %s", esp_err_to_name(sensor_error));
    }

    enter_state(sensor_fault ? State::FAULT : State::LOCKED);
}

static void enter_fault_with_safe_output()
{
    const esp_err_t error = motor_output::set_safe();
    if (error != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to apply safe ESC output while entering FAULT: %s", esp_err_to_name(error));
    }

    if (motor_run_timer != nullptr && xTimerStop(motor_run_timer, 0) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to stop motor-run timer while entering FAULT");
    }

    enter_state(State::FAULT);
}

static void handle_receiver_pulse(uint32_t pulse_width_us, int64_t pulse_start_time_us)
{
    if (current_state == State::READY)
    {
        if (pulse_start_time_us < ready_since_time_us)
        {
            return;
        }

        const esp_err_t output_error = motor_output::set_pulse_us(pulse_width_us);
        if (output_error != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to relay receiver pulse: %s", esp_err_to_name(output_error));
            enter_fault_with_safe_output();
            return;
        }

        if (pulse_width_us > MOTOR_START_THRESHOLD_US)
        {
            if (xTimerStart(motor_run_timer, 0) != pdPASS)
            {
                ESP_LOGE(TAG, "Failed to start motor-run timer");
                enter_fault_with_safe_output();
                return;
            }

            const esp_err_t sensor_error = bmp280::start_altitude_monitoring();
            if (sensor_error != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to start altitude monitoring: %s", esp_err_to_name(sensor_error));
                enter_fault_with_safe_output();
                return;
            }

            enter_state(State::MOTOR_RUNNING);
        }
        return;
    }

    if (current_state != State::MOTOR_RUNNING)
    {
        return;
    }

    if (pulse_width_us < MOTOR_START_THRESHOLD_US)
    {
        stop_motor_and_lock(false);
        return;
    }

    const esp_err_t error = motor_output::set_pulse_us(pulse_width_us);
    if (error != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to relay receiver pulse: %s", esp_err_to_name(error));
        stop_motor_and_lock(true);
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
            enter_state(ground_reference_valid ? State::READY : State::GROUND_REFERENCE);
            return;
        }
        if (message.event == Event::GROUND_REFERENCE_CAPTURED)
        {
            portENTER_CRITICAL(&ground_reference_mux);
            ground_reference_pressure_pa = message.pressure_pa;
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
            ground_reference_pressure_pa = message.pressure_pa;
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
    case State::MOTOR_RUNNING:
        if (message.event == Event::RECEIVER_PULSE_RECEIVED)
        {
            handle_receiver_pulse(message.pulse_width_us, message.pulse_start_time_us);
            return;
        }
        if (current_state == State::MOTOR_RUNNING
            && message.event == Event::ALTITUDE_LIMIT_REACHED)
        {
            stop_motor_and_lock(false);
            return;
        }
        if (current_state == State::MOTOR_RUNNING
            && message.event == Event::MOTOR_TIME_LIMIT_REACHED)
        {
            stop_motor_and_lock(false);
            return;
        }
        if (current_state == State::MOTOR_RUNNING
            && message.event == Event::ALTITUDE_MONITOR_FAILED)
        {
            stop_motor_and_lock(true);
            return;
        }
        break;

    case State::LOCKED:
    case State::FAULT:
        break;
    }

    if (message.event == Event::RECEIVER_PULSE_RECEIVED)
    {
        return;
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

static void motor_run_timer_callback(TimerHandle_t)
{
    const esp_err_t error = post_event(Event::MOTOR_TIME_LIMIT_REACHED);
    if (error != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to post motor time limit: %s", esp_err_to_name(error));
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
    motor_run_timer = xTimerCreate(
        "motor_run_limit",
        pdMS_TO_TICKS(MOTOR_RUN_LIMIT_MS),
        pdFALSE,
        nullptr,
        motor_run_timer_callback);
    if (throttle_guard_timer == nullptr || motor_run_timer == nullptr)
    {
        if (throttle_guard_timer != nullptr)
        {
            xTimerDelete(throttle_guard_timer, 0);
            throttle_guard_timer = nullptr;
        }
        if (motor_run_timer != nullptr)
        {
            xTimerDelete(motor_run_timer, 0);
            motor_run_timer = nullptr;
        }
        vQueueDelete(event_queue);
        event_queue = nullptr;
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreate(fsm_task, "almrt_fsm", FSM_TASK_STACK_SIZE, nullptr,
                    FSM_TASK_PRIORITY, nullptr) != pdPASS)
    {
        xTimerDelete(throttle_guard_timer, 0);
        xTimerDelete(motor_run_timer, 0);
        throttle_guard_timer = nullptr;
        motor_run_timer = nullptr;
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

    const EventMessage message = {event, value, 0, 0};
    return xQueueSend(event_queue, &message, 0) == pdPASS ? ESP_OK : ESP_ERR_TIMEOUT;
}

bool post_receiver_pulse_from_isr(
    uint32_t pulse_width_us,
    int64_t pulse_start_time_us,
    BaseType_t *higher_priority_task_woken)
{
    if (event_queue == nullptr || higher_priority_task_woken == nullptr)
    {
        return false;
    }

    const EventMessage message = {
        Event::RECEIVER_PULSE_RECEIVED,
        0.0F,
        pulse_width_us,
        pulse_start_time_us};
    return xQueueSendFromISR(event_queue, &message, higher_priority_task_woken) == pdPASS;
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

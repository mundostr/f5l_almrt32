#include "fsm.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

namespace almrt
{
    namespace fsm
    {
        namespace
        {

            const char *TAG = "almrt_fsm";
            const UBaseType_t EVENT_QUEUE_LENGTH = 8;
            const uint32_t FSM_TASK_STACK_SIZE = 3072;
            const UBaseType_t FSM_TASK_PRIORITY = 5;

            enum class State : unsigned char
            {
                BOOT,
                THROTTLE_GUARD,
                GROUND_REFERENCE,
                READY,
                MOTOR_RUNNING,
                LOCKED
            };

            QueueHandle_t event_queue = nullptr;
            State current_state = State::BOOT;

            const char *state_name(State state)
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
                }

                return "UNKNOWN";
            }

            void transition_to(State next_state)
            {
                if (current_state == next_state)
                {
                    return;
                }

                ESP_LOGI(TAG, "%s -> %s", state_name(current_state), state_name(next_state));
                current_state = next_state;
            }

            void process_event(Event event)
            {
                switch (current_state)
                {
                case State::BOOT:
                    if (event == Event::SYSTEM_STARTED)
                    {
                        transition_to(State::THROTTLE_GUARD);
                        return;
                    }
                    break;

                case State::THROTTLE_GUARD:
                    if (event == Event::ARMING_DELAY_ELAPSED)
                    {
                        transition_to(State::GROUND_REFERENCE);
                        return;
                    }
                    break;

                case State::GROUND_REFERENCE:
                    if (event == Event::GROUND_REFERENCE_CAPTURED)
                    {
                        transition_to(State::READY);
                        return;
                    }
                    break;

                case State::READY:
                    if (event == Event::MOTOR_START_DETECTED)
                    {
                        transition_to(State::MOTOR_RUNNING);
                        return;
                    }
                    break;

                case State::MOTOR_RUNNING:
                    if (event == Event::MANUAL_MOTOR_STOP || event == Event::ALTITUDE_LIMIT_REACHED || event == Event::MOTOR_TIME_LIMIT_REACHED)
                    {
                        transition_to(State::LOCKED);
                        return;
                    }
                    break;

                case State::LOCKED:
                    break;
                }

                ESP_LOGW(TAG, "Ignoring event %u in state %s",
                         static_cast<unsigned int>(event), state_name(current_state));
            }

            void fsm_task(void *)
            {
                Event event;
                for (;;)
                {
                    if (xQueueReceive(event_queue, &event, portMAX_DELAY) == pdPASS)
                    {
                        process_event(event);
                    }
                }
            }

        }

        esp_err_t start()
        {
            if (event_queue != nullptr)
            {
                return ESP_ERR_INVALID_STATE;
            }

            event_queue = xQueueCreate(EVENT_QUEUE_LENGTH, sizeof(Event));
            if (event_queue == nullptr)
            {
                return ESP_ERR_NO_MEM;
            }

            const Event initial_event = Event::SYSTEM_STARTED;
            if (xQueueSend(event_queue, &initial_event, 0) != pdPASS)
            {
                vQueueDelete(event_queue);
                event_queue = nullptr;
                return ESP_FAIL;
            }

            if (xTaskCreate(fsm_task, "almrt_fsm", FSM_TASK_STACK_SIZE, nullptr,
                            FSM_TASK_PRIORITY, nullptr) != pdPASS)
            {
                vQueueDelete(event_queue);
                event_queue = nullptr;
                return ESP_ERR_NO_MEM;
            }

            return ESP_OK;
        }

        esp_err_t post_event(Event event)
        {
            if (event_queue == nullptr)
            {
                return ESP_ERR_INVALID_STATE;
            }

            return xQueueSend(event_queue, &event, 0) == pdPASS ? ESP_OK : ESP_ERR_TIMEOUT;
        }

    }
}

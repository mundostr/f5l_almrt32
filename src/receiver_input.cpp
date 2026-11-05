#include "receiver_input.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "board_config.h"
#include "fsm.h"

namespace receiver_input
{
    constexpr uint32_t MINIMUM_VALID_PULSE_US = 800;
    constexpr uint32_t MAXIMUM_VALID_PULSE_US = 2200;

    static int64_t rising_edge_time_us = 0;
    static bool is_initialized = false;

    static void receiver_gpio_isr(void *)
    {
        const int64_t now_us = esp_timer_get_time();
        if (gpio_get_level(board::RECEIVER_SIGNAL_GPIO) != 0)
        {
            rising_edge_time_us = now_us;
            return;
        }

        if (rising_edge_time_us == 0)
        {
            return;
        }

        const int64_t pulse_start_time_us = rising_edge_time_us;
        const int64_t pulse_width_us = now_us - pulse_start_time_us;
        rising_edge_time_us = 0;
        if (pulse_width_us < MINIMUM_VALID_PULSE_US || pulse_width_us > MAXIMUM_VALID_PULSE_US)
        {
            return;
        }

        BaseType_t higher_priority_task_woken = pdFALSE;
        if (!fsm::post_receiver_pulse_from_isr(
                static_cast<uint32_t>(pulse_width_us),
                pulse_start_time_us,
                &higher_priority_task_woken))
        {
            ESP_DRAM_LOGE("rx", "FSM event queue full; receiver pulse dropped");
        }
        if (higher_priority_task_woken == pdTRUE)
        {
            portYIELD_FROM_ISR();
        }
    }

    esp_err_t init()
    {
        if (is_initialized)
        {
            return ESP_ERR_INVALID_STATE;
        }

        gpio_config_t input_config = {};
        input_config.pin_bit_mask = 1ULL << board::RECEIVER_SIGNAL_GPIO;
        input_config.mode = GPIO_MODE_INPUT;
        input_config.pull_up_en = GPIO_PULLUP_DISABLE;
        input_config.pull_down_en = GPIO_PULLDOWN_ENABLE;
        input_config.intr_type = GPIO_INTR_ANYEDGE;

        esp_err_t error = gpio_config(&input_config);
        if (error != ESP_OK)
        {
            return error;
        }

        error = gpio_install_isr_service(0);
        if (error != ESP_OK && error != ESP_ERR_INVALID_STATE)
        {
            return error;
        }

        error = gpio_isr_handler_add(board::RECEIVER_SIGNAL_GPIO, receiver_gpio_isr, nullptr);
        if (error != ESP_OK)
        {
            return error;
        }

        is_initialized = true;
        ESP_LOGI("receiver_input", "Receiver pulse capture initialized on GPIO %d", board::RECEIVER_SIGNAL_GPIO);
        return ESP_OK;
    }
}

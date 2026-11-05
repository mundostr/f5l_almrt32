#include "motor_output.h"

#include "board_config.h"
#include "driver/ledc.h"

namespace motor_output
{
    static const uint32_t PWM_FREQUENCY_HZ = 50;
    static const uint32_t PWM_PERIOD_US = 1000000 / PWM_FREQUENCY_HZ;
    static const uint32_t SAFE_PULSE_US = 900;
    static const ledc_timer_bit_t PWM_RESOLUTION = LEDC_TIMER_14_BIT;
    static const uint32_t PWM_MAX_DUTY = (1U << PWM_RESOLUTION) - 1U;
    static bool initialized = false;

    static esp_err_t update_pulse(uint32_t pulse_width_us, bool round_to_nearest)
    {
        if (!initialized)
        {
            return ESP_ERR_INVALID_STATE;
        }

        const uint32_t scaled_pulse = pulse_width_us * PWM_MAX_DUTY;
        const uint32_t duty = round_to_nearest
                                  ? (scaled_pulse + PWM_PERIOD_US / 2) / PWM_PERIOD_US
                                  : scaled_pulse / PWM_PERIOD_US;
        esp_err_t error = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
        if (error != ESP_OK)
        {
            return error;
        }

        return ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }

    esp_err_t init_safe()
    {
        ledc_timer_config_t timer_config = {};
        timer_config.speed_mode = LEDC_LOW_SPEED_MODE;
        timer_config.duty_resolution = PWM_RESOLUTION;
        timer_config.timer_num = LEDC_TIMER_0;
        timer_config.freq_hz = PWM_FREQUENCY_HZ;
        timer_config.clk_cfg = LEDC_AUTO_CLK;

        esp_err_t error = ledc_timer_config(&timer_config);
        if (error != ESP_OK)
        {
            return error;
        }

        const uint32_t safe_duty = (SAFE_PULSE_US * PWM_MAX_DUTY) / PWM_PERIOD_US;

        ledc_channel_config_t channel_config = {};
        channel_config.gpio_num = board::ESC_SIGNAL_GPIO;
        channel_config.speed_mode = LEDC_LOW_SPEED_MODE;
        channel_config.channel = LEDC_CHANNEL_0;
        channel_config.intr_type = LEDC_INTR_DISABLE;
        channel_config.timer_sel = LEDC_TIMER_0;
        channel_config.duty = safe_duty;
        channel_config.hpoint = 0;

        error = ledc_channel_config(&channel_config);
        if (error != ESP_OK)
        {
            return error;
        }

        initialized = true;
        return ESP_OK;
    }

    esp_err_t set_pulse_us(uint32_t pulse_width_us)
    {
        if (pulse_width_us < 800 || pulse_width_us > 2200)
        {
            return ESP_ERR_INVALID_ARG;
        }

        return update_pulse(pulse_width_us, true);
    }

    esp_err_t set_safe()
    {
        return update_pulse(SAFE_PULSE_US, false);
    }
}

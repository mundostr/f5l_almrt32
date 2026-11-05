#include "esp_check.h"
#include "fsm.h"
#include "motor_output.h"
#include "receiver_input.h"

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(motor_output::init_safe());
    ESP_ERROR_CHECK(fsm::start());
    ESP_ERROR_CHECK(receiver_input::init());
}

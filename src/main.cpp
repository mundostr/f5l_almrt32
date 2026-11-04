#include "esp_check.h"
#include "fsm.h"
#include "motor_output.h"

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(motor_output::init_safe());
    ESP_ERROR_CHECK(fsm::start());
}

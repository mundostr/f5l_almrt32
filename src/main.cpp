#include "fsm.h"

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(almrt::fsm::start());
}

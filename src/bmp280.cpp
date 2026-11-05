#include "bmp280.h"

#include <algorithm>
#include <array>
#include <cmath>

#include "board_config.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "fsm.h"

namespace bmp280
{
    constexpr char TAG[] = "bmp280";
    constexpr uint8_t BMP280_ADDRESS_PRIMARY = 0x76;
    constexpr uint8_t BMP280_ADDRESS_SECONDARY = 0x77;
    constexpr uint8_t BMP280_CHIP_ID = 0x58;
    constexpr uint8_t REG_CHIP_ID = 0xD0;
    constexpr uint8_t REG_RESET = 0xE0;
    constexpr uint8_t REG_CALIBRATION = 0x88;
    constexpr uint8_t REG_CTRL_MEAS = 0xF4;
    constexpr uint8_t REG_CONFIG = 0xF5;
    constexpr uint8_t REG_PRESSURE_DATA = 0xF7;
    constexpr uint32_t I2C_FREQUENCY_HZ = 400000;
    constexpr uint32_t I2C_TIMEOUT_MS = 100;
    constexpr uint32_t BMP280_TASK_STACK_SIZE = 4096;
    constexpr UBaseType_t BMP280_TASK_PRIORITY = 4;
    constexpr uint32_t BMP280_CONVERSION_DELAY_MS = 50;
    constexpr float MINIMUM_PRESSURE_PA = 70000.0F;
    constexpr float MAXIMUM_PRESSURE_PA = 110000.0F;
    constexpr double MINIMUM_SENSOR_TEMPERATURE_C = -40.0;
    constexpr double MAXIMUM_SENSOR_TEMPERATURE_C = 85.0;
    constexpr uint8_t BMP280_RESET_COMMAND = 0xB6;
    constexpr uint8_t BMP280_CONFIG_VALUE = 0x08;
    constexpr uint8_t BMP280_FORCED_MEASUREMENT = 0x55;
    constexpr uint32_t ALTITUDE_MONITOR_START_NOTIFICATION = 1U << 0;
    constexpr uint32_t ALTITUDE_MONITOR_STOP_NOTIFICATION = 1U << 1;
    constexpr double ISA_ALTITUDE_COEFFICIENT_M = 44330.76923;
    constexpr double ISA_STANDARD_PRESSURE_HPA = 1013.25;
    constexpr double ISA_ALTITUDE_EXPONENT = 0.190266669;

    struct CalibrationData
    {
        uint16_t temperature_1;
        int16_t temperature_2;
        int16_t temperature_3;
        uint16_t pressure_1;
        int16_t pressure_2;
        int16_t pressure_3;
        int16_t pressure_4;
        int16_t pressure_5;
        int16_t pressure_6;
        int16_t pressure_7;
        int16_t pressure_8;
        int16_t pressure_9;
    };

    static i2c_master_bus_handle_t i2c_bus = nullptr;
    static i2c_master_dev_handle_t sensor_device = nullptr;
    static CalibrationData calibration = {};
    static bool task_started = false;
    static bool ground_reference_ready = false;
    static double ground_reference_altitude_m = 0.0;
    static TaskHandle_t sensor_task_handle = nullptr;

    static uint16_t read_u16_le(const uint8_t *data)
    {
        return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
    }

    static int16_t read_i16_le(const uint8_t *data)
    {
        return static_cast<int16_t>(read_u16_le(data));
    }

    static esp_err_t read_registers(uint8_t register_address, uint8_t *data, size_t length)
    {
        return i2c_master_transmit_receive(
            sensor_device,
            &register_address,
            sizeof(register_address),
            data,
            length,
            I2C_TIMEOUT_MS);
    }

    static esp_err_t write_register(uint8_t register_address, uint8_t value)
    {
        const uint8_t data[] = {register_address, value};
        return i2c_master_transmit(sensor_device, data, sizeof(data), I2C_TIMEOUT_MS);
    }

    static esp_err_t initialize_sensor()
    {
        i2c_master_bus_config_t bus_config = {};
        bus_config.i2c_port = I2C_NUM_0;
        bus_config.sda_io_num = board::BMP280_SDA_GPIO;
        bus_config.scl_io_num = board::BMP280_SCL_GPIO;
        bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
        bus_config.glitch_ignore_cnt = 7;
        bus_config.flags.enable_internal_pullup = true;

        esp_err_t error = i2c_new_master_bus(&bus_config, &i2c_bus);
        if (error != ESP_OK)
        {
            return error;
        }

        uint8_t sensor_address = BMP280_ADDRESS_PRIMARY;
        error = i2c_master_probe(i2c_bus, sensor_address, I2C_TIMEOUT_MS);
        if (error != ESP_OK)
        {
            sensor_address = BMP280_ADDRESS_SECONDARY;
            error = i2c_master_probe(i2c_bus, sensor_address, I2C_TIMEOUT_MS);
            if (error != ESP_OK)
            {
                return error;
            }
        }

        i2c_device_config_t device_config = {};
        device_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        device_config.device_address = sensor_address;
        device_config.scl_speed_hz = I2C_FREQUENCY_HZ;

        error = i2c_master_bus_add_device(i2c_bus, &device_config, &sensor_device);
        if (error != ESP_OK)
        {
            return error;
        }

        uint8_t chip_id = 0;
        error = read_registers(REG_CHIP_ID, &chip_id, sizeof(chip_id));
        if (error != ESP_OK)
        {
            return error;
        }
        if (chip_id != BMP280_CHIP_ID)
        {
            ESP_LOGE(TAG, "Unexpected sensor ID 0x%02X", chip_id);
            return ESP_ERR_NOT_SUPPORTED;
        }

        error = write_register(REG_RESET, BMP280_RESET_COMMAND);
        if (error != ESP_OK)
        {
            return error;
        }
        vTaskDelay(pdMS_TO_TICKS(5));

        uint8_t calibration_bytes[24] = {};
        error = read_registers(REG_CALIBRATION, calibration_bytes, sizeof(calibration_bytes));
        if (error != ESP_OK)
        {
            return error;
        }

        calibration.temperature_1 = read_u16_le(&calibration_bytes[0]);
        calibration.temperature_2 = read_i16_le(&calibration_bytes[2]);
        calibration.temperature_3 = read_i16_le(&calibration_bytes[4]);
        calibration.pressure_1 = read_u16_le(&calibration_bytes[6]);
        calibration.pressure_2 = read_i16_le(&calibration_bytes[8]);
        calibration.pressure_3 = read_i16_le(&calibration_bytes[10]);
        calibration.pressure_4 = read_i16_le(&calibration_bytes[12]);
        calibration.pressure_5 = read_i16_le(&calibration_bytes[14]);
        calibration.pressure_6 = read_i16_le(&calibration_bytes[16]);
        calibration.pressure_7 = read_i16_le(&calibration_bytes[18]);
        calibration.pressure_8 = read_i16_le(&calibration_bytes[20]);
        calibration.pressure_9 = read_i16_le(&calibration_bytes[22]);

        if (calibration.pressure_1 == 0)
        {
            ESP_LOGE(TAG, "Invalid pressure calibration data");
            return ESP_ERR_INVALID_RESPONSE;
        }

        return write_register(REG_CONFIG, BMP280_CONFIG_VALUE);
    }

    static esp_err_t read_pressure_pa(float *pressure_pa)
    {
        esp_err_t error = write_register(REG_CTRL_MEAS, BMP280_FORCED_MEASUREMENT);
        if (error != ESP_OK)
        {
            return error;
        }

        vTaskDelay(pdMS_TO_TICKS(BMP280_CONVERSION_DELAY_MS));

        uint8_t data[6] = {};
        error = read_registers(REG_PRESSURE_DATA, data, sizeof(data));
        if (error != ESP_OK)
        {
            return error;
        }

        const int32_t raw_pressure =
            (static_cast<int32_t>(data[0]) << 12) | (static_cast<int32_t>(data[1]) << 4) | (static_cast<int32_t>(data[2]) >> 4);
        const int32_t raw_temperature =
            (static_cast<int32_t>(data[3]) << 12) | (static_cast<int32_t>(data[4]) << 4) | (static_cast<int32_t>(data[5]) >> 4);

        const double temperature_value_1 =
            (static_cast<double>(raw_temperature) / 16384.0 - static_cast<double>(calibration.temperature_1) / 1024.0) * static_cast<double>(calibration.temperature_2);
        const double temperature_delta =
            static_cast<double>(raw_temperature) / 131072.0 - static_cast<double>(calibration.temperature_1) / 8192.0;
        const double temperature_value_2 =
            temperature_delta * temperature_delta * static_cast<double>(calibration.temperature_3);
        const double fine_temperature = temperature_value_1 + temperature_value_2;
        const double temperature_c = fine_temperature / 5120.0;
        if (temperature_c < MINIMUM_SENSOR_TEMPERATURE_C
            || temperature_c > MAXIMUM_SENSOR_TEMPERATURE_C)
        {
            return ESP_ERR_INVALID_RESPONSE;
        }

        double pressure_value_1 = fine_temperature / 2.0 - 64000.0;
        double pressure_value_2 = pressure_value_1 * pressure_value_1 * static_cast<double>(calibration.pressure_6) / 32768.0;
        pressure_value_2 += pressure_value_1 * static_cast<double>(calibration.pressure_5) * 2.0;
        pressure_value_2 = pressure_value_2 / 4.0 + static_cast<double>(calibration.pressure_4) * 65536.0;
        pressure_value_1 =
            (static_cast<double>(calibration.pressure_3) * pressure_value_1 * pressure_value_1 / 524288.0 + static_cast<double>(calibration.pressure_2) * pressure_value_1) / 524288.0;
        pressure_value_1 =
            (1.0 + pressure_value_1 / 32768.0) * static_cast<double>(calibration.pressure_1);

        if (pressure_value_1 == 0.0)
        {
            return ESP_ERR_INVALID_RESPONSE;
        }

        double pressure = 1048576.0 - static_cast<double>(raw_pressure);
        pressure = (pressure - pressure_value_2 / 4096.0) * 6250.0 / pressure_value_1;
        const double pressure_correction_1 =
            static_cast<double>(calibration.pressure_9) * pressure * pressure / 2147483648.0;
        const double pressure_correction_2 =
            pressure * static_cast<double>(calibration.pressure_8) / 32768.0;
        pressure += (pressure_correction_1 + pressure_correction_2 + static_cast<double>(calibration.pressure_7)) / 16.0;

        if (!std::isfinite(pressure) || pressure < MINIMUM_PRESSURE_PA || pressure > MAXIMUM_PRESSURE_PA)
        {
            return ESP_ERR_INVALID_RESPONSE;
        }

        *pressure_pa = static_cast<float>(pressure);
        return ESP_OK;
    }

    static double calculate_isa_altitude_m(float pressure_pa)
    {
        const double pressure_hpa = static_cast<double>(pressure_pa) / 100.0;
        return ISA_ALTITUDE_COEFFICIENT_M
            * (1.0 - std::pow(
                pressure_hpa / ISA_STANDARD_PRESSURE_HPA,
                ISA_ALTITUDE_EXPONENT));
    }

    static esp_err_t collect_ground_reference(float *reference_pressure_pa)
    {
        std::array<float, GROUND_REFERENCE_SAMPLE_COUNT> samples = {};
        TickType_t last_wake_time = xTaskGetTickCount();

        for (size_t index = 0; index < samples.size(); ++index)
        {
            esp_err_t error = read_pressure_pa(&samples[index]);
            if (error != ESP_OK)
            {
                return error;
            }

            if (index + 1 < samples.size())
            {
                vTaskDelayUntil(
                    &last_wake_time,
                    pdMS_TO_TICKS(PRESSURE_READING_INTERVAL_MS));
            }
        }

        std::sort(samples.begin(), samples.end());

        float trimmed_sum = 0.0F;
        for (size_t index = 1; index + 1 < samples.size(); ++index)
        {
            trimmed_sum += samples[index];
        }

        *reference_pressure_pa = trimmed_sum / static_cast<float>(samples.size() - 2);
        return ESP_OK;
    }

    static void report_monitor_error(esp_err_t error)
    {
        ESP_LOGE(TAG, "Altitude monitoring failed: %s", esp_err_to_name(error));
        const esp_err_t post_error = fsm::post_event(fsm::Event::ALTITUDE_MONITOR_FAILED);
        if (post_error != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to notify FSM of altitude-monitor error: %s", esp_err_to_name(post_error));
        }
    }

    static bool wait_for_monitor_start()
    {
        uint32_t notification_value = 0;
        if (xTaskNotifyWait(0, UINT32_MAX, &notification_value, portMAX_DELAY) != pdTRUE)
        {
            return false;
        }

        return (notification_value & ALTITUDE_MONITOR_START_NOTIFICATION) != 0
            && (notification_value & ALTITUDE_MONITOR_STOP_NOTIFICATION) == 0;
    }

    static bool altitude_monitoring_stopped()
    {
        uint32_t notification_value = 0;
        const BaseType_t received = xTaskNotifyWait(
            0,
            ALTITUDE_MONITOR_STOP_NOTIFICATION,
            &notification_value,
            0);
        return received == pdTRUE
            && (notification_value & ALTITUDE_MONITOR_STOP_NOTIFICATION) != 0;
    }

    static bool monitor_altitude()
    {
        TickType_t last_wake_time = xTaskGetTickCount();
        for (;;)
        {
            if (altitude_monitoring_stopped())
            {
                return true;
            }

            float pressure_pa = 0.0F;
            const esp_err_t error = read_pressure_pa(&pressure_pa);
            if (error != ESP_OK)
            {
                report_monitor_error(error);
                return false;
            }

            const double current_altitude_m = calculate_isa_altitude_m(pressure_pa);
            const double altitude_delta_m =
                current_altitude_m - ground_reference_altitude_m;

            if (!std::isfinite(altitude_delta_m))
            {
                report_monitor_error(ESP_ERR_INVALID_RESPONSE);
                return false;
            }

            if (altitude_delta_m >= MOTOR_CUTOFF_ALTITUDE_M)
            {
                const esp_err_t post_error = fsm::post_event(fsm::Event::ALTITUDE_LIMIT_REACHED);
                if (post_error != ESP_OK)
                {
                    ESP_LOGE(TAG, "Failed to notify FSM of altitude limit: %s", esp_err_to_name(post_error));
                }
                return false;
            }

            vTaskDelayUntil(
                &last_wake_time,
                pdMS_TO_TICKS(PRESSURE_READING_INTERVAL_MS));
        }
    }

    static void report_sensor_error(esp_err_t error)
    {
        ESP_LOGE(TAG, "BMP280 ground-reference capture failed: %s", esp_err_to_name(error));
        const esp_err_t post_error = fsm::post_event(fsm::Event::GROUND_REFERENCE_FAILED);
        if (post_error != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to notify FSM of sensor error: %s", esp_err_to_name(post_error));
        }
    }

    static void ground_reference_task(void *)
    {
        esp_err_t error = initialize_sensor();
        if (error != ESP_OK)
        {
            report_sensor_error(error);
            vTaskDelete(nullptr);
            return;
        }

        float reference_pressure_pa = 0.0F;
        error = collect_ground_reference(&reference_pressure_pa);
        if (error != ESP_OK)
        {
            report_sensor_error(error);
            vTaskDelete(nullptr);
            return;
        }

        ESP_LOGI(TAG, "Ground reference pressure: %.2f Pa", reference_pressure_pa);
        ground_reference_altitude_m = calculate_isa_altitude_m(reference_pressure_pa);
        if (!std::isfinite(ground_reference_altitude_m))
        {
            report_sensor_error(ESP_ERR_INVALID_RESPONSE);
            vTaskDelete(nullptr);
            return;
        }
        ground_reference_ready = true;
        error = fsm::post_event(fsm::Event::GROUND_REFERENCE_CAPTURED, reference_pressure_pa);
        if (error != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to notify FSM of ground reference: %s", esp_err_to_name(error));
        }

        for (;;)
        {
            if (wait_for_monitor_start())
            {
                if (monitor_altitude())
                {
                    vTaskDelete(nullptr);
                    return;
                }
            }
            else
            {
                vTaskDelete(nullptr);
                return;
            }
        }
    }

    esp_err_t start_ground_reference()
    {
        if (task_started)
        {
            return ESP_ERR_INVALID_STATE;
        }

        task_started = true;
        if (xTaskCreate(
                ground_reference_task,
                "bmp280_ref",
                BMP280_TASK_STACK_SIZE,
                nullptr,
                BMP280_TASK_PRIORITY,
                &sensor_task_handle) != pdPASS)
        {
            task_started = false;
            return ESP_ERR_NO_MEM;
        }

        return ESP_OK;
    }

    esp_err_t start_altitude_monitoring()
    {
        if (!task_started || !ground_reference_ready || sensor_task_handle == nullptr)
        {
            return ESP_ERR_INVALID_STATE;
        }

        return xTaskNotify(
            sensor_task_handle,
            ALTITUDE_MONITOR_START_NOTIFICATION,
            eSetBits) == pdPASS ? ESP_OK : ESP_FAIL;
    }

    esp_err_t stop_altitude_monitoring()
    {
        if (sensor_task_handle == nullptr)
        {
            return ESP_ERR_INVALID_STATE;
        }

        return xTaskNotify(
            sensor_task_handle,
            ALTITUDE_MONITOR_STOP_NOTIFICATION,
            eSetBits) == pdPASS ? ESP_OK : ESP_FAIL;
    }
    }

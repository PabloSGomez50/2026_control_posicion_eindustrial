#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "as5600.h"
#include "l298n.h"

#define TAG "EI_MAIN"
#define DIR_GPIO_NUM GPIO_NUM_4
#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_MASTER_SCL_IO GPIO_NUM_9
#define I2C_MASTER_SDA_IO GPIO_NUM_8

void app_main(void)
{

    as5600_init_dir(DIR_GPIO_NUM);
    as5600_set_dir(DIR_GPIO_NUM, 0);
    i2c_master_bus_handle_t bus_handle;
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_PORT,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    i2c_device_config_t as5600_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_ADDRESS,
        .scl_speed_hz = 100000,
    };
    i2c_master_dev_handle_t as5600_handle;

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &as5600_cfg, &as5600_handle));

    as5600_status_t status;
    uint16_t angle;

    ESP_LOGI(TAG, "-- Inicializacion del programa -- ");

    while (1) {

        if (as5600_get_status((as5600_handle_t)as5600_handle, &status) == ESP_OK && status.valid) {
            if (as5600_get_angle((as5600_handle_t)as5600_handle, &angle) == ESP_OK) {
                ESP_LOGI(TAG, "Angle: %d", angle);
            } else {
                ESP_LOGE(TAG, "Failed to read angle");
            }
        } else {
            ESP_LOGE(TAG, "Invalid status");
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "as5600.h"
#include "l298n.h"
#include "pid.h"

#define TAG "EI_MAIN"
#define DIR_GPIO_NUM GPIO_NUM_4
#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_MASTER_SCL_IO GPIO_NUM_9
#define I2C_MASTER_SDA_IO GPIO_NUM_8

#define PWM_FREQ                4000
#define PWM_RES                 LEDC_TIMER_10_BIT
#define PWM_MAX                 (1 << PWM_RES)
#define PWM_GPIO                9
#define IN1_GPIO                10
#define IN2_GPIO                12
#define PID_TS                  10 // En ms

QueueHandle_t q_angle;

void task_read_angle(void *params) {
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

    TickType_t ticks = xTaskGetTickCount();
    as5600_status_t status;
    uint16_t angle;
    float degrees;

    while(1) {
        if (as5600_get_status((as5600_handle_t)as5600_handle, &status) == ESP_OK && status.md) {
            if (as5600_get_angle((as5600_handle_t)as5600_handle, &angle) == ESP_OK) {
                degrees = as5600_angle_to_degrees(angle);
            } else {
                ESP_LOGE(TAG, "Failed to read angle");
            }
        } else {
            ESP_LOGE(TAG, "Invalid status");
        }
        xQueueSendToBack(q_angle, &degrees, portMAX_DELAY);
        vTaskDelayUntil(&ticks, pdMS_TO_TICKS(PID_TS));
    }
    
}

void task_pid(void *params) {
    // Primera versión con valores a mano
    pid_params_t pid_params = {
        .kp = 4,
        .td = 0.1,
        .ti = 0.2,
        .ts = PID_TS, // No se si está bien
    };

    pid_variables_t pid_variables = {
        .e_0 = 0, .e_1 = 0, .e_2 = 0,
        .integral_action = 0,
        .max_out = PWM_MAX, .min_out = 0,
        .u = 0,
        .y_0 = 0, .y_1 = 0, .y_2 = 0,
    };

    // Parametros de posición 
    position_params_t position_params;
    pid_position_parameters(pid_params, &position_params);

    // Parametros de velocidad
    speed_params_t speed_params;
    pid_speed_parameters(pid_params, WITH_KICK, &speed_params);

    pwm_handle_t pwm_handle;
    pwm_config_t pwm_config = {
        .ledc_timer_config = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_10_BIT,
            .timer_num = LEDC_TIMER_0,
            .freq_hz = PWM_FREQ,
            .clk_cfg = LEDC_AUTO_CLK,
        },
        .ledc_channel_config = {
            .gpio_num = PWM_GPIO,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_0,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
        },
    };
    direction_gpio_t direction_gpio = {
        .in1 = IN1_GPIO,
        .in2 = IN2_GPIO,
    };
    ESP_ERROR_CHECK(l298n_init(pwm_config, &pwm_handle, direction_gpio));

    float angle;

    while(1) {
        xQueueReceive(q_angle, &angle, portMAX_DELAY);
    }
}

void app_main(void) {
    q_angle = xQueueCreate(1, sizeof(float));

    xTaskCreate(
        task_read_angle,
        "task_read_angle",
        1024,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL
    );

    xTaskCreate(
        task_pid,
        "task_pid",
        1024,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL
    );
}
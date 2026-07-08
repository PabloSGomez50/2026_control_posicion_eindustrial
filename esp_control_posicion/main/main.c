#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
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
#define PWM_GPIO                GPIO_NUM_0
#define IN1_GPIO                GPIO_NUM_1
#define IN2_GPIO                GPIO_NUM_2
#define PID_TS                  100 // En ms

#define UART_BAUDRATE       115200
#define UART_BUFF           256
#define UART_PATTERN_CHR    '\n'

QueueHandle_t q_angle;
QueueHandle_t q_uart;

void uart_init(void);

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
    float degrees = 0.0f;

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
        .kp = 0,
        .ki = 0,
        .kd = 0,
        .kick = NO_KICK,
        .windup = WITH_WINDUP,
    };

    pid_variables_t pid_variables = {
        .e_0 = 0, .e_1 = 0, .e_2 = 0,
        .integral_action = 0,
        .max_out = PWM_MAX, .min_out = 0,
        .u = 0,
        .y_0 = 0, .y_1 = 0, .y_2 = 0,
    };

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
    uint32_t pwm_raw;
    while(1) {
        xQueueReceive(q_angle, &angle, portMAX_DELAY);

        ESP_LOGI(TAG, "Angle: %.2f", angle);

        pwm_raw = angle / 360.0 * PWM_MAX;
        l298n_set_dc(pwm_handle, pwm_raw);
        l298n_change_dir(direction_gpio, CLOCKWISE);
    }
}

void task_uart_tx(void *params) {
    //buffer_t final_buffer;
    bool start = false;
    uint8_t header[2] = {0xAA, 0x55};

    while(1) {
        //xQueueReceive(q_filtered, &final_buffer, portMAX_DELAY);
        //if(xQueuePeek(q_start, &start, 0) != pdTRUE) start = false;
        if(start) {
            uart_write_bytes(UART_NUM_0, (char *)header, 2);
            //uart_write_bytes(UART_NUM_0, (char *)final_buffer.data, sizeof(final_buffer.data));
        }
    }
}

void task_uart_rx(void *params) {
    uart_event_t event;
    //uint8_t* dtmp = (uint8_t*) malloc(UART_BUFF + 1);
    float f, q;
    //lpf_t filter_config;
    bool start = false;

    //xQueueOverwrite(q_start, &start);

    while(1) {
        /*
        xQueueReceive(q_uart, (void *)&event, portMAX_DELAY);
        if(event.type == UART_PATTERN_DET) {
            size_t buffered_size;
            uart_get_buffered_data_len(UART_NUM_0, &buffered_size);
            int pos = uart_pattern_pop_pos(UART_NUM_0);
            if(pos != -1) {
                int read_len = uart_read_bytes(UART_NUM_0, dtmp, pos + 1, portMAX_DELAY);
                dtmp[read_len] = '\0';

                if(strncmp((char *)dtmp, "start", 5) == 0) {
                    start = true;
                    xQueueOverwrite(q_start, &start);
                    ESP_LOGI("UART_RX", "start");
                }
                else if(strncmp((char *)dtmp, "stop", 4) == 0) {
                    start = false;
                    xQueueOverwrite(q_start, &start);
                    ESP_LOGI("UART_RX", "stop");
                }
                else if (sscanf((char *)dtmp, "set %f %f", &f, &q) == 2) {
                    filter_config.f = f;
                    filter_config.q = q;
                    xQueueOverwrite(q_filter, &filter_config);
                    ESP_LOGI("UART_RX", "set %.0f %.0f", filter_config.f, filter_config.q);
                }
            }
            else {
                uart_flush_input(UART_NUM_0);
            }
        }
        */
    }
}

void app_main(void) {
    ESP_ERROR_CHECK(ledc_fade_func_install(0));
    q_angle = xQueueCreate(1, sizeof(float));
    ESP_LOGI(TAG, "Queue created");

    xTaskCreate(
        task_read_angle,
        "task_read_angle",
        1024,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL
    );
    ESP_LOGI(TAG, "Task created: task_read_angle");
    xTaskCreate(
        task_pid,
        "task_pid",
        1024,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL
    );
    ESP_LOGI(TAG, "Task created: task_pid");
}

void uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // 1. Instalamos el driver con una cola de eventos (uart0_queue)
    uart_driver_install(UART_NUM_0, UART_BUFF * 2, UART_BUFF * 2, 20, &q_uart, 0);
    uart_param_config(UART_NUM_0, &uart_config);
    uart_enable_pattern_det_baud_intr(UART_NUM_0, UART_PATTERN_CHR, 1, 9, 0, 0);
    uart_pattern_queue_reset(UART_NUM_0, 20);
}

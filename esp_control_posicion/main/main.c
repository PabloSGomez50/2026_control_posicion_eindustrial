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
#define PID_TS                  20 // En ms

#define UART_BAUDRATE       115200
#define UART_BUFF           256
#define UART_PATTERN_CHR    '\n'

// typedef enum {
//     EVENT_START,
//     EVENT_STOP,
//     EVENT_KP,
//     EVENT_KD,
//     EVENT_KI,
//     EVENT_WINDUP,
//     EVENT_KICK,
//     EVENT_REF,
// } event_t;

// typedef struct {
//     event_t event;
//     float value;
// } uart_rx_event_t;

typedef struct {
    float ref;
    float angle;
    float u_control;
} uart_tx_data_t;

QueueHandle_t q_angle;
QueueHandle_t q_uart;
QueueHandle_t q_pid_params;
QueueHandle_t q_tx_data;

TaskHandle_t task_pid_handle = NULL;


pwm_handle_t pwm_handle;
direction_gpio_t direction_gpio = {
    .in1 = IN1_GPIO,
    .in2 = IN2_GPIO,
};

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
        xQueueOverwrite(q_angle, &degrees);
        vTaskDelayUntil(&ticks, pdMS_TO_TICKS(PID_TS));
    }
}

void task_pid(void *params) {
    // Primera versión con valores a mano
    pid_variables_t pid_variables = {
        .e_0 = 0, .e_1 = 0,
        .integral_action = 0,
        .max_out = PWM_MAX, .min_out = -PWM_MAX,
        .u = 0,
        .y_0 = 0, .y_1 = 0,
    };
    float angle;
    pid_params_t pid_params;
    uart_tx_data_t uart_tx_data;
    
    while(1) {
        xQueueReceive(q_angle, &angle, portMAX_DELAY);
        xQueuePeek(q_pid_params, &pid_params, 0);
        
        float error = pid_params.ref - angle;
        pid_variables.y_0 = error;
        
        if(error > 180.0f) 
            error -= 360.0f;
        else if(error < -180.0f)
            error += 360.0f;
        
        pid_variables.e_0 = error;
        
        pid_position(pid_params, &pid_variables);
        
        if(pid_variables.u < 0) {
            l298n_change_dir(direction_gpio, COUNTER_CLOCKWISE);
        }
        else {
            l298n_change_dir(direction_gpio, CLOCKWISE);
        }
        
        l298n_set_dc(pwm_handle, abs((int32_t)pid_variables.u));
        uart_tx_data.ref = pid_params.ref;
        uart_tx_data.angle = angle;
        uart_tx_data.u_control = pid_variables.u;
        xQueueSendToBack(q_tx_data, &uart_tx_data, portMAX_DELAY);
    }
}

void task_uart_tx(void *params) {
    uart_tx_data_t final_buffer;
    char buffer[UART_BUFF];

    while(1) {
        xQueueReceive(q_tx_data, &final_buffer, portMAX_DELAY);
        sprintf(buffer, "R%.2fA%.2fU%.2f\n", final_buffer.ref, final_buffer.angle, final_buffer.u_control);
        uart_write_bytes(UART_NUM_0, buffer, strlen(buffer));
    }
}

void task_uart_rx(void *params) {
    pid_params_t pid_params;
    uart_event_t event;
    float value;
    uint8_t* dtmp = (uint8_t*) malloc(UART_BUFF + 1);

    while(1) {
        xQueueReceive(q_uart, (void *)&event, portMAX_DELAY);
        xQueuePeek(q_pid_params, &pid_params, 0);
        if(event.type == UART_PATTERN_DET) {
            size_t buffered_size;
            uart_get_buffered_data_len(UART_NUM_0, &buffered_size);
            int pos = uart_pattern_pop_pos(UART_NUM_0);
            if(pos != -1) {
                int read_len = uart_read_bytes(UART_NUM_0, dtmp, pos + 1, portMAX_DELAY);
                dtmp[read_len] = '\0';

                if(strncmp((char *)dtmp, "start", 5) == 0) {
                    // uart_event.event = EVENT_START;
                    
                    xTaskCreate(
                        task_pid,
                        "task_pid",
                        1024,
                        NULL,
                        tskIDLE_PRIORITY + 1,
                        &task_pid_handle
                    );
                    ESP_LOGI(TAG, "Task created: task_pid");
                }
                else if(strncmp((char *)dtmp, "stop", 4) == 0) {
                    // uart_event.event = EVENT_STOP;
                    l298n_set_dc(pwm_handle, 0);
                    l298n_change_dir(direction_gpio, NO_DIRECTION);
                    vTaskDelete(task_pid_handle);
                    task_pid_handle = NULL;
                }
                else if (sscanf((char *)dtmp, "set kp %f", &value) == 1) {
                    // uart_event.event = EVENT_KP;
                    pid_params.kp = value;
                }
                else if (sscanf((char *)dtmp, "set kd %f", &value) == 1) {
                    // uart_event.event = EVENT_KD;
                    pid_params.kd = value;
                }
                else if (sscanf((char *)dtmp, "set ki %f", &value) == 1) {
                    // uart_event.event = EVENT_KI;
                    pid_params.ki = value;
                }
                else if (sscanf((char *)dtmp, "set windup %f", &value) == 1) {
                    // uart_event.event = EVENT_WINDUP;
                    pid_params.windup = value != 0.0f ? WITH_WINDUP : NO_WINDUP;
                }
                else if (sscanf((char *)dtmp, "set kick %f", &value) == 1) {
                    // uart_event.event = EVENT_KICK;
                    pid_params.kick = value != 0.0f ? WITH_KICK : NO_KICK;
                }
                else if (sscanf((char *)dtmp, "set ref %f", &value) == 1) {
                    // uart_event.event = EVENT_REF;
                    pid_params.ref = value;
                } else {
                    ESP_LOGW(TAG, "Unknown command received: %s", dtmp);
                    continue;
                }
                
                xQueueOverwrite(q_pid_params, &pid_params);
                // xQueueSendToBack(q_rx_event, &event, portMAX_DELAY);
            }
            else {
                uart_flush_input(UART_NUM_0);
            }
        }
    }
}

void app_main(void) {
    ESP_ERROR_CHECK(ledc_fade_func_install(0));
    q_angle = xQueueCreate(1, sizeof(float));
    q_pid_params = xQueueCreate(1, sizeof(pid_params_t));
    q_tx_data = xQueueCreate(5, sizeof(uart_tx_data_t));
    ESP_LOGI(TAG, "Queue created");
    uart_init();

    pid_params_t pid_params = {
        .kp = 2.5,
        .ki = 0.03,
        .kd = 0.01,
        .kick = NO_KICK,
        .windup = WITH_WINDUP,
        .ref = 0.0f,
    };
    xQueueOverwrite(q_pid_params, &pid_params);

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
    ESP_ERROR_CHECK(l298n_init(pwm_config, &pwm_handle, direction_gpio));

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
        task_uart_tx,
        "task_uart_tx",
        1024,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL
    );
    ESP_LOGI(TAG, "Task created: task_uart_tx");
    xTaskCreate(
        task_uart_rx,
        "task_uart_rx",
        1024,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL
    );
    ESP_LOGI(TAG, "Task created: task_uart_rx");
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

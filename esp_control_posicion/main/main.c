#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/usb_serial_jtag.h"
#include "as5600.h"
#include "l298n.h"
#include "pid.h"

#define TAG "EI_MAIN"
#define DIR_GPIO_NUM GPIO_NUM_4
#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_MASTER_SCL_IO GPIO_NUM_9
#define I2C_MASTER_SDA_IO GPIO_NUM_8

#define PWM_FREQ                20000
#define PWM_RES                 LEDC_TIMER_10_BIT
#define PWM_MAX                 (1 << PWM_RES)
#define PWM_GPIO                GPIO_NUM_1
#define IN1_GPIO                GPIO_NUM_2
#define IN2_GPIO                GPIO_NUM_3
#define PID_TS                  20 // En ms

#define USB_BUFF           256

typedef enum {
    EVENT_START,
    EVENT_STOP,
    EVENT_KP,
    EVENT_KD,
    EVENT_KI,
    EVENT_WINDUP,
    EVENT_KICK,
    EVENT_REF,
} event_t;

typedef struct {
    event_t event;
    float value;
} uart_rx_event_t;

typedef struct {
    float ref;
    float angle;
    float u_control;
    char u_name[32];
    bool is_param;
} uart_transmit_t;

QueueHandle_t q_angle;
QueueHandle_t q_rx_event;
QueueHandle_t q_tx_data;

void usb_init(void);

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
                degrees = as5600_angle_to_degrees(angle) + 22;
                if(degrees > 360) degrees -= 360;
            } else {
                ESP_LOGE(TAG, "Failed to read angle");
            }
        } else {
            ESP_LOGE(TAG, "Invalid status");
        }
        ESP_LOGI(TAG, "angulo: %f", degrees);
        xQueueOverwrite(q_angle, &degrees);
        vTaskDelayUntil(&ticks, pdMS_TO_TICKS(PID_TS));
    }
}

void task_pid(void *params) {
    // Primera versión con valores a mano
    pid_params_t pid_params = {
        .kp = 0,
        .ki = 0,
        .kd = 0,
        .kick = WITH_KICK,
        .windup = WITH_WINDUP,
    };

    pid_variables_t pid_variables = {
        .e_0 = 0, .e_1 = 0,
        .integral_action = 0,
        .max_out = PWM_MAX, .min_out = -PWM_MAX,
        .u = 0,
        .y_0 = 0, .y_1 = 0,
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
    ESP_ERROR_CHECK(ledc_fade_func_install(0));

    float angle, ref = 0;

    bool start = false;

    uart_rx_event_t uart_event;
    uart_transmit_t uart_tx_data;
    
    while(1) {
        if (xQueueReceive(q_rx_event, &uart_event, 0) == pdTRUE) {
            switch (uart_event.event) {
                case EVENT_START:
                start = true;
                break;
                case EVENT_STOP:
                start = false;
                break;
                case EVENT_KP:
                pid_params.kp = uart_event.value;
                break;
                case EVENT_KI:
                pid_params.ki = uart_event.value;
                break;
                case EVENT_KD:
                pid_params.kd = uart_event.value;
                break;
                case EVENT_WINDUP:
                pid_params.windup = (uart_event.value != 0) ? WITH_WINDUP : NO_WINDUP;
                break;
                case EVENT_KICK:
                pid_params.kick = (uart_event.value != 0) ? WITH_KICK : NO_KICK;
                break;
                case EVENT_REF:
                ref = uart_event.value;
                break;
                default:
                ESP_LOGW(TAG, "Unknown event received");
                break;
            }
        }
         
        if(start) {
            xQueueReceive(q_angle, &angle, portMAX_DELAY);
            
            float error = ref - angle;
            pid_variables.y_0 = error;
            
            if(error > 180.0f) {
                error -= 360.0f;
            }
            else if(error < -180.0f) {
                error += 360.0f;
            }
            
            pid_variables.e_0 = error;
            
            pid_position(pid_params, &pid_variables);
            
            if(pid_variables.u < 0) {
                l298n_change_dir(direction_gpio, COUNTER_CLOCKWISE);
            }
            else {
                l298n_change_dir(direction_gpio, CLOCKWISE);
            }
            
            l298n_set_dc(pwm_handle, abs((int32_t)pid_variables.u));
            
            uart_tx_data.ref = ref;
            uart_tx_data.angle = angle;
            uart_tx_data.u_control = pid_variables.u;
            uart_tx_data.is_param = false;
            xQueueSendToBack(q_tx_data, &uart_tx_data, 0);
        }
        else {
            l298n_set_dc(pwm_handle, 0);
            l298n_change_dir(direction_gpio, NO_DIRECTION);
            
            xQueuePeek(q_rx_event, &uart_event, portMAX_DELAY);
        }
    }
}

void task_uart_tx(void *params) {
    uart_transmit_t final_buffer;
    char buffer[USB_BUFF];

    while(1) {
        xQueueReceive(q_tx_data, &final_buffer, portMAX_DELAY);

        if(final_buffer.is_param) {
            strcpy(buffer, final_buffer.u_name);
        }
        else {
            sprintf(buffer, "R%.2fA%.2fU%.2f\n", final_buffer.ref, final_buffer.angle, final_buffer.u_control);
        }
        usb_serial_jtag_write_bytes(buffer, strlen(buffer), portMAX_DELAY);
    }
}

void task_uart_rx(void *params) {
    char var_name[32];
    uart_transmit_t uart_transmit = {
        .angle = 0,
        .ref = 0,
        .u_control = 0,
        .is_param = true,
    };
    
    uart_rx_event_t uart_event;
    
    char rx_buff[USB_BUFF];
    int rx_idx = 0;

    while(1) {
        uint8_t ch;
        int read_len = usb_serial_jtag_read_bytes(&ch, 1, portMAX_DELAY);

        if(read_len > 0) {
            if(ch == '\n' || ch == '\r') {
                if(rx_idx > 0) {
                    rx_buff[rx_idx] = '\0';

                    if(strncmp(rx_buff, "start", 5) == 0) {
                        uart_event.event = EVENT_START;
                        sprintf(var_name, "ok start\n");
                    }
                    else if(strncmp(rx_buff, "stop", 4) == 0) {
                        uart_event.event = EVENT_STOP;
                        sprintf(var_name, "ok stop\n");
                    }
                    else if (sscanf(rx_buff, "set kp %f", &uart_event.value) == 1) {
                        uart_event.event = EVENT_KP;
                        sprintf(var_name, "ok kp:%f\n", uart_event.value);
                    }
                    else if (sscanf(rx_buff, "set kd %f", &uart_event.value) == 1) {
                        uart_event.event = EVENT_KD;
                        sprintf(var_name, "ok kd:%f\n", uart_event.value);
                    }
                    else if (sscanf(rx_buff, "set ki %f", &uart_event.value) == 1) {
                        uart_event.event = EVENT_KI;
                        sprintf(var_name, "ok ki:%f\n", uart_event.value);
                    }
                    else if (sscanf(rx_buff, "set windup %f", &uart_event.value) == 1) {
                        uart_event.event = EVENT_WINDUP;
                        sprintf(var_name, "ok windup:%f\n", uart_event.value);
                    }
                    else if (sscanf(rx_buff, "set kick %f", &uart_event.value) == 1) {
                        uart_event.event = EVENT_KICK;
                        sprintf(var_name, "ok kick:%f\n", uart_event.value);
                    }
                    else if (sscanf(rx_buff, "set ref %f", &uart_event.value) == 1) {
                        uart_event.event = EVENT_REF;
                        sprintf(var_name, "ok ref:%f\n", uart_event.value);
                    } else {
                        ESP_LOGW(TAG, "Unknown command received: %s", rx_buff);
                        rx_idx = 0;
                        continue;
                    }

                    strcpy(uart_transmit.u_name, var_name);    
                    xQueueSendToBack(q_tx_data, &uart_transmit, portMAX_DELAY);
                    xQueueSendToBack(q_rx_event, &uart_event, portMAX_DELAY);

                    rx_idx = 0;
                }
            }
            else {
                if(rx_idx < USB_BUFF - 1) {
                    rx_buff[rx_idx++] = ch;
                }
            }
        }
    }
}

void app_main(void) {
    q_angle = xQueueCreate(1, sizeof(float));
    q_rx_event = xQueueCreate(10, sizeof(uart_rx_event_t));
    q_tx_data = xQueueCreate(10, sizeof(uart_transmit_t));
    ESP_LOGI(TAG, "Queue created");
    usb_init();

    xTaskCreate(
        task_read_angle,
        "task_read_angle",
        1024 * 4,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL
    );
    ESP_LOGI(TAG, "Task created: task_read_angle");
    xTaskCreate(
        task_pid,
        "task_pid",
        1024 * 4,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL
    );
    ESP_LOGI(TAG, "Task created: task_pid");
    xTaskCreate(
        task_uart_tx,
        "task_uart_tx",
        1024 * 4,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL
    );
    ESP_LOGI(TAG, "Task created: task_uart_tx");
    xTaskCreate(
        task_uart_rx,
        "task_uart_rx",
        1024 * 4,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL
    );
    ESP_LOGI(TAG, "Task created: task_uart_rx");
}

void usb_init(void) {
    usb_serial_jtag_driver_config_t usb_config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    // Instalar driver de USB Serial JTAG
    usb_serial_jtag_driver_install(&usb_config);
}

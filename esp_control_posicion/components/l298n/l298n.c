#include "l298n.h"
#include "esp_log.h"

static const char *TAG = "L298N";

esp_err_t l298n_init(pwm_config_t pwm_config, pwm_handle_t *pwm_handle, direction_gpio_t direction_gpio) {
    esp_err_t err;
    
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << direction_gpio.in1) | (1ULL << direction_gpio.in2),
        .pull_down_en = 0,
        .pull_up_en = 0
    };

    err = gpio_config(&io_conf);
    if(err != ESP_OK) return err;

    err = l298n_change_dir(direction_gpio, NO_DIRECTION);
    if(err != ESP_OK) return err;

    err = ledc_timer_config(&pwm_config.ledc_timer_config);
    if(err != ESP_OK) return err;

    err = ledc_channel_config(&pwm_config.ledc_channel_config);
    if(err != ESP_OK) return err;

    pwm_handle->ledc_channel = pwm_config.ledc_channel_config.channel;
    pwm_handle->ledc_mode = pwm_config.ledc_timer_config.speed_mode;

    return ESP_OK;
}

esp_err_t l298n_set_dc(pwm_handle_t pwm_handle, uint32_t dc) {
    esp_err_t err;

    err = ledc_set_duty_and_update(pwm_handle.ledc_mode, pwm_handle.ledc_channel, dc, 0);
    if(err != ESP_OK) return err;

    return ESP_OK;
}

esp_err_t l298n_change_dir(direction_gpio_t direction_gpio, direction_t dir) {
    esp_err_t err;
    
    switch (dir) {
        case NO_DIRECTION: {
            err = gpio_set_level(direction_gpio.in1, 0);
            if(err != ESP_OK) return err;

            err = gpio_set_level(direction_gpio.in2, 0);
            if(err != ESP_OK) return err;
        } break;

        case CLOCKWISE: {
            err = gpio_set_level(direction_gpio.in1, 1);
            if(err != ESP_OK) return err;

            err = gpio_set_level(direction_gpio.in2, 0);
            if(err != ESP_OK) return err;
        } break;

        case COUNTER_CLOCKWISE: {
            err = gpio_set_level(direction_gpio.in1, 0);
            if(err != ESP_OK) return err;

            err = gpio_set_level(direction_gpio.in2, 1);
            if(err != ESP_OK) return err;
        } break;
    }

    return ESP_OK;
}

esp_err_t l298n_stop(pwm_handle_t pwm_handle) {
    esp_err_t err;

    err = ledc_set_duty_and_update(pwm_handle.ledc_mode, pwm_handle.ledc_channel, 0, 0);
    if(err != ESP_OK) return err;

    return ESP_OK;
}
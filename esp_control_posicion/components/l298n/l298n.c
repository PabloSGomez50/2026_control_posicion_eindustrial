#include "l298n.h"
#include "esp_log.h"

static const char *TAG = "L298N";

esp_err_t l298n_config(pwm_config_t pwm_config, pwm_handle_t *pwm_handle, direction_gpio_t direction_gpio) {
    esp_err_t err;
    mcpwm_oper_handle_t operator;
    mcpwm_gen_handle_t generator;
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

    err = mcpwm_new_timer(&pwm_config.timer_config, &pwm_handle->timer);
    if(err != ESP_OK) return err;

    err = mcpwm_new_operator(&pwm_config.operator_config, &operator);
    if(err != ESP_OK) return err;

    err = mcpwm_operator_connect_timer(operator, pwm_handle->timer);
    if(err != ESP_OK) return err;
    
    err = mcpwm_new_comparator(operator, &pwm_config.comparator_config, &pwm_handle->comparator);
    if(err != ESP_OK) return err;
    
    err = mcpwm_new_generator(operator, &pwm_config.generator_config, &generator);
    if(err != ESP_OK) return err;
    
    err = mcpwm_generator_set_action_on_compare_event(
        generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,
            pwm_handle->comparator,
            MCPWM_GEN_ACTION_LOW
        )
    );
    if(err != ESP_OK) return err;
    
    err = mcpwm_generator_set_action_on_compare_event(
        generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_DOWN,
            pwm_handle->comparator,
            MCPWM_GEN_ACTION_HIGH
        )
    );
    if(err != ESP_OK) return err;
    
    return ESP_OK;
}

esp_err_t l298n_start(pwm_handle_t pwm_handle) {
    esp_err_t err;

    err = mcpwm_comparator_set_compare_value(pwm_handle.comparator, 0);
    if(err != ESP_OK) return err;

    err = mcpwm_timer_enable(pwm_handle.timer);
    if(err != ESP_OK) return err;

    err = mcpwm_timer_start_stop(pwm_handle.timer, MCPWM_TIMER_START_NO_STOP);
    if(err != ESP_OK) return err;

    return ESP_OK;
}

esp_err_t l298n_set_dc(pwm_handle_t pwm_handle, uint32_t dc) {
    esp_err_t err;

    err = mcpwm_comparator_set_compare_value(pwm_handle.comparator, dc);
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

    err = mcpwm_timer_start_stop(pwm_handle.timer, MCPWM_TIMER_STOP_EMPTY);
    if(err != ESP_OK) return err;
    
    err = mcpwm_timer_disable(pwm_handle.timer);
    if(err != ESP_OK) return err;

    return ESP_OK;
}
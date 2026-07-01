#ifndef L298N_H
#define L298N_H

#include <stdint.h>
#include "driver/ledc.h"
#include "driver/gpio.h"

typedef struct {
    ledc_mode_t ledc_mode;
    ledc_channel_t ledc_channel;
} pwm_handle_t;

typedef struct {
    ledc_timer_config_t ledc_timer_config;
    ledc_channel_config_t ledc_channel_config;
} pwm_config_t;

typedef struct {
    gpio_num_t in1;
    gpio_num_t in2;
} direction_gpio_t;

typedef enum {
    CLOCKWISE,
    COUNTER_CLOCKWISE,
    NO_DIRECTION,
} direction_t;

/**
 * @brief   Initialize L298N PWM and direction pins
 * 
 * @param   pwm_config PWM configuration
 * @param   pwm_handle Handles of the timer and comparator
 * @param   direction_gpio direction gpio pins
 * @return  esp_err_t
 */
esp_err_t l298n_init(pwm_config_t pwm_config, pwm_handle_t *pwm_handle, direction_gpio_t direction_gpio);

/**
 * @brief   Sets the duty cycle of the pwm
 * 
 * @param   pwm_handle Handles of the timer and comparator
 * @param   dc duty cycle
 * @return  esp_err_t
 */
esp_err_t l298n_set_dc(pwm_handle_t pwm_handle, uint32_t dc);

/**
 * @brief   Changes the direction of the motor
 * 
 * @param   direction_gpio direction gpio pins
 * @param   dir direction of the motor
 * @return  esp_err_t
 */
esp_err_t l298n_change_dir(direction_gpio_t direction_gpio, direction_t dir);

/**
 * @brief   Stops the PWM timer
 * 
 * @param   pwm_handle Handles of the timer and comparator
 * @return  esp_err_t
 */
esp_err_t l298n_stop(pwm_handle_t pwm_handle);

#endif /* L298N_H */

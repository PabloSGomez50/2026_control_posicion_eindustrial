#include "pid.h"
#include "esp_log.h"

static const char *TAG = "PID";

/**
 * @brief   Asigns the previous variables
 * 
 * @param   pid_variables variables for the PID control
 * @return  esp_err_t
 */
esp_err_t pid_prev_variables(pid_variables_t *pid_variables);

/**
 * @brief   Limits the value of a variable
 * 
 * @param   variable variable to limit
 * @param   pid_variables variables for the PID control
 * @return  esp_err_t
 */
esp_err_t pid_limit(float *variable, pid_variables_t *pid_variables);

esp_err_t pid_position(pid_params_t pid_params, pid_variables_t *pid_variables) {
    pid_variables->integral_action += pid_params.ki * (pid_variables->e_0 + pid_variables->e_1);

    if(pid_params.windup == WITH_WINDUP) {
        pid_limit(&pid_variables->integral_action, pid_variables);
    }

    switch (pid_params.kick) {
        case WITH_KICK: {
            pid_variables->u = pid_variables->integral_action + 
                               pid_params.kd * (pid_variables->e_0 - pid_variables->e_1) +
                               pid_params.kp * pid_variables->e_0;
        } break;

        case NO_KICK: {
            pid_variables->u = pid_variables->integral_action - 
                               pid_params.kd * (pid_variables->y_0 - pid_variables->y_1) +
                               pid_params.kp * pid_variables->e_0;
        } break;
    }

    pid_limit(&pid_variables->u, pid_variables);

    pid_prev_variables(pid_variables);

    return ESP_OK;
}

esp_err_t pid_limit(float *variable, pid_variables_t *pid_variables) {
    if(*variable > pid_variables->max_out)
        *variable = pid_variables->max_out;
    if(*variable < pid_variables->min_out)
        *variable = pid_variables->min_out;
    return ESP_OK;
}

esp_err_t pid_prev_variables(pid_variables_t *pid_variables) {
    pid_variables->e_1 = pid_variables->e_0;
    pid_variables->y_1 = pid_variables->y_0;

    return ESP_OK;
}

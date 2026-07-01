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

esp_err_t pid_position_parameters(pid_params_t pid_params, position_params_t *position_params) {
    position_params->kp = pid_params.kp;
    position_params->ki = pid_params.kp * pid_params.ts / (2 * pid_params.ti);
    position_params->kd = pid_params.kp * pid_params.td / pid_params.ts;

    return ESP_OK;
}

esp_err_t pid_speed_parameters(pid_params_t pid_params, pid_kick_t pid_kick, speed_params_t *speed_params) {
    switch (pid_kick) {
        case WITH_KICK: {
            speed_params->q0 = pid_params.kp * (1 + pid_params.ts / (2 * pid_params.ti) + pid_params.td / pid_params.ts);
            speed_params->q1 = pid_params.kp * (-1 + pid_params.ts / (2 * pid_params.ti) - 2 * pid_params.td / pid_params.ts);
            speed_params->q2 = pid_params.kp * pid_params.td / pid_params.ts;
            speed_params->ki = -1;
        } break;

        case NO_KICK: {
            speed_params->q0 = -pid_params.kp * (1 + pid_params.td / pid_params.ts);
            speed_params->q1 = pid_params.kp * (1 + 2 * pid_params.td / pid_params.ts);
            speed_params->q2 = -pid_params.kp * pid_params.td / pid_params.ts;
            speed_params->ki = pid_params.kp * pid_params.ts / (2 * pid_params.ti);
        } break;
    }

    return ESP_OK;
}

esp_err_t pid_position(position_params_t position_params, pid_kick_t pid_kick, pid_windup_t pid_windup, pid_variables_t *pid_variables) {
    pid_variables->integral_action += position_params.ki * (pid_variables->e_0 + pid_variables->e_1);

    if(pid_windup == WINDUP) {
        pid_limit(&pid_variables->integral_action, pid_variables);
    }

    switch (pid_kick) {
        case WITH_KICK: {
            pid_variables->u = pid_variables->integral_action + 
                               position_params.kd * (pid_variables->e_0 - pid_variables->e_1) +
                               position_params.kp * pid_variables->e_0;
        } break;

        case NO_KICK: {
            pid_variables->u = pid_variables->integral_action - 
                               position_params.kd * (pid_variables->y_0 - pid_variables->y_1) +
                               position_params.kp * pid_variables->e_0;
        } break;
    }

    pid_limit(&pid_variables->u, pid_variables);

    pid_prev_variables(pid_variables);

    return ESP_OK;
}

esp_err_t pid_speed(speed_params_t speed_params, pid_kick_t pid_kick, pid_variables_t *pid_variables) {
    pid_kick_t type = pid_kick;

    if(pid_kick == NO_KICK && speed_params.ki == -1) {
        ESP_LOGW(TAG, "Trying using no kick without calculation, using with kick");
        type = WITH_KICK;
    }
    if(pid_kick == WITH_KICK && speed_params.ki != -1) {
        ESP_LOGW(TAG, "Trying using with kick without calculation, using no kick");
        type = NO_KICK;
    }
    
    switch (type) {
        case WITH_KICK: {
            pid_variables->u += speed_params.q0 * pid_variables->e_0 +
                                speed_params.q1 * pid_variables->e_1 +
                                speed_params.q2 * pid_variables->e_2;
        } break;

        case NO_KICK: {
            pid_variables->u += speed_params.q0 * pid_variables->y_0 +
                                speed_params.q1 * pid_variables->y_1 +
                                speed_params.q2 * pid_variables->y_2 +
                                speed_params.ki * (pid_variables->e_0 + pid_variables->e_1);
        } break;
    }

    pid_limit(&pid_variables->u, pid_variables);

    pid_prev_variables(pid_variables);

    return ESP_OK;
}

esp_err_t pid_limit(float *variable, pid_variables_t *pid_variables) {
    if(*variable > pid_variables->max_out)
        *variable = pid_variables->max_out;
    if(*variable > pid_variables->min_out)
        *variable = pid_variables->min_out;
    return ESP_OK;
}

esp_err_t pid_prev_variables(pid_variables_t *pid_variables) {
    pid_variables->e_2 = pid_variables->e_1;
    pid_variables->e_1 = pid_variables->e_0;
    pid_variables->y_2 = pid_variables->y_1;
    pid_variables->y_1 = pid_variables->y_0;

    return ESP_OK;
}

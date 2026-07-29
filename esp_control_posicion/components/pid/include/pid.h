#ifndef PID_H
#define PID_H

#include <stdint.h>
#include "esp_err.h"

typedef enum {
    NO_KICK,
    WITH_KICK,
} pid_kick_t;

typedef enum {
    NO_WINDUP,
    WITH_WINDUP,
} pid_windup_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    pid_kick_t kick;
    pid_windup_t windup;
} pid_params_t;

typedef struct {
    float e_0, e_1;
    float integral_action;
    float max_out, min_out;
    float u;
    float y_0, y_1;
} pid_variables_t;

/**
 * @brief   Computes the position PID
 * 
 * @param   pid_params kp, ki and kd parameters
 * @param   pid_variables variables for the PID control
 * @return  esp_err_t
 */
esp_err_t pid_position(pid_params_t pid_params, pid_variables_t *pid_variables);

#endif /* PID_H */

#ifndef PID_H
#define PID_H

#include <stdint.h>
#include "esp_err.h"

typedef struct {
    float kp;
    float ti;
    float td;
    float ts;
} pid_params_t;

typedef struct {
    float kp;
    float ki;
    float kd;
} position_params_t;

typedef struct {
    float q0;
    float q1;
    float q2;
    float ki;
} speed_params_t;

typedef struct {
    float e_0, e_1, e_2;
    float integral_action;
    float max_out, min_out;
    float u;
    float y_0, y_1, y_2;
} pid_variables_t;

typedef enum {
    NO_KICK,
    WITH_KICK,
} pid_kick_t;

/**
 * @brief   Calculates the position parameters for the PID
 * 
 * @param   pid_params kp, td, ti and ts parameters
 * @param   position_params kp, ki and kd parameters
 * @return  esp_err_t
 */
esp_err_t pid_position_parameters(pid_params_t pid_params, position_params_t *position_params);

/**
 * @brief   Calculates the speed parameters for the PID
 * 
 * @param   pid_params kp, td, ti and ts parameters
 * @param   pid_kick pid with or no kick
 * @param   speed_params q0, q1, q2 and ki parameters
 * @return  esp_err_t
 */
esp_err_t pid_speed_parameters(pid_params_t pid_params, pid_kick_t pid_kick, speed_params_t *speed_params);

/**
 * @brief   Computes the position PID
 * 
 * @param   position_params kp, ki and kd parameters
 * @param   pid_kick pid with or no kick
 * @param   pid_variables variables for the PID control
 * @return  esp_err_t
 */
esp_err_t pid_position(position_params_t position_params, pid_kick_t pid_kick, pid_variables_t *pid_variables);

/**
 * @brief   Computes the speed PID
 * 
 * @param   speed_params q0, q1, q2 and ki parameters
 * @param   pid_kick pid with or no kick
 * @param   pid_variables variables for the PID control
 * @return  esp_err_t
 */
esp_err_t pid_speed(speed_params_t speed_params, pid_kick_t pid_kick, pid_variables_t *pid_variables);

#endif /* PID_H */

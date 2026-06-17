#ifndef AS5600_H
#define AS5600_H

#include <stdint.h>
#include "driver/i2c_master.h"
#include "driver/gpio.h"

#define AS5600_ADDRESS 0x36

/* Registers */
#define AS5600_ANGLE_REG_HIGH 0x0E
#define AS5600_ANGLE_REG_LOW  0x0F
#define AS5600_STATUS_REG     0x0B
#define AS5600_AGC_REG        0x1A
#define AS5600_MAG_REG_HIGH   0x1B

typedef struct {
    uint8_t mh;
    uint8_t ml;
    uint8_t md;
    uint8_t valid;
} as5600_status_t;

/**
 * @brief   AS5600 device handle
 */
typedef i2c_master_dev_handle_t as5600_handle_t;

/**
 * @brief   Initialize AS5600 direction GPIO
 * 
 * @param   gpio_num GPIO number for DIR pin
 * @return  esp_err_t 
 */
esp_err_t as5600_init_dir(gpio_num_t gpio_num);

/**
 * @brief   Set AS5600 direction
 * 
 * @param   gpio_num GPIO number for DIR pin
 * @param   dir 0 for clockwise, 1 for counter-clockwise
 * @return  esp_err_t 
 */
esp_err_t as5600_set_dir(gpio_num_t gpio_num, uint8_t dir);

/**
 * @brief   Get AS5600 status
 * 
 * @param   handle AS5600 device handle
 * @param   status Pointer to status structure
 * @return  esp_err_t 
 */
esp_err_t as5600_get_status(as5600_handle_t handle, as5600_status_t *status);

/**
 * @brief   Get AS5600 angle
 * 
 * @param   handle AS5600 device handle
 * @param   angle Pointer to store 12-bit angle (0-4095)
 * @return  esp_err_t 
 */
esp_err_t as5600_get_angle(as5600_handle_t handle, uint16_t *angle);

/**
 * @brief   Get AS5600 Automatic Gain Control value
 * 
 * @param   handle AS5600 device handle
 * @param   agc Pointer to store AGC value
 * @return  esp_err_t 
 */
esp_err_t as5600_get_agc(as5600_handle_t handle, uint8_t *agc);

/**
 * @brief   Process angle relative to a reference
 * 
 * @param   angle Current angle (0-4095)
 * @param   ref_angle Reference angle (0-4095)
 * @return  int8_t Scaled difference (-127 to 127)
 */
int8_t as5600_process_angle(uint16_t angle, uint16_t ref_angle);

#endif /* AS5600_H */

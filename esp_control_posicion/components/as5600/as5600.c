#include "as5600.h"
#include "esp_log.h"

static const char *TAG = "AS5600";

esp_err_t as5600_init_dir(gpio_num_t gpio_num)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << gpio_num),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret == ESP_OK) {
        gpio_set_level(gpio_num, 0);
    }
    return ret;
}

esp_err_t as5600_set_dir(gpio_num_t gpio_num, uint8_t dir)
{
    return gpio_set_level(gpio_num, dir);
}

esp_err_t as5600_get_status(as5600_handle_t handle, as5600_status_t *status)
{
    uint8_t reg = AS5600_STATUS_REG;
    uint8_t buf;
    esp_err_t ret = i2c_master_transmit_receive(handle, &reg, 1, &buf, 1, 1000);
    if (ret != ESP_OK) {
        return ret;
    }

    buf = buf >> 3;
    status->mh = buf & 0b001;
    status->ml = (buf & 0b010) >> 1;
    status->md = (buf & 0b100) >> 2;
    status->valid = (status->md && !status->ml && !status->mh) ? 1 : 0;

    return ESP_OK;
}

esp_err_t as5600_get_angle(as5600_handle_t handle, uint16_t *angle)
{
    uint8_t reg = AS5600_ANGLE_REG_HIGH;
    uint8_t buffer[2];
    esp_err_t ret = i2c_master_transmit_receive(handle, &reg, 1, buffer, 2, 1000);
    if (ret != ESP_OK) {
        return ret;
    }

    *angle = ((uint16_t)(buffer[0] & 0x0F) << 8) | buffer[1];
    return ESP_OK;
}

esp_err_t as5600_get_agc(as5600_handle_t handle, uint8_t *agc)
{
    uint8_t reg = AS5600_AGC_REG;
    return i2c_master_transmit_receive(handle, &reg, 1, agc, 1, 1000);
}

int8_t as5600_process_angle(uint16_t angle, uint16_t ref_angle)
{
    int16_t diff = (int16_t)angle - (int16_t)ref_angle;
    // Handle wrap-around (0-4095)
    if (diff > 2048) {
        diff -= 4096;
    }
    if (diff < -2048) {
        diff += 4096;
    }
    // Scale to -127 to 127
    int32_t out_angle = ((int32_t)diff * 127) / 2048;
    if (out_angle > 127) {
        out_angle = 127;
    }
    if (out_angle < -127) {
        out_angle = -127;
    }
    return (int8_t)out_angle;
}

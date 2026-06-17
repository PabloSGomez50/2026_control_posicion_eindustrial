#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "as5600.h"
#include "l298n.h"

#include "esp_lcd_gc9a01.h"
#include "LVGL_Driver.h"

#include "ui.h"

#define TAG "EI_MAIN"
#define DIR_GPIO_NUM GPIO_NUM_5
#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_MASTER_SCL_IO GPIO_NUM_9
#define I2C_MASTER_SDA_IO GPIO_NUM_8

esp_lcd_panel_handle_t panel_handle = NULL;

void init_panel() {
    ESP_LOGI(TAG, "Initialize SPI bus");
    const spi_bus_config_t bus_config = GC9A01_PANEL_BUS_SPI_CONFIG(EXAMPLE_PIN_NUM_LCD_PCLK, EXAMPLE_PIN_NUM_LCD_MOSI,
                                                                    EXAMPLE_LCD_H_RES * 80 * sizeof(uint16_t));
    ESP_ERROR_CHECK(spi_bus_initialize(EXAMPLE_LCD_HOST, &bus_config, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = GC9A01_PANEL_IO_SPI_CONFIG(
        -1, EXAMPLE_PIN_NUM_LCD_DC, // CS set to -1 because it is physically grounded
        example_notify_lvgl_flush_ready, &disp_drv);
    
    // Reducir la velocidad de SPI a 20MHz. 80MHz es demasiado para cables largos o protoboards.
    io_config.pclk_hz = 20 * 1000 * 1000; 

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)EXAMPLE_LCD_HOST, &io_config, &io_handle));


    ESP_LOGI(TAG, "Install GC9A01 panel driver");
    
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,   // Si RST está "al aire", conéctalo a 3.3V o a un pin GPIO
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,  // GC9A01 suele usar RGB con LVGL
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(io_handle, &panel_config, &panel_handle));
    
    // Si RST es -1 y está al aire, el driver no puede resetear el chip.
    // Se recomienda encarecidamente conectar físicamente RST a 3.3V o a un GPIO.
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    
    // Invertir colores si se ve como un "negativo" (común en pantallas IPS)
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
}

void app_main(void)
{
    init_panel();
    LVGL_Init();
    ui_init();

    // as5600_init_dir(DIR_GPIO_NUM);
    // as5600_set_dir(DIR_GPIO_NUM, 0);
    // i2c_master_bus_handle_t bus_handle;
    // i2c_master_bus_config_t i2c_mst_config = {
    //     .clk_source = I2C_CLK_SRC_DEFAULT,
    //     .i2c_port = I2C_MASTER_PORT,
    //     .scl_io_num = I2C_MASTER_SCL_IO,
    //     .sda_io_num = I2C_MASTER_SDA_IO,
    //     .glitch_ignore_cnt = 7,
    //     .flags.enable_internal_pullup = true,
    // };

    // ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    // i2c_device_config_t as5600_cfg = {
    //     .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    //     .device_address = AS5600_ADDRESS,
    //     .scl_speed_hz = 100000,
    // };
    // i2c_master_dev_handle_t as5600_handle;

    // ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &as5600_cfg, &as5600_handle));

    // as5600_status_t status;
    // uint16_t angle;

    ESP_LOGI(TAG, "-- Inicializacion del programa -- ");

    while (1) {
        lv_timer_handler();

        // if (as5600_get_status((as5600_handle_t)as5600_handle, &status) == ESP_OK && status.valid) {
        //     if (as5600_get_angle((as5600_handle_t)as5600_handle, &angle) == ESP_OK) {
        //         ESP_LOGI(TAG, "Angle: %d", angle);
        //     } else {
        //         ESP_LOGE(TAG, "Failed to read angle");
        //     }
        // } else {
        //     // ESP_LOGE(TAG, "Invalid status"); // Commented out to avoid spamming if sensor not connected
        // }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
#pragma once
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "demos/lv_demos.h"

#include "esp_lcd_gc9a01.h"

// #include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
// #include "esp_err.h"
// #include "lvgl.h"
// #include "demos/lv_demos.h"
#include "driver/ledc.h"


#define EXAMPLE_LCD_HOST SPI2_HOST
#define EXAMPLE_PIN_NUM_LCD_PCLK GPIO_NUM_4
#define EXAMPLE_PIN_NUM_LCD_MOSI GPIO_NUM_6
#define EXAMPLE_PIN_NUM_LCD_CS GPIO_NUM_7
#define EXAMPLE_PIN_NUM_LCD_DC GPIO_NUM_3
#define EXAMPLE_PIN_NUM_LCD_RST -1
#define EXAMPLE_LCD_H_RES 240
#define EXAMPLE_LCD_V_RES 240

#define Offset_X 0
#define Offset_Y 0

#define LVGL_BUF_LEN  (EXAMPLE_LCD_H_RES * 20)
#define EXAMPLE_LVGL_TICK_PERIOD_MS    2

extern esp_lcd_panel_handle_t panel_handle;
extern lv_disp_draw_buf_t disp_buf;                                                 // contains internal graphic buffer(s) called draw buffer(s)
extern lv_disp_drv_t disp_drv;                                                      // contains callback functions
extern lv_disp_t *disp;    

bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx);
void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map);
/* Rotate display and touch, when rotated screen in LVGL. Called when driver parameters are updated. */
void example_lvgl_port_update_callback(lv_disp_drv_t *drv);
void example_increase_lvgl_tick(void *arg);

void LVGL_Init(void);                     // Call this function to initialize the screen (must be called in the main function) !!!!!
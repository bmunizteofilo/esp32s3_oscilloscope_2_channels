/**
 * @file main.c
 * @brief Bootstrap da aplicação com LCD ST7796, touch FT6336U e interface LVGL.
 */

#include "esp_check.h"
#include "esp_log.h"
#include "adc_scope.h"
#include "ft6336u.h"
#include "lvgl_app.h"
#include "st7796.h"
#include "st7796_port.h"

/** @brief Tag de log da aplicação. */
static const char *TAG = "app";

/**
 * @brief Inicializa o módulo de captura contínua do ADC.
 */
static void app_init_adc_scope(void)
{
    adc_scope_config_t adc_config = adc_scope_get_default_config();

    adc_config.unit = APP_ADC_UNIT;
    adc_config.channel_count = 2U;
    adc_config.channels[0] = APP_ADC_CHANNEL_1;
    adc_config.channels[1] = APP_ADC_CHANNEL_2;
    adc_config.attenuations[0] = APP_ADC_ATTENUATION_1;
    adc_config.attenuations[1] = APP_ADC_ATTENUATION_2;
    adc_config.sample_freq_hz = APP_ADC_SAMPLE_FREQ_HZ;
    adc_config.circular_buffer_capacity = APP_ADC_HISTORY_SAMPLES;
    adc_config.chart_point_count = APP_ADC_CHART_POINTS;
    adc_config.default_block_sample_count = APP_ADC_BLOCK_SAMPLES;
    adc_config.conv_frame_size = APP_ADC_DMA_FRAME_BYTES;
    adc_config.max_store_buf_size = APP_ADC_STORE_BUFFER_BYTES;
    adc_config.task_stack_size = APP_ADC_TASK_STACK_SIZE;
    adc_config.task_priority = APP_ADC_TASK_PRIORITY;

    ESP_ERROR_CHECK(adc_scope_init(&adc_config));
    ESP_ERROR_CHECK(adc_scope_start());
}

/**
 * @brief Verifica se os pinos obrigatórios do barramento já foram configurados.
 *
 * @return `true` se todos os pinos obrigatórios foram preenchidos.
 */
static bool board_pins_are_configured(void)
{
    const int required_pins[] = {
        ST7796_PIN_DC, ST7796_PIN_WR, ST7796_PIN_D0, ST7796_PIN_D1, ST7796_PIN_D2,
        ST7796_PIN_D3, ST7796_PIN_D4, ST7796_PIN_D5, ST7796_PIN_D6, ST7796_PIN_D7,
    };

    for (size_t i = 0; i < sizeof(required_pins) / sizeof(required_pins[0]); i++) {
        if (required_pins[i] < 0) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Inicializa o driver ST7796 e liga o painel.
 *
 * @param[out] out_lcd Handle retornado do driver.
 */
static void app_init_lcd(st7796_handle_t *out_lcd)
{
    const size_t draw_buffer_pixels = (size_t)ST7796_H_RES * (size_t)ST7796_DRAW_BUFFER_LINES;
    const size_t draw_buffer_size = draw_buffer_pixels * sizeof(uint16_t);

    const st7796_config_t lcd_config = {
        .reset_gpio_num = ST7796_PIN_RESET,
        .backlight_gpio_num = ST7796_PIN_BACKLIGHT,
        .backlight_on_level = ST7796_BACKLIGHT_ON_LEVEL,
        .dc_gpio_num = ST7796_PIN_DC,
        .wr_gpio_num = ST7796_PIN_WR,
        .cs_gpio_num = ST7796_PIN_CS,
        .data_gpio_nums = {ST7796_PIN_D0, ST7796_PIN_D1, ST7796_PIN_D2, ST7796_PIN_D3, ST7796_PIN_D4, ST7796_PIN_D5, ST7796_PIN_D6, ST7796_PIN_D7},
        .pclk_hz = ST7796_PIXEL_CLOCK_HZ,
        .trans_queue_depth = ST7796_DMA_QUEUE_DEPTH,
        .max_transfer_bytes = draw_buffer_size,
        .dma_burst_size = ST7796_DMA_BURST_SIZE,
        .h_res = ST7796_H_RES,
        .v_res = ST7796_V_RES,
        .x_gap = ST7796_X_GAP,
        .y_gap = ST7796_Y_GAP,
        .swap_xy = ST7796_SWAP_XY,
        .mirror_x = ST7796_MIRROR_X,
        .mirror_y = ST7796_MIRROR_Y,
        .invert_colors = ST7796_INVERT_COLORS,
        .bgr = ST7796_BGR_ORDER,
        .swap_color_bytes = ST7796_SWAP_COLOR_BYTES,
        .pclk_active_neg = ST7796_PCLK_ACTIVE_NEG,
        .pclk_idle_low = ST7796_PCLK_IDLE_LOW,
        .init_cmds = NULL,
        .init_cmds_count = 0,
    };

    ESP_ERROR_CHECK(st7796_new(&lcd_config, out_lcd));
    ESP_ERROR_CHECK(st7796_reset(*out_lcd));
    ESP_ERROR_CHECK(st7796_init(*out_lcd));
    ESP_ERROR_CHECK(st7796_set_display_on(*out_lcd, true));

    if (ST7796_PIN_BACKLIGHT >= 0) {
        ESP_ERROR_CHECK(st7796_set_backlight(*out_lcd, true));
    }
}

/**
 * @brief Inicializa o controlador touch FT6336U.
 *
 * @param[out] out_touch Handle retornado do touch.
 */
static void app_init_touch(ft6336u_handle_t *out_touch)
{
    const ft6336u_config_t touch_config = {
        .i2c_port = -1,
        .sda_gpio_num = FT6336U_PIN_SDA,
        .scl_gpio_num = FT6336U_PIN_SCL,
        .int_gpio_num = FT6336U_PIN_INT,
        .reset_gpio_num = FT6336U_PIN_RST,
        .skip_reset = FT6336U_SKIP_RESET,
        .reset_active_low = true,
        .enable_internal_pullup = true,
        .i2c_clk_speed_hz = FT6336U_I2C_CLOCK_HZ,
        .x_max = ST7796_H_RES,
        .y_max = ST7796_V_RES,
        .swap_xy = FT6336U_SWAP_XY,
        .mirror_x = FT6336U_MIRROR_X,
        .mirror_y = FT6336U_MIRROR_Y,
    };

    ESP_ERROR_CHECK(ft6336u_new(&touch_config, out_touch));
    ESP_ERROR_CHECK(ft6336u_init(*out_touch));
}

/**
 * @brief Ponto de entrada da aplicação.
 */
void app_main(void)
{
    st7796_handle_t lcd = NULL;
    ft6336u_handle_t touch = NULL;

    if (!board_pins_are_configured()) {
        ESP_LOGW(TAG, "Preencha os defines em st7796_port.h com os GPIOs reais do hardware.");
        return;
    }

    app_init_lcd(&lcd);
    app_init_touch(&touch);
    app_init_adc_scope();
    lvgl_app_start(lcd, touch);
}

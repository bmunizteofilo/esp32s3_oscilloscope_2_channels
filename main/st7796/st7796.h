#pragma once

/**
 * @file st7796.h
 * @brief Driver modular do controlador LCD ST7796 usando barramento paralelo Intel 8080 de 8 bits.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_lcd_panel_io.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct st7796_t *st7796_handle_t;

/**
 * @brief Item de inicialização customizada do ST7796.
 */
typedef struct {
    uint8_t cmd;
    const uint8_t *data;
    size_t data_size;
    uint32_t delay_ms;
} st7796_init_cmd_t;

/**
 * @brief Configuração completa do driver ST7796.
 */
typedef struct {
    int reset_gpio_num;
    int backlight_gpio_num;
    uint32_t backlight_on_level;
    int dc_gpio_num;
    int wr_gpio_num;
    int cs_gpio_num;
    int data_gpio_nums[8];
    uint32_t pclk_hz;
    size_t trans_queue_depth;
    size_t max_transfer_bytes;
    size_t dma_burst_size;
    uint16_t h_res;
    uint16_t v_res;
    uint16_t x_gap;
    uint16_t y_gap;
    bool swap_xy;
    bool mirror_x;
    bool mirror_y;
    bool invert_colors;
    bool bgr;
    bool swap_color_bytes;
    bool pclk_active_neg;
    bool pclk_idle_low;
    const st7796_init_cmd_t *init_cmds;
    size_t init_cmds_count;
} st7796_config_t;

/** @brief Cria a instância do driver ST7796. */
esp_err_t st7796_new(const st7796_config_t *config, st7796_handle_t *out_handle);

/** @brief Libera todos os recursos do driver ST7796. */
esp_err_t st7796_del(st7796_handle_t handle);

/** @brief Executa reset por hardware ou software no painel. */
esp_err_t st7796_reset(st7796_handle_t handle);

/** @brief Inicializa o ST7796 com a sequência padrão ou a sequência customizada. */
esp_err_t st7796_init(st7796_handle_t handle);

/** @brief Liga ou desliga a saída de imagem do painel. */
esp_err_t st7796_set_display_on(st7796_handle_t handle, bool on);

/** @brief Coloca o painel em sleep ou retira o painel de sleep. */
esp_err_t st7796_set_sleep(st7796_handle_t handle, bool sleep);

/** @brief Liga ou desliga a inversão de cores do painel. */
esp_err_t st7796_set_invert(st7796_handle_t handle, bool enabled);

/** @brief Liga ou desliga o backlight quando ele foi configurado no driver. */
esp_err_t st7796_set_backlight(st7796_handle_t handle, bool on);

/** @brief Atualiza os gaps usados na programação da janela de escrita. */
esp_err_t st7796_set_gap(st7796_handle_t handle, uint16_t x_gap, uint16_t y_gap);

/** @brief Atualiza espelhamento e troca de eixos via MADCTL. */
esp_err_t st7796_set_rotation(st7796_handle_t handle, bool mirror_x, bool mirror_y, bool swap_xy);

/** @brief Envia um comando curto por polling. */
esp_err_t st7796_tx_param(st7796_handle_t handle, uint8_t cmd, const void *data, size_t data_size);

/** @brief Programa a janela de acesso à GRAM do painel. */
esp_err_t st7796_set_address_window(st7796_handle_t handle, uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end);

/** @brief Envia um frame RGB565 e aguarda o término da DMA. */
esp_err_t st7796_draw_bitmap_blocking(st7796_handle_t handle, uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, const void *frame_data);

/** @brief Envia um frame RGB565 de forma assíncrona usando DMA. */
esp_err_t st7796_draw_bitmap_async(st7796_handle_t handle, uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, const void *frame_data);

/** @brief Aguarda a conclusão de todas as transferências DMA pendentes. */
esp_err_t st7796_wait_all_dma_done(st7796_handle_t handle, uint32_t timeout_ms);

/** @brief Aloca um buffer adequado para transferências DMA do barramento I80. */
void *st7796_alloc_dma_buffer(st7796_handle_t handle, size_t size);

/** @brief Retorna a largura lógica configurada do painel. */
uint16_t st7796_get_width(st7796_handle_t handle);

/** @brief Retorna a altura lógica configurada do painel. */
uint16_t st7796_get_height(st7796_handle_t handle);

#ifdef __cplusplus
}
#endif

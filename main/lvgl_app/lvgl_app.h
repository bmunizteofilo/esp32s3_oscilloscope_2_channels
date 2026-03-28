#pragma once

/**
 * @file lvgl_app.h
 * @brief Módulo de integração do LVGL com o display ST7796 e o touch FT6336U.
 */

#include "st7796.h"
#include "ft6336u.h"
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Estado compartilhado entre a UI local e a interface web.
 */
typedef struct {
    uint16_t sample_channel_mode;   /**< Canal exibido atualmente: 0=Ch1, 1=Ch2, 2=Ch1+Ch2. */
    uint16_t timebase_index;        /**< Índice da base de tempo selecionada. */
    uint16_t voltscale_index;       /**< Índice da escala vertical selecionada. */
    uint16_t trigger_channel_index; /**< Índice do canal de trigger selecionado. */
    uint16_t trigger_mode;          /**< Borda do trigger: 1=subida, 2=descida. */
    uint16_t trigger_run_mode;      /**< Modo do trigger: 0=off, 1=auto, 2=normal, 3=single. */
    bool paused;                    /**< Indica se a captura está pausada. */
} lvgl_app_control_state_t;

/** @brief Inicializa o LVGL, a interface de osciloscópio e a task dedicada da UI. */
void lvgl_app_start(st7796_handle_t lcd, ft6336u_handle_t touch);

/**
 * @brief Retorna o estado atual compartilhado da interface.
 *
 * @param[out] out_state Estrutura preenchida com o estado atual.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
esp_err_t lvgl_app_get_control_state(lvgl_app_control_state_t *out_state);

/**
 * @brief Agenda a aplicação de um novo estado vindo de uma interface externa.
 *
 * @param[in] state Novo estado desejado.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
esp_err_t lvgl_app_request_control_state(const lvgl_app_control_state_t *state);

/**
 * @brief Agenda a execução do Auto Set no contexto da task do LVGL.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
esp_err_t lvgl_app_request_auto_set(void);

/**
 * @brief Agenda a alternância do modo foco do chart, equivalente ao atalho de triple-tap.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
esp_err_t lvgl_app_request_toggle_focus(void);

#ifdef __cplusplus
}
#endif

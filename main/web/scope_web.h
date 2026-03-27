#pragma once

/**
 * @file scope_web.h
 * @brief Interface web do osciloscópio via SoftAP e navegador.
 */

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Define quais saídas visuais do osciloscópio estão ativas.
 */
typedef enum {
    SCOPE_OUTPUT_MODE_BOTH = 0,         /**< Mantém display local e interface web ativos. */
    SCOPE_OUTPUT_MODE_WEB_ONLY,         /**< Prioriza apenas a interface web. */
    SCOPE_OUTPUT_MODE_DISPLAY_ONLY,     /**< Prioriza apenas o display local. */
} scope_output_mode_t;

/**
 * @brief Inicializa o SoftAP e o servidor HTTP da interface web.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
esp_err_t scope_web_start(void);

/**
 * @brief Define o modo de saída visual ativo do osciloscópio.
 *
 * @param[in] mode Novo modo desejado.
 */
void scope_web_set_output_mode(scope_output_mode_t mode);

/**
 * @brief Retorna o modo de saída visual ativo do osciloscópio.
 *
 * @return Modo atual.
 */
scope_output_mode_t scope_web_get_output_mode(void);

#ifdef __cplusplus
}
#endif

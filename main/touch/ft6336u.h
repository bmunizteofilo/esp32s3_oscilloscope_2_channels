#pragma once

/**
 * @file ft6336u.h
 * @brief Driver do controlador touch capacitivo FT6336U usando I2C.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Handle opaco da instância do driver FT6336U. */
typedef struct ft6336u_t *ft6336u_handle_t;

/** @brief Estrutura com um ponto de toque lido do controlador. */
typedef struct {
    bool pressed;        /*!< Indica se o ponto está pressionado. */
    uint16_t x;          /*!< Coordenada X do ponto. */
    uint16_t y;          /*!< Coordenada Y do ponto. */
    uint8_t id;          /*!< ID do toque reportado pelo controlador. */
    uint8_t event;       /*!< Tipo de evento bruto vindo do FT6336U. */
    uint8_t weight;      /*!< Peso do toque. */
    uint8_t area;        /*!< Área do toque. */
} ft6336u_point_t;

/** @brief Resultado completo da leitura do controlador. */
typedef struct {
    uint8_t touch_count;         /*!< Quantidade de toques detectados no instante da leitura. */
    ft6336u_point_t points[2];   /*!< Até dois pontos suportados pelo FT6336U. */
} ft6336u_touch_data_t;

/** @brief Configuração do driver FT6336U. */
typedef struct {
    i2c_port_num_t i2c_port;     /*!< Porta I2C usada. Use `-1` para seleção automática. */
    int sda_gpio_num;            /*!< GPIO da linha SDA. */
    int scl_gpio_num;            /*!< GPIO da linha SCL. */
    int int_gpio_num;            /*!< GPIO da linha de interrupção. Use `-1` se não estiver ligado. */
    int reset_gpio_num;          /*!< GPIO do reset do touch. Use `-1` para desabilitar reset dedicado. */
    bool skip_reset;             /*!< Quando `true`, não executa reset de hardware no init. Útil para reset compartilhado com o LCD. */
    bool reset_active_low;       /*!< Polaridade ativa do reset do touch. */
    bool enable_internal_pullup; /*!< Habilita pull-up interno nas linhas I2C. */
    uint32_t i2c_clk_speed_hz;   /*!< Clock do barramento I2C. */
    uint16_t x_max;              /*!< Limite máximo da coordenada X para a integração com a UI. */
    uint16_t y_max;              /*!< Limite máximo da coordenada Y para a integração com a UI. */
    bool swap_xy;                /*!< Troca os eixos X e Y na leitura. */
    bool mirror_x;               /*!< Espelha a coordenada X. */
    bool mirror_y;               /*!< Espelha a coordenada Y. */
} ft6336u_config_t;

/** @brief Cria e inicializa a instância do driver FT6336U. */
esp_err_t ft6336u_new(const ft6336u_config_t *config, ft6336u_handle_t *out_handle);

/** @brief Libera os recursos da instância do FT6336U. */
esp_err_t ft6336u_del(ft6336u_handle_t handle);

/** @brief Executa reset por hardware quando configurado. */
esp_err_t ft6336u_reset(ft6336u_handle_t handle);

/** @brief Inicializa o controlador touch e valida a comunicação básica. */
esp_err_t ft6336u_init(ft6336u_handle_t handle);

/** @brief Lê um registrador de 8 bits do FT6336U. */
esp_err_t ft6336u_read_reg(ft6336u_handle_t handle, uint8_t reg_addr, uint8_t *value);

/** @brief Escreve um registrador de 8 bits do FT6336U. */
esp_err_t ft6336u_write_reg(ft6336u_handle_t handle, uint8_t reg_addr, uint8_t value);

/** @brief Lê o quadro atual de toque com até dois pontos. */
esp_err_t ft6336u_read_touch_data(ft6336u_handle_t handle, ft6336u_touch_data_t *touch_data);

/** @brief Retorna `true` quando a linha de interrupção indica atividade de toque. */
bool ft6336u_is_interrupt_active(ft6336u_handle_t handle);

/** @brief Atualiza a transformação das coordenadas lidas. */
esp_err_t ft6336u_set_transform(ft6336u_handle_t handle, bool swap_xy, bool mirror_x, bool mirror_y);

/** @brief Retorna a versão de firmware reportada pelo controlador. */
esp_err_t ft6336u_get_firmware_id(ft6336u_handle_t handle, uint8_t *firmware_id);

/** @brief Retorna o chip ID reportado pelo controlador. */
esp_err_t ft6336u_get_chip_id(ft6336u_handle_t handle, uint8_t *chip_id);

#ifdef __cplusplus
}
#endif

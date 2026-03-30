#pragma once

/**
 * @file adc_scope.h
 * @brief Captura contínua de ADC com DMA e buffers nos modos circular e bloco.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "hal/adc_types.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Quantidade máxima de canais analógicos tratados pelo módulo. */
#define ADC_SCOPE_MAX_CHANNELS (2U)
#define ADC_SCOPE_FREE_HISTORY_BLOCKS (32U)

/**
 * @brief Modos de aquisição suportados pelo módulo.
 */
typedef enum {
    ADC_SCOPE_MODE_CIRCULAR = 0,  /**< Mantém uma janela deslizante com as amostras mais recentes. */
    ADC_SCOPE_MODE_BLOCK,         /**< Captura um bloco finito de amostras e congela o resultado. */
} adc_scope_mode_t;

/**
 * @brief Modos de trigger suportados para composição da janela do gráfico.
 */
typedef enum {
    ADC_SCOPE_TRIGGER_FREE = 0,  /**< Exibe a janela mais recente sem procurar borda. */
    ADC_SCOPE_TRIGGER_RISE,      /**< Procura uma borda de subida antes de montar a janela. */
    ADC_SCOPE_TRIGGER_FALL,      /**< Procura uma borda de descida antes de montar a janela. */
} adc_scope_trigger_mode_t;

/**
 * @brief Modos de execução do sistema de trigger.
 */
typedef enum {
    ADC_SCOPE_TRIGGER_RUN_OFF = 0,    /**< Trigger desligado; a tela se comporta como modo livre. */
    ADC_SCOPE_TRIGGER_RUN_AUTO,       /**< Atualiza a tela mesmo sem trigger válido. */
    ADC_SCOPE_TRIGGER_RUN_NORMAL,     /**< Só atualiza a tela quando encontra trigger válido. */
    ADC_SCOPE_TRIGGER_RUN_SINGLE,     /**< Para a aquisição após o primeiro trigger válido. */
} adc_scope_trigger_run_mode_t;

/**
 * @brief Configuração inicial do módulo de aquisição.
 */
typedef struct {
    adc_unit_t unit;                                   /**< Unidade ADC compartilhada pelos canais configurados. */
    size_t channel_count;                              /**< Quantidade de canais habilitados. */
    adc_channel_t channels[ADC_SCOPE_MAX_CHANNELS];    /**< Canais ADC amostrados. */
    adc_atten_t attenuations[ADC_SCOPE_MAX_CHANNELS];  /**< Atenuação aplicada a cada canal. */
    adc_bitwidth_t bitwidth;                           /**< Resolução bruta entregue pelo hardware. */
    uint32_t sample_freq_hz;                           /**< Frequência de amostragem desejada em hertz por canal. */
    uint32_t max_store_buf_size;                       /**< Tamanho do pool interno do driver contínuo em bytes. */
    uint32_t conv_frame_size;                          /**< Tamanho de um frame DMA em bytes. */
    size_t circular_buffer_capacity;                   /**< Quantidade de amostras retidas no histórico circular. */
    size_t chart_point_count;                          /**< Quantidade de pontos usados para exibir o gráfico. */
    size_t default_block_sample_count;                 /**< Quantidade padrão de amostras para captura em bloco. */
    uint32_t task_priority;                            /**< Prioridade da task de aquisição. */
    uint32_t task_stack_size;                          /**< Tamanho da stack da task de aquisição em bytes. */
} adc_scope_config_t;

/**
 * @brief Metadados da última janela copiada para a interface.
 */
typedef struct {
    adc_scope_mode_t mode;                                  /**< Modo de aquisição ativo. */
    size_t channel_count;                                   /**< Quantidade de canais disponíveis no snapshot. */
    size_t sample_count;                                    /**< Quantidade válida de pontos presentes na janela lógica. */
    size_t capacity;                                        /**< Capacidade total do gráfico. */
    size_t history_count;                                   /**< Quantidade total de amostras disponíveis no histórico atual. */
    size_t history_capacity;                                /**< Capacidade total do buffer de histórico atual. */
    uint64_t latest_sequence[ADC_SCOPE_MAX_CHANNELS];       /**< Sequência absoluta mais recente processada por canal. */
    uint64_t window_start_fp_q10[ADC_SCOPE_MAX_CHANNELS];   /**< Início absoluto da janela renderizada em amostras Q10 por canal. */
    uint32_t latest_raw[ADC_SCOPE_MAX_CHANNELS];            /**< Última amostra bruta recebida por canal. */
    int32_t latest_mv[ADC_SCOPE_MAX_CHANNELS];              /**< Última amostra convertida para milivolts por canal. */
    int32_t min_mv[ADC_SCOPE_MAX_CHANNELS];                 /**< Menor valor da janela atual em milivolts por canal. */
    int32_t max_mv[ADC_SCOPE_MAX_CHANNELS];                 /**< Maior valor da janela atual em milivolts por canal. */
    uint32_t frequency_tenths_hz[ADC_SCOPE_MAX_CHANNELS];   /**< Frequência estimada por canal em décimos de hertz. */
    uint32_t duty_tenths_percent[ADC_SCOPE_MAX_CHANNELS];   /**< Duty cycle estimado por canal em décimos de porcentagem. */
    bool measurements_valid[ADC_SCOPE_MAX_CHANNELS];        /**< Indica se frequência e duty foram calculados com sucesso. */
    size_t trigger_channel_index;                           /**< Índice do canal usado como referência do trigger. */
    int32_t trigger_level_mv;                               /**< Nível de trigger usado na busca da borda. */
    uint32_t trigger_hysteresis_mv;                         /**< Histerese usada na busca da borda. */
    size_t trigger_sample_index;                            /**< Índice relativo da borda encontrada dentro da janela lógica. */
    bool calibrated[ADC_SCOPE_MAX_CHANNELS];                /**< Indica se a conversão para tensão usa calibração em cada canal. */
    bool capture_ready;                                     /**< Indica se a última captura em bloco foi concluída. */
    bool acquisition_running;                               /**< Indica se a aquisição contínua está em execução. */
    bool trigger_found;                                     /**< Indica se a janela atual foi ancorada por um trigger válido. */
    bool trigger_pending;                                   /**< Indica se já existe evento de trigger aguardando sweep completa. */
    bool overflow_seen;                                     /**< Indica se houve overflow do pool interno desde a inicialização. */
} adc_scope_snapshot_t;

/**
 * @brief Retorna uma configuração padrão adequada para o exemplo.
 *
 * @return Estrutura preenchida com valores seguros para partida.
 */
adc_scope_config_t adc_scope_get_default_config(void);

/**
 * @brief Inicializa o módulo e cria a task de aquisição.
 *
 * @param[in] config Configuração desejada.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
esp_err_t adc_scope_init(const adc_scope_config_t *config);

/**
 * @brief Inicia a aquisição contínua via ADC + DMA.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
esp_err_t adc_scope_start(void);

/**
 * @brief Interrompe a aquisição contínua via ADC + DMA.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
esp_err_t adc_scope_stop(void);

/**
 * @brief Altera o modo de aquisição.
 *
 * @param[in] mode Novo modo desejado.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
esp_err_t adc_scope_set_mode(adc_scope_mode_t mode);

/**
 * @brief Agenda uma captura em bloco com quantidade definida de amostras.
 *
 * @param[in] sample_count Número de amostras desejadas. Se zero, usa o valor padrão da configuração.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
esp_err_t adc_scope_capture_block(size_t sample_count);

/**
 * @brief Copia janelas alinhadas de até dois canais para buffers do gráfico.
 *
 * @param[out] dest_per_channel Vetor com buffers de destino por canal. Posições `NULL` serão ignoradas.
 * @param[in] point_count Quantidade de posições de cada buffer de destino.
 * @param[in] requested_samples Quantidade de amostras reais desejadas para compor a janela.
 * @param[in] trigger_mode Tipo de borda usada pelo trigger.
 * @param[in] trigger_run_mode Modo de execução do trigger.
 * @param[in] trigger_channel_index Índice do canal usado como referência do trigger.
 * @param[in] trigger_point_index Posição horizontal desejada para a borda de trigger no gráfico.
 * @param[in] history_offset_samples Quantidade de amostras recuadas em relação ao trecho mais recente.
 * @param[in] trigger_level_mv Nível de trigger em milivolts. Use `INT32_MIN` para nível automático.
 * @param[in] trigger_hysteresis_mv Histerese em milivolts. Use `0` para histerese automática.
 * @param[out] out_snapshot Metadados opcionais da janela copiada.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
esp_err_t adc_scope_copy_chart_points_multi(int32_t *dest_per_channel[ADC_SCOPE_MAX_CHANNELS],
                                            size_t point_count,
                                            size_t requested_samples,
                                            adc_scope_trigger_mode_t trigger_mode,
                                            adc_scope_trigger_run_mode_t trigger_run_mode,
                                            size_t trigger_channel_index,
                                            size_t trigger_point_index,
                                            size_t history_offset_samples,
                                            bool anchor_to_sequence,
                                            uint64_t anchor_sequence,
                                            int32_t trigger_level_mv,
                                            uint32_t trigger_hysteresis_mv,
                                            adc_scope_snapshot_t *out_snapshot);

/**
 * @brief Retorna a frequência de amostragem configurada para o ADC.
 *
 * @param[out] out_sample_freq_hz Frequência configurada em hertz.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
esp_err_t adc_scope_get_sample_freq_hz(uint32_t *out_sample_freq_hz);

esp_err_t adc_scope_get_effective_sample_freq_hz(uint32_t *out_sample_freq_hz);

esp_err_t adc_scope_get_circular_status(size_t channel_index,
                                        uint64_t *out_latest_sequence,
                                        size_t *out_shared_history_count);

esp_err_t adc_scope_copy_free_run_circular_window_multi(int32_t *dest_per_channel[ADC_SCOPE_MAX_CHANNELS],
                                                        size_t point_count,
                                                        size_t requested_samples,
                                                        uint64_t display_end_sequence,
                                                        int32_t trigger_level_mv,
                                                        adc_scope_snapshot_t *out_snapshot);

esp_err_t adc_scope_copy_trigger_window_multi(int32_t *dest_per_channel[ADC_SCOPE_MAX_CHANNELS],
                                              size_t point_count,
                                              size_t requested_samples,
                                              adc_scope_trigger_mode_t trigger_mode,
                                              adc_scope_trigger_run_mode_t trigger_run_mode,
                                              size_t trigger_channel_index,
                                              size_t trigger_point_index,
                                              int32_t trigger_level_mv,
                                              uint32_t trigger_hysteresis_mv,
                                              adc_scope_snapshot_t *out_snapshot);

/**
 * @brief Limpa o histórico atual de captura e invalida sweeps/cache dependentes.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
esp_err_t adc_scope_clear_history(void);

esp_err_t adc_scope_freeze_history_snapshot(void);

esp_err_t adc_scope_release_history_snapshot(void);

esp_err_t adc_scope_configure_free_run_block(size_t sample_count);

esp_err_t adc_scope_get_free_run_block(int32_t *dest_per_channel[ADC_SCOPE_MAX_CHANNELS],
                                       size_t point_count,
                                       size_t history_block_offset,
                                       adc_scope_snapshot_t *out_snapshot,
                                       bool *out_available);

/**
 * @brief Reconfigura a frequência de amostragem por canal do ADC contínuo.
 *
 * @param[in] sample_freq_hz Nova frequência desejada em hertz por canal.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
esp_err_t adc_scope_set_sample_freq_hz(uint32_t sample_freq_hz);

/**
 * @brief Reconfigura quantos canais físicos o ADC contínuo deve amostrar.
 *
 * @param[in] sample_channel_mode 0=Ch1, 1=Ch2, 2=Ch1+Ch2.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
esp_err_t adc_scope_set_active_channel_mode(uint16_t sample_channel_mode);

/**
 * @brief Retorna a quantidade de canais configurados.
 *
 * @param[out] out_channel_count Quantidade de canais habilitados.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
esp_err_t adc_scope_get_channel_count(size_t *out_channel_count);

/**
 * @brief Retorna o GPIO físico associado ao canal configurado.
 *
 * @param[in] channel_index Índice lógico do canal.
 * @param[out] out_gpio GPIO retornado pelo driver.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
esp_err_t adc_scope_get_gpio_num(size_t channel_index, int *out_gpio);

/**
 * @brief Configura o monitor de trigger por hardware do ADC contínuo.
 *
 * @param[in] trigger_channel_index Índice lógico do canal usado como fonte do trigger.
 * @param[in] trigger_mode Tipo de borda desejada. Use `ADC_SCOPE_TRIGGER_FREE` para desabilitar.
 * @param[in] trigger_level_mv Nível de trigger em milivolts.
 * @param[in] trigger_hysteresis_mv Histerese em milivolts. Use `0` para automático.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
esp_err_t adc_scope_configure_trigger_monitor(size_t trigger_channel_index,
                                              adc_scope_trigger_mode_t trigger_mode,
                                              int32_t trigger_level_mv,
                                              uint32_t trigger_hysteresis_mv,
                                              size_t requested_samples,
                                              size_t trigger_point_index,
                                              adc_scope_trigger_run_mode_t trigger_run_mode);

#ifdef __cplusplus
}
#endif

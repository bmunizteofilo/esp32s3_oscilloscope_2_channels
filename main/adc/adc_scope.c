/**
 * @file adc_scope.c
 * @brief Implementação da captura contínua do ADC com DMA e buffers para gráfico.
 */

#include "adc_scope.h"
#include <inttypes.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"

/** @brief Tag de log do módulo de aquisição. */
static const char *TAG = "adc_scope";

/** @brief Tempo máximo de espera no `adc_continuous_read` em milissegundos. */
#define ADC_SCOPE_READ_TIMEOUT_MS  (10U)

/** @brief Bits de notificação usados pela task de aquisição. */
#define ADC_SCOPE_NOTIFY_DATA      (1UL << 0)
#define ADC_SCOPE_NOTIFY_CONTROL   (1UL << 1)

/** @brief Estado interno do módulo de aquisição. */
typedef struct {
    adc_scope_config_t config;                                 /**< Cópia local da configuração do usuário. */
    adc_continuous_handle_t adc_handle;                        /**< Handle do driver contínuo do ADC. */
    adc_cali_handle_t cali_handle[ADC_SCOPE_MAX_CHANNELS];     /**< Handle de calibração por canal. */
    TaskHandle_t task_handle;                                  /**< Task que consome frames do ADC. */
    SemaphoreHandle_t mutex;                                   /**< Mutex que protege buffers compartilhados. */
    SemaphoreHandle_t control_done;                            /**< Sinaliza o término de comandos start/stop. */
    uint8_t *read_buffer;                                      /**< Buffer temporário para leitura do driver contínuo. */
    int32_t *circular_mv_buffer[ADC_SCOPE_MAX_CHANNELS];       /**< Buffer circular de tensão em milivolts por canal. */
    int32_t *block_mv_buffer[ADC_SCOPE_MAX_CHANNELS];          /**< Buffer usado na captura em bloco por canal. */
    size_t circular_head[ADC_SCOPE_MAX_CHANNELS];              /**< Próxima posição de escrita no buffer circular de cada canal. */
    size_t circular_count[ADC_SCOPE_MAX_CHANNELS];             /**< Quantidade válida de amostras no buffer circular de cada canal. */
    size_t block_count[ADC_SCOPE_MAX_CHANNELS];                /**< Quantidade válida de amostras no buffer em bloco de cada canal. */
    size_t block_target;                                       /**< Quantidade alvo da captura em bloco corrente. */
    adc_scope_mode_t mode;                                     /**< Modo de aquisição ativo. */
    bool initialized;                                          /**< Indica se o módulo já foi inicializado. */
    bool started;                                              /**< Indica se o ADC contínuo já foi iniciado. */
    bool control_start_pending;                                /**< Indica requisição pendente de start. */
    bool control_stop_pending;                                 /**< Indica requisição pendente de stop. */
    bool calibrated[ADC_SCOPE_MAX_CHANNELS];                   /**< Indica se o esquema de calibração foi criado por canal. */
    bool block_capture_ready;                                  /**< Indica se a captura em bloco foi concluída. */
    bool overflow_seen;                                        /**< Indica se ocorreu overflow do pool do driver. */
    uint32_t latest_raw[ADC_SCOPE_MAX_CHANNELS];               /**< Último valor bruto recebido por canal. */
    int32_t latest_mv[ADC_SCOPE_MAX_CHANNELS];                 /**< Último valor convertido por canal. */
    int gpio_num[ADC_SCOPE_MAX_CHANNELS];                      /**< GPIO físico associado a cada canal configurado. */
    esp_err_t control_result;                                  /**< Resultado do último comando de start/stop. */
} adc_scope_state_t;

/** @brief Estado global do módulo. Mantido em RAM interna para interação com ISR. */
static adc_scope_state_t s_scope = {
    .gpio_num = {-1, -1},
};

/**
 * @brief Retorna o índice lógico de um canal dentro da configuração ativa.
 *
 * @param[in] unit Unidade reportada pela amostra DMA.
 * @param[in] channel Canal reportado pela amostra DMA.
 *
 * @return Índice lógico do canal ou `-1` quando a amostra não pertence ao escopo.
 */
static int adc_scope_find_channel_index(adc_unit_t unit, adc_channel_t channel)
{
    if (unit != s_scope.config.unit) {
        return -1;
    }

    for (size_t i = 0; i < s_scope.config.channel_count; i++) {
        if (s_scope.config.channels[i] == channel) {
            return (int)i;
        }
    }

    return -1;
}

/**
 * @brief Retorna o menor contador de histórico disponível entre os canais ativos.
 *
 * @param[in] circular_source Indica se a origem é o buffer circular.
 *
 * @return Quantidade de amostras válidas compartilhadas.
 */
static size_t adc_scope_get_shared_count_locked(bool circular_source)
{
    size_t count = SIZE_MAX;

    for (size_t i = 0; i < s_scope.config.channel_count; i++) {
        const size_t current = circular_source ? s_scope.circular_count[i] : s_scope.block_count[i];
        if (current < count) {
            count = current;
        }
    }

    return (count == SIZE_MAX) ? 0U : count;
}

/**
 * @brief Lê uma amostra relativa ao início lógico do histórico de um canal.
 *
 * @param[in] channel_index Índice lógico do canal.
 * @param[in] circular_source Indica se a origem é o buffer circular.
 * @param[in] oldest_start Índice do elemento mais antigo no buffer circular do canal.
 * @param[in] rel_index Índice relativo a partir do início lógico da origem.
 *
 * @return Amostra em milivolts.
 */
static int32_t adc_scope_get_source_sample_locked(size_t channel_index,
                                                  bool circular_source,
                                                  size_t oldest_start,
                                                  size_t rel_index)
{
    if (circular_source) {
        return s_scope.circular_mv_buffer[channel_index][(oldest_start + rel_index) % s_scope.config.circular_buffer_capacity];
    }

    return s_scope.block_mv_buffer[channel_index][rel_index];
}

/**
 * @brief Renderiza uma janela do histórico real de um canal no buffer do gráfico.
 *
 * @param[in] channel_index Índice lógico do canal.
 * @param[in] circular_source Indica se a origem é o buffer circular.
 * @param[in] oldest_start Índice do elemento mais antigo no buffer circular do canal.
 * @param[in] window_start Índice relativo inicial da janela.
 * @param[in] window_len Quantidade de amostras reais na janela.
 * @param[out] dest Buffer de destino do gráfico.
 * @param[in] point_count Quantidade de pontos do gráfico.
 * @param[out] out_min_mv Menor valor encontrado na janela.
 * @param[out] out_max_mv Maior valor encontrado na janela.
 */
static void adc_scope_render_window_locked(size_t channel_index,
                                           bool circular_source,
                                           size_t oldest_start,
                                           size_t window_start,
                                           size_t window_len,
                                           int32_t *dest,
                                           size_t point_count,
                                           int32_t *out_min_mv,
                                           int32_t *out_max_mv)
{
    int32_t min_mv = INT32_MAX;
    int32_t max_mv = INT32_MIN;

    if (dest == NULL) {
        if (out_min_mv != NULL) {
            *out_min_mv = 0;
        }
        if (out_max_mv != NULL) {
            *out_max_mv = 0;
        }
        return;
    }

    for (size_t i = 0; i < point_count; i++) {
        dest[i] = INT32_MAX;
    }

    if (window_len == 0U) {
        if (out_min_mv != NULL) {
            *out_min_mv = 0;
        }
        if (out_max_mv != NULL) {
            *out_max_mv = 0;
        }
        return;
    }

    if (window_len == 1U) {
        const int32_t mv = adc_scope_get_source_sample_locked(channel_index, circular_source, oldest_start, window_start);
        for (size_t i = 0; i < point_count; i++) {
            dest[i] = mv;
        }
        if (out_min_mv != NULL) {
            *out_min_mv = mv;
        }
        if (out_max_mv != NULL) {
            *out_max_mv = mv;
        }
        return;
    }

    for (size_t i = 0; i < point_count; i++) {
        const uint64_t scaled_pos = ((uint64_t)i * (uint64_t)(window_len - 1U) * 1024ULL) / (uint64_t)(point_count - 1U);
        const size_t src_index = (size_t)(scaled_pos / 1024ULL);
        const uint32_t frac = (uint32_t)(scaled_pos % 1024ULL);
        const int32_t sample_a =
            adc_scope_get_source_sample_locked(channel_index, circular_source, oldest_start, window_start + src_index);
        int32_t mv = sample_a;

        if ((src_index + 1U) < window_len && frac > 0U) {
            const int32_t sample_b =
                adc_scope_get_source_sample_locked(channel_index, circular_source, oldest_start, window_start + src_index + 1U);
            const int64_t interp =
                ((int64_t)sample_a * (int64_t)(1024U - frac)) +
                ((int64_t)sample_b * (int64_t)frac);
            mv = (int32_t)(interp / 1024LL);
        }

        dest[i] = mv;
        if (mv < min_mv) {
            min_mv = mv;
        }
        if (mv > max_mv) {
            max_mv = mv;
        }
    }

    if (out_min_mv != NULL) {
        *out_min_mv = (min_mv == INT32_MAX) ? 0 : min_mv;
    }
    if (out_max_mv != NULL) {
        *out_max_mv = (max_mv == INT32_MIN) ? 0 : max_mv;
    }
}

/**
 * @brief Calcula frequência e duty cycle usando a janela real de um canal.
 *
 * @param[in] channel_index Índice lógico do canal.
 * @param[in] circular_source Indica se a origem é o buffer circular.
 * @param[in] oldest_start Índice do elemento mais antigo no buffer circular do canal.
 * @param[in] window_start Índice relativo inicial da janela.
 * @param[in] window_len Quantidade de amostras reais na janela.
 * @param[in] threshold_mv Nível de trigger usado como referência.
 * @param[out] out_freq_tenths_hz Frequência estimada em décimos de hertz.
 * @param[out] out_duty_tenths_percent Duty estimado em décimos de porcentagem.
 * @param[out] out_valid Indica se as medições puderam ser calculadas.
 */
static void adc_scope_measure_window_locked(size_t channel_index,
                                            bool circular_source,
                                            size_t oldest_start,
                                            size_t window_start,
                                            size_t window_len,
                                            int32_t threshold_mv,
                                            uint32_t *out_freq_tenths_hz,
                                            uint32_t *out_duty_tenths_percent,
                                            bool *out_valid)
{
    size_t crossing_index[24] = {0};
    bool crossing_rising[24] = {0};
    size_t crossing_count = 0U;
    size_t first_edge = 0U;
    size_t last_edge = 0U;
    size_t matching_edges = 0U;
    bool matching_polarity = false;

    if (out_freq_tenths_hz != NULL) {
        *out_freq_tenths_hz = 0U;
    }
    if (out_duty_tenths_percent != NULL) {
        *out_duty_tenths_percent = 0U;
    }
    if (out_valid != NULL) {
        *out_valid = false;
    }

    if (window_len < 3U || s_scope.config.sample_freq_hz == 0U) {
        return;
    }

    for (size_t i = 1U; i < window_len && crossing_count < (sizeof(crossing_index) / sizeof(crossing_index[0])); i++) {
        const int32_t prev = adc_scope_get_source_sample_locked(channel_index, circular_source, oldest_start, window_start + i - 1U);
        const int32_t curr = adc_scope_get_source_sample_locked(channel_index, circular_source, oldest_start, window_start + i);
        const bool rising = (prev < threshold_mv) && (curr >= threshold_mv);
        const bool falling = (prev > threshold_mv) && (curr <= threshold_mv);

        if (!rising && !falling) {
            continue;
        }

        crossing_index[crossing_count] = i;
        crossing_rising[crossing_count] = rising;
        crossing_count++;
    }

    for (size_t i = 0; i < crossing_count; i++) {
        if (matching_edges == 0U) {
            matching_polarity = crossing_rising[i];
            first_edge = crossing_index[i];
            last_edge = crossing_index[i];
            matching_edges = 1U;
            continue;
        }

        if (crossing_rising[i] == matching_polarity) {
            last_edge = crossing_index[i];
            matching_edges++;
        }
    }

    if (matching_edges < 2U || last_edge <= first_edge) {
        return;
    }

    if (out_freq_tenths_hz != NULL) {
        const uint64_t numerator = (uint64_t)s_scope.config.sample_freq_hz * 10ULL;
        const uint64_t avg_period_samples =
            ((uint64_t)(last_edge - first_edge) + ((uint64_t)(matching_edges - 1U) / 2ULL)) /
            (uint64_t)(matching_edges - 1U);

        if (avg_period_samples == 0U) {
            return;
        }

        *out_freq_tenths_hz = (uint32_t)((numerator + (avg_period_samples / 2ULL)) / avg_period_samples);
    }

    if (out_duty_tenths_percent != NULL) {
        uint32_t high_samples = 0U;

        for (size_t i = first_edge; i < last_edge; i++) {
            const int32_t mv = adc_scope_get_source_sample_locked(channel_index, circular_source, oldest_start, window_start + i);
            if (mv >= threshold_mv) {
                high_samples++;
            }
        }

        *out_duty_tenths_percent = (uint32_t)(((uint64_t)high_samples * 1000ULL) / (uint64_t)(last_edge - first_edge));
    }

    if (out_valid != NULL) {
        *out_valid = true;
    }
}

/**
 * @brief Armazena uma amostra já convertida no modo circular.
 *
 * @param[in] channel_index Índice lógico do canal.
 * @param[in] mv Amostra convertida em milivolts.
 */
static void adc_scope_store_circular_sample_locked(size_t channel_index, int32_t mv)
{
    s_scope.circular_mv_buffer[channel_index][s_scope.circular_head[channel_index]] = mv;
    s_scope.circular_head[channel_index] = (s_scope.circular_head[channel_index] + 1U) % s_scope.config.circular_buffer_capacity;

    if (s_scope.circular_count[channel_index] < s_scope.config.circular_buffer_capacity) {
        s_scope.circular_count[channel_index]++;
    }
}

/**
 * @brief Armazena uma amostra no modo bloco, respeitando a quantidade solicitada.
 *
 * @param[in] channel_index Índice lógico do canal.
 * @param[in] mv Amostra convertida em milivolts.
 */
static void adc_scope_store_block_sample_locked(size_t channel_index, int32_t mv)
{
    bool all_done = true;

    if (s_scope.block_target == 0U || s_scope.block_capture_ready) {
        return;
    }

    if (s_scope.block_count[channel_index] < s_scope.block_target) {
        s_scope.block_mv_buffer[channel_index][s_scope.block_count[channel_index]++] = mv;
    }

    for (size_t i = 0; i < s_scope.config.channel_count; i++) {
        if (s_scope.block_count[i] < s_scope.block_target) {
            all_done = false;
            break;
        }
    }

    s_scope.block_capture_ready = all_done;
}

/**
 * @brief Converte uma amostra bruta do ADC para milivolts.
 *
 * @param[in] channel_index Índice lógico do canal.
 * @param[in] raw Valor bruto retornado pelo ADC.
 *
 * @return Amostra convertida em milivolts.
 */
static int32_t adc_scope_convert_raw_to_mv(size_t channel_index, uint32_t raw)
{
    int voltage_mv = (int)raw;

    if (s_scope.calibrated[channel_index]) {
        if (adc_cali_raw_to_voltage(s_scope.cali_handle[channel_index], (int)raw, &voltage_mv) != ESP_OK) {
            voltage_mv = (int)raw;
        }
    }

    return (int32_t)voltage_mv;
}

/**
 * @brief Processa um lote de frames lidos do ADC.
 *
 * @param[in] data Dados crus recebidos via `adc_continuous_read`.
 * @param[in] size Quantidade de bytes válidos em `data`.
 */
static void adc_scope_process_samples(const uint8_t *data, uint32_t size)
{
    if (xSemaphoreTake(s_scope.mutex, portMAX_DELAY) != pdTRUE) {
        return;
    }

    for (uint32_t offset = 0; offset + SOC_ADC_DIGI_RESULT_BYTES <= size; offset += SOC_ADC_DIGI_RESULT_BYTES) {
        const adc_digi_output_data_t *sample = (const adc_digi_output_data_t *)&data[offset];
        const adc_unit_t unit = sample->type2.unit ? ADC_UNIT_2 : ADC_UNIT_1;
        const adc_channel_t channel = (adc_channel_t)sample->type2.channel;
        const uint32_t raw = sample->type2.data;
        const int index = adc_scope_find_channel_index(unit, channel);
        int32_t mv = 0;

        if (index < 0) {
            continue;
        }

        mv = adc_scope_convert_raw_to_mv((size_t)index, raw);
        s_scope.latest_raw[index] = raw;
        s_scope.latest_mv[index] = mv;

        if (s_scope.mode == ADC_SCOPE_MODE_CIRCULAR) {
            adc_scope_store_circular_sample_locked((size_t)index, mv);
        } else {
            adc_scope_store_block_sample_locked((size_t)index, mv);
        }
    }

    xSemaphoreGive(s_scope.mutex);
}

/**
 * @brief Callback disparada quando um frame DMA de conversões fica pronto.
 *
 * @param[in] handle Handle do ADC contínuo.
 * @param[in] edata Metadados do evento de conversão.
 * @param[in] user_data Contexto registrado no driver.
 *
 * @return `true` quando uma task de maior prioridade deve ser acordada.
 */
static bool IRAM_ATTR adc_scope_on_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t task_woken = pdFALSE;
    adc_scope_state_t *state = (adc_scope_state_t *)user_data;
    (void)handle;
    (void)edata;

    xTaskNotifyFromISR(state->task_handle, ADC_SCOPE_NOTIFY_DATA, eSetBits, &task_woken);
    return task_woken == pdTRUE;
}

/**
 * @brief Callback disparada quando o pool interno do driver sofre overflow.
 *
 * @param[in] handle Handle do ADC contínuo.
 * @param[in] edata Metadados do evento.
 * @param[in] user_data Contexto registrado no driver.
 *
 * @return `false`, pois não há troca imediata de contexto.
 */
static bool IRAM_ATTR adc_scope_on_pool_ovf_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    adc_scope_state_t *state = (adc_scope_state_t *)user_data;
    (void)handle;
    (void)edata;

    state->overflow_seen = true;
    return false;
}

/**
 * @brief Task responsável por consumir os frames produzidos pelo ADC contínuo.
 *
 * @param[in] arg Contexto não utilizado.
 */
static void adc_scope_task(void *arg)
{
    (void)arg;

    while (true) {
        uint32_t notify_value = 0;
        uint32_t bytes_read = 0;
        esp_err_t ret = ESP_ERR_TIMEOUT;

        xTaskNotifyWait(0U, UINT32_MAX, &notify_value, pdMS_TO_TICKS(ADC_SCOPE_READ_TIMEOUT_MS));

        if (s_scope.control_stop_pending && s_scope.started) {
            s_scope.control_result = adc_continuous_stop(s_scope.adc_handle);
            if (s_scope.control_result == ESP_OK) {
                s_scope.started = false;
            }
            s_scope.control_stop_pending = false;
            xSemaphoreGive(s_scope.control_done);
        }

        if (s_scope.control_start_pending && !s_scope.started) {
            s_scope.control_result = adc_continuous_start(s_scope.adc_handle);
            if (s_scope.control_result == ESP_OK) {
                s_scope.started = true;
            }
            s_scope.control_start_pending = false;
            xSemaphoreGive(s_scope.control_done);
        }

        if (!s_scope.started) {
            continue;
        }

        do {
            ret = adc_continuous_read(s_scope.adc_handle,
                                      s_scope.read_buffer,
                                      s_scope.config.conv_frame_size,
                                      &bytes_read,
                                      0);
            if (ret == ESP_OK && bytes_read > 0U) {
                adc_scope_process_samples(s_scope.read_buffer, bytes_read);
            }
        } while (ret == ESP_OK && bytes_read > 0U);
    }
}

/**
 * @brief Retorna uma configuração padrão adequada para inicializar o módulo.
 *
 * @return Estrutura preenchida com valores iniciais seguros para o ESP32-S3.
 */
adc_scope_config_t adc_scope_get_default_config(void)
{
    const adc_scope_config_t config = {
        .unit = ADC_UNIT_1,
        .channel_count = 2U,
        .channels = {ADC_CHANNEL_9, ADC_CHANNEL_0},
        .attenuations = {ADC_ATTEN_DB_12, ADC_ATTEN_DB_12},
        .bitwidth = ADC_BITWIDTH_12,
        .sample_freq_hz = 20000U,
        .max_store_buf_size = 2048U,
        .conv_frame_size = 256U,
        .circular_buffer_capacity = 20000U,
        .chart_point_count = 240U,
        .default_block_sample_count = 240U,
        .task_priority = 5U,
        .task_stack_size = 4096U,
    };

    return config;
}

/**
 * @brief Inicializa o driver contínuo do ADC, a calibração e a task de aquisição.
 *
 * @param[in] config Configuração desejada para o módulo.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
esp_err_t adc_scope_init(const adc_scope_config_t *config)
{
    adc_continuous_handle_cfg_t handle_config = {0};
    adc_continuous_config_t adc_config = {0};
    adc_digi_pattern_config_t patterns[ADC_SCOPE_MAX_CHANNELS] = {0};
    adc_continuous_evt_cbs_t callbacks = {0};
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "config nulo");
    ESP_RETURN_ON_FALSE(config->channel_count > 0U && config->channel_count <= ADC_SCOPE_MAX_CHANNELS, ESP_ERR_INVALID_ARG, TAG, "channel_count invalido");
    ESP_RETURN_ON_FALSE(config->circular_buffer_capacity >= config->chart_point_count, ESP_ERR_INVALID_ARG, TAG, "historico circular invalido");
    ESP_RETURN_ON_FALSE(config->chart_point_count > 0U, ESP_ERR_INVALID_ARG, TAG, "chart_point_count invalido");
    ESP_RETURN_ON_FALSE(config->default_block_sample_count > 0U, ESP_ERR_INVALID_ARG, TAG, "default_block_sample_count invalido");
    ESP_RETURN_ON_FALSE(config->default_block_sample_count <= config->chart_point_count, ESP_ERR_INVALID_ARG, TAG, "bloco excede grafico");
    ESP_RETURN_ON_FALSE(config->conv_frame_size >= SOC_ADC_DIGI_RESULT_BYTES, ESP_ERR_INVALID_ARG, TAG, "frame DMA invalido");
    ESP_RETURN_ON_FALSE((config->conv_frame_size % SOC_ADC_DIGI_RESULT_BYTES) == 0U, ESP_ERR_INVALID_ARG, TAG, "frame DMA desalinhado");
    ESP_RETURN_ON_FALSE(config->unit == ADC_UNIT_1, ESP_ERR_INVALID_ARG, TAG, "somente ADC1 e suportado neste projeto");
    ESP_RETURN_ON_FALSE(!s_scope.initialized, ESP_ERR_INVALID_STATE, TAG, "modulo ja inicializado");

    memset(&s_scope, 0, sizeof(s_scope));
    for (size_t i = 0; i < ADC_SCOPE_MAX_CHANNELS; i++) {
        s_scope.gpio_num[i] = -1;
    }

    s_scope.config = *config;
    s_scope.mode = ADC_SCOPE_MODE_CIRCULAR;
    s_scope.block_target = config->default_block_sample_count;

    s_scope.mutex = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_scope.mutex != NULL, ESP_ERR_NO_MEM, TAG, "falha ao criar mutex");
    s_scope.control_done = xSemaphoreCreateBinary();
    ESP_GOTO_ON_FALSE(s_scope.control_done != NULL, ESP_ERR_NO_MEM, err, TAG, "falha ao criar semaforo de controle");

    s_scope.read_buffer = heap_caps_malloc(config->conv_frame_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    ESP_GOTO_ON_FALSE(s_scope.read_buffer != NULL, ESP_ERR_NO_MEM, err, TAG, "falha ao alocar read_buffer");

    for (size_t i = 0; i < config->channel_count; i++) {
        s_scope.circular_mv_buffer[i] = calloc(config->circular_buffer_capacity, sizeof(int32_t));
        s_scope.block_mv_buffer[i] = calloc(config->chart_point_count, sizeof(int32_t));
        ESP_GOTO_ON_FALSE(s_scope.circular_mv_buffer[i] != NULL, ESP_ERR_NO_MEM, err, TAG, "falha ao alocar buffer circular");
        ESP_GOTO_ON_FALSE(s_scope.block_mv_buffer[i] != NULL, ESP_ERR_NO_MEM, err, TAG, "falha ao alocar buffer bloco");
    }

    handle_config.max_store_buf_size = config->max_store_buf_size;
    handle_config.conv_frame_size = config->conv_frame_size;
    handle_config.flags.flush_pool = 1U;
    ESP_GOTO_ON_ERROR(adc_continuous_new_handle(&handle_config, &s_scope.adc_handle), err, TAG, "falha ao criar handle ADC");

    for (size_t i = 0; i < config->channel_count; i++) {
        patterns[i].atten = (uint8_t)config->attenuations[i];
        patterns[i].channel = (uint8_t)config->channels[i];
        patterns[i].unit = (uint8_t)config->unit;
        patterns[i].bit_width = (uint8_t)config->bitwidth;
    }

    adc_config.pattern_num = config->channel_count;
    adc_config.adc_pattern = patterns;
    adc_config.sample_freq_hz = config->sample_freq_hz * (uint32_t)config->channel_count;
    adc_config.conv_mode = ADC_CONV_SINGLE_UNIT_1;
    adc_config.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
    ESP_GOTO_ON_ERROR(adc_continuous_config(s_scope.adc_handle, &adc_config), err, TAG, "falha ao configurar ADC continuo");

    callbacks.on_conv_done = adc_scope_on_conv_done_cb;
    callbacks.on_pool_ovf = adc_scope_on_pool_ovf_cb;
    ESP_GOTO_ON_ERROR(adc_continuous_register_event_callbacks(s_scope.adc_handle, &callbacks, &s_scope), err, TAG, "falha ao registrar callbacks");

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    for (size_t i = 0; i < config->channel_count; i++) {
        const adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = config->unit,
            .chan = config->channels[i],
            .atten = config->attenuations[i],
            .bitwidth = config->bitwidth,
        };

        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &s_scope.cali_handle[i]);
        if (ret == ESP_OK) {
            s_scope.calibrated[i] = true;
        } else {
            ESP_LOGW(TAG, "Calibracao indisponivel no canal %u (%s). Seguiremos com valor cru.", (unsigned)i, esp_err_to_name(ret));
            ret = ESP_OK;
        }
    }
#endif

    for (size_t i = 0; i < config->channel_count; i++) {
        ESP_GOTO_ON_ERROR(adc_continuous_channel_to_io(config->unit, config->channels[i], &s_scope.gpio_num[i]), err, TAG, "falha ao resolver GPIO ADC");
    }

    BaseType_t task_ok = xTaskCreate(adc_scope_task,
                                     "adc_scope",
                                     config->task_stack_size,
                                     NULL,
                                     (UBaseType_t)config->task_priority,
                                     &s_scope.task_handle);
    ESP_GOTO_ON_FALSE(task_ok == pdPASS, ESP_ERR_NO_MEM, err, TAG, "falha ao criar task ADC");

    s_scope.initialized = true;
    ESP_LOGI(TAG,
             "ADC pronto: CH1 GPIO %d, CH2 GPIO %d, freq=%" PRIu32 " Hz/canal, pontos=%u",
             s_scope.gpio_num[0],
             (config->channel_count > 1U) ? s_scope.gpio_num[1] : -1,
             config->sample_freq_hz,
             (unsigned)config->chart_point_count);
    return ESP_OK;

err:
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    for (size_t i = 0; i < ADC_SCOPE_MAX_CHANNELS; i++) {
        if (s_scope.cali_handle[i] != NULL) {
            adc_cali_delete_scheme_curve_fitting(s_scope.cali_handle[i]);
            s_scope.cali_handle[i] = NULL;
        }
    }
#endif
    if (s_scope.adc_handle != NULL) {
        adc_continuous_deinit(s_scope.adc_handle);
        s_scope.adc_handle = NULL;
    }
    for (size_t i = 0; i < ADC_SCOPE_MAX_CHANNELS; i++) {
        if (s_scope.block_mv_buffer[i] != NULL) {
            free(s_scope.block_mv_buffer[i]);
            s_scope.block_mv_buffer[i] = NULL;
        }
        if (s_scope.circular_mv_buffer[i] != NULL) {
            free(s_scope.circular_mv_buffer[i]);
            s_scope.circular_mv_buffer[i] = NULL;
        }
    }
    if (s_scope.read_buffer != NULL) {
        heap_caps_free(s_scope.read_buffer);
        s_scope.read_buffer = NULL;
    }
    if (s_scope.mutex != NULL) {
        vSemaphoreDelete(s_scope.mutex);
        s_scope.mutex = NULL;
    }
    if (s_scope.control_done != NULL) {
        vSemaphoreDelete(s_scope.control_done);
        s_scope.control_done = NULL;
    }
    return ret;
}

/**
 * @brief Inicia a aquisição contínua de amostras pelo ADC com DMA.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
esp_err_t adc_scope_start(void)
{
    ESP_RETURN_ON_FALSE(s_scope.initialized, ESP_ERR_INVALID_STATE, TAG, "modulo nao inicializado");
    ESP_RETURN_ON_FALSE(!s_scope.started, ESP_ERR_INVALID_STATE, TAG, "ADC ja iniciado");

    s_scope.control_result = ESP_ERR_INVALID_STATE;
    s_scope.control_start_pending = true;
    xTaskNotify(s_scope.task_handle, ADC_SCOPE_NOTIFY_CONTROL, eSetBits);
    if (xSemaphoreTake(s_scope.control_done, pdMS_TO_TICKS(250)) != pdTRUE) {
        s_scope.control_start_pending = false;
        return ESP_ERR_TIMEOUT;
    }

    ESP_RETURN_ON_ERROR(s_scope.control_result, TAG, "falha ao iniciar ADC continuo");
    return ESP_OK;
}

/**
 * @brief Interrompe a aquisição contínua de amostras pelo ADC com DMA.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
esp_err_t adc_scope_stop(void)
{
    ESP_RETURN_ON_FALSE(s_scope.initialized, ESP_ERR_INVALID_STATE, TAG, "modulo nao inicializado");
    ESP_RETURN_ON_FALSE(s_scope.started, ESP_ERR_INVALID_STATE, TAG, "ADC nao iniciado");

    s_scope.control_result = ESP_ERR_INVALID_STATE;
    s_scope.control_stop_pending = true;
    xTaskNotify(s_scope.task_handle, ADC_SCOPE_NOTIFY_CONTROL, eSetBits);
    if (xSemaphoreTake(s_scope.control_done, pdMS_TO_TICKS(250)) != pdTRUE) {
        s_scope.control_stop_pending = false;
        return ESP_ERR_TIMEOUT;
    }

    ESP_RETURN_ON_ERROR(s_scope.control_result, TAG, "falha ao interromper ADC continuo");
    return ESP_OK;
}

/**
 * @brief Altera o modo de aquisição entre circular e bloco.
 *
 * @param[in] mode Novo modo desejado.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
esp_err_t adc_scope_set_mode(adc_scope_mode_t mode)
{
    ESP_RETURN_ON_FALSE(s_scope.initialized, ESP_ERR_INVALID_STATE, TAG, "modulo nao inicializado");

    if (xSemaphoreTake(s_scope.mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    s_scope.mode = mode;
    if (mode == ADC_SCOPE_MODE_CIRCULAR) {
        s_scope.block_capture_ready = false;
    } else {
        for (size_t i = 0; i < s_scope.config.channel_count; i++) {
            s_scope.block_count[i] = 0U;
        }
        s_scope.block_target = s_scope.config.default_block_sample_count;
        s_scope.block_capture_ready = false;
    }

    xSemaphoreGive(s_scope.mutex);
    return ESP_OK;
}

/**
 * @brief Prepara uma nova captura em bloco com quantidade configurável de amostras.
 *
 * @param[in] sample_count Quantidade desejada de amostras. Se zero, usa o valor padrão.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
esp_err_t adc_scope_capture_block(size_t sample_count)
{
    ESP_RETURN_ON_FALSE(s_scope.initialized, ESP_ERR_INVALID_STATE, TAG, "modulo nao inicializado");

    if (sample_count == 0U) {
        sample_count = s_scope.config.default_block_sample_count;
    }

    ESP_RETURN_ON_FALSE(sample_count <= s_scope.config.chart_point_count, ESP_ERR_INVALID_ARG, TAG, "sample_count excede grafico");

    if (xSemaphoreTake(s_scope.mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    s_scope.mode = ADC_SCOPE_MODE_BLOCK;
    for (size_t i = 0; i < s_scope.config.channel_count; i++) {
        s_scope.block_count[i] = 0U;
    }
    s_scope.block_target = sample_count;
    s_scope.block_capture_ready = false;

    xSemaphoreGive(s_scope.mutex);
    return ESP_OK;
}

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
 * @param[in] trigger_level_mv Nível de trigger em milivolts ou `INT32_MIN` para automático.
 * @param[in] trigger_hysteresis_mv Histerese em milivolts ou `0` para automático.
 * @param[out] out_snapshot Estrutura opcional com metadados da janela copiada.
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
                                            int32_t trigger_level_mv,
                                            uint32_t trigger_hysteresis_mv,
                                            adc_scope_snapshot_t *out_snapshot)
{
    adc_scope_snapshot_t snapshot = {0};
    size_t oldest_start[ADC_SCOPE_MAX_CHANNELS] = {0};
    size_t count = 0U;
    size_t window_start = 0U;
    size_t window_len = 0U;
    bool circular_source = false;

    ESP_RETURN_ON_FALSE(s_scope.initialized, ESP_ERR_INVALID_STATE, TAG, "modulo nao inicializado");
    ESP_RETURN_ON_FALSE(dest_per_channel != NULL, ESP_ERR_INVALID_ARG, TAG, "destinos nulos");
    ESP_RETURN_ON_FALSE(point_count == s_scope.config.chart_point_count, ESP_ERR_INVALID_ARG, TAG, "point_count divergente");
    ESP_RETURN_ON_FALSE(requested_samples > 0U, ESP_ERR_INVALID_ARG, TAG, "requested_samples invalido");
    ESP_RETURN_ON_FALSE(trigger_channel_index < s_scope.config.channel_count, ESP_ERR_INVALID_ARG, TAG, "trigger_channel_index invalido");
    ESP_RETURN_ON_FALSE(trigger_point_index < point_count, ESP_ERR_INVALID_ARG, TAG, "trigger_point_index invalido");

    if (xSemaphoreTake(s_scope.mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    snapshot.mode = s_scope.mode;
    snapshot.channel_count = s_scope.config.channel_count;
    snapshot.capacity = point_count;
    snapshot.history_capacity = (s_scope.mode == ADC_SCOPE_MODE_CIRCULAR) ? s_scope.config.circular_buffer_capacity : s_scope.block_target;
    snapshot.capture_ready = s_scope.block_capture_ready;
    snapshot.acquisition_running = s_scope.started;
    snapshot.trigger_found = false;
    snapshot.overflow_seen = s_scope.overflow_seen;
    snapshot.trigger_channel_index = trigger_channel_index;
    for (size_t i = 0; i < s_scope.config.channel_count; i++) {
        snapshot.latest_raw[i] = s_scope.latest_raw[i];
        snapshot.latest_mv[i] = s_scope.latest_mv[i];
        snapshot.calibrated[i] = s_scope.calibrated[i];
        snapshot.min_mv[i] = 0;
        snapshot.max_mv[i] = 0;
        snapshot.frequency_tenths_hz[i] = 0U;
        snapshot.duty_tenths_percent[i] = 0U;
        snapshot.measurements_valid[i] = false;
    }

    circular_source = (s_scope.mode == ADC_SCOPE_MODE_CIRCULAR);
    count = adc_scope_get_shared_count_locked(circular_source);
    snapshot.history_count = count;

    for (size_t i = 0; i < s_scope.config.channel_count; i++) {
        if (circular_source) {
            oldest_start[i] = (s_scope.circular_head[i] + s_scope.config.circular_buffer_capacity - s_scope.circular_count[i]) % s_scope.config.circular_buffer_capacity;
        }
    }

    if (history_offset_samples > count) {
        history_offset_samples = count;
    }

    if (count > history_offset_samples) {
        count -= history_offset_samples;
    } else {
        count = 0U;
    }

    window_len = (requested_samples < count) ? requested_samples : count;
    window_start = (count > window_len) ? (count - window_len) : 0U;

    if (trigger_mode != ADC_SCOPE_TRIGGER_FREE && requested_samples > 1U && count >= requested_samples) {
        const size_t pretrigger_samples = ((requested_samples - 1U) * trigger_point_index) / (point_count - 1U);
        const size_t posttrigger_samples = requested_samples - pretrigger_samples - 1U;
        int32_t source_min = INT32_MAX;
        int32_t source_max = INT32_MIN;
        int32_t threshold = 0;
        int32_t low_threshold = 0;
        int32_t high_threshold = 0;
        int32_t hysteresis_mv = 0;

        for (size_t i = 0; i < count; i++) {
            const int32_t mv = adc_scope_get_source_sample_locked(trigger_channel_index, circular_source, oldest_start[trigger_channel_index], i);
            if (mv < source_min) {
                source_min = mv;
            }
            if (mv > source_max) {
                source_max = mv;
            }
        }

        threshold = (trigger_level_mv == INT32_MIN) ? (source_min + ((source_max - source_min) / 2)) : trigger_level_mv;
        hysteresis_mv = (int32_t)trigger_hysteresis_mv;
        if (hysteresis_mv <= 0) {
            hysteresis_mv = (source_max - source_min) / 20;
            if (hysteresis_mv < 8) {
                hysteresis_mv = 8;
            }
        }

        low_threshold = threshold - (hysteresis_mv / 2);
        high_threshold = threshold + (hysteresis_mv / 2);
        snapshot.trigger_level_mv = threshold;
        snapshot.trigger_hysteresis_mv = (uint32_t)hysteresis_mv;

        if (count > posttrigger_samples + 1U) {
            for (size_t rel = count - posttrigger_samples - 1U; rel > pretrigger_samples; rel--) {
                const int32_t prev = adc_scope_get_source_sample_locked(trigger_channel_index, circular_source, oldest_start[trigger_channel_index], rel - 1U);
                const int32_t curr = adc_scope_get_source_sample_locked(trigger_channel_index, circular_source, oldest_start[trigger_channel_index], rel);
                const bool rising = (prev < low_threshold) && (curr >= high_threshold);
                const bool falling = (prev > high_threshold) && (curr <= low_threshold);

                if ((trigger_mode == ADC_SCOPE_TRIGGER_RISE && rising) ||
                    (trigger_mode == ADC_SCOPE_TRIGGER_FALL && falling)) {
                    window_start = rel - pretrigger_samples;
                    window_len = requested_samples;
                    snapshot.trigger_sample_index = rel - window_start;
                    snapshot.trigger_found = true;
                    break;
                }
            }
        }

        if (!snapshot.trigger_found && trigger_run_mode != ADC_SCOPE_TRIGGER_RUN_AUTO) {
            window_len = 0U;
        }
    } else {
        snapshot.trigger_level_mv = (trigger_level_mv == INT32_MIN) ? 0 : trigger_level_mv;
        snapshot.trigger_hysteresis_mv = trigger_hysteresis_mv;
    }

    snapshot.sample_count = window_len;
    for (size_t i = 0; i < s_scope.config.channel_count; i++) {
        adc_scope_render_window_locked(i,
                                       circular_source,
                                       oldest_start[i],
                                       window_start,
                                       window_len,
                                       dest_per_channel[i],
                                       point_count,
                                       &snapshot.min_mv[i],
                                       &snapshot.max_mv[i]);

        adc_scope_measure_window_locked(i,
                                        circular_source,
                                        oldest_start[i],
                                        window_start,
                                        window_len,
                                        snapshot.trigger_level_mv,
                                        &snapshot.frequency_tenths_hz[i],
                                        &snapshot.duty_tenths_percent[i],
                                        &snapshot.measurements_valid[i]);
    }

    xSemaphoreGive(s_scope.mutex);

    if (out_snapshot != NULL) {
        *out_snapshot = snapshot;
    }

    return ESP_OK;
}

/**
 * @brief Retorna a frequência de amostragem configurada para o ADC.
 *
 * @param[out] out_sample_freq_hz Frequência configurada em hertz.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
esp_err_t adc_scope_get_sample_freq_hz(uint32_t *out_sample_freq_hz)
{
    ESP_RETURN_ON_FALSE(s_scope.initialized, ESP_ERR_INVALID_STATE, TAG, "modulo nao inicializado");
    ESP_RETURN_ON_FALSE(out_sample_freq_hz != NULL, ESP_ERR_INVALID_ARG, TAG, "saida nula");

    *out_sample_freq_hz = s_scope.config.sample_freq_hz;
    return ESP_OK;
}

/**
 * @brief Retorna a quantidade de canais configurados.
 *
 * @param[out] out_channel_count Quantidade de canais habilitados.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
esp_err_t adc_scope_get_channel_count(size_t *out_channel_count)
{
    ESP_RETURN_ON_FALSE(s_scope.initialized, ESP_ERR_INVALID_STATE, TAG, "modulo nao inicializado");
    ESP_RETURN_ON_FALSE(out_channel_count != NULL, ESP_ERR_INVALID_ARG, TAG, "saida nula");

    *out_channel_count = s_scope.config.channel_count;
    return ESP_OK;
}

/**
 * @brief Retorna o GPIO físico associado ao canal ADC configurado.
 *
 * @param[in] channel_index Índice lógico do canal configurado.
 * @param[out] out_gpio GPIO correspondente ao canal configurado.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
esp_err_t adc_scope_get_gpio_num(size_t channel_index, int *out_gpio)
{
    ESP_RETURN_ON_FALSE(s_scope.initialized, ESP_ERR_INVALID_STATE, TAG, "modulo nao inicializado");
    ESP_RETURN_ON_FALSE(out_gpio != NULL, ESP_ERR_INVALID_ARG, TAG, "saida nula");
    ESP_RETURN_ON_FALSE(channel_index < s_scope.config.channel_count, ESP_ERR_INVALID_ARG, TAG, "channel_index invalido");

    *out_gpio = s_scope.gpio_num[channel_index];
    return ESP_OK;
}

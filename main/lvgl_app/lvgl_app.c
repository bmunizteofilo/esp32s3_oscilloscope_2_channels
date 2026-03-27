/**
 * @file lvgl_app.c
 * @brief Implementação da interface LVGL com visual de osciloscópio.
 */

#include "lvgl_app.h"
#include <math.h>
#include <inttypes.h>
#include "adc_scope.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "scope_web.h"
#include "st7796_port.h"

/** @brief Tag de log do módulo LVGL. */
static const char *TAG = "lvgl_app";

/** @brief Período do tick do LVGL em milissegundos. */
#define LVGL_TICK_PERIOD_MS     (2U)

/** @brief Período máximo do loop do handler do LVGL em milissegundos. */
#define LVGL_TASK_PERIOD_MS     (5U)

/** @brief Período de atualização do gráfico em milissegundos. */
#define LVGL_SCOPE_REFRESH_MS   (40U)

/** @brief Tempo de exibição da splash screen em milissegundos. */
#define LVGL_SPLASH_DURATION_MS (5000U)

/** @brief Quantidade de divisões horizontais usada para a base de tempo. */
#define LVGL_SCOPE_TIME_DIVS    (8U)

/** @brief Quantidade de linhas verticais internas desenhadas na grade do chart. */
#define LVGL_SCOPE_TIME_GRID_LINES (9U)

/** @brief Quantidade de linhas horizontais internas desenhadas na grade do chart. */
#define LVGL_SCOPE_VOLT_GRID_LINES (5U)

/** @brief Quantidade de divisões horizontais visíveis usada para a leitura de tensão. */
#define LVGL_SCOPE_VOLT_DIVS    (4U)

/** @brief Posição horizontal alvo do trigger no gráfico. */
#define LVGL_SCOPE_TRIGGER_POS  ((APP_ADC_CHART_POINTS - 1U) / 2U)

/** @brief Cor do grupo de base de tempo. */
#define LVGL_COLOR_TIMEBASE     0xFCA5A5

/** @brief Cor do grupo de base de tensão. */
#define LVGL_COLOR_VOLTS        0xD8B4FE

/** @brief Cor do grupo de seleção de canal amostrado. */
#define LVGL_COLOR_CHANNEL      0xFB7185

/** @brief Cor do grupo de seleção de canal de trigger. */
#define LVGL_COLOR_TRIGCH       0x60A5FA

/** @brief Cor do grupo de trigger. */
#define LVGL_COLOR_TRIGGER      0xFACC15

/** @brief Cor do grupo de modo de trigger. */
#define LVGL_COLOR_TRIGMODE     0xC084FC

/** @brief Cor do grupo de status. */
#define LVGL_COLOR_STATUS       0x4ADE80

/** @brief Cor do grupo de cursores. */
#define LVGL_COLOR_CURSOR       0xF472B6

/** @brief Cor do grupo de seleção de linha do cursor. */
#define LVGL_COLOR_CURSORLINE   0xA78BFA

/** @brief Cor do grupo de autoajuste. */
#define LVGL_COLOR_AUTOSET      0x22C55E

/** @brief Cor do traço do canal 1 no chart. */
#define LVGL_COLOR_TRACE_CH1    0x39FF14

/** @brief Cor do traço do canal 2 no chart. */
#define LVGL_COLOR_TRACE_CH2    0x38BDF8

/** @brief Distância máxima, em pixels, para capturar uma linha de cursor com o dedo. */
#define LVGL_CURSOR_HIT_SLOP_PX (14)

/** @brief Tempo de permanência visual do trigger após interação do usuário. */
#define LVGL_TRIGGER_VISUAL_HOLD_MS (3000U)

/** @brief Janela máxima entre toques para reconhecer o atalho de triple-tap. */
#define LVGL_FULLSCREEN_TAP_WINDOW_MS (700U)

/** @brief Distância máxima para considerar um toque como tap, e não arraste. */
#define LVGL_FULLSCREEN_TAP_SLOP_PX (18)

/** @brief Formato de cor do LVGL para o framebuffer RGB565 do display. */
#define LVGL_DISPLAY_COLOR_FORMAT LV_COLOR_FORMAT_RGB565

/**
 * @brief Modos de cursor disponíveis na UI.
 */
typedef enum {
    LVGL_CURSOR_MODE_OFF = 0,      /**< Cursores desligados. */
    LVGL_CURSOR_MODE_TIME,         /**< Cursores verticais para medir delta de tempo. */
    LVGL_CURSOR_MODE_VOLTAGE,      /**< Cursores horizontais para medir delta de tensão. */
} lvgl_cursor_mode_t;

/**
 * @brief Base de tempo disponível no dropdown.
 */
typedef struct {
    const char *label;      /**< Texto visível no dropdown. */
    uint32_t total_window_us; /**< Janela total exibida no gráfico em microssegundos. */
} lvgl_scope_timebase_t;

/**
 * @brief Escalas de tensão disponíveis no dropdown.
 */
typedef struct {
    const char *label;      /**< Texto visível no dropdown. */
    int32_t max_mv;         /**< Tensão máxima mostrada no topo da tela. */
} lvgl_scope_voltscale_t;

/** @brief Opções de base de tempo disponíveis no seletor. */
static const lvgl_scope_timebase_t s_timebase_options[] = {
    {.label = "5 ms",   .total_window_us = 5000U},
    {.label = "10 ms",  .total_window_us = 10000U},
    {.label = "15 ms",  .total_window_us = 15000U},
    {.label = "20 ms",  .total_window_us = 20000U},
    {.label = "25 ms",  .total_window_us = 25000U},
    {.label = "50 ms",  .total_window_us = 50000U},
    {.label = "100 ms", .total_window_us = 100000U},
    {.label = "250 ms", .total_window_us = 250000U},
    {.label = "500 ms", .total_window_us = 500000U},
    {.label = "1 s",    .total_window_us = 1000000U},
};

/** @brief Opções de escala vertical disponíveis no seletor. */
static const lvgl_scope_voltscale_t s_voltscale_options[] = {
    {.label = "100 mV", .max_mv = 100},
    {.label = "200 mV", .max_mv = 200},
    {.label = "500 mV", .max_mv = 500},
    {.label = "1 V",    .max_mv = 1000},
    {.label = "2 V",    .max_mv = 2000},
    {.label = "3.3 V",  .max_mv = 3300},
};

/** @brief Handle global do LCD usado pelos callbacks do LVGL. */
static st7796_handle_t s_lcd = NULL;

/** @brief Handle global do touch usado pelos callbacks do LVGL. */
static ft6336u_handle_t s_touch = NULL;

/** @brief Timer periódico que alimenta o tick interno do LVGL. */
static esp_timer_handle_t s_lvgl_tick_timer = NULL;

/** @brief Chart principal que exibe a forma de onda. */
static lv_obj_t *s_scope_chart = NULL;

/** @brief Série do chart com os pontos de tensão. */
static lv_chart_series_t *s_scope_series = NULL;

/** @brief Série secundária do chart para o segundo canal. */
static lv_chart_series_t *s_scope_series_ch2 = NULL;

/** @brief Dropdown com o canal mostrado no gráfico. */
static lv_obj_t *s_channel_dropdown = NULL;

/** @brief Dropdown com o canal usado como trigger. */
static lv_obj_t *s_trigger_channel_dropdown = NULL;

/** @brief Dropdown com a escala vertical atual. */
static lv_obj_t *s_volts_dropdown = NULL;

/** @brief Dropdown com a base de tempo atual. */
static lv_obj_t *s_timebase_dropdown = NULL;

/** @brief Dropdown com o modo de trigger atual. */
static lv_obj_t *s_trigger_dropdown = NULL;

/** @brief Dropdown com o estado de captura atual. */
static lv_obj_t *s_status_dropdown = NULL;

/** @brief Barra inferior que exibe as métricas do(s) canal(is). */
static lv_obj_t *s_metrics_strip = NULL;

/** @brief Barra superior com as configurações do osciloscópio. */
static lv_obj_t *s_controls_strip = NULL;

/** @brief Dropdown com o modo de execução do trigger. */
static lv_obj_t *s_trigger_run_dropdown = NULL;

/** @brief Dropdown com o modo de cursor atual. */
static lv_obj_t *s_cursor_dropdown = NULL;

/** @brief Dropdown com a linha de cursor atualmente selecionada. */
static lv_obj_t *s_cursor_line_dropdown = NULL;

/** @brief Dropdown que dispara o autoajuste da visualização. */
static lv_obj_t *s_auto_dropdown = NULL;

/** @brief Label exibido na barra inferior com a ocupação do histórico. */
static lv_obj_t *s_buffer_label = NULL;

/** @brief Label com o delta medido pelos cursores. */
static lv_obj_t *s_cursor_delta_label = NULL;

/** @brief Label com o valor da primeira linha de cursor. */
static lv_obj_t *s_cursor_value_1_label = NULL;

/** @brief Label com o valor da segunda linha de cursor. */
static lv_obj_t *s_cursor_value_2_label = NULL;

/** @brief Label central temporário para avisos operacionais ao usuário. */
static lv_obj_t *s_center_notice_label = NULL;

/** @brief Instante mínimo, em microssegundos, até o qual o aviso central deve permanecer visível. */
static int64_t s_center_notice_hold_until_us = 0;

/** @brief Indica que o aviso central deve ser ocultado assim que o tempo mínimo expirar. */
static bool s_center_notice_pending_hide = false;

/** @brief Indica se o aviso central atual foi colocado pelo modo somente web. */
static bool s_web_only_notice_active = false;

/** @brief Mutex que protege o estado compartilhado entre LVGL e web. */
static SemaphoreHandle_t s_control_state_mutex = NULL;

/** @brief Último estado compartilhado publicado pela UI local. */
static lvgl_app_control_state_t s_shared_control_state = {
    .sample_channel_mode = 0U,
    .timebase_index = 1U,
    .voltscale_index = 5U,
    .trigger_channel_index = 0U,
    .trigger_mode = 0U,
    .trigger_run_mode = 0U,
    .paused = false,
};

/** @brief Estado pendente solicitado por uma interface externa. */
static lvgl_app_control_state_t s_pending_control_state = {0};

/** @brief Indica se existe um estado externo pendente para aplicação na UI local. */
static bool s_has_pending_control_state = false;

/** @brief Indica se o Auto Set foi solicitado por uma interface externa. */
static bool s_pending_auto_set = false;

/** @brief Indica se a alternância do modo foco foi solicitada por uma interface externa. */
static bool s_pending_toggle_focus = false;

/** @brief Evita recursão de callbacks ao atualizar dropdowns programaticamente. */
static bool s_suppress_dropdown_events = false;

/** @brief Indica se a tela principal do osciloscópio já foi criada. */
static bool s_scope_ui_ready = false;

/** @brief Linha horizontal que representa o nível do trigger. */
static lv_obj_t *s_trigger_level_line = NULL;

/** @brief Label com o valor atual do nível de trigger. */
static lv_obj_t *s_trigger_level_label = NULL;

/** @brief Label com o valor de tempo por divisão. */
static lv_obj_t *s_time_div_label = NULL;

/** @brief Label com o valor de tensão por divisão. */
static lv_obj_t *s_volt_div_label = NULL;

/** @brief Primeira linha de cursor desenhada sobre o chart. */
static lv_obj_t *s_cursor_line_1 = NULL;

/** @brief Segunda linha de cursor desenhada sobre o chart. */
static lv_obj_t *s_cursor_line_2 = NULL;

/** @brief Linha vertical central destacada no chart. */
static lv_obj_t *s_center_vertical_line = NULL;

/** @brief Linha horizontal central destacada no chart. */
static lv_obj_t *s_center_horizontal_line = NULL;

/** @brief Label com o valor RMS da janela exibida. */
static lv_obj_t *s_rms_label = NULL;

/** @brief Label com o valor RMS do segundo canal. */
static lv_obj_t *s_rms_label_ch2 = NULL;

/** @brief Label com o pico positivo da janela exibida. */
static lv_obj_t *s_pk_label = NULL;

/** @brief Label com o pico positivo do segundo canal. */
static lv_obj_t *s_pk_label_ch2 = NULL;

/** @brief Label com o pico negativo da janela exibida. */
static lv_obj_t *s_pk_neg_label = NULL;

/** @brief Label com o pico negativo do segundo canal. */
static lv_obj_t *s_pk_neg_label_ch2 = NULL;

/** @brief Label com a frequência estimada da janela exibida. */
static lv_obj_t *s_freq_label = NULL;

/** @brief Label com a frequência estimada do segundo canal. */
static lv_obj_t *s_freq_label_ch2 = NULL;

/** @brief Label com o duty cycle estimado da janela exibida. */
static lv_obj_t *s_duty_label = NULL;

/** @brief Label com o duty cycle estimado da janela exibida para o segundo canal. */
static lv_obj_t *s_duty_label_ch2 = NULL;

/** @brief Buffer externo usado pelo `lv_chart`. */
static int32_t s_chart_points[APP_ADC_CHART_POINTS] = {0};

/** @brief Buffer externo usado pelo `lv_chart` para o segundo canal. */
static int32_t s_chart_points_ch2[APP_ADC_CHART_POINTS] = {0};

/** @brief Buffer temporário para compor a próxima janela antes de publicar no chart. */
static int32_t s_chart_points_pending[APP_ADC_CHART_POINTS] = {0};

/** @brief Buffer temporário para compor a próxima janela do segundo canal antes de publicar no chart. */
static int32_t s_chart_points_pending_ch2[APP_ADC_CHART_POINTS] = {0};

/** @brief Último snapshot copiado do módulo ADC. */
static adc_scope_snapshot_t s_scope_snapshot = {0};

/** @brief Índice atual da base de tempo selecionada. */
static uint16_t s_timebase_index = 1U;

/** @brief Índice atual da escala vertical selecionada. */
static uint16_t s_voltscale_index = 5U;

/** @brief Modo de trigger atualmente selecionado. */
static adc_scope_trigger_mode_t s_trigger_mode = ADC_SCOPE_TRIGGER_FREE;

/** @brief Índice do canal exibido atualmente no gráfico. */
static uint16_t s_sample_channel_mode = 0U;

/** @brief Índice do canal usado como referência de trigger. */
static uint16_t s_trigger_channel_index = 0U;

/** @brief Modo de cursor atualmente selecionado. */
static lvgl_cursor_mode_t s_cursor_mode = LVGL_CURSOR_MODE_OFF;

/** @brief Índice da linha de cursor atualmente selecionada. */
static uint16_t s_cursor_selected_line = 0U;

/** @brief Posição do cursor 1 no eixo X do chart, em pontos visuais. */
static uint16_t s_cursor_time_pos_1 = APP_ADC_CHART_POINTS / 3U;

/** @brief Posição do cursor 2 no eixo X do chart, em pontos visuais. */
static uint16_t s_cursor_time_pos_2 = (APP_ADC_CHART_POINTS * 2U) / 3U;

/** @brief Posição do cursor 1 no eixo Y do chart, em milivolts. */
static int32_t s_cursor_voltage_mv_1 = 1100;

/** @brief Posição do cursor 2 no eixo Y do chart, em milivolts. */
static int32_t s_cursor_voltage_mv_2 = 2200;

/** @brief Indica se o arraste atual realmente capturou a linha de cursor selecionada. */
static bool s_cursor_drag_active = false;

/** @brief Modo de execução do trigger atualmente selecionado. */
static adc_scope_trigger_run_mode_t s_trigger_run_mode = ADC_SCOPE_TRIGGER_RUN_AUTO;

/** @brief Indica se a UI está em modo de pausa. */
static bool s_scope_paused = false;

/** @brief Offset temporal aplicado à janela durante a pausa. */
static size_t s_history_offset_samples = 0U;

/** @brief Indica se o efeito visual de varredura contínua está ativo no modo livre. */
static bool s_free_run_sweep_active = false;

/** @brief Quantidade de pontos já revelados durante o enchimento inicial da varredura livre. */
static size_t s_free_run_visible_points = 0U;

/** @brief Última largura temporal usada pela varredura livre para detectar mudanças de configuração. */
static size_t s_free_run_last_requested_samples = 0U;

/** @brief Último modo de canal exibido usado pela varredura livre para detectar mudanças de configuração. */
static uint16_t s_free_run_last_channel_mode = 0U;

/** @brief Última posição X registrada ao iniciar um arraste horizontal. */
static int16_t s_drag_start_x = 0;

/** @brief Offset temporal no início do último arraste. */
static size_t s_drag_start_offset_samples = 0U;

/** @brief Última posição Y registrada ao iniciar ajuste vertical do trigger. */
static int16_t s_drag_start_y = 0;

/** @brief Nível de trigger no início do último arraste vertical. */
static int32_t s_drag_start_trigger_level_mv = 1650;

/** @brief Nível atual do trigger em milivolts. */
static int32_t s_trigger_level_mv = 1650;

/** @brief Instante, em microssegundos, até o qual a linha visual do trigger deve permanecer visível. */
static int64_t s_trigger_visual_hold_until_us = 0;

/** @brief Indica se o chart está em modo foco ocupando quase toda a tela. */
static bool s_scope_fullscreen = false;

/** @brief Quantidade de toques curtos consecutivos detectados no chart. */
static uint8_t s_scope_tap_count = 0U;

/** @brief Instante do último toque curto detectado no chart, em microssegundos. */
static int64_t s_scope_last_tap_time_us = 0;

/**
 * @brief Configura um overlay do chart para não interceptar gestos do usuário.
 *
 * @param[in] obj Objeto filho do chart que deve repassar eventos ao pai.
 */
static void lvgl_make_chart_overlay_passthrough(lv_obj_t *obj)
{
    if (obj == NULL) {
        return;
    }

    lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(obj, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_add_flag(obj, LV_OBJ_FLAG_GESTURE_BUBBLE);
}

/**
 * @brief Informa se a base de tempo atual ainda permite usar trigger.
 *
 * @return `true` quando a janela atual é menor que 1 segundo.
 */
static bool lvgl_timebase_allows_trigger(void)
{
    return s_timebase_options[s_timebase_index].total_window_us < 500000U;
}

/**
 * @brief Mantém os indicadores visuais do trigger aparentes por um tempo mínimo.
 *
 * @param[in] hold_ms Tempo, em milissegundos, de permanência visual.
 */
static void lvgl_hold_trigger_visual(uint32_t hold_ms)
{
    const int64_t hold_until_us = esp_timer_get_time() + ((int64_t)hold_ms * 1000LL);

    if (hold_until_us > s_trigger_visual_hold_until_us) {
        s_trigger_visual_hold_until_us = hold_until_us;
    }
}

/**
 * @brief Atualiza o chart com a janela correspondente à base de tempo escolhida.
 *
 * @param[in] timer Timer do LVGL que disparou a atualização.
 */
static void lvgl_scope_refresh_timer_cb(lv_timer_t *timer);

/**
 * @brief Atualiza a linha e o label visuais do nível de trigger.
 */
static void lvgl_update_trigger_level_visuals(void);

/**
 * @brief Atualiza as cores da faixa inferior conforme o canal exibido.
 */
static void lvgl_update_metric_strip_styles(void);

/**
 * @brief Reorganiza o layout do osciloscópio após mudanças de modo ou canal.
 */
static void lvgl_update_scope_layout(void);

/**
 * @brief Atualiza as linhas e labels dos cursores conforme o estado atual.
 */
static void lvgl_update_cursor_visuals(void);

/**
 * @brief Atualiza o label sobre o chart com buffer ou medições de cursor.
 */
static void lvgl_update_buffer_label(void);

/**
 * @brief Executa a heurística de Auto Set no contexto da UI local.
 */
static void lvgl_apply_auto_settings(void);

/**
 * @brief Callback periódica que incrementa o tick interno do LVGL.
 *
 * @param[in] arg Contexto não utilizado.
 */
static void lvgl_tick_cb(void *arg)
{
    (void)arg;
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

/**
 * @brief Callback de leitura do touch para o LVGL.
 *
 * @param[in] indev Dispositivo de entrada do LVGL.
 * @param[out] data Estrutura preenchida com o estado do toque.
 */
static void lvgl_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    ft6336u_touch_data_t touch_data = {0};
    (void)indev;

    if (!s_touch || ft6336u_read_touch_data(s_touch, &touch_data) != ESP_OK || touch_data.touch_count == 0 || !touch_data.points[0].pressed) {
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }

    data->state = LV_INDEV_STATE_PRESSED;
    data->point.x = (int16_t)touch_data.points[0].x;
    data->point.y = (int16_t)touch_data.points[0].y;
}

/**
 * @brief Callback de flush do LVGL para enviar uma área renderizada ao ST7796.
 *
 * @param[in] display Display do LVGL que solicitou o flush.
 * @param[in] area Área retangular atualizada.
 * @param[in] px_map Buffer de pixels RGB565 produzido pelo LVGL.
 */
static void lvgl_flush_cb(lv_display_t *display, const lv_area_t *area, uint8_t *px_map)
{
    const uint16_t x_start = (uint16_t)area->x1;
    const uint16_t y_start = (uint16_t)area->y1;
    const uint16_t x_end = (uint16_t)(area->x2 + 1);
    const uint16_t y_end = (uint16_t)(area->y2 + 1);

    ESP_ERROR_CHECK(st7796_draw_bitmap_blocking(s_lcd, x_start, y_start, x_end, y_end, px_map));
    lv_display_flush_ready(display);
}

/**
 * @brief Retorna a base de tempo selecionada no dropdown.
 *
 * @return Estrutura da base de tempo atualmente ativa.
 */
static const lvgl_scope_timebase_t *lvgl_get_selected_timebase(void)
{
    return &s_timebase_options[s_timebase_index];
}

/**
 * @brief Retorna a escala vertical selecionada no dropdown.
 *
 * @return Estrutura da escala vertical atualmente ativa.
 */
static const lvgl_scope_voltscale_t *lvgl_get_selected_voltscale(void)
{
    return &s_voltscale_options[s_voltscale_index];
}

/**
 * @brief Retorna a tensão máxima mostrada no topo do chart.
 *
 * @return Tensão máxima atual em milivolts.
 */
static int32_t lvgl_get_chart_y_max_mv(void)
{
    return lvgl_get_selected_voltscale()->max_mv;
}

/**
 * @brief Atualiza os labels que indicam o valor de cada divisão da grade.
 */
static void lvgl_update_grid_scale_labels(void)
{
    char text[40];
    uint32_t time_per_div_us = 0U;
    int32_t volt_per_div_mv = 0;

    if (s_time_div_label == NULL || s_volt_div_label == NULL) {
        return;
    }

    time_per_div_us = lvgl_get_selected_timebase()->total_window_us / LVGL_SCOPE_TIME_DIVS;
    volt_per_div_mv = lvgl_get_chart_y_max_mv() / (int32_t)LVGL_SCOPE_VOLT_DIVS;

    if (time_per_div_us >= 1000U) {
        lv_snprintf(text, sizeof(text), "T/div: %lu.%02lums",
                    (unsigned long)(time_per_div_us / 1000U),
                    (unsigned long)((time_per_div_us % 1000U) / 10U));
    } else {
        lv_snprintf(text, sizeof(text), "T/div: %luus", (unsigned long)time_per_div_us);
    }
    lv_label_set_text(s_time_div_label, text);

    if (volt_per_div_mv >= 1000) {
        lv_snprintf(text, sizeof(text), "V/div: %ld.%02ldV",
                    (long)(volt_per_div_mv / 1000),
                    (long)((volt_per_div_mv % 1000) / 10));
    } else {
        lv_snprintf(text, sizeof(text), "V/div: %ldmV", (long)volt_per_div_mv);
    }
    lv_label_set_text(s_volt_div_label, text);
}

/**
 * @brief Atualiza as linhas centrais destacadas do chart.
 */
static void lvgl_update_center_grid_lines(void)
{
    int32_t chart_width = 0;
    int32_t chart_height = 0;
    int32_t center_x = 0;
    int32_t center_y = 0;

    if (s_scope_chart == NULL || s_center_vertical_line == NULL || s_center_horizontal_line == NULL) {
        return;
    }

    chart_width = lv_obj_get_content_width(s_scope_chart);
    chart_height = lv_obj_get_content_height(s_scope_chart);
    center_x = chart_width / 2;
    center_y = chart_height / 2;

    if (chart_width <= 1 || chart_height <= 1) {
        return;
    }

    lv_obj_set_size(s_center_vertical_line, 1, chart_height);
    lv_obj_set_pos(s_center_vertical_line, center_x, 0);
    lv_obj_move_to_index(s_center_vertical_line, 0);

    lv_obj_set_size(s_center_horizontal_line, chart_width, 1);
    lv_obj_set_pos(s_center_horizontal_line, 0, center_y);
    lv_obj_move_to_index(s_center_horizontal_line, 0);
}

/**
 * @brief Mostra ou oculta um aviso central temporário na tela.
 *
 * @param[in] text Texto do aviso. Quando `NULL`, o aviso é ocultado.
 */
static void lvgl_set_center_notice(const char *text)
{
    if (s_center_notice_label == NULL) {
        return;
    }

    if (text == NULL || text[0] == '\0') {
        lv_obj_add_flag(s_center_notice_label, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    lv_label_set_text(s_center_notice_label, text);
    lv_obj_remove_flag(s_center_notice_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_to_index(s_center_notice_label, -1);
}

/**
 * @brief Solicita a ocultação do aviso central após um tempo mínimo.
 *
 * @param[in] hold_ms Tempo mínimo restante, em milissegundos.
 */
static void lvgl_hold_center_notice(uint32_t hold_ms)
{
    const int64_t now_us = esp_timer_get_time();
    const int64_t hold_until_us = now_us + ((int64_t)hold_ms * 1000LL);

    if (hold_until_us > s_center_notice_hold_until_us) {
        s_center_notice_hold_until_us = hold_until_us;
    }
    s_center_notice_pending_hide = true;
}

/**
 * @brief Reinicia o efeito visual de varredura contínua do modo livre.
 */
static void lvgl_reset_free_run_sweep(void);

/**
 * @brief Copia a janela recém-renderizada integralmente para o chart.
 */
static void lvgl_publish_pending_points_full(void);

/**
 * @brief Aplica um efeito visual de varredura contínua no modo livre.
 *
 * @param[in] requested_samples Janela temporal atual em amostras reais.
 */
static void lvgl_publish_pending_points_free_run(size_t requested_samples);

/**
 * @brief Publica o estado atual da UI local para leitura pela interface web.
 */
static void lvgl_publish_control_state(void)
{
    if (s_control_state_mutex == NULL) {
        return;
    }

    if (xSemaphoreTake(s_control_state_mutex, portMAX_DELAY) == pdTRUE) {
        s_shared_control_state.sample_channel_mode = s_sample_channel_mode;
        s_shared_control_state.timebase_index = s_timebase_index;
        s_shared_control_state.voltscale_index = s_voltscale_index;
        s_shared_control_state.trigger_channel_index = s_trigger_channel_index;
        s_shared_control_state.trigger_mode = (uint16_t)s_trigger_mode;
        s_shared_control_state.trigger_run_mode = (uint16_t)s_trigger_run_mode;
        s_shared_control_state.paused = s_scope_paused;
        xSemaphoreGive(s_control_state_mutex);
    }
}

/**
 * @brief Aplica um estado externo pendente no contexto da task do LVGL.
 *
 * @param[in] state Estado desejado vindo da web.
 */
static void lvgl_apply_external_control_state(const lvgl_app_control_state_t *state)
{
    if (state == NULL) {
        return;
    }

    s_suppress_dropdown_events = true;

    s_sample_channel_mode = (state->sample_channel_mode <= 2U) ? state->sample_channel_mode : 0U;
    s_timebase_index = (state->timebase_index < (uint16_t)(sizeof(s_timebase_options) / sizeof(s_timebase_options[0]))) ? state->timebase_index : 0U;
    s_voltscale_index = (state->voltscale_index < (uint16_t)(sizeof(s_voltscale_options) / sizeof(s_voltscale_options[0]))) ?
        state->voltscale_index : ((uint16_t)(sizeof(s_voltscale_options) / sizeof(s_voltscale_options[0])) - 1U);
    s_trigger_channel_index = (state->trigger_channel_index <= 1U) ? state->trigger_channel_index : 0U;
    s_trigger_mode = (state->trigger_mode <= (uint16_t)ADC_SCOPE_TRIGGER_FALL) ?
        (adc_scope_trigger_mode_t)state->trigger_mode : ADC_SCOPE_TRIGGER_FREE;
    s_trigger_run_mode = (state->trigger_run_mode <= (uint16_t)ADC_SCOPE_TRIGGER_RUN_SINGLE) ?
        (adc_scope_trigger_run_mode_t)state->trigger_run_mode : ADC_SCOPE_TRIGGER_RUN_AUTO;

    if (!lvgl_timebase_allows_trigger()) {
        s_trigger_mode = ADC_SCOPE_TRIGGER_FREE;
    }

    if (s_channel_dropdown != NULL) {
        lv_dropdown_set_selected(s_channel_dropdown, s_sample_channel_mode);
    }
    if (s_timebase_dropdown != NULL) {
        lv_dropdown_set_selected(s_timebase_dropdown, s_timebase_index);
    }
    if (s_volts_dropdown != NULL) {
        lv_dropdown_set_selected(s_volts_dropdown, s_voltscale_index);
    }
    if (s_trigger_channel_dropdown != NULL) {
        lv_dropdown_set_selected(s_trigger_channel_dropdown, s_trigger_channel_index);
    }
    if (s_trigger_dropdown != NULL) {
        lv_dropdown_set_selected(s_trigger_dropdown, (uint16_t)s_trigger_mode);
    }
    if (s_trigger_run_dropdown != NULL) {
        lv_dropdown_set_selected(s_trigger_run_dropdown, (uint16_t)s_trigger_run_mode);
    }

    if (state->paused != s_scope_paused) {
        if (state->paused) {
            if (adc_scope_stop() == ESP_OK) {
                s_scope_paused = true;
                s_history_offset_samples = 0U;
            }
        } else {
            if (adc_scope_start() == ESP_OK) {
                s_scope_paused = false;
                s_history_offset_samples = 0U;
            }
        }
    }
    if (s_status_dropdown != NULL) {
        lv_dropdown_set_selected(s_status_dropdown, s_scope_paused ? 1U : 0U);
    }

    if (s_trigger_level_mv > lvgl_get_chart_y_max_mv()) {
        s_trigger_level_mv = lvgl_get_chart_y_max_mv();
    }
    if (s_cursor_voltage_mv_1 > lvgl_get_chart_y_max_mv()) {
        s_cursor_voltage_mv_1 = lvgl_get_chart_y_max_mv();
    }
    if (s_cursor_voltage_mv_2 > lvgl_get_chart_y_max_mv()) {
        s_cursor_voltage_mv_2 = lvgl_get_chart_y_max_mv();
    }

    lvgl_update_scope_layout();
    lvgl_update_grid_scale_labels();
    lvgl_update_trigger_level_visuals();
    lvgl_update_cursor_visuals();
    lvgl_update_buffer_label();
    lvgl_publish_control_state();
    s_suppress_dropdown_events = false;
    lvgl_scope_refresh_timer_cb(NULL);
}

/**
 * @brief Processa solicitações externas pendentes dentro da task do LVGL.
 */
static void lvgl_process_pending_external_requests(void)
{
    lvgl_app_control_state_t pending_state = {0};
    bool has_pending_state = false;
    bool pending_auto_set = false;
    bool pending_toggle_focus = false;

    if (!s_scope_ui_ready || s_control_state_mutex == NULL) {
        return;
    }

    if (xSemaphoreTake(s_control_state_mutex, 0) != pdTRUE) {
        return;
    }

    has_pending_state = s_has_pending_control_state;
    if (has_pending_state) {
        pending_state = s_pending_control_state;
        s_has_pending_control_state = false;
    }
    pending_auto_set = s_pending_auto_set;
    s_pending_auto_set = false;
    pending_toggle_focus = s_pending_toggle_focus;
    s_pending_toggle_focus = false;
    xSemaphoreGive(s_control_state_mutex);

    if (has_pending_state) {
        lvgl_apply_external_control_state(&pending_state);
    }
    if (pending_auto_set) {
        lvgl_apply_auto_settings();
        lvgl_publish_control_state();
    }
    if (pending_toggle_focus) {
        s_scope_fullscreen = !s_scope_fullscreen;
        lvgl_update_scope_layout();
        lvgl_scope_refresh_timer_cb(NULL);
    }
}

/**
 * @brief Estima a frequência de uma janela reamostrada usando cruzamentos de nível médio.
 *
 * @param[in] points Pontos da janela exibida.
 * @param[in] point_count Quantidade de pontos válidos no vetor.
 * @param[in] window_us Duração total da janela em microssegundos.
 * @param[in] min_mv Menor valor da janela.
 * @param[in] max_mv Maior valor da janela.
 *
 * @return Frequência estimada em décimos de hertz, ou zero quando não foi possível estimar.
 */
static uint32_t lvgl_estimate_frequency_tenths_from_points(const int32_t *points,
                                                           size_t point_count,
                                                           uint32_t window_us,
                                                           int32_t min_mv,
                                                           int32_t max_mv)
{
    int32_t threshold = 0;
    int32_t first_cross = -1;
    int32_t second_cross = -1;

    if (points == NULL || point_count < 4U || window_us == 0U || max_mv <= min_mv) {
        return 0U;
    }

    threshold = min_mv + ((max_mv - min_mv) / 2);

    for (size_t i = 1U; i < point_count; i++) {
        const int32_t prev = points[i - 1U];
        const int32_t curr = points[i];
        const bool rising = (prev < threshold) && (curr >= threshold);

        if (!rising) {
            continue;
        }

        if (first_cross < 0) {
            first_cross = (int32_t)i;
        } else {
            second_cross = (int32_t)i;
            break;
        }
    }

    if (first_cross < 0 || second_cross <= first_cross) {
        return 0U;
    }

    {
        const uint64_t delta_points = (uint64_t)(second_cross - first_cross);
        const uint64_t period_us = (delta_points * (uint64_t)window_us) / (uint64_t)(point_count - 1U);
        if (period_us == 0U) {
            return 0U;
        }

        return (uint32_t)((10000000ULL + (period_us / 2ULL)) / period_us);
    }
}

/**
 * @brief Busca uma estimativa de frequência usando janelas progressivamente maiores do histórico.
 *
 * @param[in] analysis_channel Canal usado como referência.
 * @param[out] out_freq_tenths_hz Frequência estimada em décimos de hertz.
 *
 * @return `true` quando conseguiu estimar a frequência.
 */
static bool lvgl_try_estimate_frequency_from_history(size_t analysis_channel, uint32_t *out_freq_tenths_hz)
{
    int32_t analysis_ch1[APP_ADC_CHART_POINTS] = {0};
    int32_t analysis_ch2[APP_ADC_CHART_POINTS] = {0};
    int32_t *dest_buffers[ADC_SCOPE_MAX_CHANNELS] = {
        analysis_ch1,
        analysis_ch2,
    };
    adc_scope_snapshot_t analysis_snapshot = {0};
    uint32_t sample_freq_hz = 0U;

    if (out_freq_tenths_hz == NULL) {
        return false;
    }

    *out_freq_tenths_hz = 0U;

    if (adc_scope_get_sample_freq_hz(&sample_freq_hz) != ESP_OK || sample_freq_hz == 0U) {
        return false;
    }

    for (uint16_t i = s_timebase_index; i < (uint16_t)(sizeof(s_timebase_options) / sizeof(s_timebase_options[0])); i++) {
        const uint64_t requested_samples_64 = ((uint64_t)sample_freq_hz * (uint64_t)s_timebase_options[i].total_window_us) / 1000000ULL;
        size_t requested_samples = (size_t)((requested_samples_64 == 0U) ? 1U : requested_samples_64);

        if (requested_samples < APP_ADC_CHART_POINTS) {
            requested_samples = APP_ADC_CHART_POINTS;
        }

        if (adc_scope_copy_chart_points_multi(dest_buffers,
                                              APP_ADC_CHART_POINTS,
                                              requested_samples,
                                              ADC_SCOPE_TRIGGER_FREE,
                                              ADC_SCOPE_TRIGGER_RUN_AUTO,
                                              s_trigger_channel_index,
                                              LVGL_SCOPE_TRIGGER_POS,
                                              0U,
                                              s_trigger_level_mv,
                                              0U,
                                              &analysis_snapshot) != ESP_OK) {
            continue;
        }

        {
            const int32_t *analysis_points = (analysis_channel == 0U) ? analysis_ch1 : analysis_ch2;
            const uint32_t estimated = lvgl_estimate_frequency_tenths_from_points(analysis_points,
                                                                                  APP_ADC_CHART_POINTS,
                                                                                  s_timebase_options[i].total_window_us,
                                                                                  analysis_snapshot.min_mv[analysis_channel],
                                                                                  analysis_snapshot.max_mv[analysis_channel]);
            if (estimated > 0U) {
                *out_freq_tenths_hz = estimated;
                return true;
            }
        }
    }

    return false;
}

/**
 * @brief Escolhe e aplica automaticamente uma escala vertical e uma base de tempo.
 */
static void lvgl_apply_auto_settings(void)
{
    size_t analysis_channel = (s_sample_channel_mode == 1U) ? 1U : 0U;
    uint32_t estimated_freq_tenths_hz = 0U;
    uint32_t desired_window_us = 0U;
    int32_t peak_mv = 0;

    lvgl_set_center_notice("Auto Set...\nCapturando e ajustando");
    s_center_notice_pending_hide = false;
    s_center_notice_hold_until_us = esp_timer_get_time();
    lv_refr_now(NULL);

    peak_mv = s_scope_snapshot.max_mv[analysis_channel];

    for (uint16_t i = 0; i < (uint16_t)(sizeof(s_voltscale_options) / sizeof(s_voltscale_options[0])); i++) {
        if (peak_mv <= s_voltscale_options[i].max_mv) {
            s_voltscale_index = i;
            break;
        }
        s_voltscale_index = (uint16_t)(sizeof(s_voltscale_options) / sizeof(s_voltscale_options[0])) - 1U;
    }

    if (analysis_channel < ADC_SCOPE_MAX_CHANNELS && s_scope_snapshot.measurements_valid[analysis_channel]) {
        estimated_freq_tenths_hz = s_scope_snapshot.frequency_tenths_hz[analysis_channel];
    } else {
        const int32_t *analysis_points = (analysis_channel == 0U) ? s_chart_points : s_chart_points_ch2;
        estimated_freq_tenths_hz = lvgl_estimate_frequency_tenths_from_points(analysis_points,
                                                                              APP_ADC_CHART_POINTS,
                                                                              lvgl_get_selected_timebase()->total_window_us,
                                                                              s_scope_snapshot.min_mv[analysis_channel],
                                                                              s_scope_snapshot.max_mv[analysis_channel]);
        if (estimated_freq_tenths_hz == 0U) {
            (void)lvgl_try_estimate_frequency_from_history(analysis_channel, &estimated_freq_tenths_hz);
        }
    }

    if (estimated_freq_tenths_hz > 0U) {
        const uint64_t period_us = 10000000ULL / estimated_freq_tenths_hz;
        desired_window_us = (uint32_t)(period_us * 4ULL);

        for (uint16_t i = 0; i < (uint16_t)(sizeof(s_timebase_options) / sizeof(s_timebase_options[0])); i++) {
            if (desired_window_us <= s_timebase_options[i].total_window_us) {
                s_timebase_index = i;
                break;
            }
            s_timebase_index = (uint16_t)(sizeof(s_timebase_options) / sizeof(s_timebase_options[0])) - 1U;
        }
    }

    if (s_volts_dropdown != NULL) {
        lv_dropdown_set_selected(s_volts_dropdown, s_voltscale_index);
    }
    if (s_timebase_dropdown != NULL) {
        lv_dropdown_set_selected(s_timebase_dropdown, s_timebase_index);
    }

    if (s_trigger_level_mv > lvgl_get_chart_y_max_mv()) {
        s_trigger_level_mv = lvgl_get_chart_y_max_mv();
    }
    if (s_cursor_voltage_mv_1 > lvgl_get_chart_y_max_mv()) {
        s_cursor_voltage_mv_1 = lvgl_get_chart_y_max_mv();
    }
    if (s_cursor_voltage_mv_2 > lvgl_get_chart_y_max_mv()) {
        s_cursor_voltage_mv_2 = lvgl_get_chart_y_max_mv();
    }

    lvgl_scope_refresh_timer_cb(NULL);
    lvgl_hold_center_notice(2000U);
}

/**
 * @brief Calcula quantas amostras recentes devem ser exibidas no gráfico.
 *
 * @return Quantidade de amostras desejadas para a janela atual.
 */
static size_t lvgl_get_requested_window_samples(void)
{
    uint32_t sample_freq_hz = 0;
    uint64_t requested_samples = 0;
    const lvgl_scope_timebase_t *timebase = lvgl_get_selected_timebase();

    if (adc_scope_get_sample_freq_hz(&sample_freq_hz) != ESP_OK) {
        return APP_ADC_CHART_POINTS;
    }

    requested_samples = ((uint64_t)sample_freq_hz * (uint64_t)timebase->total_window_us) / 1000000ULL;
    if (requested_samples == 0U) {
        requested_samples = 1U;
    }

    return (size_t)requested_samples;
}

/**
 * @brief Indica se um canal deve ser exibido no gráfico na configuração atual.
 *
 * @param[in] channel_index Índice lógico do canal.
 *
 * @return `true` quando o canal deve permanecer visível.
 */
static bool lvgl_channel_is_visible(size_t channel_index)
{
    if (s_sample_channel_mode == 2U) {
        return true;
    }

    return s_sample_channel_mode == (uint16_t)channel_index;
}

/**
 * @brief Atualiza as linhas de cursor e o delta visível no chart.
 */
static void lvgl_update_cursor_visuals(void)
{
    char text[40];
    char text_1[40];
    char text_2[40];
    int32_t chart_width = 0;
    int32_t chart_height = 0;

    if (s_scope_chart == NULL || s_cursor_line_1 == NULL || s_cursor_line_2 == NULL ||
        s_cursor_delta_label == NULL || s_cursor_value_1_label == NULL || s_cursor_value_2_label == NULL) {
        return;
    }

    chart_width = lv_obj_get_content_width(s_scope_chart);
    chart_height = lv_obj_get_content_height(s_scope_chart);
    if (chart_width <= 1 || chart_height <= 1) {
        return;
    }

    if (s_cursor_mode == LVGL_CURSOR_MODE_OFF) {
        lv_obj_add_flag(s_cursor_line_1, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_cursor_line_2, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_cursor_delta_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_cursor_value_1_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_cursor_value_2_label, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    lv_obj_remove_flag(s_cursor_line_1, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(s_cursor_line_2, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(s_cursor_delta_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(s_cursor_value_1_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(s_cursor_value_2_label, LV_OBJ_FLAG_HIDDEN);

    if (s_cursor_mode == LVGL_CURSOR_MODE_TIME) {
        const int32_t x1 = (int32_t)((((uint32_t)s_cursor_time_pos_1) * (uint32_t)(chart_width - 1)) / (APP_ADC_CHART_POINTS - 1U));
        const int32_t x2 = (int32_t)((((uint32_t)s_cursor_time_pos_2) * (uint32_t)(chart_width - 1)) / (APP_ADC_CHART_POINTS - 1U));
        const uint16_t min_pos = (s_cursor_time_pos_1 < s_cursor_time_pos_2) ? s_cursor_time_pos_1 : s_cursor_time_pos_2;
        const uint16_t max_pos = (s_cursor_time_pos_1 > s_cursor_time_pos_2) ? s_cursor_time_pos_1 : s_cursor_time_pos_2;

        lv_obj_set_size(s_cursor_line_1, 2, chart_height);
        lv_obj_set_size(s_cursor_line_2, 2, chart_height);
        lv_obj_set_pos(s_cursor_line_1, x1, 0);
        lv_obj_set_pos(s_cursor_line_2, x2, 0);

        if (s_scope_snapshot.sample_count > 1U) {
            uint32_t sample_freq_hz = 0U;
            if (adc_scope_get_sample_freq_hz(&sample_freq_hz) == ESP_OK && sample_freq_hz > 0U) {
                const uint64_t samples_1 = ((uint64_t)s_cursor_time_pos_1 * (uint64_t)(s_scope_snapshot.sample_count - 1U)) / (APP_ADC_CHART_POINTS - 1U);
                const uint64_t samples_2 = ((uint64_t)s_cursor_time_pos_2 * (uint64_t)(s_scope_snapshot.sample_count - 1U)) / (APP_ADC_CHART_POINTS - 1U);
                const uint64_t delta_points = (uint64_t)(max_pos - min_pos);
                const uint64_t delta_samples = (delta_points * (uint64_t)(s_scope_snapshot.sample_count - 1U)) / (APP_ADC_CHART_POINTS - 1U);
                const uint64_t time_1_us = (samples_1 * 1000000ULL) / sample_freq_hz;
                const uint64_t time_2_us = (samples_2 * 1000000ULL) / sample_freq_hz;
                const uint64_t delta_us = (delta_samples * 1000000ULL) / sample_freq_hz;

                if (time_1_us >= 1000ULL) {
                    lv_snprintf(text_1, sizeof(text_1), "T1: %lu.%02lums",
                                (unsigned long)(time_1_us / 1000ULL),
                                (unsigned long)((time_1_us % 1000ULL) / 10ULL));
                } else {
                    lv_snprintf(text_1, sizeof(text_1), "T1: %luus", (unsigned long)time_1_us);
                }

                if (time_2_us >= 1000ULL) {
                    lv_snprintf(text_2, sizeof(text_2), "T2: %lu.%02lums",
                                (unsigned long)(time_2_us / 1000ULL),
                                (unsigned long)((time_2_us % 1000ULL) / 10ULL));
                } else {
                    lv_snprintf(text_2, sizeof(text_2), "T2: %luus", (unsigned long)time_2_us);
                }

                if (delta_us >= 1000ULL) {
                    lv_snprintf(text, sizeof(text), "Dt: %lu.%02lums",
                                (unsigned long)(delta_us / 1000ULL),
                                (unsigned long)((delta_us % 1000ULL) / 10ULL));
                } else {
                    lv_snprintf(text, sizeof(text), "Dt: %luus", (unsigned long)delta_us);
                }
                lv_label_set_text(s_cursor_value_1_label, text_1);
                lv_label_set_text(s_cursor_value_2_label, text_2);
                lv_label_set_text(s_cursor_delta_label, text);
            } else {
                lv_label_set_text(s_cursor_value_1_label, "T1: --");
                lv_label_set_text(s_cursor_value_2_label, "T2: --");
                lv_label_set_text(s_cursor_delta_label, "Dt: --");
            }
        } else {
            lv_label_set_text(s_cursor_value_1_label, "T1: --");
            lv_label_set_text(s_cursor_value_2_label, "T2: --");
            lv_label_set_text(s_cursor_delta_label, "Dt: --");
        }
    } else {
        const int32_t y_max_mv = lvgl_get_chart_y_max_mv();
        const int32_t clamped_mv_1 = (s_cursor_voltage_mv_1 < 0) ? 0 : ((s_cursor_voltage_mv_1 > y_max_mv) ? y_max_mv : s_cursor_voltage_mv_1);
        const int32_t clamped_mv_2 = (s_cursor_voltage_mv_2 < 0) ? 0 : ((s_cursor_voltage_mv_2 > y_max_mv) ? y_max_mv : s_cursor_voltage_mv_2);
        const int32_t y1 = ((y_max_mv - clamped_mv_1) * (chart_height - 1)) / y_max_mv;
        const int32_t y2 = ((y_max_mv - clamped_mv_2) * (chart_height - 1)) / y_max_mv;
        const int32_t delta_mv = (clamped_mv_1 > clamped_mv_2) ? (clamped_mv_1 - clamped_mv_2) : (clamped_mv_2 - clamped_mv_1);

        lv_obj_set_size(s_cursor_line_1, chart_width, 2);
        lv_obj_set_size(s_cursor_line_2, chart_width, 2);
        lv_obj_set_pos(s_cursor_line_1, 0, y1);
        lv_obj_set_pos(s_cursor_line_2, 0, y2);
        lv_snprintf(text_1, sizeof(text_1), "V1: %ld.%02ldV",
                    (long)(clamped_mv_1 / 1000),
                    (long)((clamped_mv_1 % 1000) / 10));
        lv_snprintf(text_2, sizeof(text_2), "V2: %ld.%02ldV",
                    (long)(clamped_mv_2 / 1000),
                    (long)((clamped_mv_2 % 1000) / 10));
        lv_snprintf(text, sizeof(text), "DV: %ld.%02ldV",
                    (long)(delta_mv / 1000),
                    (long)((delta_mv % 1000) / 10));
        lv_label_set_text(s_cursor_value_1_label, text_1);
        lv_label_set_text(s_cursor_value_2_label, text_2);
        lv_label_set_text(s_cursor_delta_label, text);
    }

    lv_obj_set_style_bg_opa(s_cursor_line_1, (s_cursor_selected_line == 0U) ? LV_OPA_COVER : LV_OPA_60, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_cursor_line_2, (s_cursor_selected_line == 1U) ? LV_OPA_COVER : LV_OPA_60, LV_PART_MAIN);
    lv_obj_move_to_index(s_cursor_line_1, -1);
    lv_obj_move_to_index(s_cursor_line_2, -1);
    lv_obj_move_to_index(s_cursor_value_1_label, -1);
    lv_obj_move_to_index(s_cursor_value_2_label, -1);
    lv_obj_move_to_index(s_cursor_delta_label, -1);
}

/**
 * @brief Ajusta o layout entre modo de um canal e modo de dois canais.
 */
static void lvgl_update_scope_layout(void)
{
    const bool dual_channel = (s_sample_channel_mode == 2U);

    if (s_scope_chart == NULL || s_metrics_strip == NULL || s_controls_strip == NULL) {
        return;
    }

    if (s_scope_fullscreen) {
        lv_obj_add_flag(s_controls_strip, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_metrics_strip, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_size(s_scope_chart, ST7796_H_RES - 6, ST7796_V_RES - 6);
        lv_obj_align(s_scope_chart, LV_ALIGN_CENTER, 0, 0);
        lvgl_update_center_grid_lines();
        lvgl_update_trigger_level_visuals();
        lvgl_update_cursor_visuals();
        return;
    }

    lv_obj_remove_flag(s_controls_strip, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(s_metrics_strip, LV_OBJ_FLAG_HIDDEN);

    lv_obj_set_size(s_scope_chart, ST7796_H_RES - 10, dual_channel ? 192 : 217);
    lv_obj_align(s_scope_chart, LV_ALIGN_BOTTOM_MID, 0, dual_channel ? -52 : -27);

    lv_obj_set_size(s_metrics_strip, ST7796_H_RES, dual_channel ? 50 : 25);
    lv_obj_align(s_metrics_strip, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    lvgl_update_metric_strip_styles();

    if (s_rms_label_ch2 != NULL) {
        if (dual_channel) {
            lv_obj_remove_flag(s_rms_label_ch2, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(s_rms_label_ch2, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (s_pk_label_ch2 != NULL) {
        if (dual_channel) {
            lv_obj_remove_flag(s_pk_label_ch2, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(s_pk_label_ch2, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (s_pk_neg_label_ch2 != NULL) {
        if (dual_channel) {
            lv_obj_remove_flag(s_pk_neg_label_ch2, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(s_pk_neg_label_ch2, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (s_freq_label_ch2 != NULL) {
        if (dual_channel) {
            lv_obj_remove_flag(s_freq_label_ch2, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(s_freq_label_ch2, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (s_duty_label_ch2 != NULL) {
        if (dual_channel) {
            lv_obj_remove_flag(s_duty_label_ch2, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(s_duty_label_ch2, LV_OBJ_FLAG_HIDDEN);
        }
    }

    lvgl_update_cursor_visuals();
}

/**
 * @brief Atualiza o label interno com a ocupação do buffer de histórico.
 */
static void lvgl_update_buffer_label(void)
{
    char buffer_text[40];
    unsigned percent = 0U;
    size_t window_start = 0U;
    size_t window_end = 0U;
    size_t visible_history = 0U;

    if (s_buffer_label == NULL) {
        return;
    }

    if (!s_scope_paused) {
        if (s_scope_snapshot.history_capacity > 0U) {
            percent = (unsigned)((s_scope_snapshot.history_count * 100U) / s_scope_snapshot.history_capacity);
            if (percent > 100U) {
                percent = 100U;
            }
        }
        lv_snprintf(buffer_text, sizeof(buffer_text), "Buffer: %u%%", percent);
        lv_label_set_text(s_buffer_label, buffer_text);
        return;
    }

    if (s_scope_snapshot.history_count <= 1U || s_scope_snapshot.sample_count == 0U) {
        lv_label_set_text(s_buffer_label, "Buffer: 0%");
        return;
    }

    visible_history = s_scope_snapshot.history_count;
    if (s_history_offset_samples > visible_history) {
        lv_label_set_text(s_buffer_label, "Buffer: 0%");
        return;
    }

    visible_history -= s_history_offset_samples;
    if (visible_history > s_scope_snapshot.sample_count) {
        window_start = visible_history - s_scope_snapshot.sample_count;
    }

    if (s_scope_snapshot.sample_count > 0U) {
        window_end = window_start + s_scope_snapshot.sample_count - 1U;
    }
    percent = (unsigned)((window_end * 100U) / (s_scope_snapshot.history_count - 1U));
    if (percent > 100U) {
        percent = 100U;
    }

    lv_snprintf(buffer_text, sizeof(buffer_text), "Buffer: %u%%", percent);
    lv_label_set_text(s_buffer_label, buffer_text);
}

/**
 * @brief Atualiza a linha e o label visuais do nível de trigger.
 */
static void lvgl_update_trigger_level_visuals(void)
{
    char trigger_text[32];
    const int32_t y_max_mv = lvgl_get_chart_y_max_mv();
    int32_t clamped_mv = s_trigger_level_mv;
    int32_t chart_height = 0;
    int32_t chart_width = 0;
    int32_t y = 0;

    if (clamped_mv < 0) {
        clamped_mv = 0;
    } else if (clamped_mv > y_max_mv) {
        clamped_mv = y_max_mv;
    }
    s_trigger_level_mv = clamped_mv;

    if (s_trigger_level_label != NULL) {
        lv_snprintf(trigger_text,
                    sizeof(trigger_text),
                    "Trig: %ld.%02ldV",
                    (long)(clamped_mv / 1000),
                    (long)((clamped_mv % 1000) / 10));
        lv_label_set_text(s_trigger_level_label, trigger_text);
    }

    if (s_trigger_level_line == NULL) {
        return;
    }

    if (s_trigger_mode == ADC_SCOPE_TRIGGER_FREE || !lvgl_timebase_allows_trigger()) {
        lv_obj_add_flag(s_trigger_level_line, LV_OBJ_FLAG_HIDDEN);
        if (s_trigger_level_label != NULL) {
            lv_obj_add_flag(s_trigger_level_label, LV_OBJ_FLAG_HIDDEN);
        }
        return;
    }

    chart_height = lv_obj_get_content_height(s_scope_chart);
    chart_width = lv_obj_get_content_width(s_scope_chart);
    if (chart_height <= 1 || chart_width <= 16) {
        return;
    }

    if (esp_timer_get_time() >= s_trigger_visual_hold_until_us) {
        lv_obj_add_flag(s_trigger_level_line, LV_OBJ_FLAG_HIDDEN);
        if (s_trigger_level_label != NULL) {
            lv_obj_add_flag(s_trigger_level_label, LV_OBJ_FLAG_HIDDEN);
        }
        return;
    }

    y = ((y_max_mv - clamped_mv) * (chart_height - 1)) / y_max_mv;
    lv_obj_remove_flag(s_trigger_level_line, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_size(s_trigger_level_line, chart_width - 16, 2);
    lv_obj_set_pos(s_trigger_level_line, 8, y);
    lv_obj_move_to_index(s_trigger_level_line, -1);
    lv_obj_set_y(s_trigger_level_line, y);

    if (s_trigger_level_label != NULL) {
        lv_obj_remove_flag(s_trigger_level_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_align(s_trigger_level_label, LV_ALIGN_TOP_LEFT, 8, 52);
        lv_obj_move_to_index(s_trigger_level_label, -1);
    }
}

/**
 * @brief Aplica a identidade visual de um grupo ao dropdown correspondente.
 *
 * @param[in] dropdown Dropdown que receberá o estilo.
 * @param[in] accent_color Cor de destaque usada no label do grupo.
 */
static void lvgl_style_dropdown(lv_obj_t *dropdown, lv_color_t accent_color)
{
    lv_obj_t *list = lv_dropdown_get_list(dropdown);

    lv_obj_set_style_bg_color(dropdown, accent_color, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(dropdown, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_text_color(dropdown, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_border_color(dropdown, accent_color, LV_PART_MAIN);
    lv_obj_set_style_border_width(dropdown, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(dropdown, 6, LV_PART_MAIN);

    lv_obj_set_style_bg_color(dropdown, lv_color_hex(0x000000), LV_PART_ITEMS);
    lv_obj_set_style_bg_opa(dropdown, LV_OPA_COVER, LV_PART_ITEMS);
    lv_obj_set_style_text_color(dropdown, lv_color_hex(0xF9FAFB), LV_PART_ITEMS);

    if (list != NULL) {
        lv_obj_set_style_bg_color(list, lv_color_hex(0x000000), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(list, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_border_color(list, accent_color, LV_PART_MAIN);
        lv_obj_set_style_border_width(list, 1, LV_PART_MAIN);
        lv_obj_set_style_text_color(list, lv_color_hex(0xF9FAFB), LV_PART_MAIN);
        lv_obj_set_style_bg_color(list, accent_color, LV_PART_SELECTED);
        lv_obj_set_style_bg_opa(list, LV_OPA_COVER, LV_PART_SELECTED);
        lv_obj_set_style_text_color(list, lv_color_hex(0x000000), LV_PART_SELECTED);
        lv_obj_set_style_bg_color(list, lv_color_hex(0x111827), LV_PART_SCROLLBAR);
        lv_obj_set_style_bg_opa(list, LV_OPA_40, LV_PART_SCROLLBAR);
    }
}

/**
 * @brief Aplica o visual de fundo de uma métrica inferior com a cor do canal.
 *
 * @param[in] label Label da métrica a estilizar.
 * @param[in] bg_color Cor de fundo associada ao canal.
 */
static void lvgl_style_metric_label(lv_obj_t *label, lv_color_t bg_color)
{
    lv_obj_set_width(label, lv_pct(100));
    lv_obj_set_height(label, 23);
    lv_obj_set_style_bg_color(label, bg_color, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(label, LV_OPA_70, LV_PART_MAIN);
    lv_obj_set_style_text_color(label, lv_color_hex(0x020617), LV_PART_MAIN);
    lv_obj_set_style_radius(label, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_left(label, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_right(label, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_top(label, 1, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(label, 1, LV_PART_MAIN);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
}

/**
 * @brief Atualiza as cores da faixa inferior conforme o canal exibido.
 */
static void lvgl_update_metric_strip_styles(void)
{
    lv_color_t primary_color = lv_color_hex(LVGL_COLOR_TRACE_CH1);
    lv_color_t secondary_color = lv_color_hex(LVGL_COLOR_TRACE_CH2);

    if (s_sample_channel_mode == 1U) {
        primary_color = lv_color_hex(LVGL_COLOR_TRACE_CH2);
    }

    if (s_rms_label != NULL) {
        lvgl_style_metric_label(s_rms_label, primary_color);
    }
    if (s_pk_label != NULL) {
        lvgl_style_metric_label(s_pk_label, primary_color);
    }
    if (s_pk_neg_label != NULL) {
        lvgl_style_metric_label(s_pk_neg_label, primary_color);
    }
    if (s_freq_label != NULL) {
        lvgl_style_metric_label(s_freq_label, primary_color);
    }
    if (s_duty_label != NULL) {
        lvgl_style_metric_label(s_duty_label, primary_color);
    }

    if (s_rms_label_ch2 != NULL) {
        lvgl_style_metric_label(s_rms_label_ch2, secondary_color);
    }
    if (s_pk_label_ch2 != NULL) {
        lvgl_style_metric_label(s_pk_label_ch2, secondary_color);
    }
    if (s_pk_neg_label_ch2 != NULL) {
        lvgl_style_metric_label(s_pk_neg_label_ch2, secondary_color);
    }
    if (s_freq_label_ch2 != NULL) {
        lvgl_style_metric_label(s_freq_label_ch2, secondary_color);
    }
    if (s_duty_label_ch2 != NULL) {
        lvgl_style_metric_label(s_duty_label_ch2, secondary_color);
    }
}

/**
 * @brief Preenche os labels de uma métrica textual no formato `Nome: valor`.
 *
 * @param[in] label Label de destino.
 * @param[in] prefix Texto fixo antes do valor.
 * @param[in] valid Indica se existe valor calculado.
 * @param[in] value_mv Valor em milivolts.
 */
static void lvgl_set_metric_voltage_label(lv_obj_t *label, const char *prefix, bool valid, int32_t value_mv)
{
    char text[40];

    if (label == NULL) {
        return;
    }

    if (!valid) {
        lv_snprintf(text, sizeof(text), "%s: --", prefix);
        lv_label_set_text(label, text);
        return;
    }

    lv_snprintf(text, sizeof(text), "%s: %ld.%02ldV",
                prefix,
                (long)(value_mv / 1000),
                (long)((value_mv % 1000) / 10));
    lv_label_set_text(label, text);
}

/**
 * @brief Atualiza um label textual de frequência.
 *
 * @param[in] label Label de destino.
 * @param[in] valid Indica se existe valor calculado.
 * @param[in] freq_tenths_hz Valor em décimos de hertz.
 */
static void lvgl_set_metric_frequency_label(lv_obj_t *label, bool valid, uint32_t freq_tenths_hz)
{
    char text[32];

    if (label == NULL) {
        return;
    }

    if (!valid) {
        lv_label_set_text(label, "Freq: --");
        return;
    }

    lv_snprintf(text, sizeof(text), "Freq: %lu.%1luHz",
                (unsigned long)(freq_tenths_hz / 10U),
                (unsigned long)(freq_tenths_hz % 10U));
    lv_label_set_text(label, text);
}

/**
 * @brief Atualiza um label textual de duty cycle.
 *
 * @param[in] label Label de destino.
 * @param[in] valid Indica se existe valor calculado.
 * @param[in] duty_tenths_percent Valor em décimos de porcentagem.
 */
static void lvgl_set_metric_duty_label(lv_obj_t *label, bool valid, uint32_t duty_tenths_percent)
{
    char text[32];

    if (label == NULL) {
        return;
    }

    if (!valid) {
        lv_label_set_text(label, "Duty: --");
        return;
    }

    lv_snprintf(text, sizeof(text), "Duty: %lu.%1lu%%",
                (unsigned long)(duty_tenths_percent / 10U),
                (unsigned long)(duty_tenths_percent % 10U));
    lv_label_set_text(label, text);
}

/**
 * @brief Calcula métricas da forma de onda mostrada para um canal.
 *
 * @param[in] points Vetor de pontos visíveis do canal.
 * @param[in] channel_visible Indica se o canal deve aparecer na UI atual.
 * @param[in] rms_label Label de RMS do canal.
 * @param[in] pk_label Label de pico positivo do canal.
 * @param[in] freq_label Label de frequência do canal.
 * @param[in] duty_label Label de duty do canal.
 */
static void lvgl_update_channel_measurements(const int32_t *points,
                                             size_t channel_index,
                                             bool channel_visible,
                                             lv_obj_t *rms_label,
                                             lv_obj_t *pk_label,
                                             lv_obj_t *pk_neg_label,
                                             lv_obj_t *freq_label,
                                             lv_obj_t *duty_label)
{
    double sum_sq = 0.0;
    uint32_t valid_count = 0U;
    const int32_t y_max_mv = lvgl_get_chart_y_max_mv();
    int32_t peak_mv = 0;
    int32_t min_mv = INT32_MAX;

    if (!channel_visible || points == NULL) {
        lvgl_set_metric_voltage_label(rms_label, "RMS", false, 0);
        lvgl_set_metric_voltage_label(pk_label, "Pk+", false, 0);
        lvgl_set_metric_voltage_label(pk_neg_label, "Pk-", false, 0);
        lvgl_set_metric_frequency_label(freq_label, false, 0);
        lvgl_set_metric_duty_label(duty_label, false, 0);
        return;
    }

    for (uint32_t i = 0; i < APP_ADC_CHART_POINTS; i++) {
        const int32_t mv = points[i];
        if (mv == INT32_MAX) {
            continue;
        }
        sum_sq += (double)mv * (double)mv;
        if (mv > peak_mv) {
            peak_mv = mv;
        }
        if (mv < min_mv) {
            min_mv = mv;
        }
        valid_count++;
    }

    if (valid_count == 0U) {
        lvgl_set_metric_voltage_label(rms_label, "RMS", false, 0);
        lvgl_set_metric_voltage_label(pk_label, "Pk+", false, 0);
        lvgl_set_metric_voltage_label(pk_neg_label, "Pk-", false, 0);
        lvgl_set_metric_frequency_label(freq_label, false, 0);
        lvgl_set_metric_duty_label(duty_label, false, 0);
        return;
    }

    lvgl_set_metric_voltage_label(rms_label, "RMS", true, (int32_t)(sqrt(sum_sq / (double)valid_count) + 0.5));
    if (pk_label != NULL) {
        if (peak_mv > y_max_mv) {
            lv_label_set_text(pk_label, "Pk+: Overflow");
        } else {
            lvgl_set_metric_voltage_label(pk_label, "Pk+", true, peak_mv);
        }
    }
    lvgl_set_metric_voltage_label(pk_neg_label, "Pk-", true, (min_mv == INT32_MAX) ? 0 : min_mv);

    if (s_trigger_mode != ADC_SCOPE_TRIGGER_FREE &&
        s_scope_snapshot.trigger_found &&
        channel_index < ADC_SCOPE_MAX_CHANNELS &&
        s_scope_snapshot.measurements_valid[channel_index]) {
        lvgl_set_metric_frequency_label(freq_label, true, s_scope_snapshot.frequency_tenths_hz[channel_index]);
        lvgl_set_metric_duty_label(duty_label, true, s_scope_snapshot.duty_tenths_percent[channel_index]);
        return;
    }

    lvgl_set_metric_frequency_label(freq_label, false, 0);
    lvgl_set_metric_duty_label(duty_label, false, 0);
}

/**
 * @brief Atualiza os indicadores inferiores com métricas das janelas exibidas.
 */
static void lvgl_update_measurements(void)
{
    if (s_sample_channel_mode == 2U) {
        lvgl_update_channel_measurements(s_chart_points,
                                         0U,
                                         true,
                                         s_rms_label,
                                         s_pk_label,
                                         s_pk_neg_label,
                                         s_freq_label,
                                         s_duty_label);

        lvgl_update_channel_measurements(s_chart_points_ch2,
                                         1U,
                                         true,
                                         s_rms_label_ch2,
                                         s_pk_label_ch2,
                                         s_pk_neg_label_ch2,
                                         s_freq_label_ch2,
                                         s_duty_label_ch2);
        return;
    }

    if (s_sample_channel_mode == 0U) {
        lvgl_update_channel_measurements(s_chart_points,
                                         0U,
                                         true,
                                         s_rms_label,
                                         s_pk_label,
                                         s_pk_neg_label,
                                         s_freq_label,
                                         s_duty_label);
    } else {
        lvgl_update_channel_measurements(s_chart_points_ch2,
                                         1U,
                                         true,
                                         s_rms_label,
                                         s_pk_label,
                                         s_pk_neg_label,
                                         s_freq_label,
                                         s_duty_label);
    }

    lv_label_set_text(s_rms_label_ch2, "");
    lv_label_set_text(s_pk_label_ch2, "");
    lv_label_set_text(s_pk_neg_label_ch2, "");
    lv_label_set_text(s_freq_label_ch2, "");
    lv_label_set_text(s_duty_label_ch2, "");
}

/**
 * @brief Atualiza o chart com a janela correspondente à base de tempo escolhida.
 *
 * @param[in] timer Timer do LVGL que disparou a atualização.
 */
static void lvgl_scope_refresh_timer_cb(lv_timer_t *timer)
{
    const size_t requested_samples = lvgl_get_requested_window_samples();
    const adc_scope_trigger_mode_t active_trigger_mode =
        (s_scope_paused || !lvgl_timebase_allows_trigger()) ? ADC_SCOPE_TRIGGER_FREE : s_trigger_mode;
    const adc_scope_trigger_run_mode_t active_trigger_run_mode = s_scope_paused ? ADC_SCOPE_TRIGGER_RUN_AUTO : s_trigger_run_mode;
    const bool use_free_run_sweep =
        (!s_scope_paused &&
         active_trigger_mode == ADC_SCOPE_TRIGGER_FREE &&
         s_history_offset_samples == 0U);
    int32_t *dest_buffers[ADC_SCOPE_MAX_CHANNELS] = {
        s_chart_points_pending,
        s_chart_points_pending_ch2,
    };
    adc_scope_snapshot_t next_snapshot = {0};
    (void)timer;

    if (scope_web_get_output_mode() == SCOPE_OUTPUT_MODE_WEB_ONLY) {
        lvgl_set_center_notice("So web ativo\nDisplay pausado");
        s_center_notice_pending_hide = false;
        s_web_only_notice_active = true;
        return;
    }

    if (s_web_only_notice_active) {
        lvgl_set_center_notice(NULL);
        s_web_only_notice_active = false;
    }

    if (adc_scope_copy_chart_points_multi(dest_buffers,
                                          APP_ADC_CHART_POINTS,
                                          requested_samples,
                                          active_trigger_mode,
                                          active_trigger_run_mode,
                                          s_trigger_channel_index,
                                          LVGL_SCOPE_TRIGGER_POS,
                                          s_history_offset_samples,
                                          s_trigger_level_mv,
                                          0U,
                                          &next_snapshot) != ESP_OK) {
        return;
    }

    if (active_trigger_mode != ADC_SCOPE_TRIGGER_FREE &&
        active_trigger_run_mode != ADC_SCOPE_TRIGGER_RUN_AUTO &&
        !next_snapshot.trigger_found) {
        lvgl_reset_free_run_sweep();
        s_scope_snapshot = next_snapshot;
        lvgl_update_grid_scale_labels();
        lvgl_update_buffer_label();
        lvgl_update_trigger_level_visuals();
        lvgl_update_cursor_visuals();
        return;
    }

    if (use_free_run_sweep) {
        lvgl_publish_pending_points_free_run(requested_samples);
    } else {
        lvgl_reset_free_run_sweep();
        lvgl_publish_pending_points_full();
    }

    s_scope_snapshot = next_snapshot;

    lv_chart_set_axis_range(s_scope_chart, LV_CHART_AXIS_PRIMARY_Y, 0, lvgl_get_chart_y_max_mv());
    lv_chart_refresh(s_scope_chart);
    lvgl_update_center_grid_lines();
    lvgl_update_grid_scale_labels();
    lvgl_update_buffer_label();
    lvgl_update_trigger_level_visuals();
    lvgl_update_cursor_visuals();
    lvgl_update_measurements();

    if (s_center_notice_pending_hide && esp_timer_get_time() >= s_center_notice_hold_until_us) {
        lvgl_set_center_notice(NULL);
        s_center_notice_pending_hide = false;
    }

    if (!s_scope_paused &&
        active_trigger_mode != ADC_SCOPE_TRIGGER_FREE &&
        active_trigger_run_mode == ADC_SCOPE_TRIGGER_RUN_SINGLE &&
        s_scope_snapshot.trigger_found) {
        if (adc_scope_stop() == ESP_OK) {
            s_scope_paused = true;
            s_history_offset_samples = 0U;
            if (s_status_dropdown != NULL) {
                lv_dropdown_set_selected(s_status_dropdown, 1U);
            }
        }
    }
}

/**
 * @brief Callback do dropdown de seleção de canal exibido.
 *
 * @param[in] e Evento gerado pelo LVGL.
 */
static void lvgl_channel_dropdown_event_cb(lv_event_t *e)
{
    if (s_suppress_dropdown_events) {
        return;
    }
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) {
        return;
    }

    s_sample_channel_mode = (uint16_t)lv_dropdown_get_selected(lv_event_get_target_obj(e));
    if (s_sample_channel_mode > 2U) {
        s_sample_channel_mode = 2U;
    }

    lvgl_update_scope_layout();
    lvgl_publish_control_state();
    lvgl_scope_refresh_timer_cb(NULL);
}

/**
 * @brief Reinicia o efeito visual de varredura contínua do modo livre.
 */
static void lvgl_reset_free_run_sweep(void)
{
    s_free_run_sweep_active = false;
    s_free_run_visible_points = 0U;
    s_free_run_last_requested_samples = 0U;
    s_free_run_last_channel_mode = s_sample_channel_mode;
}

/**
 * @brief Copia a janela recém-renderizada integralmente para o chart.
 */
static void lvgl_publish_pending_points_full(void)
{
    for (size_t i = 0; i < APP_ADC_CHART_POINTS; i++) {
        s_chart_points[i] = lvgl_channel_is_visible(0U) ? s_chart_points_pending[i] : INT32_MAX;
        s_chart_points_ch2[i] = lvgl_channel_is_visible(1U) ? s_chart_points_pending_ch2[i] : INT32_MAX;
    }
}

/**
 * @brief Aplica um efeito visual de varredura contínua no modo livre.
 *
 * @param[in] requested_samples Janela temporal atual em amostras reais.
 */
static void lvgl_publish_pending_points_free_run(size_t requested_samples)
{
    uint32_t sample_freq_hz = APP_ADC_SAMPLE_FREQ_HZ;
    uint64_t new_samples_est = 0U;
    size_t delta_points = 1U;

    if (!s_free_run_sweep_active ||
        s_free_run_last_requested_samples != requested_samples ||
        s_free_run_last_channel_mode != s_sample_channel_mode) {
        lvgl_reset_free_run_sweep();
        s_free_run_sweep_active = true;
        s_free_run_last_requested_samples = requested_samples;
        s_free_run_last_channel_mode = s_sample_channel_mode;
    }

    (void)adc_scope_get_sample_freq_hz(&sample_freq_hz);
    new_samples_est = ((uint64_t)sample_freq_hz * (uint64_t)LVGL_SCOPE_REFRESH_MS) / 1000ULL;
    if (new_samples_est == 0U) {
        new_samples_est = 1U;
    }

    if (requested_samples > 0U) {
        delta_points = (size_t)(((new_samples_est * APP_ADC_CHART_POINTS) + requested_samples - 1U) / requested_samples);
        if (delta_points == 0U) {
            delta_points = 1U;
        }
        if (delta_points > APP_ADC_CHART_POINTS) {
            delta_points = APP_ADC_CHART_POINTS;
        }
    }

    if (s_free_run_visible_points < APP_ADC_CHART_POINTS) {
        size_t visible_points = s_free_run_visible_points + delta_points;
        if (visible_points > APP_ADC_CHART_POINTS) {
            visible_points = APP_ADC_CHART_POINTS;
        }

        for (size_t i = 0; i < APP_ADC_CHART_POINTS; i++) {
            if (i < visible_points) {
                s_chart_points[i] = lvgl_channel_is_visible(0U) ? s_chart_points_pending[i] : INT32_MAX;
                s_chart_points_ch2[i] = lvgl_channel_is_visible(1U) ? s_chart_points_pending_ch2[i] : INT32_MAX;
            } else {
                s_chart_points[i] = INT32_MAX;
                s_chart_points_ch2[i] = INT32_MAX;
            }
        }

        s_free_run_visible_points = visible_points;
        return;
    }

    lvgl_publish_pending_points_full();
}

/**
 * @brief Callback do dropdown de seleção do canal de trigger.
 *
 * @param[in] e Evento gerado pelo LVGL.
 */
static void lvgl_trigger_channel_dropdown_event_cb(lv_event_t *e)
{
    if (s_suppress_dropdown_events) {
        return;
    }
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) {
        return;
    }

    s_trigger_channel_index = (uint16_t)lv_dropdown_get_selected(lv_event_get_target_obj(e));
    if (s_trigger_channel_index > 1U) {
        s_trigger_channel_index = 0U;
    }

    lvgl_publish_control_state();
    lvgl_scope_refresh_timer_cb(NULL);
}

/**
 * @brief Callback do dropdown do modo de cursor.
 *
 * @param[in] e Evento gerado pelo LVGL.
 */
static void lvgl_cursor_dropdown_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) {
        return;
    }

    s_cursor_mode = (lvgl_cursor_mode_t)lv_dropdown_get_selected(lv_event_get_target_obj(e));
    if (s_cursor_mode > LVGL_CURSOR_MODE_VOLTAGE) {
        s_cursor_mode = LVGL_CURSOR_MODE_OFF;
    }

    lvgl_update_cursor_visuals();
    lvgl_scope_refresh_timer_cb(NULL);
}

/**
 * @brief Callback do dropdown da linha de cursor selecionada.
 *
 * @param[in] e Evento gerado pelo LVGL.
 */
static void lvgl_cursor_line_dropdown_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) {
        return;
    }

    s_cursor_selected_line = (uint16_t)lv_dropdown_get_selected(lv_event_get_target_obj(e));
    if (s_cursor_selected_line > 1U) {
        s_cursor_selected_line = 0U;
    }

    lvgl_update_cursor_visuals();
}

/**
 * @brief Callback do dropdown de autoajuste.
 *
 * @param[in] e Evento gerado pelo LVGL.
 */
static void lvgl_auto_dropdown_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) {
        return;
    }

    if (lv_dropdown_get_selected(lv_event_get_target_obj(e)) != 1U) {
        return;
    }

    lvgl_apply_auto_settings();
    if (s_auto_dropdown != NULL) {
        lv_dropdown_set_selected(s_auto_dropdown, 0U);
    }
}

/**
 * @brief Callback do dropdown de base de tempo.
 *
 * @param[in] e Evento gerado pelo LVGL.
 */
static void lvgl_timebase_dropdown_event_cb(lv_event_t *e)
{
    if (s_suppress_dropdown_events) {
        return;
    }
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) {
        return;
    }

    s_timebase_index = (uint16_t)lv_dropdown_get_selected(lv_event_get_target_obj(e));
    if (s_timebase_index >= (uint16_t)(sizeof(s_timebase_options) / sizeof(s_timebase_options[0]))) {
        s_timebase_index = 0U;
    }

    if (!lvgl_timebase_allows_trigger()) {
        s_trigger_mode = ADC_SCOPE_TRIGGER_FREE;
        s_trigger_visual_hold_until_us = 0;
        if (s_trigger_dropdown != NULL) {
            lv_dropdown_set_selected(s_trigger_dropdown, (uint16_t)ADC_SCOPE_TRIGGER_FREE);
        }
    }

    lvgl_publish_control_state();
    lvgl_scope_refresh_timer_cb(NULL);
}

/**
 * @brief Callback do dropdown de escala vertical.
 *
 * @param[in] e Evento gerado pelo LVGL.
 */
static void lvgl_volts_dropdown_event_cb(lv_event_t *e)
{
    if (s_suppress_dropdown_events) {
        return;
    }
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) {
        return;
    }

    s_voltscale_index = (uint16_t)lv_dropdown_get_selected(lv_event_get_target_obj(e));
    if (s_voltscale_index >= (uint16_t)(sizeof(s_voltscale_options) / sizeof(s_voltscale_options[0]))) {
        s_voltscale_index = (uint16_t)(sizeof(s_voltscale_options) / sizeof(s_voltscale_options[0])) - 1U;
    }

    if (s_trigger_level_mv > lvgl_get_chart_y_max_mv()) {
        s_trigger_level_mv = lvgl_get_chart_y_max_mv();
    }
    if (s_cursor_voltage_mv_1 > lvgl_get_chart_y_max_mv()) {
        s_cursor_voltage_mv_1 = lvgl_get_chart_y_max_mv();
    }
    if (s_cursor_voltage_mv_2 > lvgl_get_chart_y_max_mv()) {
        s_cursor_voltage_mv_2 = lvgl_get_chart_y_max_mv();
    }

    lvgl_publish_control_state();
    lvgl_scope_refresh_timer_cb(NULL);
}

/**
 * @brief Callback do dropdown de trigger.
 *
 * @param[in] e Evento gerado pelo LVGL.
 */
static void lvgl_trigger_dropdown_event_cb(lv_event_t *e)
{
    uint16_t selected = 0;

    if (s_suppress_dropdown_events) {
        return;
    }
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) {
        return;
    }

    selected = (uint16_t)lv_dropdown_get_selected(lv_event_get_target_obj(e));
    if (selected > (uint16_t)ADC_SCOPE_TRIGGER_FALL) {
        selected = (uint16_t)ADC_SCOPE_TRIGGER_FREE;
    }

    if (selected != (uint16_t)ADC_SCOPE_TRIGGER_FREE && !lvgl_timebase_allows_trigger()) {
        s_trigger_mode = ADC_SCOPE_TRIGGER_FREE;
        s_trigger_visual_hold_until_us = 0;
        if (s_trigger_dropdown != NULL) {
            lv_dropdown_set_selected(s_trigger_dropdown, (uint16_t)ADC_SCOPE_TRIGGER_FREE);
        }
        lvgl_update_trigger_level_visuals();
        lvgl_set_center_notice("Trigger desabilitado\npara esta base de tempo");
        lvgl_hold_center_notice(1800U);
        lvgl_scope_refresh_timer_cb(NULL);
        return;
    }

    s_trigger_mode = (adc_scope_trigger_mode_t)selected;
    if (s_trigger_mode != ADC_SCOPE_TRIGGER_FREE) {
        lvgl_hold_trigger_visual(LVGL_TRIGGER_VISUAL_HOLD_MS);
    } else {
        s_trigger_visual_hold_until_us = 0;
    }
    lvgl_update_trigger_level_visuals();
    lvgl_publish_control_state();
    lvgl_scope_refresh_timer_cb(NULL);
}

/**
 * @brief Callback do dropdown do modo de execução do trigger.
 *
 * @param[in] e Evento gerado pelo LVGL.
 */
static void lvgl_trigger_run_dropdown_event_cb(lv_event_t *e)
{
    uint16_t selected = 0;

    if (s_suppress_dropdown_events) {
        return;
    }
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) {
        return;
    }

    selected = (uint16_t)lv_dropdown_get_selected(lv_event_get_target_obj(e));
    if (selected > (uint16_t)ADC_SCOPE_TRIGGER_RUN_SINGLE) {
        selected = (uint16_t)ADC_SCOPE_TRIGGER_RUN_AUTO;
    }

    s_trigger_run_mode = (adc_scope_trigger_run_mode_t)selected;
    lvgl_publish_control_state();
    lvgl_scope_refresh_timer_cb(NULL);
}

/**
 * @brief Callback do dropdown de status de captura.
 *
 * @param[in] e Evento gerado pelo LVGL.
 */
static void lvgl_status_dropdown_event_cb(lv_event_t *e)
{
    const uint16_t selected = (uint16_t)lv_dropdown_get_selected(lv_event_get_target_obj(e));

    if (s_suppress_dropdown_events) {
        return;
    }
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) {
        return;
    }

    if (selected == 0U) {
        s_history_offset_samples = 0U;
        if (s_scope_paused) {
            if (adc_scope_start() == ESP_OK) {
                s_scope_paused = false;
            }
        }
    } else {
        if (!s_scope_paused) {
            if (adc_scope_stop() == ESP_OK) {
                s_scope_paused = true;
                s_history_offset_samples = 0U;
            }
        }
    }

    lvgl_publish_control_state();
    lvgl_scope_refresh_timer_cb(NULL);
}

/**
 * @brief Permite navegar lateralmente pelo histórico quando a captura está pausada.
 *
 * @param[in] e Evento do LVGL gerado no chart.
 */
static void lvgl_scope_chart_event_cb(lv_event_t *e)
{
    const lv_event_code_t code = lv_event_get_code(e);
    const size_t requested_samples = lvgl_get_requested_window_samples();
    const size_t width_px = (size_t)lv_obj_get_content_width(s_scope_chart);
    const size_t height_px = (size_t)lv_obj_get_content_height(s_scope_chart);

    if (code == LV_EVENT_PRESSED) {
        lv_indev_t *indev = lv_event_get_indev(e);
        lv_point_t point = {0};
        lv_area_t content_coords = {0};
        int32_t chart_width = 0;
        int32_t chart_height = 0;
        int32_t local_x = 0;
        int32_t local_y = 0;
        if (indev == NULL) {
            return;
        }
        lv_indev_get_point(indev, &point);
        s_drag_start_x = point.x;
        s_drag_start_y = point.y;
        s_drag_start_offset_samples = s_history_offset_samples;
        s_drag_start_trigger_level_mv = s_trigger_level_mv;
        s_cursor_drag_active = false;

        if (s_cursor_mode != LVGL_CURSOR_MODE_OFF) {
            chart_width = lv_obj_get_content_width(s_scope_chart);
            chart_height = lv_obj_get_content_height(s_scope_chart);
            lv_obj_get_content_coords(s_scope_chart, &content_coords);
            local_x = point.x - content_coords.x1;
            local_y = point.y - content_coords.y1;

            if (s_cursor_mode == LVGL_CURSOR_MODE_TIME && chart_width > 0) {
                const uint16_t cursor_pos = (s_cursor_selected_line == 0U) ? s_cursor_time_pos_1 : s_cursor_time_pos_2;
                const int32_t cursor_x = (int32_t)((((uint32_t)cursor_pos) * (uint32_t)(chart_width - 1)) / (APP_ADC_CHART_POINTS - 1U));
                const int32_t delta_x = local_x - cursor_x;
                s_cursor_drag_active = (delta_x >= -LVGL_CURSOR_HIT_SLOP_PX) && (delta_x <= LVGL_CURSOR_HIT_SLOP_PX);
            } else if (s_cursor_mode == LVGL_CURSOR_MODE_VOLTAGE && chart_height > 0) {
                const int32_t cursor_mv = (s_cursor_selected_line == 0U) ? s_cursor_voltage_mv_1 : s_cursor_voltage_mv_2;
                const int32_t y_max_mv = lvgl_get_chart_y_max_mv();
                const int32_t clamped_cursor_mv = (cursor_mv < 0) ? 0 : ((cursor_mv > y_max_mv) ? y_max_mv : cursor_mv);
                const int32_t cursor_y = ((y_max_mv - clamped_cursor_mv) * (chart_height - 1)) / y_max_mv;
                const int32_t delta_y = local_y - cursor_y;
                s_cursor_drag_active = (delta_y >= -LVGL_CURSOR_HIT_SLOP_PX) && (delta_y <= LVGL_CURSOR_HIT_SLOP_PX);
            }
        }
        return;
    }

    if (code == LV_EVENT_RELEASED) {
        lv_indev_t *indev = lv_event_get_indev(e);
        lv_point_t point = {0};
        int32_t delta_x = 0;
        int32_t delta_y = 0;
        int64_t now_us = 0;

        if (indev == NULL) {
            return;
        }

        lv_indev_get_point(indev, &point);
        delta_x = (int32_t)point.x - (int32_t)s_drag_start_x;
        delta_y = (int32_t)point.y - (int32_t)s_drag_start_y;

        if (delta_x < 0) {
            delta_x = -delta_x;
        }
        if (delta_y < 0) {
            delta_y = -delta_y;
        }

        if (delta_x <= LVGL_FULLSCREEN_TAP_SLOP_PX && delta_y <= LVGL_FULLSCREEN_TAP_SLOP_PX) {
            now_us = esp_timer_get_time();
            if ((now_us - s_scope_last_tap_time_us) <= ((int64_t)LVGL_FULLSCREEN_TAP_WINDOW_MS * 1000LL)) {
                s_scope_tap_count++;
            } else {
                s_scope_tap_count = 1U;
            }
            s_scope_last_tap_time_us = now_us;

            if (s_scope_tap_count >= 3U) {
                s_scope_tap_count = 0U;
                s_scope_last_tap_time_us = 0;
                s_scope_fullscreen = !s_scope_fullscreen;
                lvgl_update_scope_layout();
                lvgl_scope_refresh_timer_cb(NULL);
            }
        } else {
            s_scope_tap_count = 0U;
        }
        return;
    }

    if (code == LV_EVENT_PRESSING) {
        lv_indev_t *indev = lv_event_get_indev(e);
        lv_point_t point = {0};
        lv_area_t content_coords = {0};
        size_t max_offset = 0U;
        int32_t delta_x = 0;
        int32_t delta_y = 0;
        size_t delta_samples = 0U;
        int32_t delta_mv = 0;
        int32_t local_x = 0;
        int32_t local_y = 0;

        if (indev == NULL) {
            return;
        }

        lv_indev_get_point(indev, &point);
        lv_obj_get_content_coords(s_scope_chart, &content_coords);
        local_x = point.x - content_coords.x1;
        local_y = point.y - content_coords.y1;
        delta_x = (int32_t)point.x - (int32_t)s_drag_start_x;
        delta_y = (int32_t)point.y - (int32_t)s_drag_start_y;

        if (s_cursor_mode != LVGL_CURSOR_MODE_OFF && s_cursor_drag_active) {
            if (s_cursor_mode == LVGL_CURSOR_MODE_TIME) {
                if (width_px == 0U) {
                    return;
                }
                int32_t pos = (int32_t)((((int64_t)local_x) * (APP_ADC_CHART_POINTS - 1U)) / (int64_t)width_px);
                if (pos < 0) {
                    pos = 0;
                } else if (pos >= (int32_t)APP_ADC_CHART_POINTS) {
                    pos = (int32_t)APP_ADC_CHART_POINTS - 1;
                }

                if (s_cursor_selected_line == 0U) {
                    s_cursor_time_pos_1 = (uint16_t)pos;
                } else {
                    s_cursor_time_pos_2 = (uint16_t)pos;
                }
            } else if (height_px > 0U) {
                const int32_t y_max_mv = lvgl_get_chart_y_max_mv();
                int32_t mv = y_max_mv - (int32_t)(((int64_t)local_y * (int64_t)y_max_mv) / (int64_t)height_px);
                if (mv < 0) {
                    mv = 0;
                } else if (mv > y_max_mv) {
                    mv = y_max_mv;
                }

                if (s_cursor_selected_line == 0U) {
                    s_cursor_voltage_mv_1 = mv;
                } else {
                    s_cursor_voltage_mv_2 = mv;
                }
            }

            lvgl_update_cursor_visuals();
            return;
        }

        if (!s_scope_paused && s_trigger_mode != ADC_SCOPE_TRIGGER_FREE) {
            if (height_px == 0U) {
                return;
            }

            delta_mv = (int32_t)(((int64_t)(-delta_y) * (int64_t)lvgl_get_chart_y_max_mv()) / (int64_t)height_px);
            s_trigger_level_mv = s_drag_start_trigger_level_mv + delta_mv;
            if (s_trigger_level_mv < 0) {
                s_trigger_level_mv = 0;
            } else if (s_trigger_level_mv > lvgl_get_chart_y_max_mv()) {
                s_trigger_level_mv = lvgl_get_chart_y_max_mv();
            }
            lvgl_hold_trigger_visual(LVGL_TRIGGER_VISUAL_HOLD_MS);
            lvgl_scope_refresh_timer_cb(NULL);
            return;
        }

        if (!s_scope_paused || width_px == 0U) {
            return;
        }

        if (s_scope_snapshot.history_count > requested_samples) {
            max_offset = s_scope_snapshot.history_count - requested_samples;
        }

        delta_samples = (size_t)(((uint64_t)(delta_x >= 0 ? delta_x : -delta_x) * (uint64_t)requested_samples) / width_px);

        if (delta_x > 0) {
            s_history_offset_samples = s_drag_start_offset_samples + delta_samples;
            if (s_history_offset_samples > max_offset) {
                s_history_offset_samples = max_offset;
            }
        } else {
            if (delta_samples >= s_drag_start_offset_samples) {
                s_history_offset_samples = 0U;
            } else {
                s_history_offset_samples = s_drag_start_offset_samples - delta_samples;
            }
        }

        lvgl_scope_refresh_timer_cb(NULL);
    }
}

/**
 * @brief Cria a interface gráfica do osciloscópio.
 */
static void lvgl_create_scope_ui(void)
{
    lv_obj_t *screen = lv_screen_active();
    lv_obj_t *channel_group = NULL;
    lv_obj_t *channel_caption = NULL;
    lv_obj_t *trigger_channel_group = NULL;
    lv_obj_t *trigger_channel_caption = NULL;
    lv_obj_t *volts_group = NULL;
    lv_obj_t *volts_caption = NULL;
    lv_obj_t *timebase_group = NULL;
    lv_obj_t *timebase_caption = NULL;
    lv_obj_t *trigger_group = NULL;
    lv_obj_t *trigger_caption = NULL;
    lv_obj_t *trigger_mode_group = NULL;
    lv_obj_t *trigger_mode_caption = NULL;
    lv_obj_t *status_group = NULL;
    lv_obj_t *status_caption = NULL;
    lv_obj_t *cursor_group = NULL;
    lv_obj_t *cursor_caption = NULL;
    lv_obj_t *cursor_line_group = NULL;
    lv_obj_t *cursor_line_caption = NULL;
    lv_obj_t *auto_group = NULL;
    lv_obj_t *auto_caption = NULL;
    lv_obj_t *rms_group = NULL;
    lv_obj_t *pk_group = NULL;
    lv_obj_t *pk_neg_group = NULL;
    lv_obj_t *freq_group = NULL;
    lv_obj_t *duty_group = NULL;
    const int32_t slot_width = (int32_t)(ST7796_H_RES / 4);

    lv_obj_clean(screen);
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

    s_controls_strip = lv_obj_create(screen);
    lv_obj_remove_style_all(s_controls_strip);
    lv_obj_set_size(s_controls_strip, ST7796_H_RES, 68);
    lv_obj_align(s_controls_strip, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_scroll_dir(s_controls_strip, LV_DIR_HOR);
    lv_obj_set_scrollbar_mode(s_controls_strip, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_flag(s_controls_strip, LV_OBJ_FLAG_SCROLLABLE);

    channel_group = lv_obj_create(s_controls_strip);
    lv_obj_remove_style_all(channel_group);
    lv_obj_set_size(channel_group, slot_width, 64);
    lv_obj_set_pos(channel_group, 0, 2);

    channel_caption = lv_label_create(channel_group);
    lv_label_set_text(channel_caption, "Canal");
    lv_obj_set_style_text_color(channel_caption, lv_color_hex(LVGL_COLOR_CHANNEL), LV_PART_MAIN);
    lv_obj_align(channel_caption, LV_ALIGN_TOP_LEFT, 10, 4);

    s_channel_dropdown = lv_dropdown_create(channel_group);
    lv_dropdown_set_options_static(s_channel_dropdown,
                                   "Ch 1\n"
                                   "Ch 2\n"
                                   "Ch 1 e 2");
    lv_dropdown_set_selected(s_channel_dropdown, s_sample_channel_mode);
    lv_obj_set_size(s_channel_dropdown, slot_width - 20, LV_SIZE_CONTENT);
    lv_obj_align_to(s_channel_dropdown, channel_caption, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 4);
    lvgl_style_dropdown(s_channel_dropdown, lv_color_hex(LVGL_COLOR_CHANNEL));
    lv_obj_add_event_cb(s_channel_dropdown, lvgl_channel_dropdown_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    trigger_channel_group = lv_obj_create(s_controls_strip);
    lv_obj_remove_style_all(trigger_channel_group);
    lv_obj_set_size(trigger_channel_group, slot_width, 64);
    lv_obj_set_pos(trigger_channel_group, slot_width, 2);

    trigger_channel_caption = lv_label_create(trigger_channel_group);
    lv_label_set_text(trigger_channel_caption, "Canal Trig");
    lv_obj_set_style_text_color(trigger_channel_caption, lv_color_hex(LVGL_COLOR_TRIGCH), LV_PART_MAIN);
    lv_obj_align(trigger_channel_caption, LV_ALIGN_TOP_LEFT, 10, 4);

    s_trigger_channel_dropdown = lv_dropdown_create(trigger_channel_group);
    lv_dropdown_set_options_static(s_trigger_channel_dropdown,
                                   "Ch 1\n"
                                   "Ch 2");
    lv_dropdown_set_selected(s_trigger_channel_dropdown, s_trigger_channel_index);
    lv_obj_set_size(s_trigger_channel_dropdown, slot_width - 20, LV_SIZE_CONTENT);
    lv_obj_align_to(s_trigger_channel_dropdown, trigger_channel_caption, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 4);
    lvgl_style_dropdown(s_trigger_channel_dropdown, lv_color_hex(LVGL_COLOR_TRIGCH));
    lv_obj_add_event_cb(s_trigger_channel_dropdown, lvgl_trigger_channel_dropdown_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    volts_group = lv_obj_create(s_controls_strip);
    lv_obj_remove_style_all(volts_group);
    lv_obj_set_size(volts_group, slot_width, 64);
    lv_obj_set_pos(volts_group, slot_width * 2, 2);

    volts_caption = lv_label_create(volts_group);
    lv_label_set_text(volts_caption, "Div/Volts");
    lv_obj_set_style_text_color(volts_caption, lv_color_hex(LVGL_COLOR_VOLTS), LV_PART_MAIN);
    lv_obj_align(volts_caption, LV_ALIGN_TOP_LEFT, 10, 4);

    s_volts_dropdown = lv_dropdown_create(volts_group);
    lv_dropdown_set_options_static(s_volts_dropdown,
                                   "100 mV\n"
                                   "200 mV\n"
                                   "500 mV\n"
                                   "1 V\n"
                                   "2 V\n"
                                   "3.3 V");
    lv_dropdown_set_selected(s_volts_dropdown, s_voltscale_index);
    lv_obj_set_size(s_volts_dropdown, slot_width - 20, LV_SIZE_CONTENT);
    lv_obj_align_to(s_volts_dropdown, volts_caption, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 4);
    lvgl_style_dropdown(s_volts_dropdown, lv_color_hex(LVGL_COLOR_VOLTS));
    lv_obj_add_event_cb(s_volts_dropdown, lvgl_volts_dropdown_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    timebase_group = lv_obj_create(s_controls_strip);
    lv_obj_remove_style_all(timebase_group);
    lv_obj_set_size(timebase_group, slot_width, 64);
    lv_obj_set_pos(timebase_group, slot_width * 3, 2);

    timebase_caption = lv_label_create(timebase_group);
    lv_label_set_text(timebase_caption, "Div/Time");
    lv_obj_set_style_text_color(timebase_caption, lv_color_hex(LVGL_COLOR_TIMEBASE), LV_PART_MAIN);
    lv_obj_align(timebase_caption, LV_ALIGN_TOP_LEFT, 10, 4);

    s_timebase_dropdown = lv_dropdown_create(timebase_group);
    lv_dropdown_set_options_static(s_timebase_dropdown,
                                   "5 ms\n"
                                   "10 ms\n"
                                   "15 ms\n"
                                   "20 ms\n"
                                   "25 ms\n"
                                   "50 ms\n"
                                   "100 ms\n"
                                   "250 ms\n"
                                   "500 ms\n"
                                   "1 s");
    lv_dropdown_set_selected(s_timebase_dropdown, s_timebase_index);
    lv_obj_set_size(s_timebase_dropdown, slot_width - 20, LV_SIZE_CONTENT);
    lv_obj_align_to(s_timebase_dropdown, timebase_caption, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 4);
    lvgl_style_dropdown(s_timebase_dropdown, lv_color_hex(LVGL_COLOR_TIMEBASE));
    lv_obj_add_event_cb(s_timebase_dropdown, lvgl_timebase_dropdown_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    trigger_group = lv_obj_create(s_controls_strip);
    lv_obj_remove_style_all(trigger_group);
    lv_obj_set_size(trigger_group, slot_width, 64);
    lv_obj_set_pos(trigger_group, slot_width * 4, 2);

    trigger_caption = lv_label_create(trigger_group);
    lv_label_set_text(trigger_caption, "Trigger");
    lv_obj_set_style_text_color(trigger_caption, lv_color_hex(LVGL_COLOR_TRIGGER), LV_PART_MAIN);
    lv_obj_align(trigger_caption, LV_ALIGN_TOP_LEFT, 10, 4);

    s_trigger_dropdown = lv_dropdown_create(trigger_group);
    lv_dropdown_set_options_static(s_trigger_dropdown,
                                   "Off\n"
                                   "Subida\n"
                                   "Descida");
    lv_dropdown_set_selected(s_trigger_dropdown, (uint16_t)s_trigger_mode);
    lv_obj_set_size(s_trigger_dropdown, slot_width - 20, LV_SIZE_CONTENT);
    lv_obj_align_to(s_trigger_dropdown, trigger_caption, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 4);
    lvgl_style_dropdown(s_trigger_dropdown, lv_color_hex(LVGL_COLOR_TRIGGER));
    lv_obj_add_event_cb(s_trigger_dropdown, lvgl_trigger_dropdown_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    trigger_mode_group = lv_obj_create(s_controls_strip);
    lv_obj_remove_style_all(trigger_mode_group);
    lv_obj_set_size(trigger_mode_group, slot_width, 64);
    lv_obj_set_pos(trigger_mode_group, slot_width * 5, 2);

    trigger_mode_caption = lv_label_create(trigger_mode_group);
    lv_label_set_text(trigger_mode_caption, "Trig Mode");
    lv_obj_set_style_text_color(trigger_mode_caption, lv_color_hex(LVGL_COLOR_TRIGMODE), LV_PART_MAIN);
    lv_obj_align(trigger_mode_caption, LV_ALIGN_TOP_LEFT, 10, 4);

    s_trigger_run_dropdown = lv_dropdown_create(trigger_mode_group);
    lv_dropdown_set_options_static(s_trigger_run_dropdown,
                                   "Auto\n"
                                   "Normal\n"
                                   "Single");
    lv_dropdown_set_selected(s_trigger_run_dropdown, (uint16_t)s_trigger_run_mode);
    lv_obj_set_size(s_trigger_run_dropdown, slot_width - 20, LV_SIZE_CONTENT);
    lv_obj_align_to(s_trigger_run_dropdown, trigger_mode_caption, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 4);
    lvgl_style_dropdown(s_trigger_run_dropdown, lv_color_hex(LVGL_COLOR_TRIGMODE));
    lv_obj_add_event_cb(s_trigger_run_dropdown, lvgl_trigger_run_dropdown_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    status_group = lv_obj_create(s_controls_strip);
    lv_obj_remove_style_all(status_group);
    lv_obj_set_size(status_group, slot_width, 64);
    lv_obj_set_pos(status_group, slot_width * 6, 2);

    status_caption = lv_label_create(status_group);
    lv_label_set_text(status_caption, "Status");
    lv_obj_set_style_text_color(status_caption, lv_color_hex(LVGL_COLOR_STATUS), LV_PART_MAIN);
    lv_obj_align(status_caption, LV_ALIGN_TOP_LEFT, 10, 4);

    s_status_dropdown = lv_dropdown_create(status_group);
    lv_dropdown_set_options_static(s_status_dropdown,
                                   "Capture\n"
                                   "Pause");
    lv_dropdown_set_selected(s_status_dropdown, 0U);
    lv_obj_set_size(s_status_dropdown, slot_width - 20, LV_SIZE_CONTENT);
    lv_obj_align_to(s_status_dropdown, status_caption, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 4);
    lvgl_style_dropdown(s_status_dropdown, lv_color_hex(LVGL_COLOR_STATUS));
    lv_obj_add_event_cb(s_status_dropdown, lvgl_status_dropdown_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    cursor_group = lv_obj_create(s_controls_strip);
    lv_obj_remove_style_all(cursor_group);
    lv_obj_set_size(cursor_group, slot_width, 64);
    lv_obj_set_pos(cursor_group, slot_width * 7, 2);

    cursor_caption = lv_label_create(cursor_group);
    lv_label_set_text(cursor_caption, "Cursor");
    lv_obj_set_style_text_color(cursor_caption, lv_color_hex(LVGL_COLOR_CURSOR), LV_PART_MAIN);
    lv_obj_align(cursor_caption, LV_ALIGN_TOP_LEFT, 10, 4);

    s_cursor_dropdown = lv_dropdown_create(cursor_group);
    lv_dropdown_set_options_static(s_cursor_dropdown,
                                   "Off\n"
                                   "Tempo\n"
                                   "Tensao");
    lv_dropdown_set_selected(s_cursor_dropdown, (uint16_t)s_cursor_mode);
    lv_obj_set_size(s_cursor_dropdown, slot_width - 20, LV_SIZE_CONTENT);
    lv_obj_align_to(s_cursor_dropdown, cursor_caption, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 4);
    lvgl_style_dropdown(s_cursor_dropdown, lv_color_hex(LVGL_COLOR_CURSOR));
    lv_obj_add_event_cb(s_cursor_dropdown, lvgl_cursor_dropdown_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    cursor_line_group = lv_obj_create(s_controls_strip);
    lv_obj_remove_style_all(cursor_line_group);
    lv_obj_set_size(cursor_line_group, slot_width, 64);
    lv_obj_set_pos(cursor_line_group, slot_width * 8, 2);

    cursor_line_caption = lv_label_create(cursor_line_group);
    lv_label_set_text(cursor_line_caption, "Linha");
    lv_obj_set_style_text_color(cursor_line_caption, lv_color_hex(LVGL_COLOR_CURSORLINE), LV_PART_MAIN);
    lv_obj_align(cursor_line_caption, LV_ALIGN_TOP_LEFT, 10, 4);

    s_cursor_line_dropdown = lv_dropdown_create(cursor_line_group);
    lv_dropdown_set_options_static(s_cursor_line_dropdown,
                                   "1\n"
                                   "2");
    lv_dropdown_set_selected(s_cursor_line_dropdown, s_cursor_selected_line);
    lv_obj_set_size(s_cursor_line_dropdown, slot_width - 20, LV_SIZE_CONTENT);
    lv_obj_align_to(s_cursor_line_dropdown, cursor_line_caption, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 4);
    lvgl_style_dropdown(s_cursor_line_dropdown, lv_color_hex(LVGL_COLOR_CURSORLINE));
    lv_obj_add_event_cb(s_cursor_line_dropdown, lvgl_cursor_line_dropdown_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    auto_group = lv_obj_create(s_controls_strip);
    lv_obj_remove_style_all(auto_group);
    lv_obj_set_size(auto_group, slot_width, 64);
    lv_obj_set_pos(auto_group, slot_width * 9, 2);

    auto_caption = lv_label_create(auto_group);
    lv_label_set_text(auto_caption, "Auto Set");
    lv_obj_set_style_text_color(auto_caption, lv_color_hex(LVGL_COLOR_AUTOSET), LV_PART_MAIN);
    lv_obj_align(auto_caption, LV_ALIGN_TOP_LEFT, 10, 4);

    s_auto_dropdown = lv_dropdown_create(auto_group);
    lv_dropdown_set_options_static(s_auto_dropdown,
                                   "Off\n"
                                   "Aplicar");
    lv_dropdown_set_selected(s_auto_dropdown, 0U);
    lv_obj_set_size(s_auto_dropdown, slot_width - 20, LV_SIZE_CONTENT);
    lv_obj_align_to(s_auto_dropdown, auto_caption, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 4);
    lvgl_style_dropdown(s_auto_dropdown, lv_color_hex(LVGL_COLOR_AUTOSET));
    lv_obj_add_event_cb(s_auto_dropdown, lvgl_auto_dropdown_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    s_scope_chart = lv_chart_create(screen);
    lv_obj_set_size(s_scope_chart, ST7796_H_RES - 10, 192);
    lv_obj_align(s_scope_chart, LV_ALIGN_BOTTOM_MID, 0, -52);
    lv_obj_add_flag(s_scope_chart, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(s_scope_chart, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(s_scope_chart, lv_color_hex(0x050505), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_scope_chart, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(s_scope_chart, lv_color_hex(0xF8FAFC), LV_PART_MAIN);
    lv_obj_set_style_border_width(s_scope_chart, 2, LV_PART_MAIN);
    lv_obj_set_style_pad_left(s_scope_chart, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_right(s_scope_chart, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_top(s_scope_chart, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(s_scope_chart, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(s_scope_chart, 6, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(s_scope_chart, 14, LV_PART_MAIN);
    lv_obj_set_style_shadow_color(s_scope_chart, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_shadow_opa(s_scope_chart, LV_OPA_60, LV_PART_MAIN);
    lv_obj_set_style_line_color(s_scope_chart, lv_color_hex(0xF8FAFC), LV_PART_MAIN);
    lv_obj_set_style_line_opa(s_scope_chart, LV_OPA_20, LV_PART_MAIN);
    lv_obj_set_style_line_color(s_scope_chart, lv_color_hex(0x39FF14), LV_PART_ITEMS);
    lv_obj_set_style_line_width(s_scope_chart, 2, LV_PART_ITEMS);
    lv_obj_set_style_size(s_scope_chart, 0, 0, LV_PART_INDICATOR);
    lv_chart_set_type(s_scope_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(s_scope_chart, APP_ADC_CHART_POINTS);
    lv_chart_set_div_line_count(s_scope_chart, LVGL_SCOPE_VOLT_GRID_LINES, LVGL_SCOPE_TIME_GRID_LINES);
    lv_chart_set_axis_range(s_scope_chart, LV_CHART_AXIS_PRIMARY_Y, 0, lvgl_get_chart_y_max_mv());

    s_center_vertical_line = lv_obj_create(s_scope_chart);
    lv_obj_remove_style_all(s_center_vertical_line);
    lv_obj_set_style_bg_color(s_center_vertical_line, lv_color_hex(0xE2E8F0), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_center_vertical_line, LV_OPA_50, LV_PART_MAIN);
    lvgl_make_chart_overlay_passthrough(s_center_vertical_line);

    s_center_horizontal_line = lv_obj_create(s_scope_chart);
    lv_obj_remove_style_all(s_center_horizontal_line);
    lv_obj_set_style_bg_color(s_center_horizontal_line, lv_color_hex(0xE2E8F0), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_center_horizontal_line, LV_OPA_50, LV_PART_MAIN);
    lvgl_make_chart_overlay_passthrough(s_center_horizontal_line);

    s_scope_series = lv_chart_add_series(s_scope_chart, lv_color_hex(LVGL_COLOR_TRACE_CH1), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_set_series_ext_y_array(s_scope_chart, s_scope_series, s_chart_points);
    s_scope_series_ch2 = lv_chart_add_series(s_scope_chart, lv_color_hex(LVGL_COLOR_TRACE_CH2), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_set_series_ext_y_array(s_scope_chart, s_scope_series_ch2, s_chart_points_ch2);
    lv_obj_add_event_cb(s_scope_chart, lvgl_scope_chart_event_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(s_scope_chart, lvgl_scope_chart_event_cb, LV_EVENT_PRESSING, NULL);
    lv_obj_add_event_cb(s_scope_chart, lvgl_scope_chart_event_cb, LV_EVENT_RELEASED, NULL);

    s_trigger_level_line = lv_obj_create(s_scope_chart);
    lv_obj_remove_style_all(s_trigger_level_line);
    lv_obj_set_size(s_trigger_level_line, lv_obj_get_content_width(s_scope_chart) - 16, 2);
    lv_obj_set_style_bg_color(s_trigger_level_line, lv_color_hex(0xFACC15), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_trigger_level_line, LV_OPA_90, LV_PART_MAIN);
    lv_obj_set_x(s_trigger_level_line, 8);
    lv_obj_add_flag(s_trigger_level_line, LV_OBJ_FLAG_HIDDEN);
    lvgl_make_chart_overlay_passthrough(s_trigger_level_line);

    s_trigger_level_label = lv_label_create(s_scope_chart);
    lv_label_set_text(s_trigger_level_label, "Trig: 1.65V");
    lv_obj_set_style_text_color(s_trigger_level_label, lv_color_hex(0xFEF08A), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_trigger_level_label, lv_color_hex(0x111827), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_trigger_level_label, LV_OPA_40, LV_PART_MAIN);
    lv_obj_set_style_pad_left(s_trigger_level_label, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_right(s_trigger_level_label, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_top(s_trigger_level_label, 2, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(s_trigger_level_label, 2, LV_PART_MAIN);
    lv_obj_align(s_trigger_level_label, LV_ALIGN_TOP_LEFT, 8, 8);
    lv_obj_add_flag(s_trigger_level_label, LV_OBJ_FLAG_HIDDEN);
    lvgl_make_chart_overlay_passthrough(s_trigger_level_label);

    s_time_div_label = lv_label_create(s_scope_chart);
    lv_label_set_text(s_time_div_label, "T/div: --");
    lv_obj_set_style_text_color(s_time_div_label, lv_color_hex(0xCBD5E1), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_time_div_label, lv_color_hex(0x111827), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_time_div_label, LV_OPA_40, LV_PART_MAIN);
    lv_obj_set_style_pad_left(s_time_div_label, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_right(s_time_div_label, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_top(s_time_div_label, 2, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(s_time_div_label, 2, LV_PART_MAIN);
    lv_obj_align(s_time_div_label, LV_ALIGN_TOP_LEFT, 8, 8);
    lvgl_make_chart_overlay_passthrough(s_time_div_label);

    s_volt_div_label = lv_label_create(s_scope_chart);
    lv_label_set_text(s_volt_div_label, "V/div: --");
    lv_obj_set_style_text_color(s_volt_div_label, lv_color_hex(0xCBD5E1), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_volt_div_label, lv_color_hex(0x111827), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_volt_div_label, LV_OPA_40, LV_PART_MAIN);
    lv_obj_set_style_pad_left(s_volt_div_label, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_right(s_volt_div_label, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_top(s_volt_div_label, 2, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(s_volt_div_label, 2, LV_PART_MAIN);
    lv_obj_align(s_volt_div_label, LV_ALIGN_TOP_LEFT, 8, 30);
    lvgl_make_chart_overlay_passthrough(s_volt_div_label);

    s_buffer_label = lv_label_create(s_scope_chart);
    lv_label_set_text(s_buffer_label, "Buffer: 0%");
    lv_obj_set_style_text_color(s_buffer_label, lv_color_hex(0xE5E7EB), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_buffer_label, lv_color_hex(0x111827), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_buffer_label, LV_OPA_40, LV_PART_MAIN);
    lv_obj_set_style_pad_left(s_buffer_label, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_right(s_buffer_label, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_top(s_buffer_label, 2, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(s_buffer_label, 2, LV_PART_MAIN);
    lv_obj_align(s_buffer_label, LV_ALIGN_TOP_RIGHT, -8, 8);
    lvgl_make_chart_overlay_passthrough(s_buffer_label);

    s_cursor_delta_label = lv_label_create(s_scope_chart);
    lv_label_set_text(s_cursor_delta_label, "Dt: --");
    lv_obj_set_style_text_color(s_cursor_delta_label, lv_color_hex(0xF9FAFB), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_cursor_delta_label, lv_color_hex(0x111827), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_cursor_delta_label, LV_OPA_40, LV_PART_MAIN);
    lv_obj_set_style_pad_left(s_cursor_delta_label, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_right(s_cursor_delta_label, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_top(s_cursor_delta_label, 2, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(s_cursor_delta_label, 2, LV_PART_MAIN);
    lv_obj_align(s_cursor_delta_label, LV_ALIGN_TOP_RIGHT, -8, 30);
    lv_obj_add_flag(s_cursor_delta_label, LV_OBJ_FLAG_HIDDEN);
    lvgl_make_chart_overlay_passthrough(s_cursor_delta_label);

    s_cursor_value_1_label = lv_label_create(s_scope_chart);
    lv_label_set_text(s_cursor_value_1_label, "T1: --");
    lv_obj_set_style_text_color(s_cursor_value_1_label, lv_color_hex(0xF9FAFB), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_cursor_value_1_label, lv_color_hex(0x111827), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_cursor_value_1_label, LV_OPA_40, LV_PART_MAIN);
    lv_obj_set_style_pad_left(s_cursor_value_1_label, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_right(s_cursor_value_1_label, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_top(s_cursor_value_1_label, 2, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(s_cursor_value_1_label, 2, LV_PART_MAIN);
    lv_obj_align(s_cursor_value_1_label, LV_ALIGN_TOP_RIGHT, -8, 52);
    lv_obj_add_flag(s_cursor_value_1_label, LV_OBJ_FLAG_HIDDEN);
    lvgl_make_chart_overlay_passthrough(s_cursor_value_1_label);

    s_cursor_value_2_label = lv_label_create(s_scope_chart);
    lv_label_set_text(s_cursor_value_2_label, "T2: --");
    lv_obj_set_style_text_color(s_cursor_value_2_label, lv_color_hex(0xF9FAFB), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_cursor_value_2_label, lv_color_hex(0x111827), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_cursor_value_2_label, LV_OPA_40, LV_PART_MAIN);
    lv_obj_set_style_pad_left(s_cursor_value_2_label, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_right(s_cursor_value_2_label, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_top(s_cursor_value_2_label, 2, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(s_cursor_value_2_label, 2, LV_PART_MAIN);
    lv_obj_align(s_cursor_value_2_label, LV_ALIGN_TOP_RIGHT, -8, 74);
    lv_obj_add_flag(s_cursor_value_2_label, LV_OBJ_FLAG_HIDDEN);
    lvgl_make_chart_overlay_passthrough(s_cursor_value_2_label);

    s_center_notice_label = lv_label_create(screen);
    lv_label_set_text(s_center_notice_label, "");
    lv_obj_set_style_text_align(s_center_notice_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_center_notice_label, lv_color_hex(0xF9FAFB), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_center_notice_label, lv_color_hex(0x111827), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_center_notice_label, LV_OPA_80, LV_PART_MAIN);
    lv_obj_set_style_border_color(s_center_notice_label, lv_color_hex(0x334155), LV_PART_MAIN);
    lv_obj_set_style_border_width(s_center_notice_label, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(s_center_notice_label, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_left(s_center_notice_label, 14, LV_PART_MAIN);
    lv_obj_set_style_pad_right(s_center_notice_label, 14, LV_PART_MAIN);
    lv_obj_set_style_pad_top(s_center_notice_label, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(s_center_notice_label, 8, LV_PART_MAIN);
    lv_obj_align(s_center_notice_label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(s_center_notice_label, LV_OBJ_FLAG_HIDDEN);

    s_cursor_line_1 = lv_obj_create(s_scope_chart);
    lv_obj_remove_style_all(s_cursor_line_1);
    lv_obj_set_style_bg_color(s_cursor_line_1, lv_color_hex(0xF472B6), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_cursor_line_1, LV_OPA_90, LV_PART_MAIN);
    lv_obj_add_flag(s_cursor_line_1, LV_OBJ_FLAG_HIDDEN);
    lvgl_make_chart_overlay_passthrough(s_cursor_line_1);

    s_cursor_line_2 = lv_obj_create(s_scope_chart);
    lv_obj_remove_style_all(s_cursor_line_2);
    lv_obj_set_style_bg_color(s_cursor_line_2, lv_color_hex(0xA78BFA), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_cursor_line_2, LV_OPA_90, LV_PART_MAIN);
    lv_obj_add_flag(s_cursor_line_2, LV_OBJ_FLAG_HIDDEN);
    lvgl_make_chart_overlay_passthrough(s_cursor_line_2);

    s_metrics_strip = lv_obj_create(screen);
    lv_obj_remove_style_all(s_metrics_strip);
    lv_obj_set_size(s_metrics_strip, ST7796_H_RES, 50);
    lv_obj_align(s_metrics_strip, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    lv_obj_set_scroll_dir(s_metrics_strip, LV_DIR_HOR);
    lv_obj_set_scrollbar_mode(s_metrics_strip, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_flag(s_metrics_strip, LV_OBJ_FLAG_SCROLLABLE);

    rms_group = lv_obj_create(s_metrics_strip);
    lv_obj_remove_style_all(rms_group);
    lv_obj_set_size(rms_group, slot_width, 50);
    lv_obj_set_pos(rms_group, 0, 0);
    s_rms_label = lv_label_create(rms_group);
    lv_label_set_text(s_rms_label, "RMS: --");
    lvgl_style_metric_label(s_rms_label, lv_color_hex(LVGL_COLOR_TRACE_CH1));
    lv_obj_align(s_rms_label, LV_ALIGN_TOP_LEFT, 0, 1);
    s_rms_label_ch2 = lv_label_create(rms_group);
    lv_label_set_text(s_rms_label_ch2, "RMS: --");
    lvgl_style_metric_label(s_rms_label_ch2, lv_color_hex(LVGL_COLOR_TRACE_CH2));
    lv_obj_align(s_rms_label_ch2, LV_ALIGN_BOTTOM_LEFT, 0, -1);

    pk_group = lv_obj_create(s_metrics_strip);
    lv_obj_remove_style_all(pk_group);
    lv_obj_set_size(pk_group, slot_width, 50);
    lv_obj_set_pos(pk_group, slot_width, 0);
    s_pk_label = lv_label_create(pk_group);
    lv_label_set_text(s_pk_label, "Pk+: --");
    lvgl_style_metric_label(s_pk_label, lv_color_hex(LVGL_COLOR_TRACE_CH1));
    lv_obj_align(s_pk_label, LV_ALIGN_TOP_LEFT, 0, 1);
    s_pk_label_ch2 = lv_label_create(pk_group);
    lv_label_set_text(s_pk_label_ch2, "Pk+: --");
    lvgl_style_metric_label(s_pk_label_ch2, lv_color_hex(LVGL_COLOR_TRACE_CH2));
    lv_obj_align(s_pk_label_ch2, LV_ALIGN_BOTTOM_LEFT, 0, -1);

    pk_neg_group = lv_obj_create(s_metrics_strip);
    lv_obj_remove_style_all(pk_neg_group);
    lv_obj_set_size(pk_neg_group, slot_width, 50);
    lv_obj_set_pos(pk_neg_group, slot_width * 2, 0);
    s_pk_neg_label = lv_label_create(pk_neg_group);
    lv_label_set_text(s_pk_neg_label, "Pk-: --");
    lvgl_style_metric_label(s_pk_neg_label, lv_color_hex(LVGL_COLOR_TRACE_CH1));
    lv_obj_align(s_pk_neg_label, LV_ALIGN_TOP_LEFT, 0, 1);
    s_pk_neg_label_ch2 = lv_label_create(pk_neg_group);
    lv_label_set_text(s_pk_neg_label_ch2, "Pk-: --");
    lvgl_style_metric_label(s_pk_neg_label_ch2, lv_color_hex(LVGL_COLOR_TRACE_CH2));
    lv_obj_align(s_pk_neg_label_ch2, LV_ALIGN_BOTTOM_LEFT, 0, -1);

    freq_group = lv_obj_create(s_metrics_strip);
    lv_obj_remove_style_all(freq_group);
    lv_obj_set_size(freq_group, slot_width, 50);
    lv_obj_set_pos(freq_group, slot_width * 3, 0);
    s_freq_label = lv_label_create(freq_group);
    lv_label_set_text(s_freq_label, "Freq: --");
    lvgl_style_metric_label(s_freq_label, lv_color_hex(LVGL_COLOR_TRACE_CH1));
    lv_obj_align(s_freq_label, LV_ALIGN_TOP_LEFT, 0, 1);
    s_freq_label_ch2 = lv_label_create(freq_group);
    lv_label_set_text(s_freq_label_ch2, "Freq: --");
    lvgl_style_metric_label(s_freq_label_ch2, lv_color_hex(LVGL_COLOR_TRACE_CH2));
    lv_obj_align(s_freq_label_ch2, LV_ALIGN_BOTTOM_LEFT, 0, -1);

    duty_group = lv_obj_create(s_metrics_strip);
    lv_obj_remove_style_all(duty_group);
    lv_obj_set_size(duty_group, slot_width, 50);
    lv_obj_set_pos(duty_group, slot_width * 4, 0);
    s_duty_label = lv_label_create(duty_group);
    lv_label_set_text(s_duty_label, "Duty: --");
    lvgl_style_metric_label(s_duty_label, lv_color_hex(LVGL_COLOR_TRACE_CH1));
    lv_obj_align(s_duty_label, LV_ALIGN_TOP_LEFT, 0, 1);
    s_duty_label_ch2 = lv_label_create(duty_group);
    lv_label_set_text(s_duty_label_ch2, "Duty: --");
    lvgl_style_metric_label(s_duty_label_ch2, lv_color_hex(LVGL_COLOR_TRACE_CH2));
    lv_obj_align(s_duty_label_ch2, LV_ALIGN_BOTTOM_LEFT, 0, -1);

    lvgl_update_scope_layout();
    lvgl_update_center_grid_lines();
    lv_timer_create(lvgl_scope_refresh_timer_cb, LVGL_SCOPE_REFRESH_MS, NULL);
    s_scope_ui_ready = true;
    lvgl_publish_control_state();
    lvgl_scope_refresh_timer_cb(NULL);
}

/**
 * @brief Cria a splash screen inicial do projeto.
 */
static void lvgl_create_splash_ui(void)
{
    lv_obj_t *screen = lv_screen_active();
    lv_obj_t *title = NULL;
    lv_obj_t *subtitle = NULL;
    lv_obj_t *author = NULL;
    lv_obj_t *version = NULL;

    lv_obj_clean(screen);
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

    subtitle = lv_label_create(screen);
    lv_label_set_text(subtitle, "Bem-vindo");
    lv_obj_set_style_text_color(subtitle, lv_color_hex(0xFACC15), LV_PART_MAIN);
    lv_obj_set_style_text_font(subtitle, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_align(subtitle, LV_ALIGN_CENTER, 0, -72);

    title = lv_label_create(screen);
    lv_label_set_text(title, "Mini Osciloscopio 2 Canais");
    lv_obj_set_style_text_color(title, lv_color_hex(0xF9FAFB), LV_PART_MAIN);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_28, LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_CENTER, 0, -20);

    author = lv_label_create(screen);
    lv_label_set_text(author, "Bruno Muniz\nEngenheiro eletronico\nEngenheiro de software");
    lv_label_set_long_mode(author, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(author, 280);
    lv_obj_set_style_text_align(author, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_color(author, lv_color_hex(0x94A3B8), LV_PART_MAIN);
    lv_obj_set_style_text_font(author, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_align(author, LV_ALIGN_CENTER, 0, 58);

    version = lv_label_create(screen);
    lv_label_set_text(version, "Versao 1.0");
    lv_obj_set_style_text_color(version, lv_color_hex(0x64748B), LV_PART_MAIN);
    lv_obj_set_style_text_font(version, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_align(version, LV_ALIGN_CENTER, 0, 118);
}

/**
 * @brief Inicializa o display e o dispositivo de entrada do LVGL.
 */
static void lvgl_init_runtime(void)
{
    const size_t draw_buffer_pixels = (size_t)ST7796_H_RES * (size_t)ST7796_DRAW_BUFFER_LINES;
    const size_t draw_buffer_size = draw_buffer_pixels * sizeof(uint16_t);
    void *buf1 = NULL;
    void *buf2 = NULL;
    lv_display_t *display = NULL;
    lv_indev_t *indev = NULL;

    lv_init();

    buf1 = st7796_alloc_dma_buffer(s_lcd, draw_buffer_size);
    buf2 = st7796_alloc_dma_buffer(s_lcd, draw_buffer_size);
    ESP_ERROR_CHECK(buf1 ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(buf2 ? ESP_OK : ESP_ERR_NO_MEM);

    display = lv_display_create(ST7796_H_RES, ST7796_V_RES);
    ESP_ERROR_CHECK(display ? ESP_OK : ESP_FAIL);
    lv_display_set_color_format(display, LVGL_DISPLAY_COLOR_FORMAT);
    lv_display_set_flush_cb(display, lvgl_flush_cb);
    lv_display_set_buffers(display, buf1, buf2, draw_buffer_size, LV_DISPLAY_RENDER_MODE_PARTIAL);

    indev = lv_indev_create();
    ESP_ERROR_CHECK(indev ? ESP_OK : ESP_FAIL);
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, lvgl_touch_read_cb);

    const esp_timer_create_args_t timer_args = {
        .callback = lvgl_tick_cb,
        .name = "lvgl_tick",
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000ULL));
}

/**
 * @brief Task dedicada ao processamento do LVGL.
 *
 * @param[in] arg Contexto não utilizado.
 */
static void lvgl_task(void *arg)
{
    uint32_t elapsed_ms = 0U;
    (void)arg;

    lvgl_create_splash_ui();
    while (elapsed_ms < LVGL_SPLASH_DURATION_MS) {
        lvgl_process_pending_external_requests();
        uint32_t delay_ms = lv_timer_handler();
        if (delay_ms > LVGL_TASK_PERIOD_MS) {
            delay_ms = LVGL_TASK_PERIOD_MS;
        }
        if ((elapsed_ms + delay_ms) > LVGL_SPLASH_DURATION_MS) {
            delay_ms = LVGL_SPLASH_DURATION_MS - elapsed_ms;
        }
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        elapsed_ms += delay_ms;
    }

    lvgl_create_scope_ui();
    ESP_LOGI(TAG, "LVGL 9 integrado ao ST7796 com visual de osciloscopio.");

    while (true) {
        lvgl_process_pending_external_requests();
        uint32_t delay_ms = lv_timer_handler();
        if (delay_ms > LVGL_TASK_PERIOD_MS) {
            delay_ms = LVGL_TASK_PERIOD_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

/**
 * @brief Inicializa o LVGL, a interface do osciloscópio e a task dedicada.
 */
void lvgl_app_start(st7796_handle_t lcd, ft6336u_handle_t touch)
{
    s_lcd = lcd;
    s_touch = touch;

    ESP_ERROR_CHECK(s_lcd ? ESP_OK : ESP_ERR_INVALID_ARG);
    s_control_state_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(s_control_state_mutex ? ESP_OK : ESP_ERR_NO_MEM);
    lvgl_init_runtime();
    xTaskCreatePinnedToCore(lvgl_task,
                            "lvgl",
                            ST7796_LVGL_TASK_STACK_SIZE,
                            NULL,
                            ST7796_LVGL_TASK_PRIORITY,
                            NULL,
                            ST7796_LVGL_TASK_CORE_ID);
}

esp_err_t lvgl_app_get_control_state(lvgl_app_control_state_t *out_state)
{
    ESP_RETURN_ON_FALSE(out_state != NULL, ESP_ERR_INVALID_ARG, TAG, "estado invalido");
    ESP_RETURN_ON_FALSE(s_control_state_mutex != NULL, ESP_ERR_INVALID_STATE, TAG, "mutex nao inicializado");

    if (xSemaphoreTake(s_control_state_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    *out_state = s_shared_control_state;
    xSemaphoreGive(s_control_state_mutex);
    return ESP_OK;
}

esp_err_t lvgl_app_request_control_state(const lvgl_app_control_state_t *state)
{
    ESP_RETURN_ON_FALSE(state != NULL, ESP_ERR_INVALID_ARG, TAG, "estado invalido");
    ESP_RETURN_ON_FALSE(s_control_state_mutex != NULL, ESP_ERR_INVALID_STATE, TAG, "mutex nao inicializado");

    if (xSemaphoreTake(s_control_state_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    s_pending_control_state = *state;
    s_has_pending_control_state = true;
    xSemaphoreGive(s_control_state_mutex);
    return ESP_OK;
}

esp_err_t lvgl_app_request_auto_set(void)
{
    ESP_RETURN_ON_FALSE(s_control_state_mutex != NULL, ESP_ERR_INVALID_STATE, TAG, "mutex nao inicializado");

    if (xSemaphoreTake(s_control_state_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    s_pending_auto_set = true;
    xSemaphoreGive(s_control_state_mutex);
    return ESP_OK;
}

esp_err_t lvgl_app_request_toggle_focus(void)
{
    ESP_RETURN_ON_FALSE(s_control_state_mutex != NULL, ESP_ERR_INVALID_STATE, TAG, "mutex nao inicializado");

    if (xSemaphoreTake(s_control_state_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    s_pending_toggle_focus = true;
    xSemaphoreGive(s_control_state_mutex);
    return ESP_OK;
}

#include "scope_web.h"

/**
 * @file scope_web.c
 * @brief Servidor web simples para visualizar o osciloscópio no navegador.
 */

#include <inttypes.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "adc_scope.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "lvgl_app.h"
#include "nvs_flash.h"
#include "st7796_port.h"

/** @brief Tag de log do módulo web. */
static const char *TAG = "scope_web";

/** @brief URI da página principal da interface web. */
#define SCOPE_WEB_URI_INDEX "/"

/** @brief URI do endpoint com a janela atual do osciloscópio. */
#define SCOPE_WEB_URI_API "/api/scope"

/** @brief URI do endpoint de leitura e escrita do modo de saída. */
#define SCOPE_WEB_URI_MODE "/api/mode"

/** @brief URI do endpoint dedicado à alternância do modo foco no display. */
#define SCOPE_WEB_URI_FOCUS "/api/focus"

/** @brief URI do websocket usado para streaming da waveform. */
#define SCOPE_WEB_URI_WS "/ws"

/** @brief Quantidade de pontos copiados do módulo ADC. */
#define SCOPE_WEB_SOURCE_POINT_COUNT APP_ADC_CHART_POINTS

/** @brief Quantidade de pontos enviados ao navegador por atualização. */
#define SCOPE_WEB_POINT_COUNT (120U)

/** @brief Quantidade de pontos enviados ao navegador em modo somente web. */
#define SCOPE_WEB_POINT_COUNT_WEB_ONLY APP_ADC_CHART_POINTS

/** @brief Tamanho máximo da query string processada pela API. */
#define SCOPE_WEB_QUERY_LEN (96U)

/** @brief Tamanho do cabeçalho JSON montado na stack do handler HTTP. */
#define SCOPE_WEB_JSON_HEAD_CAPACITY (640U)

/** @brief Período de refresh da página no modo compartilhado. */
#define SCOPE_WEB_REFRESH_MS_BOTH (500U)

/** @brief Período de refresh da página no modo somente web. */
#define SCOPE_WEB_REFRESH_MS_WEB_ONLY (100U)

/** @brief Posição horizontal alvo do trigger na visualização web. */
#define SCOPE_WEB_TRIGGER_POS ((APP_ADC_CHART_POINTS - 1U) / 2U)

/** @brief Capacidade do buffer JSON usado no streaming websocket. */
#define SCOPE_WEB_WS_JSON_CAPACITY (16384U)

/** @brief Opção de base de tempo disponível na interface web. */
typedef struct {
    const char *label;          /**< Texto visível no seletor web. */
    uint32_t total_window_us;   /**< Janela total exibida em microssegundos. */
} scope_web_timebase_t;

/** @brief Opção de escala vertical disponível na interface web. */
typedef struct {
    const char *label;      /**< Texto visível no seletor web. */
    int32_t max_mv;         /**< Tensão máxima mostrada no topo da tela. */
} scope_web_voltscale_t;

/** @brief Formato JSON servido pela API web. */
typedef struct {
    uint32_t requested_samples;                     /**< Janela real solicitada pelo cliente em amostras. */
    uint32_t sample_freq_hz;                        /**< Frequência de amostragem atual. */
    size_t history_count;                           /**< Quantidade de amostras válidas no histórico. */
    size_t channel_count;                           /**< Quantidade de canais disponíveis. */
    adc_scope_snapshot_t snapshot;                  /**< Metadados produzidos pelo módulo ADC. */
    int32_t points_ch1[SCOPE_WEB_SOURCE_POINT_COUNT];      /**< Pontos renderizados do canal 1. */
    int32_t points_ch2[SCOPE_WEB_SOURCE_POINT_COUNT];      /**< Pontos renderizados do canal 2. */
} scope_web_payload_t;

/**
 * @brief Estado local usado pela interface web para compor a janela e overlays.
 */
typedef struct {
    uint16_t sample_channel_mode;                   /**< Canal exibido na web: 0=Ch1, 1=Ch2, 2=Ch1+Ch2. */
    uint16_t timebase_index;                        /**< Índice da base de tempo usada na web. */
    uint16_t voltscale_index;                       /**< Índice da escala vertical usada na web. */
    uint16_t trigger_channel_index;                 /**< Canal usado como referência do trigger na web. */
    adc_scope_trigger_mode_t trigger_mode;          /**< Tipo de trigger usado na web. */
    adc_scope_trigger_run_mode_t trigger_run_mode;  /**< Modo de execução do trigger usado na web. */
    bool paused;                                    /**< Indica se a visualização web está pausada. */
    int32_t trigger_level_mv;                       /**< Nível de trigger usado na web. */
} scope_web_view_state_t;

/** @brief Opções de base de tempo disponíveis na interface web. */
static const scope_web_timebase_t s_scope_web_timebases[] = {
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

/** @brief Opções de escala vertical disponíveis na interface web. */
static const scope_web_voltscale_t s_scope_web_voltscales[] = {
    {.label = "100 mV", .max_mv = 100},
    {.label = "200 mV", .max_mv = 200},
    {.label = "500 mV", .max_mv = 500},
    {.label = "1 V",    .max_mv = 1000},
    {.label = "2 V",    .max_mv = 2000},
    {.label = "3.3 V",  .max_mv = 3300},
};

/** @brief Servidor HTTP criado pelo módulo web. */
static httpd_handle_t s_server = NULL;

/** @brief Netif do SoftAP usado pela interface web. */
static esp_netif_t *s_ap_netif = NULL;

/** @brief Indica se o stack básico de rede já foi inicializado. */
static bool s_network_ready = false;

/** @brief Modo visual atual compartilhado entre a web e o display local. */
static volatile scope_output_mode_t s_output_mode = SCOPE_OUTPUT_MODE_BOTH;

/** @brief Configuração padrão inicial da interface web independente. */
static const scope_web_view_state_t s_scope_web_default_state = {
    .sample_channel_mode = 0U,
    .timebase_index = 1U,
    .voltscale_index = 5U,
    .trigger_channel_index = 0U,
    .trigger_mode = ADC_SCOPE_TRIGGER_FREE,
    .trigger_run_mode = ADC_SCOPE_TRIGGER_RUN_AUTO,
    .paused = false,
    .trigger_level_mv = 1650,
};

/** @brief Estado persistido da interface web usado no streaming e nos handlers. */
static scope_web_view_state_t s_scope_web_runtime_state = {
    .sample_channel_mode = 0U,
    .timebase_index = 1U,
    .voltscale_index = 5U,
    .trigger_channel_index = 0U,
    .trigger_mode = ADC_SCOPE_TRIGGER_FREE,
    .trigger_run_mode = ADC_SCOPE_TRIGGER_RUN_AUTO,
    .paused = false,
    .trigger_level_mv = 1650,
};

/** @brief Indica se a página web está em modo comando. */
static bool s_scope_web_command_mode = false;

/** @brief Mutex que protege o estado persistido da interface web. */
static SemaphoreHandle_t s_scope_web_mutex = NULL;

/** @brief Descriptor do cliente websocket ativo. */
static int s_scope_web_ws_fd = -1;

/** @brief Task que faz o streaming periódico da waveform via websocket. */
static TaskHandle_t s_scope_web_stream_task = NULL;

/** @brief Página HTML principal servida pelo ESP32. */
static const char s_scope_web_html[] =
    "<!doctype html>\n"
    "<html lang=\"pt-BR\">\n"
    "<head>\n"
    "  <meta charset=\"utf-8\">\n"
    "  <meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">\n"
    "  <title>Mini Osciloscopio</title>\n"
    "  <style>\n"
    "    :root { color-scheme: dark; }\n"
    "    body { margin: 0; font-family: Arial, sans-serif; background: #020617; color: #e5e7eb; }\n"
    "    .wrap { max-width: 960px; margin: 0 auto; padding: 16px; }\n"
    "    h1 { margin: 0 0 8px; font-size: 24px; }\n"
    "    p { margin: 0 0 16px; color: #94a3b8; }\n"
    "    .toolbar { display: flex; gap: 12px; flex-wrap: wrap; margin-bottom: 12px; }\n"
    "    .card { background: #0f172a; border: 1px solid #334155; border-radius: 12px; padding: 12px; }\n"
    "    label { display: block; font-size: 12px; color: #cbd5e1; margin-bottom: 4px; }\n"
    "    select { width: 100%; min-width: 140px; background: #020617; color: #f8fafc; border: 1px solid #475569; border-radius: 8px; padding: 8px; }\n"
    "    canvas { width: 100%; height: auto; display: block; background: #000; border: 1px solid #f8fafc; border-radius: 12px; touch-action: pan-y pinch-zoom; }\n"
    "    .metrics { margin-top: 12px; display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 8px; }\n"
    "    .metric { background: #111827; border-radius: 10px; padding: 10px; }\n"
    "    .metric strong { display: block; color: #94a3b8; font-size: 12px; margin-bottom: 4px; }\n"
    "    .ch1 { border-left: 4px solid #39ff14; }\n"
    "    .ch2 { border-left: 4px solid #38bdf8; }\n"
    "  </style>\n"
    "</head>\n"
    "<body>\n"
    "  <div class=\"wrap\">\n"
    "    <h1>Mini Osciloscopio</h1>\n"
    "    <p>Conectado ao SoftAP do ESP32-S3. A leitura abaixo mostra a janela mais recente do ADC.</p>\n"
    "    <div class=\"toolbar\">\n"
    "      <div class=\"card\">\n"
    "        <label for=\"channel\">Canal</label>\n"
    "        <select id=\"channel\">\n"
    "          <option value=\"0\">Ch 1</option>\n"
    "          <option value=\"1\">Ch 2</option>\n"
    "          <option value=\"2\">Ch 1 e 2</option>\n"
    "        </select>\n"
    "      </div>\n"
    "      <div class=\"card\">\n"
    "        <label for=\"outputMode\">Saida</label>\n"
    "        <select id=\"outputMode\">\n"
    "          <option value=\"both\" selected>Ambos</option>\n"
    "          <option value=\"web\">Web</option>\n"
    "          <option value=\"display\">Display</option>\n"
    "        </select>\n"
    "      </div>\n"
    "      <div class=\"card\">\n"
    "        <label for=\"commandMode\">Controle</label>\n"
    "        <select id=\"commandMode\">\n"
    "          <option value=\"0\" selected>Monitor</option>\n"
    "          <option value=\"1\">Comando</option>\n"
    "        </select>\n"
    "      </div>\n"
    "      <div class=\"card\">\n"
    "        <label for=\"timebase\">Janela</label>\n"
    "        <select id=\"timebase\">\n"
    "          <option value=\"0\">5 ms</option>\n"
    "          <option value=\"1\" selected>10 ms</option>\n"
    "          <option value=\"2\">15 ms</option>\n"
    "          <option value=\"3\">20 ms</option>\n"
    "          <option value=\"4\">25 ms</option>\n"
    "          <option value=\"5\">50 ms</option>\n"
    "          <option value=\"6\">100 ms</option>\n"
    "          <option value=\"7\">250 ms</option>\n"
    "          <option value=\"8\">500 ms</option>\n"
    "          <option value=\"9\">1 s</option>\n"
    "        </select>\n"
    "      </div>\n"
    "      <div class=\"card\">\n"
    "        <label for=\"voltscale\">Div/Volts</label>\n"
    "        <select id=\"voltscale\">\n"
    "          <option value=\"0\">100 mV</option>\n"
    "          <option value=\"1\">200 mV</option>\n"
    "          <option value=\"2\">500 mV</option>\n"
    "          <option value=\"3\">1 V</option>\n"
    "          <option value=\"4\">2 V</option>\n"
    "          <option value=\"5\" selected>3.3 V</option>\n"
    "        </select>\n"
    "      </div>\n"
    "      <div class=\"card\">\n"
    "        <label for=\"triggerMode\">Trigger</label>\n"
    "        <select id=\"triggerMode\">\n"
    "          <option value=\"0\" selected>Off</option>\n"
    "          <option value=\"1\">Subida</option>\n"
    "          <option value=\"2\">Descida</option>\n"
    "        </select>\n"
    "      </div>\n"
    "      <div class=\"card\">\n"
    "        <label for=\"triggerRunMode\">Trig Run</label>\n"
    "        <select id=\"triggerRunMode\">\n"
    "          <option value=\"0\" selected>Auto</option>\n"
    "          <option value=\"1\">Normal</option>\n"
    "          <option value=\"2\">Single</option>\n"
    "        </select>\n"
    "      </div>\n"
    "      <div class=\"card\">\n"
    "        <label for=\"triggerChannel\">Canal Trig</label>\n"
    "        <select id=\"triggerChannel\">\n"
    "          <option value=\"0\" selected>Ch 1</option>\n"
    "          <option value=\"1\">Ch 2</option>\n"
    "        </select>\n"
    "      </div>\n"
    "      <div class=\"card\">\n"
    "        <label for=\"statusMode\">Status</label>\n"
    "        <select id=\"statusMode\">\n"
    "          <option value=\"0\" selected>Capture</option>\n"
    "          <option value=\"1\">Pause</option>\n"
    "        </select>\n"
    "      </div>\n"
    "      <div class=\"card\">\n"
    "        <label for=\"cursorMode\">Cursor</label>\n"
    "        <select id=\"cursorMode\">\n"
    "          <option value=\"0\" selected>Off</option>\n"
    "          <option value=\"1\">Tempo</option>\n"
    "          <option value=\"2\">Tensao</option>\n"
    "        </select>\n"
    "      </div>\n"
    "      <div class=\"card\">\n"
    "        <label for=\"cursorLine\">Linha</label>\n"
    "        <select id=\"cursorLine\">\n"
    "          <option value=\"0\" selected>1</option>\n"
    "          <option value=\"1\">2</option>\n"
    "        </select>\n"
    "      </div>\n"
    "      <div class=\"card\">\n"
    "        <label for=\"autoSetBtn\">Auto Set</label>\n"
    "        <button id=\"autoSetBtn\" style=\"width:100%;background:#22c55e;color:#020617;border:0;border-radius:8px;padding:10px;font-weight:bold;\">Aplicar</button>\n"
    "      </div>\n"
    "      <div class=\"card\">\n"
    "        <label for=\"focusBtn\">Foco Display</label>\n"
    "        <button id=\"focusBtn\" style=\"width:100%;background:#e2e8f0;color:#020617;border:0;border-radius:8px;padding:10px;font-weight:bold;\">Alternar</button>\n"
    "      </div>\n"
    "    </div>\n"
    "    <canvas id=\"scope\" width=\"920\" height=\"360\"></canvas>\n"
    "    <div class=\"metrics\">\n"
    "      <div class=\"metric ch1\"><strong>Canal 1</strong><span id=\"meta-ch1\">RMS: -- | Pk+: -- | Pk-: --</span></div>\n"
    "      <div class=\"metric ch2\"><strong>Canal 2</strong><span id=\"meta-ch2\">RMS: -- | Pk+: -- | Pk-: --</span></div>\n"
    "    </div>\n"
    "    <p id=\"modeNote\" style=\"margin-top:10px;font-size:12px;color:#94a3b8;\">Modo monitor: a web usa configuracoes proprias e nao segue o display.</p>\n"
    "  </div>\n"
    "  <script>\n"
    "    const canvas = document.getElementById('scope');\n"
    "    const ctx = canvas.getContext('2d');\n"
    "    const channelSel = document.getElementById('channel');\n"
    "    const outputModeSel = document.getElementById('outputMode');\n"
    "    const commandModeSel = document.getElementById('commandMode');\n"
    "    const timebaseSel = document.getElementById('timebase');\n"
    "    const voltscaleSel = document.getElementById('voltscale');\n"
    "    const triggerModeSel = document.getElementById('triggerMode');\n"
    "    const triggerRunModeSel = document.getElementById('triggerRunMode');\n"
    "    const triggerChannelSel = document.getElementById('triggerChannel');\n"
    "    const statusModeSel = document.getElementById('statusMode');\n"
    "    const cursorModeSel = document.getElementById('cursorMode');\n"
    "    const cursorLineSel = document.getElementById('cursorLine');\n"
    "    const autoSetBtn = document.getElementById('autoSetBtn');\n"
    "    const focusBtn = document.getElementById('focusBtn');\n"
    "    const metaCh1 = document.getElementById('meta-ch1');\n"
    "    const metaCh2 = document.getElementById('meta-ch2');\n"
    "    const modeNote = document.getElementById('modeNote');\n"
    "    const TIMEBASES_US = [5000,10000,15000,20000,25000,50000,100000,250000,500000,1000000];\n"
    "    const VOLTSCALES_MV = [100,200,500,1000,2000,3300];\n"
    "    const webState = { channel:0, outputMode:'both', commandMode:false, timebase:1, voltscale:5, triggerMode:0, triggerRunMode:0, triggerChannel:0, paused:false, cursorMode:0, cursorLine:0, triggerLevelMv:1650 };\n"
    "    let refreshTimer = null;\n"
    "    let requestInFlight = false;\n"
    "    let pendingRefresh = false;\n"
    "    let lastPayload = null;\n"
    "    let lastPayloadAt = 0;\n"
    "    let dragMode = '';\n"
    "    let ws = null;\n"
    "    let wsReconnectTimer = null;\n"
    "    let cursorTimePos1 = 40;\n"
    "    let cursorTimePos2 = 80;\n"
    "    let cursorVoltageMv1 = 1100;\n"
    "    let cursorVoltageMv2 = 2200;\n"
    "    let triggerFetchTimer = null;\n"
    "    function getWindowUs() { return TIMEBASES_US[webState.timebase] || TIMEBASES_US[1]; }\n"
    "    function getYMaxMv() { return VOLTSCALES_MV[webState.voltscale] || 3300; }\n"
    "    function getActivePointCount() { return lastPayload && lastPayload.points_ch1 && lastPayload.points_ch1.length ? lastPayload.points_ch1.length : (webState.outputMode === 'web' ? 240 : 120); }\n"
    "    function normalizeCursorState() { const count = Math.max(getActivePointCount() - 1, 1); cursorTimePos1 = Math.max(0, Math.min(count, cursorTimePos1)); cursorTimePos2 = Math.max(0, Math.min(count, cursorTimePos2)); cursorVoltageMv1 = Math.max(0, Math.min(getYMaxMv(), cursorVoltageMv1)); cursorVoltageMv2 = Math.max(0, Math.min(getYMaxMv(), cursorVoltageMv2)); }\n"
    "    function updateGesturePolicy() {\n"
    "      const lockCanvas = webState.cursorMode !== 0 || webState.triggerMode !== 0;\n"
    "      canvas.style.touchAction = lockCanvas ? 'none' : 'pan-y pinch-zoom';\n"
    "      document.body.style.overscrollBehaviorY = lockCanvas ? 'none' : 'auto';\n"
    "      document.documentElement.style.overscrollBehaviorY = lockCanvas ? 'none' : 'auto';\n"
    "      document.body.style.overflowY = lockCanvas ? 'hidden' : 'auto';\n"
    "    }\n"
    "    function syncControls() {\n"
    "      channelSel.value = String(webState.channel);\n"
    "      outputModeSel.value = webState.outputMode;\n"
    "      commandModeSel.value = webState.commandMode ? '1' : '0';\n"
    "      timebaseSel.value = String(webState.timebase);\n"
    "      voltscaleSel.value = String(webState.voltscale);\n"
    "      triggerModeSel.value = String(webState.triggerMode);\n"
    "      triggerRunModeSel.value = String(webState.triggerRunMode);\n"
    "      triggerChannelSel.value = String(webState.triggerChannel);\n"
    "      statusModeSel.value = webState.paused ? '1' : '0';\n"
    "      cursorModeSel.value = String(webState.cursorMode);\n"
    "      cursorLineSel.value = String(webState.cursorLine);\n"
    "      modeNote.textContent = webState.commandMode ? 'Modo comando: alteracoes compartilhadas agora tambem atualizam o display LVGL.' : 'Modo monitor: a web usa configuracoes proprias e nao segue o display.';\n"
    "      updateGesturePolicy();\n"
    "    }\n"
    "    function readControls() {\n"
    "      webState.channel = Number(channelSel.value);\n"
    "      webState.outputMode = outputModeSel.value;\n"
    "      webState.commandMode = commandModeSel.value === '1';\n"
    "      webState.timebase = Number(timebaseSel.value);\n"
    "      webState.voltscale = Number(voltscaleSel.value);\n"
    "      webState.triggerMode = Number(triggerModeSel.value);\n"
    "      webState.triggerRunMode = Number(triggerRunModeSel.value);\n"
    "      webState.triggerChannel = Number(triggerChannelSel.value);\n"
    "      webState.paused = statusModeSel.value === '1';\n"
    "      webState.cursorMode = Number(cursorModeSel.value);\n"
    "      webState.cursorLine = Number(cursorLineSel.value);\n"
    "      webState.triggerLevelMv = Math.max(0, Math.min(getYMaxMv(), webState.triggerLevelMv));\n"
    "    }\n"
    "    function calcRms(points) {\n"
    "      if (!points || !points.length) return 0;\n"
    "      let sum = 0;\n"
    "      for (const mv of points) sum += mv * mv;\n"
    "      return Math.sqrt(sum / points.length);\n"
    "    }\n"
    "    function calcMax(points) {\n"
    "      return points && points.length ? Math.max(...points) : 0;\n"
    "    }\n"
    "    function calcMin(points) {\n"
    "      return points && points.length ? Math.min(...points) : 0;\n"
    "    }\n"
    "    function drawGrid() {\n"
    "      ctx.fillStyle = '#000000';\n"
    "      ctx.fillRect(0, 0, canvas.width, canvas.height);\n"
    "      ctx.strokeStyle = 'rgba(255,255,255,0.14)';\n"
    "      ctx.lineWidth = 1;\n"
    "      for (let i = 0; i <= 10; i++) {\n"
    "        const x = (canvas.width * i) / 10;\n"
    "        ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, canvas.height); ctx.stroke();\n"
    "      }\n"
    "      for (let i = 0; i <= 4; i++) {\n"
    "        const y = (canvas.height * i) / 4;\n"
    "        ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(canvas.width, y); ctx.stroke();\n"
    "      }\n"
    "      ctx.strokeStyle = 'rgba(255,255,255,0.42)';\n"
    "      ctx.beginPath(); ctx.moveTo(canvas.width / 2, 0); ctx.lineTo(canvas.width / 2, canvas.height); ctx.stroke();\n"
    "      ctx.beginPath(); ctx.moveTo(0, canvas.height / 2); ctx.lineTo(canvas.width, canvas.height / 2); ctx.stroke();\n"
    "    }\n"
    "    function drawTrace(points, color, yMaxMv) {\n"
    "      if (!points || !points.length) return;\n"
    "      ctx.strokeStyle = color;\n"
    "      ctx.lineWidth = 2;\n"
    "      ctx.beginPath();\n"
    "      let started = false;\n"
    "      points.forEach((mv, i) => {\n"
    "        const x = (i * (canvas.width - 1)) / Math.max(points.length - 1, 1);\n"
    "        const clamped = Math.max(0, Math.min(yMaxMv, mv));\n"
    "        const y = canvas.height - ((clamped / yMaxMv) * (canvas.height - 1));\n"
    "        if (!started) { ctx.moveTo(x, y); started = true; } else { ctx.lineTo(x, y); }\n"
    "      });\n"
    "      ctx.stroke();\n"
    "    }\n"
    "    function fmtMv(mv) { return (mv / 1000).toFixed(3) + ' V'; }\n"
    "    function fmtUs(us) { if (us >= 1000000) return (us / 1000000).toFixed(3) + ' s'; if (us >= 1000) return (us / 1000).toFixed(3) + ' ms'; return Math.round(us) + ' us'; }\n"
    "    function drawTriggerOverlay() { if (webState.triggerMode === 0) return; const yMax = getYMaxMv(); const y = canvas.height - ((Math.max(0, Math.min(yMax, webState.triggerLevelMv)) / yMax) * (canvas.height - 1)); ctx.strokeStyle = '#facc15'; ctx.lineWidth = 2; ctx.setLineDash([8,6]); ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(canvas.width, y); ctx.stroke(); ctx.setLineDash([]); ctx.fillStyle = '#facc15'; ctx.fillText(`Trig ${(webState.triggerLevelMv / 1000).toFixed(3)} V`, 12, Math.max(50, y - 8)); }\n"
    "    function drawCursorOverlay() { if (webState.cursorMode === 0) return; normalizeCursorState(); const yMax = getYMaxMv(); ctx.fillStyle = '#f8fafc'; if (webState.cursorMode === 1) { const count = Math.max(getActivePointCount() - 1, 1); const x1 = (cursorTimePos1 * (canvas.width - 1)) / count; const x2 = (cursorTimePos2 * (canvas.width - 1)) / count; ctx.strokeStyle = '#f472b6'; ctx.lineWidth = webState.cursorLine === 0 ? 3 : 2; ctx.beginPath(); ctx.moveTo(x1,0); ctx.lineTo(x1,canvas.height); ctx.stroke(); ctx.strokeStyle = '#a78bfa'; ctx.lineWidth = webState.cursorLine === 1 ? 3 : 2; ctx.beginPath(); ctx.moveTo(x2,0); ctx.lineTo(x2,canvas.height); ctx.stroke(); const t1 = (cursorTimePos1 / count) * getWindowUs(); const t2 = (cursorTimePos2 / count) * getWindowUs(); ctx.fillText(`T1 ${fmtUs(t1)}`, canvas.width - 210, 22); ctx.fillText(`T2 ${fmtUs(t2)}`, canvas.width - 210, 40); ctx.fillText(`Dt ${fmtUs(Math.abs(t2 - t1))}`, canvas.width - 210, 58); } else { const y1 = canvas.height - ((Math.max(0, Math.min(yMax, cursorVoltageMv1)) / yMax) * (canvas.height - 1)); const y2 = canvas.height - ((Math.max(0, Math.min(yMax, cursorVoltageMv2)) / yMax) * (canvas.height - 1)); ctx.strokeStyle = '#f472b6'; ctx.lineWidth = webState.cursorLine === 0 ? 3 : 2; ctx.beginPath(); ctx.moveTo(0,y1); ctx.lineTo(canvas.width,y1); ctx.stroke(); ctx.strokeStyle = '#a78bfa'; ctx.lineWidth = webState.cursorLine === 1 ? 3 : 2; ctx.beginPath(); ctx.moveTo(0,y2); ctx.lineTo(canvas.width,y2); ctx.stroke(); ctx.fillText(`V1 ${fmtMv(cursorVoltageMv1)}`, canvas.width - 210, 22); ctx.fillText(`V2 ${fmtMv(cursorVoltageMv2)}`, canvas.width - 210, 40); ctx.fillText(`DV ${fmtMv(Math.abs(cursorVoltageMv2 - cursorVoltageMv1))}`, canvas.width - 210, 58); } }\n"
    "    function renderScope(data = lastPayload) { drawGrid(); if (data) { const mode = webState.channel; if (mode === 0 || mode === 2) drawTrace(data.points_ch1, '#39ff14', getYMaxMv()); if (mode === 1 || mode === 2) drawTrace(data.points_ch2, '#38bdf8', getYMaxMv()); metaCh1.textContent = `RMS: ${fmtMv(calcRms(data.points_ch1))} | Pk+: ${fmtMv(calcMax(data.points_ch1))} | Pk-: ${fmtMv(calcMin(data.points_ch1))}`; metaCh2.textContent = `RMS: ${fmtMv(calcRms(data.points_ch2))} | Pk+: ${fmtMv(calcMax(data.points_ch2))} | Pk-: ${fmtMv(calcMin(data.points_ch2))}`; } drawTriggerOverlay(); drawCursorOverlay(); }\n"
    "    function buildScopeQuery() { const params = new URLSearchParams(); params.set('channel', String(webState.channel)); params.set('timebase', String(webState.timebase)); params.set('volts', String(webState.voltscale)); params.set('trigger', String(webState.triggerMode)); params.set('status', webState.paused ? '1' : '0'); params.set('triglvl', String(Math.round(webState.triggerLevelMv))); return params.toString(); }\n"
    "    async function postControls(extra = '') { const params = new URLSearchParams(); params.set('command', webState.commandMode ? '1' : '0'); params.set('channel', String(webState.channel)); params.set('timebase', String(webState.timebase)); params.set('volts', String(webState.voltscale)); params.set('trigger', String(webState.triggerMode)); params.set('trigrun', String(webState.triggerRunMode)); params.set('trigch', String(webState.triggerChannel)); params.set('status', webState.paused ? '1' : '0'); params.set('triglvl', String(Math.round(webState.triggerLevelMv))); if (extra) params.set(extra, '1'); await fetch(`/api/control?${params.toString()}`, { method: 'POST', cache: 'no-store' }); }\n"
    "    function scheduleWsReconnect() { if (wsReconnectTimer || webState.outputMode === 'display' || webState.paused) return; wsReconnectTimer = setTimeout(() => { wsReconnectTimer = null; connectWs(); }, webState.outputMode === 'web' ? 300 : 700); }\n"
    "    function closeWs() { if (wsReconnectTimer) { clearTimeout(wsReconnectTimer); wsReconnectTimer = null; } if (ws) { const current = ws; ws = null; current.onopen = null; current.onmessage = null; current.onerror = null; current.onclose = null; try { current.close(); } catch (e) {} } }\n"
    "    function connectWs() { if (webState.outputMode === 'display' || webState.paused) { closeWs(); return; } if (ws && (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING)) return; const proto = location.protocol === 'https:' ? 'wss' : 'ws'; ws = new WebSocket(`${proto}://${location.host}/ws`); ws.onopen = () => { lastPayloadAt = 0; postControls().catch(console.error); }; ws.onmessage = ev => { try { lastPayload = JSON.parse(ev.data); lastPayloadAt = Date.now(); renderScope(lastPayload); } catch (err) { console.error(err); } }; ws.onerror = () => { if (ws) try { ws.close(); } catch (e) {} }; ws.onclose = () => { ws = null; scheduleWsReconnect(); }; }\n"
    "    function applyRefreshTimer() { if (refreshTimer) { clearInterval(refreshTimer); refreshTimer = null; } if (webState.outputMode === 'display' || webState.paused) { closeWs(); return; } connectWs(); const period = webState.outputMode === 'web' ? 1000 : 1500; refreshTimer = setInterval(() => { const now = Date.now(); const stale = !lastPayloadAt || (now - lastPayloadAt) > (webState.outputMode === 'web' ? 1200 : 2000); if (!ws || ws.readyState !== WebSocket.OPEN || stale) { if (stale) closeWs(); connectWs(); refreshScope().catch(console.error); } }, period); }\n"
    "    async function setOutputMode(mode) { await fetch(`/api/mode?value=${mode}`, { method: 'POST', cache: 'no-store' }); webState.outputMode = mode; await postControls(); applyRefreshTimer(); if (mode !== 'display' && !webState.paused) refreshScope().catch(console.error); else renderScope(); }\n"
    "    async function refreshScope() { if (webState.outputMode === 'display' || webState.paused) { renderScope(); return; } const now = Date.now(); if (ws && ws.readyState === WebSocket.OPEN && lastPayloadAt && (now - lastPayloadAt) < (webState.outputMode === 'web' ? 1200 : 2000)) return; if (requestInFlight) { pendingRefresh = true; return; } requestInFlight = true; pendingRefresh = false; try { await postControls(); const response = await fetch(`/api/scope`, { cache: 'no-store' }); if (response.status === 204) { renderScope(); return; } lastPayload = await response.json(); lastPayloadAt = Date.now(); renderScope(lastPayload); } finally { requestInFlight = false; if (pendingRefresh) refreshScope().catch(console.error); } }\n"
    "    function estimateFreqHz(points) { if (!points || points.length < 4) return 0; let min = Number.POSITIVE_INFINITY, max = Number.NEGATIVE_INFINITY; for (const mv of points) { if (mv < min) min = mv; if (mv > max) max = mv; } if (!(max > min)) return 0; const thr = (min + max) / 2; let first = -1, second = -1; for (let i = 1; i < points.length; i++) { if (points[i - 1] < thr && points[i] >= thr) { if (first < 0) first = i; else { second = i; break; } } } if (first < 0 || second <= first) return 0; const periodUs = ((second - first) * getWindowUs()) / Math.max(points.length - 1, 1); return periodUs > 0 ? (1000000 / periodUs) : 0; }\n"
    "    function applyLocalAutoSet() { const points = webState.channel === 1 ? (lastPayload ? lastPayload.points_ch2 : null) : (lastPayload ? lastPayload.points_ch1 : null); if (!points || !points.length) return; const peak = calcMax(points); for (let i = 0; i < VOLTSCALES_MV.length; i++) { if (peak <= VOLTSCALES_MV[i]) { webState.voltscale = i; break; } webState.voltscale = VOLTSCALES_MV.length - 1; } const freqHz = estimateFreqHz(points); if (freqHz > 0) { const desiredWindowUs = (1000000 / freqHz) * 4; for (let i = 0; i < TIMEBASES_US.length; i++) { if (desiredWindowUs <= TIMEBASES_US[i]) { webState.timebase = i; break; } webState.timebase = TIMEBASES_US.length - 1; } } webState.triggerLevelMv = Math.min(getYMaxMv(), Math.max(0, Math.round((calcMin(points) + calcMax(points)) / 2))); syncControls(); }\n"
    "    function getCanvasPos(evt) { const rect = canvas.getBoundingClientRect(); return { x: ((evt.clientX - rect.left) * canvas.width) / rect.width, y: ((evt.clientY - rect.top) * canvas.height) / rect.height }; }\n"
    "    function scheduleTriggerRefresh() { if (triggerFetchTimer) return; triggerFetchTimer = setTimeout(() => { triggerFetchTimer = null; postControls().then(() => refreshScope()).catch(console.error); }, 90); }\n"
    "    canvas.addEventListener('pointerdown', evt => { const pos = getCanvasPos(evt); const yMax = getYMaxMv(); const pointCount = Math.max(getActivePointCount() - 1, 1); if (webState.cursorMode === 1) { const cursorPos = webState.cursorLine === 0 ? cursorTimePos1 : cursorTimePos2; const cursorX = (cursorPos * (canvas.width - 1)) / pointCount; if (Math.abs(pos.x - cursorX) <= 18) dragMode = 'cursor-time'; } else if (webState.cursorMode === 2) { const cursorMv = webState.cursorLine === 0 ? cursorVoltageMv1 : cursorVoltageMv2; const cursorY = canvas.height - ((cursorMv / yMax) * (canvas.height - 1)); if (Math.abs(pos.y - cursorY) <= 18) dragMode = 'cursor-voltage'; } else if (webState.triggerMode !== 0) { dragMode = 'trigger'; } if (dragMode) { canvas.setPointerCapture(evt.pointerId); evt.preventDefault(); } });\n"
    "    canvas.addEventListener('pointermove', evt => { if (!dragMode) return; evt.preventDefault(); const pos = getCanvasPos(evt); const yMax = getYMaxMv(); const pointCount = Math.max(getActivePointCount() - 1, 1); if (dragMode === 'cursor-time') { const idx = Math.max(0, Math.min(pointCount, Math.round((pos.x * pointCount) / Math.max(canvas.width - 1, 1)))); if (webState.cursorLine === 0) cursorTimePos1 = idx; else cursorTimePos2 = idx; renderScope(); } else if (dragMode === 'cursor-voltage') { const mv = Math.max(0, Math.min(yMax, Math.round(((canvas.height - 1 - pos.y) * yMax) / Math.max(canvas.height - 1, 1)))); if (webState.cursorLine === 0) cursorVoltageMv1 = mv; else cursorVoltageMv2 = mv; renderScope(); } else if (dragMode === 'trigger') { webState.triggerLevelMv = Math.max(0, Math.min(yMax, Math.round(((canvas.height - 1 - pos.y) * yMax) / Math.max(canvas.height - 1, 1)))); renderScope(); scheduleTriggerRefresh(); } });\n"
    "    function endDrag() { if (dragMode === 'trigger') postControls().then(() => refreshScope()).catch(console.error); dragMode = ''; }\n"
    "    canvas.addEventListener('pointerup', endDrag);\n"
    "    canvas.addEventListener('pointercancel', endDrag);\n"
    "    canvas.addEventListener('touchmove', evt => { if (webState.cursorMode !== 0 || webState.triggerMode !== 0) evt.preventDefault(); }, { passive: false });\n"
    "    document.addEventListener('touchmove', evt => { if (dragMode && (webState.cursorMode !== 0 || webState.triggerMode !== 0)) evt.preventDefault(); }, { passive: false });\n"
    "    outputModeSel.addEventListener('change', () => { readControls(); setOutputMode(webState.outputMode).catch(console.error); });\n"
    "    commandModeSel.addEventListener('change', () => { readControls(); syncControls(); postControls().catch(console.error); });\n"
    "    channelSel.addEventListener('change', () => { readControls(); renderScope(); postControls().then(() => refreshScope()).catch(console.error); });\n"
    "    timebaseSel.addEventListener('change', () => { readControls(); applyRefreshTimer(); postControls().then(() => refreshScope()).catch(console.error); });\n"
    "    voltscaleSel.addEventListener('change', () => { readControls(); webState.triggerLevelMv = Math.min(getYMaxMv(), webState.triggerLevelMv); cursorVoltageMv1 = Math.min(getYMaxMv(), cursorVoltageMv1); cursorVoltageMv2 = Math.min(getYMaxMv(), cursorVoltageMv2); renderScope(); postControls().then(() => refreshScope()).catch(console.error); });\n"
    "    triggerModeSel.addEventListener('change', () => { readControls(); renderScope(); postControls().then(() => refreshScope()).catch(console.error); });\n"
    "    triggerRunModeSel.addEventListener('change', () => { readControls(); postControls().then(() => refreshScope()).catch(console.error); });\n"
    "    triggerChannelSel.addEventListener('change', () => { readControls(); postControls().then(() => refreshScope()).catch(console.error); });\n"
    "    statusModeSel.addEventListener('change', () => { readControls(); applyRefreshTimer(); postControls().then(() => refreshScope()).catch(console.error); });\n"
    "    cursorModeSel.addEventListener('change', () => { readControls(); renderScope(); });\n"
    "    cursorLineSel.addEventListener('change', () => { readControls(); renderScope(); });\n"
    "    autoSetBtn.addEventListener('click', () => { applyLocalAutoSet(); applyRefreshTimer(); postControls().then(() => refreshScope()).catch(console.error); });\n"
    "    focusBtn.addEventListener('click', () => { fetch('/api/focus', { method: 'POST', cache: 'no-store' }).catch(console.error); });\n"
    "    syncControls();\n"
    "    drawGrid();\n"
    "    setOutputMode(webState.outputMode).catch(console.error);\n"
    "  </script>\n"
    "</body>\n"
    "</html>\n";

/**
 * @brief Registra eventos de associação ao SoftAP.
 *
 * @param[in] arg Contexto não utilizado.
 * @param[in] event_base Base do evento recebido.
 * @param[in] event_id Identificador do evento.
 * @param[in] event_data Dados do evento.
 */
static void scope_web_wifi_event_handler(void *arg,
                                         esp_event_base_t event_base,
                                         int32_t event_id,
                                         void *event_data)
{
    (void)arg;

    if (event_base != WIFI_EVENT) {
        return;
    }

    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        const wifi_event_ap_staconnected_t *event = (const wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "Cliente conectado ao SoftAP, AID=%d", event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        const wifi_event_ap_stadisconnected_t *event = (const wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "Cliente desconectado do SoftAP, AID=%d", event->aid);
        s_scope_web_ws_fd = -1;
        if (scope_web_get_output_mode() == SCOPE_OUTPUT_MODE_WEB_ONLY) {
            scope_web_set_output_mode(SCOPE_OUTPUT_MODE_BOTH);
        }
    }
}

/**
 * @brief Lê o estado persistido da interface web.
 *
 * @param[out] out_state Estado lido.
 * @param[out] out_command_mode Indica se a web está em modo comando.
 */
static void scope_web_get_runtime_state(scope_web_view_state_t *out_state, bool *out_command_mode)
{
    if (out_state == NULL || s_scope_web_mutex == NULL) {
        return;
    }

    if (xSemaphoreTake(s_scope_web_mutex, portMAX_DELAY) == pdTRUE) {
        *out_state = s_scope_web_runtime_state;
        if (out_command_mode != NULL) {
            *out_command_mode = s_scope_web_command_mode;
        }
        xSemaphoreGive(s_scope_web_mutex);
    }
}

/**
 * @brief Atualiza o estado persistido da interface web.
 *
 * @param[in] state Novo estado.
 * @param[in] command_mode Novo modo comando.
 */
static void scope_web_set_runtime_state(const scope_web_view_state_t *state, bool command_mode)
{
    if (state == NULL || s_scope_web_mutex == NULL) {
        return;
    }

    if (xSemaphoreTake(s_scope_web_mutex, portMAX_DELAY) == pdTRUE) {
        s_scope_web_runtime_state = *state;
        s_scope_web_command_mode = command_mode;
        xSemaphoreGive(s_scope_web_mutex);
    }
}

/**
 * @brief Converte a query string em um estado local da interface web.
 *
 * @param[in] req Requisição HTTP atual.
 * @param[out] out_state Estado local preenchido com defaults e overrides da query.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
static esp_err_t scope_web_get_control_state(lvgl_app_control_state_t *out_state)
{
    ESP_RETURN_ON_FALSE(out_state != NULL, ESP_ERR_INVALID_ARG, TAG, "estado invalido");
    return lvgl_app_get_control_state(out_state);
}

/**
 * @brief Converte a query string em um estado local da interface web.
 *
 * @param[in] req Requisição HTTP atual.
 * @param[out] out_state Estado local preenchido com defaults e overrides da query.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
static esp_err_t scope_web_get_view_state_from_req(httpd_req_t *req, scope_web_view_state_t *out_state)
{
    char query[SCOPE_WEB_QUERY_LEN] = {0};
    char value[24] = {0};

    ESP_RETURN_ON_FALSE(out_state != NULL, ESP_ERR_INVALID_ARG, TAG, "estado web invalido");

    *out_state = s_scope_web_default_state;
    if (req == NULL || httpd_req_get_url_query_len(req) <= 0) {
        return ESP_OK;
    }
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        return ESP_OK;
    }

    if (httpd_query_key_value(query, "channel", value, sizeof(value)) == ESP_OK) {
        out_state->sample_channel_mode = (uint16_t)strtoul(value, NULL, 10);
        if (out_state->sample_channel_mode > 2U) {
            out_state->sample_channel_mode = s_scope_web_default_state.sample_channel_mode;
        }
    }
    if (httpd_query_key_value(query, "timebase", value, sizeof(value)) == ESP_OK) {
        out_state->timebase_index = (uint16_t)strtoul(value, NULL, 10);
        if (out_state->timebase_index >= (uint16_t)(sizeof(s_scope_web_timebases) / sizeof(s_scope_web_timebases[0]))) {
            out_state->timebase_index = s_scope_web_default_state.timebase_index;
        }
    }
    if (httpd_query_key_value(query, "volts", value, sizeof(value)) == ESP_OK) {
        out_state->voltscale_index = (uint16_t)strtoul(value, NULL, 10);
        if (out_state->voltscale_index >= (uint16_t)(sizeof(s_scope_web_voltscales) / sizeof(s_scope_web_voltscales[0]))) {
            out_state->voltscale_index = s_scope_web_default_state.voltscale_index;
        }
    }
    if (httpd_query_key_value(query, "trigger", value, sizeof(value)) == ESP_OK) {
        uint16_t trigger = (uint16_t)strtoul(value, NULL, 10);
        if (trigger > (uint16_t)ADC_SCOPE_TRIGGER_FALL) {
            trigger = (uint16_t)ADC_SCOPE_TRIGGER_FREE;
        }
        out_state->trigger_mode = (adc_scope_trigger_mode_t)trigger;
    }
    if (httpd_query_key_value(query, "status", value, sizeof(value)) == ESP_OK) {
        out_state->paused = (strtoul(value, NULL, 10) != 0UL);
    }
    if (httpd_query_key_value(query, "triglvl", value, sizeof(value)) == ESP_OK) {
        long parsed = strtol(value, NULL, 10);
        if (parsed < 0L) {
            parsed = 0L;
        }
        if (parsed > (long)s_scope_web_voltscales[out_state->voltscale_index].max_mv) {
            parsed = (long)s_scope_web_voltscales[out_state->voltscale_index].max_mv;
        }
        out_state->trigger_level_mv = (int32_t)parsed;
    } else if (out_state->trigger_level_mv > s_scope_web_voltscales[out_state->voltscale_index].max_mv) {
        out_state->trigger_level_mv = s_scope_web_voltscales[out_state->voltscale_index].max_mv;
    }

    return ESP_OK;
}

/**
 * @brief Retorna a quantidade de amostras baseada na base de tempo selecionada na web.
 *
 * @param[in] state Estado local da interface web.
 *
 * @return Quantidade de amostras reais desejada para a janela web.
 */
static uint32_t scope_web_get_requested_samples_from_state(const scope_web_view_state_t *state)
{
    uint32_t sample_freq_hz = APP_ADC_SAMPLE_FREQ_HZ;
    uint64_t requested = 0U;
    uint16_t timebase_index = 1U;

    if (state != NULL) {
        timebase_index = state->timebase_index;
    }
    if (timebase_index >= (uint16_t)(sizeof(s_scope_web_timebases) / sizeof(s_scope_web_timebases[0]))) {
        timebase_index = 0U;
    }

    (void)adc_scope_get_sample_freq_hz(&sample_freq_hz);
    requested = ((uint64_t)sample_freq_hz * (uint64_t)s_scope_web_timebases[timebase_index].total_window_us) / 1000000ULL;
    if (requested == 0U) {
        requested = 1U;
    }
    if (requested > APP_ADC_HISTORY_SAMPLES) {
        requested = APP_ADC_HISTORY_SAMPLES;
    }
    return (uint32_t)requested;
}

/**
 * @brief Retorna a tensão máxima usada no eixo Y da web.
 *
 * @param[in] state Estado compartilhado usado como referência.
 *
 * @return Tensão máxima da escala vertical em milivolts.
 */
static int32_t scope_web_get_y_max_mv(const scope_web_view_state_t *state)
{
    uint16_t voltscale_index = 5U;

    if (state != NULL) {
        voltscale_index = state->voltscale_index;
    }
    if (voltscale_index >= (uint16_t)(sizeof(s_scope_web_voltscales) / sizeof(s_scope_web_voltscales[0]))) {
        voltscale_index = (uint16_t)(sizeof(s_scope_web_voltscales) / sizeof(s_scope_web_voltscales[0])) - 1U;
    }
    return s_scope_web_voltscales[voltscale_index].max_mv;
}

/**
 * @brief Coleta a janela atual do osciloscópio para envio ao navegador.
 *
 * @param[in] state Estado compartilhado atual entre web e display.
 * @param[in] requested_samples Quantidade de amostras reais desejada para a janela.
 * @param[out] out_payload Estrutura preenchida com pontos e metadados.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
static esp_err_t scope_web_collect_payload(const scope_web_view_state_t *state,
                                           uint32_t requested_samples,
                                           scope_web_payload_t *out_payload)
{
    int32_t *dest_buffers[ADC_SCOPE_MAX_CHANNELS] = {0};
    adc_scope_trigger_mode_t trigger_mode = ADC_SCOPE_TRIGGER_FREE;

    ESP_RETURN_ON_FALSE(state != NULL && out_payload != NULL, ESP_ERR_INVALID_ARG, TAG, "payload invalido");

    memset(out_payload, 0, sizeof(*out_payload));
    out_payload->requested_samples = requested_samples;
    dest_buffers[0] = out_payload->points_ch1;
    dest_buffers[1] = out_payload->points_ch2;

    ESP_RETURN_ON_ERROR(adc_scope_get_sample_freq_hz(&out_payload->sample_freq_hz), TAG, "falha ao ler sample rate");
    ESP_RETURN_ON_ERROR(adc_scope_get_channel_count(&out_payload->channel_count), TAG, "falha ao ler canais");
    if (!state->paused && state->trigger_mode <= (uint16_t)ADC_SCOPE_TRIGGER_FALL) {
        trigger_mode = (adc_scope_trigger_mode_t)state->trigger_mode;
    }
    ESP_RETURN_ON_ERROR(adc_scope_copy_chart_points_multi(dest_buffers,
                                                          SCOPE_WEB_SOURCE_POINT_COUNT,
                                                          requested_samples,
                                                          trigger_mode,
                                                          state->trigger_run_mode,
                                                          state->trigger_channel_index,
                                                          SCOPE_WEB_TRIGGER_POS,
                                                          0U,
                                                          state->trigger_level_mv,
                                                          0U,
                                                          &out_payload->snapshot),
                        TAG,
                        "falha ao compor janela web");

    out_payload->history_count = out_payload->snapshot.history_count;
    return ESP_OK;
}

/**
 * @brief Retorna a quantidade de pontos que deve ser enviada à interface web.
 *
 * @return Quantidade de pontos do payload JSON.
 */
static size_t scope_web_get_output_point_count(void)
{
    return (scope_web_get_output_mode() == SCOPE_OUTPUT_MODE_WEB_ONLY) ? SCOPE_WEB_POINT_COUNT_WEB_ONLY : SCOPE_WEB_POINT_COUNT;
}

/**
 * @brief Envia um array de pontos em JSON para a resposta chunked.
 *
 * @param[in] req Requisição HTTP atual.
 * @param[in] name Nome do campo JSON.
 * @param[in] points Vetor de pontos.
 * @param[in] count Quantidade de pontos válidos.
 * @param[in] append_comma Indica se deve anexar vírgula ao final do campo.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
static esp_err_t scope_web_send_point_array(httpd_req_t *req,
                                            const char *name,
                                            const int32_t *points,
                                            size_t count,
                                            bool append_comma)
{
    char number[24];
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(req != NULL && name != NULL && points != NULL, ESP_ERR_INVALID_ARG, TAG, "array invalido");
    ret = httpd_resp_sendstr_chunk(req, "\"");
    if (ret != ESP_OK) {
        return ret;
    }
    ret = httpd_resp_sendstr_chunk(req, name);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = httpd_resp_sendstr_chunk(req, "\":[");
    if (ret != ESP_OK) {
        return ret;
    }

    for (size_t i = 0; i < count; i++) {
        const size_t src_index = (count <= 1U) ? 0U : (i * (SCOPE_WEB_SOURCE_POINT_COUNT - 1U)) / (count - 1U);
        snprintf(number, sizeof(number), "%" PRId32, points[src_index]);
        ret = httpd_resp_sendstr_chunk(req, number);
        if (ret != ESP_OK) {
            return ret;
        }
        if ((i + 1U) < count) {
            ret = httpd_resp_sendstr_chunk(req, ",");
            if (ret != ESP_OK) {
                return ret;
            }
        }
    }

    return httpd_resp_sendstr_chunk(req, append_comma ? "]," : "]");
}

/**
 * @brief Handler da página principal.
 *
 * @param[in] req Requisição HTTP atual.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
static esp_err_t scope_web_index_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, s_scope_web_html, HTTPD_RESP_USE_STRLEN);
}

/**
 * @brief Converte texto da query string para o modo de saída visual.
 *
 * @param[in] text Texto recebido em `value`.
 * @param[out] out_mode Modo convertido.
 *
 * @return `true` se a conversão foi bem-sucedida.
 */
static bool scope_web_parse_output_mode(const char *text, scope_output_mode_t *out_mode)
{
    if (text == NULL || out_mode == NULL) {
        return false;
    }

    if (strcmp(text, "both") == 0) {
        *out_mode = SCOPE_OUTPUT_MODE_BOTH;
        return true;
    }
    if (strcmp(text, "web") == 0) {
        *out_mode = SCOPE_OUTPUT_MODE_WEB_ONLY;
        return true;
    }
    if (strcmp(text, "display") == 0) {
        *out_mode = SCOPE_OUTPUT_MODE_DISPLAY_ONLY;
        return true;
    }

    return false;
}

/**
 * @brief Handler da API de controles da interface web.
 *
 * @param[in] req Requisição HTTP atual.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
static esp_err_t scope_web_control_handler(httpd_req_t *req)
{
    char query[SCOPE_WEB_QUERY_LEN] = {0};
    char value[16] = {0};
    lvgl_app_control_state_t lvgl_state = {0};
    scope_web_view_state_t state = {0};
    bool command_mode = false;

    scope_web_get_runtime_state(&state, &command_mode);

    if (httpd_req_get_url_query_len(req) > 0 &&
        httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        if (httpd_query_key_value(query, "command", value, sizeof(value)) == ESP_OK) {
            command_mode = (strtoul(value, NULL, 10) != 0UL);
        }
        if (httpd_query_key_value(query, "channel", value, sizeof(value)) == ESP_OK) {
            state.sample_channel_mode = (uint16_t)strtoul(value, NULL, 10);
            if (state.sample_channel_mode > 2U) {
                state.sample_channel_mode = 0U;
            }
        }
        if (httpd_query_key_value(query, "timebase", value, sizeof(value)) == ESP_OK) {
            state.timebase_index = (uint16_t)strtoul(value, NULL, 10);
            if (state.timebase_index >= (uint16_t)(sizeof(s_scope_web_timebases) / sizeof(s_scope_web_timebases[0]))) {
                state.timebase_index = 0U;
            }
        }
        if (httpd_query_key_value(query, "volts", value, sizeof(value)) == ESP_OK) {
            state.voltscale_index = (uint16_t)strtoul(value, NULL, 10);
            if (state.voltscale_index >= (uint16_t)(sizeof(s_scope_web_voltscales) / sizeof(s_scope_web_voltscales[0]))) {
                state.voltscale_index = (uint16_t)(sizeof(s_scope_web_voltscales) / sizeof(s_scope_web_voltscales[0])) - 1U;
            }
            if (state.trigger_level_mv > s_scope_web_voltscales[state.voltscale_index].max_mv) {
                state.trigger_level_mv = s_scope_web_voltscales[state.voltscale_index].max_mv;
            }
        }
        if (httpd_query_key_value(query, "trigger", value, sizeof(value)) == ESP_OK) {
            uint16_t trigger = (uint16_t)strtoul(value, NULL, 10);
            if (trigger > (uint16_t)ADC_SCOPE_TRIGGER_FALL) {
                trigger = (uint16_t)ADC_SCOPE_TRIGGER_FREE;
            }
            state.trigger_mode = (adc_scope_trigger_mode_t)trigger;
        }
        if (httpd_query_key_value(query, "trigrun", value, sizeof(value)) == ESP_OK) {
            uint16_t run_mode = (uint16_t)strtoul(value, NULL, 10);
            if (run_mode > (uint16_t)ADC_SCOPE_TRIGGER_RUN_SINGLE) {
                run_mode = (uint16_t)ADC_SCOPE_TRIGGER_RUN_AUTO;
            }
            state.trigger_run_mode = (adc_scope_trigger_run_mode_t)run_mode;
        }
        if (httpd_query_key_value(query, "trigch", value, sizeof(value)) == ESP_OK) {
            uint16_t trigch = (uint16_t)strtoul(value, NULL, 10);
            if (trigch > 1U) {
                trigch = 0U;
            }
            state.trigger_channel_index = trigch;
        }
        if (httpd_query_key_value(query, "status", value, sizeof(value)) == ESP_OK) {
            uint16_t status = (uint16_t)strtoul(value, NULL, 10);
            state.paused = (status != 0U);
        }
        if (httpd_query_key_value(query, "triglvl", value, sizeof(value)) == ESP_OK) {
            long parsed = strtol(value, NULL, 10);
            if (parsed < 0L) {
                parsed = 0L;
            }
            if (parsed > s_scope_web_voltscales[state.voltscale_index].max_mv) {
                parsed = s_scope_web_voltscales[state.voltscale_index].max_mv;
            }
            state.trigger_level_mv = (int32_t)parsed;
        }
        if (httpd_query_key_value(query, "autoset", value, sizeof(value)) == ESP_OK) {
            if (command_mode) {
                ESP_RETURN_ON_ERROR(lvgl_app_request_auto_set(), TAG, "falha ao solicitar auto set");
            }
        }
        if (httpd_query_key_value(query, "focus", value, sizeof(value)) == ESP_OK) {
            ESP_RETURN_ON_ERROR(lvgl_app_request_toggle_focus(), TAG, "falha ao solicitar foco do display");
        }
    }

    scope_web_set_runtime_state(&state, command_mode);
    httpd_resp_set_type(req, "application/json");
    if (command_mode) {
        ESP_RETURN_ON_ERROR(scope_web_get_control_state(&lvgl_state), TAG, "falha ao ler estado compartilhado");
        lvgl_state.sample_channel_mode = state.sample_channel_mode;
        lvgl_state.timebase_index = state.timebase_index;
        lvgl_state.voltscale_index = state.voltscale_index;
        lvgl_state.trigger_channel_index = state.trigger_channel_index;
        lvgl_state.trigger_mode = (uint16_t)state.trigger_mode;
        lvgl_state.trigger_run_mode = (uint16_t)state.trigger_run_mode;
        lvgl_state.paused = state.paused;
        ESP_RETURN_ON_ERROR(lvgl_app_request_control_state(&lvgl_state), TAG, "falha ao solicitar estado compartilhado");
    }
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

/**
 * @brief Handler da API de modo visual.
 *
 * @param[in] req Requisição HTTP atual.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
static esp_err_t scope_web_mode_handler(httpd_req_t *req)
{
    char query[SCOPE_WEB_QUERY_LEN] = {0};
    char value[16] = {0};
    char response[48];
    scope_output_mode_t mode = SCOPE_OUTPUT_MODE_BOTH;

    if (req->method == HTTP_POST) {
        if (httpd_req_get_url_query_len(req) > 0 &&
            httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
            httpd_query_key_value(query, "value", value, sizeof(value)) == ESP_OK &&
            scope_web_parse_output_mode(value, &mode)) {
            scope_web_set_output_mode(mode);
        }
    } else {
        mode = scope_web_get_output_mode();
    }

    snprintf(response,
             sizeof(response),
             "{\"mode\":\"%s\"}",
             (scope_web_get_output_mode() == SCOPE_OUTPUT_MODE_WEB_ONLY) ? "web" :
             (scope_web_get_output_mode() == SCOPE_OUTPUT_MODE_DISPLAY_ONLY) ? "display" : "both");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
}

/**
 * @brief Handler dedicado para alternar o modo foco do display local.
 *
 * @param[in] req Requisição HTTP atual.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
static esp_err_t scope_web_focus_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    ESP_RETURN_ON_ERROR(lvgl_app_request_toggle_focus(), TAG, "falha ao solicitar foco do display");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

/**
 * @brief Handler da API JSON com a janela atual do osciloscópio.
 *
 * @param[in] req Requisição HTTP atual.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
static esp_err_t scope_web_api_get_handler(httpd_req_t *req)
{
    scope_web_payload_t *payload = NULL;
    scope_web_view_state_t state = {0};
    uint32_t requested_samples = 0U;
    char head[SCOPE_WEB_JSON_HEAD_CAPACITY];
    esp_err_t ret = ESP_OK;

    if (scope_web_get_output_mode() == SCOPE_OUTPUT_MODE_DISPLAY_ONLY) {
        httpd_resp_set_status(req, "204 No Content");
        return httpd_resp_send(req, NULL, 0);
    }

    scope_web_get_runtime_state(&state, NULL);
    requested_samples = scope_web_get_requested_samples_from_state(&state);

    payload = calloc(1, sizeof(*payload));
    ESP_RETURN_ON_FALSE(payload != NULL, ESP_ERR_NO_MEM, TAG, "falha ao alocar payload web");

    ret = scope_web_collect_payload(&state, requested_samples, payload);
    if (ret != ESP_OK) {
        free(payload);
        ESP_LOGE(TAG, "falha ao coletar payload web: %s", esp_err_to_name(ret));
        return ret;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    snprintf(head,
             sizeof(head),
             "{"
             "\"requested_samples\":%" PRIu32 ","
             "\"sample_freq_hz\":%" PRIu32 ","
             "\"sample_channel_mode\":%u,"
             "\"timebase_index\":%u,"
             "\"voltscale_index\":%u,"
             "\"trigger_mode\":%u,"
             "\"trigger_run_mode\":%u,"
             "\"trigger_channel_index\":%u,"
             "\"trigger_level_mv\":%" PRId32 ","
             "\"paused\":%s,"
             "\"output_mode\":\"%s\","
             "\"y_max_mv\":%" PRId32 ","
             "\"channel_count\":%u,"
             "\"history_count\":%u,"
             "\"trigger_found\":%s,"
             "\"latest_mv\":[%" PRId32 ",%" PRId32 "],"
             "\"min_mv\":[%" PRId32 ",%" PRId32 "],"
             "\"max_mv\":[%" PRId32 ",%" PRId32 "],",
             payload->requested_samples,
             payload->sample_freq_hz,
             (unsigned)state.sample_channel_mode,
             (unsigned)state.timebase_index,
             (unsigned)state.voltscale_index,
             (unsigned)state.trigger_mode,
             (unsigned)state.trigger_run_mode,
             (unsigned)state.trigger_channel_index,
             state.trigger_level_mv,
             state.paused ? "true" : "false",
             (scope_web_get_output_mode() == SCOPE_OUTPUT_MODE_WEB_ONLY) ? "web" :
             (scope_web_get_output_mode() == SCOPE_OUTPUT_MODE_DISPLAY_ONLY) ? "display" : "both",
             scope_web_get_y_max_mv(&state),
             (unsigned)payload->channel_count,
             (unsigned)payload->history_count,
             payload->snapshot.trigger_found ? "true" : "false",
             payload->snapshot.latest_mv[0],
             payload->snapshot.latest_mv[1],
             payload->snapshot.min_mv[0],
             payload->snapshot.min_mv[1],
             payload->snapshot.max_mv[0],
             payload->snapshot.max_mv[1]);

    ret = httpd_resp_sendstr_chunk(req, head);
    if (ret == ESP_OK) {
        ret = scope_web_send_point_array(req, "points_ch1", payload->points_ch1, scope_web_get_output_point_count(), true);
    }
    if (ret == ESP_OK) {
        ret = scope_web_send_point_array(req, "points_ch2", payload->points_ch2, scope_web_get_output_point_count(), false);
    }
    if (ret == ESP_OK) {
        ret = httpd_resp_sendstr_chunk(req, "}");
    }
    if (ret == ESP_OK) {
        ret = httpd_resp_sendstr_chunk(req, NULL);
    }

    free(payload);
    if (ret == ESP_ERR_HTTPD_RESP_SEND) {
        return ESP_OK;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "falha ao responder API web: %s", esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Monta um payload JSON completo para envio via websocket.
 *
 * @param[in] state Estado web usado na composição da janela.
 * @param[in] payload Janela já coletada.
 * @param[out] out_json Buffer de saída.
 * @param[in] json_capacity Capacidade do buffer de saída.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
static esp_err_t scope_web_build_json(const scope_web_view_state_t *state,
                                      const scope_web_payload_t *payload,
                                      char *out_json,
                                      size_t json_capacity)
{
    size_t offset = 0U;
    const size_t point_count = scope_web_get_output_point_count();

    ESP_RETURN_ON_FALSE(state != NULL && payload != NULL && out_json != NULL, ESP_ERR_INVALID_ARG, TAG, "json invalido");

    offset += (size_t)snprintf(out_json + offset,
                               json_capacity - offset,
                               "{"
                               "\"requested_samples\":%" PRIu32 ","
                               "\"sample_freq_hz\":%" PRIu32 ","
                               "\"sample_channel_mode\":%u,"
                               "\"timebase_index\":%u,"
                               "\"voltscale_index\":%u,"
                               "\"trigger_mode\":%u,"
                               "\"trigger_level_mv\":%" PRId32 ","
                               "\"paused\":%s,"
                               "\"output_mode\":\"%s\","
                               "\"y_max_mv\":%" PRId32 ","
                               "\"channel_count\":%u,"
                               "\"history_count\":%u,"
                               "\"trigger_found\":%s,"
                               "\"latest_mv\":[%" PRId32 ",%" PRId32 "],"
                               "\"min_mv\":[%" PRId32 ",%" PRId32 "],"
                               "\"max_mv\":[%" PRId32 ",%" PRId32 "],"
                               "\"points_ch1\":[",
                               payload->requested_samples,
                               payload->sample_freq_hz,
                               (unsigned)state->sample_channel_mode,
                               (unsigned)state->timebase_index,
                               (unsigned)state->voltscale_index,
                               (unsigned)state->trigger_mode,
                               state->trigger_level_mv,
                               state->paused ? "true" : "false",
                               (scope_web_get_output_mode() == SCOPE_OUTPUT_MODE_WEB_ONLY) ? "web" :
                               (scope_web_get_output_mode() == SCOPE_OUTPUT_MODE_DISPLAY_ONLY) ? "display" : "both",
                               scope_web_get_y_max_mv(state),
                               (unsigned)payload->channel_count,
                               (unsigned)payload->history_count,
                               payload->snapshot.trigger_found ? "true" : "false",
                               payload->snapshot.latest_mv[0],
                               payload->snapshot.latest_mv[1],
                               payload->snapshot.min_mv[0],
                               payload->snapshot.min_mv[1],
                               payload->snapshot.max_mv[0],
                               payload->snapshot.max_mv[1]);
    ESP_RETURN_ON_FALSE(offset < json_capacity, ESP_ERR_NO_MEM, TAG, "json curto");

    for (size_t i = 0; i < point_count; i++) {
        const size_t src_index = (point_count <= 1U) ? 0U : (i * (SCOPE_WEB_SOURCE_POINT_COUNT - 1U)) / (point_count - 1U);
        offset += (size_t)snprintf(out_json + offset,
                                   json_capacity - offset,
                                   (i + 1U < point_count) ? "%" PRId32 "," : "%" PRId32 "],\"points_ch2\":[",
                                   payload->points_ch1[src_index]);
        ESP_RETURN_ON_FALSE(offset < json_capacity, ESP_ERR_NO_MEM, TAG, "json curto");
    }

    for (size_t i = 0; i < point_count; i++) {
        const size_t src_index = (point_count <= 1U) ? 0U : (i * (SCOPE_WEB_SOURCE_POINT_COUNT - 1U)) / (point_count - 1U);
        offset += (size_t)snprintf(out_json + offset,
                                   json_capacity - offset,
                                   (i + 1U < point_count) ? "%" PRId32 "," : "%" PRId32 "]}",
                                   payload->points_ch2[src_index]);
        ESP_RETURN_ON_FALSE(offset < json_capacity, ESP_ERR_NO_MEM, TAG, "json curto");
    }

    return ESP_OK;
}

/**
 * @brief Handler do websocket da waveform.
 *
 * @param[in] req Requisição HTTP atual.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
static esp_err_t scope_web_ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        s_scope_web_ws_fd = httpd_req_to_sockfd(req);
        ESP_LOGI(TAG, "WebSocket conectado, fd=%d", s_scope_web_ws_fd);
        return ESP_OK;
    }

    httpd_ws_frame_t frame = {
        .payload = NULL,
        .len = 0,
        .type = HTTPD_WS_TYPE_TEXT,
    };
    esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
    if (ret != ESP_OK) {
        if (s_scope_web_ws_fd == httpd_req_to_sockfd(req)) {
            s_scope_web_ws_fd = -1;
        }
        return ret;
    }

    if (frame.type == HTTPD_WS_TYPE_CLOSE) {
        if (s_scope_web_ws_fd == httpd_req_to_sockfd(req)) {
            s_scope_web_ws_fd = -1;
        }
    }

    return ESP_OK;
}

/**
 * @brief Task que envia continuamente a waveform atual via websocket.
 *
 * @param[in] arg Contexto não utilizado.
 */
static void scope_web_stream_task_fn(void *arg)
{
    (void)arg;

    while (true) {
        const int ws_fd = s_scope_web_ws_fd;
        const scope_output_mode_t mode = scope_web_get_output_mode();
        scope_web_view_state_t state = {0};
        scope_web_payload_t payload = {0};

        if (ws_fd >= 0 && mode != SCOPE_OUTPUT_MODE_DISPLAY_ONLY && s_server != NULL) {
            if (httpd_ws_get_fd_info(s_server, ws_fd) != HTTPD_WS_CLIENT_WEBSOCKET) {
                s_scope_web_ws_fd = -1;
                if (scope_web_get_output_mode() == SCOPE_OUTPUT_MODE_WEB_ONLY) {
                    scope_web_set_output_mode(SCOPE_OUTPUT_MODE_BOTH);
                }
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }

            char *json = calloc(1, SCOPE_WEB_WS_JSON_CAPACITY);
            if (json != NULL) {
                scope_web_get_runtime_state(&state, NULL);
                if (scope_web_collect_payload(&state,
                                              scope_web_get_requested_samples_from_state(&state),
                                              &payload) == ESP_OK &&
                    scope_web_build_json(&state, &payload, json, SCOPE_WEB_WS_JSON_CAPACITY) == ESP_OK) {
                    httpd_ws_frame_t frame = {
                        .payload = (uint8_t *)json,
                        .len = strlen(json),
                        .type = HTTPD_WS_TYPE_TEXT,
                    };
                    if (httpd_ws_send_frame_async(s_server, ws_fd, &frame) != ESP_OK) {
                        s_scope_web_ws_fd = -1;
                        if (scope_web_get_output_mode() == SCOPE_OUTPUT_MODE_WEB_ONLY) {
                            scope_web_set_output_mode(SCOPE_OUTPUT_MODE_BOTH);
                        }
                    }
                }
                free(json);
            }
        }

        vTaskDelay(pdMS_TO_TICKS((mode == SCOPE_OUTPUT_MODE_WEB_ONLY) ? SCOPE_WEB_REFRESH_MS_WEB_ONLY : SCOPE_WEB_REFRESH_MS_BOTH));
    }
}

/**
 * @brief Inicializa NVS e o stack base de rede usados pelo SoftAP.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
static esp_err_t scope_web_init_network_stack(void)
{
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "falha ao apagar NVS");
        err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(err, TAG, "falha ao inicializar NVS");

    if (!s_network_ready) {
        ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "falha no esp_netif_init");
        ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "falha ao criar event loop padrao");
        s_network_ready = true;
    }

    if (s_ap_netif == NULL) {
        s_ap_netif = esp_netif_create_default_wifi_ap();
        ESP_RETURN_ON_FALSE(s_ap_netif != NULL, ESP_FAIL, TAG, "falha ao criar netif AP");
    }

    return ESP_OK;
}

/**
 * @brief Inicializa o modo SoftAP com os parâmetros configurados para o projeto.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
static esp_err_t scope_web_start_softap(void)
{
    wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t wifi_config = {0};
    wifi_auth_mode_t authmode = WIFI_AUTH_WPA2_PSK;

    if (APP_WEB_SOFTAP_PASSWORD[0] == '\0') {
        authmode = WIFI_AUTH_OPEN;
    }

    ESP_RETURN_ON_ERROR(esp_wifi_init(&wifi_init_cfg), TAG, "falha ao inicializar Wi-Fi");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(WIFI_EVENT,
                                                            ESP_EVENT_ANY_ID,
                                                            &scope_web_wifi_event_handler,
                                                            NULL,
                                                            NULL),
                        TAG,
                        "falha ao registrar evento Wi-Fi");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "falha ao configurar storage Wi-Fi");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "falha ao configurar modo AP");

    strlcpy((char *)wifi_config.ap.ssid, APP_WEB_SOFTAP_SSID, sizeof(wifi_config.ap.ssid));
    strlcpy((char *)wifi_config.ap.password, APP_WEB_SOFTAP_PASSWORD, sizeof(wifi_config.ap.password));
    wifi_config.ap.channel = APP_WEB_SOFTAP_CHANNEL;
    wifi_config.ap.max_connection = APP_WEB_SOFTAP_MAX_CONN;
    wifi_config.ap.authmode = authmode;
    wifi_config.ap.ssid_len = strlen(APP_WEB_SOFTAP_SSID);
    wifi_config.ap.pmf_cfg.required = false;

    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &wifi_config), TAG, "falha ao aplicar config do AP");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "falha ao iniciar Wi-Fi");

    ESP_LOGI(TAG,
             "SoftAP ativo: SSID=%s senha=%s IP=http://192.168.4.1/",
             APP_WEB_SOFTAP_SSID,
             (authmode == WIFI_AUTH_OPEN) ? "<aberta>" : APP_WEB_SOFTAP_PASSWORD);
    return ESP_OK;
}

/**
 * @brief Inicia o servidor HTTP com a página do osciloscópio e a API JSON.
 *
 * @return `ESP_OK` em caso de sucesso.
 */
static esp_err_t scope_web_start_http_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    static const httpd_uri_t index_uri = {
        .uri = SCOPE_WEB_URI_INDEX,
        .method = HTTP_GET,
        .handler = scope_web_index_get_handler,
        .user_ctx = NULL,
    };
    static const httpd_uri_t api_uri = {
        .uri = SCOPE_WEB_URI_API,
        .method = HTTP_GET,
        .handler = scope_web_api_get_handler,
        .user_ctx = NULL,
    };
    static const httpd_uri_t mode_get_uri = {
        .uri = SCOPE_WEB_URI_MODE,
        .method = HTTP_GET,
        .handler = scope_web_mode_handler,
        .user_ctx = NULL,
    };
    static const httpd_uri_t mode_post_uri = {
        .uri = SCOPE_WEB_URI_MODE,
        .method = HTTP_POST,
        .handler = scope_web_mode_handler,
        .user_ctx = NULL,
    };
    static const httpd_uri_t focus_post_uri = {
        .uri = SCOPE_WEB_URI_FOCUS,
        .method = HTTP_POST,
        .handler = scope_web_focus_handler,
        .user_ctx = NULL,
    };
    static const httpd_uri_t control_post_uri = {
        .uri = "/api/control",
        .method = HTTP_POST,
        .handler = scope_web_control_handler,
        .user_ctx = NULL,
    };
    static const httpd_uri_t ws_uri = {
        .uri = SCOPE_WEB_URI_WS,
        .method = HTTP_GET,
        .handler = scope_web_ws_handler,
        .user_ctx = NULL,
        .is_websocket = true,
    };

    config.uri_match_fn = httpd_uri_match_wildcard;
    config.max_uri_handlers = 12;
    config.core_id = APP_WEB_HTTPD_CORE_ID;
    config.stack_size = 6144;

    ESP_RETURN_ON_ERROR(httpd_start(&s_server, &config), TAG, "falha ao iniciar HTTP server");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_server, &index_uri), TAG, "falha ao registrar /");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_server, &api_uri), TAG, "falha ao registrar API");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_server, &mode_get_uri), TAG, "falha ao registrar mode GET");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_server, &mode_post_uri), TAG, "falha ao registrar mode POST");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_server, &focus_post_uri), TAG, "falha ao registrar focus POST");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_server, &control_post_uri), TAG, "falha ao registrar control POST");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_server, &ws_uri), TAG, "falha ao registrar websocket");
    return ESP_OK;
}

void scope_web_set_output_mode(scope_output_mode_t mode)
{
    if (mode > SCOPE_OUTPUT_MODE_DISPLAY_ONLY) {
        mode = SCOPE_OUTPUT_MODE_BOTH;
    }
    s_output_mode = mode;
}

scope_output_mode_t scope_web_get_output_mode(void)
{
    return s_output_mode;
}

esp_err_t scope_web_start(void)
{
    if (!APP_WEB_ENABLED) {
        ESP_LOGI(TAG, "Interface web desabilitada por configuracao.");
        return ESP_OK;
    }

    if (s_server != NULL) {
        return ESP_OK;
    }

    if (s_scope_web_mutex == NULL) {
        s_scope_web_mutex = xSemaphoreCreateMutex();
        ESP_RETURN_ON_FALSE(s_scope_web_mutex != NULL, ESP_ERR_NO_MEM, TAG, "falha ao criar mutex web");
        scope_web_set_runtime_state(&s_scope_web_default_state, false);
    }

    ESP_RETURN_ON_ERROR(scope_web_init_network_stack(), TAG, "falha ao preparar rede");
    ESP_RETURN_ON_ERROR(scope_web_start_softap(), TAG, "falha ao iniciar SoftAP");
    ESP_RETURN_ON_ERROR(scope_web_start_http_server(), TAG, "falha ao iniciar servidor HTTP");

    if (s_scope_web_stream_task == NULL) {
        BaseType_t created = xTaskCreatePinnedToCore(scope_web_stream_task_fn,
                                                     "scope_web_ws",
                                                     6144,
                                                     NULL,
                                                     4,
                                                     &s_scope_web_stream_task,
                                                     APP_WEB_HTTPD_CORE_ID);
        ESP_RETURN_ON_FALSE(created == pdPASS, ESP_FAIL, TAG, "falha ao criar task websocket");
    }
    return ESP_OK;
}

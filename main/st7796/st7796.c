/**
 * @file st7796.c
 * @brief Implementação do driver ST7796 sobre o periférico LCD/I80 do ESP-IDF.
 */

#include "st7796.h"
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_lcd_io_i80.h"
#include "esp_lcd_panel_commands.h"
#include "esp_log.h"

#define ST7796_CMD_MADCTL  (0x36)
#define ST7796_CMD_COLMOD  (0x3A)

static const char *TAG = "st7796";

/**
 * @brief Estrutura interna da instância do driver ST7796.
 */
struct st7796_t {
    esp_lcd_i80_bus_handle_t bus;
    esp_lcd_panel_io_handle_t io;
    SemaphoreHandle_t done_sem;
    SemaphoreHandle_t lock;
    volatile uint32_t pending_dma;
    int reset_gpio_num;
    int backlight_gpio_num;
    uint32_t backlight_on_level;
    uint16_t h_res;
    uint16_t v_res;
    uint16_t x_gap;
    uint16_t y_gap;
    bool mirror_x;
    bool mirror_y;
    bool swap_xy;
    bool bgr;
    bool invert_colors;
    const st7796_init_cmd_t *init_cmds;
    size_t init_cmds_count;
    uint8_t madctl;
};

static const st7796_init_cmd_t s_default_init_cmds[] = {
    {LCD_CMD_SLPOUT, NULL, 0, 120},
    {ST7796_CMD_COLMOD, (const uint8_t[]) {0x55}, 1, 10},
    {LCD_CMD_NORON, NULL, 0, 10},
    {LCD_CMD_DISPON, NULL, 0, 120},
};

/**
 * @brief Callback chamada ao término de uma transferência de cor.
 */
static bool st7796_on_color_trans_done(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    (void)panel_io;
    (void)edata;

    st7796_handle_t handle = (st7796_handle_t)user_ctx;
    BaseType_t task_woken = pdFALSE;

    if (handle->pending_dma > 0) {
        handle->pending_dma--;
    }
    xSemaphoreGiveFromISR(handle->done_sem, &task_woken);
    return task_woken == pdTRUE;
}

/**
 * @brief Configura um GPIO simples de saída quando o número do pino é válido.
 */
static esp_err_t st7796_config_output_gpio_if_valid(int gpio_num)
{
    if (gpio_num < 0) {
        return ESP_OK;
    }

    gpio_config_t config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << gpio_num,
    };
    return gpio_config(&config);
}

/**
 * @brief Monta o valor do registrador MADCTL a partir do estado atual do driver.
 */
static uint8_t st7796_build_madctl(st7796_handle_t handle)
{
    uint8_t value = 0;

    if (handle->mirror_x) {
        value |= LCD_CMD_MX_BIT;
    }
    if (handle->mirror_y) {
        value |= LCD_CMD_MY_BIT;
    }
    if (handle->swap_xy) {
        value |= LCD_CMD_MV_BIT;
    }
    if (handle->bgr) {
        value |= LCD_CMD_BGR_BIT;
    }

    return value;
}

/**
 * @brief Aplica o registrador MADCTL de acordo com a orientação atual.
 */
static esp_err_t st7796_apply_madctl(st7796_handle_t handle)
{
    handle->madctl = st7796_build_madctl(handle);
    return esp_lcd_panel_io_tx_param(handle->io, ST7796_CMD_MADCTL, &handle->madctl, sizeof(handle->madctl));
}

/**
 * @brief Executa uma sequência arbitrária de inicialização do painel.
 */
static esp_err_t st7796_run_init_sequence(st7796_handle_t handle, const st7796_init_cmd_t *cmds, size_t count)
{
    for (size_t i = 0; i < count; i++) {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(handle->io, cmds[i].cmd, cmds[i].data, cmds[i].data_size), TAG, "falha cmd 0x%02X", cmds[i].cmd);
        if (cmds[i].delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(cmds[i].delay_ms));
        }
    }

    return ESP_OK;
}

/**
 * @brief Calcula o tamanho em bytes de uma área RGB565.
 */
static size_t st7796_region_size_bytes(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end)
{
    return ((size_t)(x_end - x_start) * (size_t)(y_end - y_start) * sizeof(uint16_t));
}

/**
 * @brief Valida a configuração recebida do usuário.
 */
static esp_err_t st7796_validate_config(const st7796_config_t *config)
{
    ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, TAG, "config nula");
    ESP_RETURN_ON_FALSE(config->dc_gpio_num >= 0, ESP_ERR_INVALID_ARG, TAG, "dc_gpio_num invalido");
    ESP_RETURN_ON_FALSE(config->wr_gpio_num >= 0, ESP_ERR_INVALID_ARG, TAG, "wr_gpio_num invalido");
    ESP_RETURN_ON_FALSE(config->pclk_hz > 0, ESP_ERR_INVALID_ARG, TAG, "pclk_hz invalido");
    ESP_RETURN_ON_FALSE(config->trans_queue_depth > 0, ESP_ERR_INVALID_ARG, TAG, "trans_queue_depth invalido");
    ESP_RETURN_ON_FALSE(config->max_transfer_bytes > 0, ESP_ERR_INVALID_ARG, TAG, "max_transfer_bytes invalido");
    ESP_RETURN_ON_FALSE(config->h_res > 0 && config->v_res > 0, ESP_ERR_INVALID_ARG, TAG, "resolucao invalida");

    for (size_t i = 0; i < 8; i++) {
        ESP_RETURN_ON_FALSE(config->data_gpio_nums[i] >= 0, ESP_ERR_INVALID_ARG, TAG, "data_gpio_nums[%u] invalido", (unsigned)i);
    }

    return ESP_OK;
}

/**
 * @brief Cria uma instância do driver ST7796 e inicializa o barramento I80.
 */
esp_err_t st7796_new(const st7796_config_t *config, st7796_handle_t *out_handle)
{
    esp_err_t ret = ESP_OK;
    st7796_handle_t handle = NULL;

    ESP_RETURN_ON_ERROR(st7796_validate_config(config), TAG, "configuracao invalida");
    ESP_RETURN_ON_FALSE(out_handle, ESP_ERR_INVALID_ARG, TAG, "out_handle nulo");

    handle = calloc(1, sizeof(*handle));
    ESP_GOTO_ON_FALSE(handle, ESP_ERR_NO_MEM, err, TAG, "sem memoria para handle");

    handle->done_sem = xSemaphoreCreateCounting((UBaseType_t)config->trans_queue_depth, 0);
    handle->lock = xSemaphoreCreateMutex();
    ESP_GOTO_ON_FALSE(handle->done_sem && handle->lock, ESP_ERR_NO_MEM, err, TAG, "falha ao criar sincronizacao");

    handle->reset_gpio_num = config->reset_gpio_num;
    handle->backlight_gpio_num = config->backlight_gpio_num;
    handle->backlight_on_level = config->backlight_on_level;
    handle->h_res = config->h_res;
    handle->v_res = config->v_res;
    handle->x_gap = config->x_gap;
    handle->y_gap = config->y_gap;
    handle->mirror_x = config->mirror_x;
    handle->mirror_y = config->mirror_y;
    handle->swap_xy = config->swap_xy;
    handle->bgr = config->bgr;
    handle->invert_colors = config->invert_colors;
    handle->init_cmds = config->init_cmds;
    handle->init_cmds_count = config->init_cmds_count;

    ESP_GOTO_ON_ERROR(st7796_config_output_gpio_if_valid(config->reset_gpio_num), err, TAG, "falha ao configurar reset");
    ESP_GOTO_ON_ERROR(st7796_config_output_gpio_if_valid(config->backlight_gpio_num), err, TAG, "falha ao configurar backlight");

    esp_lcd_i80_bus_config_t bus_config = {
        .dc_gpio_num = config->dc_gpio_num,
        .wr_gpio_num = config->wr_gpio_num,
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .bus_width = 8,
        .max_transfer_bytes = config->max_transfer_bytes,
        .dma_burst_size = config->dma_burst_size,
    };
    memcpy(bus_config.data_gpio_nums, config->data_gpio_nums, sizeof(config->data_gpio_nums));

    ESP_GOTO_ON_ERROR(esp_lcd_new_i80_bus(&bus_config, &handle->bus), err, TAG, "falha ao criar barramento i80");

    esp_lcd_panel_io_i80_config_t io_config = {
        .cs_gpio_num = config->cs_gpio_num,
        .pclk_hz = config->pclk_hz,
        .trans_queue_depth = config->trans_queue_depth,
        .on_color_trans_done = st7796_on_color_trans_done,
        .user_ctx = handle,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .dc_levels = {
            .dc_cmd_level = 0,
            .dc_dummy_level = 0,
            .dc_data_level = 1,
            .dc_idle_level = 0,
        },
        .flags = {
            .swap_color_bytes = config->swap_color_bytes,
            .pclk_active_neg = config->pclk_active_neg,
            .pclk_idle_low = config->pclk_idle_low,
        },
    };

    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_i80(handle->bus, &io_config, &handle->io), err, TAG, "falha ao criar io i80");

    if (handle->backlight_gpio_num >= 0) {
        gpio_set_level(handle->backlight_gpio_num, !handle->backlight_on_level);
    }

    *out_handle = handle;
    return ESP_OK;

err:
    st7796_del(handle);
    return ret;
}

/**
 * @brief Libera todos os recursos do driver ST7796.
 */
esp_err_t st7796_del(st7796_handle_t handle)
{
    if (!handle) {
        return ESP_OK;
    }

    if (handle->io) {
        esp_lcd_panel_io_del(handle->io);
    }
    if (handle->bus) {
        esp_lcd_del_i80_bus(handle->bus);
    }
    if (handle->reset_gpio_num >= 0) {
        gpio_reset_pin(handle->reset_gpio_num);
    }
    if (handle->backlight_gpio_num >= 0) {
        gpio_reset_pin(handle->backlight_gpio_num);
    }
    if (handle->done_sem) {
        vSemaphoreDelete(handle->done_sem);
    }
    if (handle->lock) {
        vSemaphoreDelete(handle->lock);
    }

    free(handle);
    return ESP_OK;
}

/**
 * @brief Executa reset por hardware ou software no painel.
 */
esp_err_t st7796_reset(st7796_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "handle nulo");

    xSemaphoreTake(handle->lock, portMAX_DELAY);
    if (handle->reset_gpio_num >= 0) {
        gpio_set_level(handle->reset_gpio_num, 0);
        vTaskDelay(pdMS_TO_TICKS(20));
        gpio_set_level(handle->reset_gpio_num, 1);
        vTaskDelay(pdMS_TO_TICKS(120));
    } else {
        esp_err_t ret = esp_lcd_panel_io_tx_param(handle->io, LCD_CMD_SWRESET, NULL, 0);
        xSemaphoreGive(handle->lock);
        ESP_RETURN_ON_ERROR(ret, TAG, "falha no swreset");
        vTaskDelay(pdMS_TO_TICKS(150));
        return ESP_OK;
    }
    xSemaphoreGive(handle->lock);

    return ESP_OK;
}

/**
 * @brief Inicializa o ST7796 com a sequência padrão ou a sequência customizada.
 */
esp_err_t st7796_init(st7796_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "handle nulo");

    const st7796_init_cmd_t *cmds = handle->init_cmds ? handle->init_cmds : s_default_init_cmds;
    const size_t cmd_count = handle->init_cmds ? handle->init_cmds_count : (sizeof(s_default_init_cmds) / sizeof(s_default_init_cmds[0]));

    xSemaphoreTake(handle->lock, portMAX_DELAY);
    esp_err_t ret = st7796_run_init_sequence(handle, cmds, cmd_count);
    if (ret == ESP_OK) {
        ret = st7796_apply_madctl(handle);
    }
    if (ret == ESP_OK) {
        ret = esp_lcd_panel_io_tx_param(handle->io, handle->invert_colors ? LCD_CMD_INVON : LCD_CMD_INVOFF, NULL, 0);
    }
    xSemaphoreGive(handle->lock);

    return ret;
}

/**
 * @brief Liga ou desliga a saída de imagem do painel.
 */
esp_err_t st7796_set_display_on(st7796_handle_t handle, bool on)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "handle nulo");
    return esp_lcd_panel_io_tx_param(handle->io, on ? LCD_CMD_DISPON : LCD_CMD_DISPOFF, NULL, 0);
}

/**
 * @brief Coloca o painel em sleep ou retira o painel de sleep.
 */
esp_err_t st7796_set_sleep(st7796_handle_t handle, bool sleep)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "handle nulo");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(handle->io, sleep ? LCD_CMD_SLPIN : LCD_CMD_SLPOUT, NULL, 0), TAG, "falha no comando sleep");
    vTaskDelay(pdMS_TO_TICKS(120));
    return ESP_OK;
}

/**
 * @brief Liga ou desliga a inversão de cores do painel.
 */
esp_err_t st7796_set_invert(st7796_handle_t handle, bool enabled)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "handle nulo");
    handle->invert_colors = enabled;
    return esp_lcd_panel_io_tx_param(handle->io, enabled ? LCD_CMD_INVON : LCD_CMD_INVOFF, NULL, 0);
}

/**
 * @brief Liga ou desliga o backlight quando ele foi configurado no driver.
 */
esp_err_t st7796_set_backlight(st7796_handle_t handle, bool on)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "handle nulo");
    ESP_RETURN_ON_FALSE(handle->backlight_gpio_num >= 0, ESP_ERR_INVALID_STATE, TAG, "backlight nao configurado");
    gpio_set_level(handle->backlight_gpio_num, on ? handle->backlight_on_level : !handle->backlight_on_level);
    return ESP_OK;
}

/**
 * @brief Atualiza os gaps usados na programação da janela de escrita.
 */
esp_err_t st7796_set_gap(st7796_handle_t handle, uint16_t x_gap, uint16_t y_gap)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "handle nulo");
    handle->x_gap = x_gap;
    handle->y_gap = y_gap;
    return ESP_OK;
}

/**
 * @brief Atualiza espelhamento e troca de eixos via MADCTL.
 */
esp_err_t st7796_set_rotation(st7796_handle_t handle, bool mirror_x, bool mirror_y, bool swap_xy)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "handle nulo");
    handle->mirror_x = mirror_x;
    handle->mirror_y = mirror_y;
    handle->swap_xy = swap_xy;
    return st7796_apply_madctl(handle);
}

/**
 * @brief Envia um comando curto por polling.
 */
esp_err_t st7796_tx_param(st7796_handle_t handle, uint8_t cmd, const void *data, size_t data_size)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "handle nulo");
    return esp_lcd_panel_io_tx_param(handle->io, cmd, data, data_size);
}

/**
 * @brief Programa a janela de acesso à GRAM do painel.
 */
esp_err_t st7796_set_address_window(st7796_handle_t handle, uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end)
{
    uint16_t xs;
    uint16_t xe;
    uint16_t ys;
    uint16_t ye;
    uint8_t caset[4];
    uint8_t raset[4];

    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "handle nulo");
    ESP_RETURN_ON_FALSE(x_start < x_end && y_start < y_end, ESP_ERR_INVALID_ARG, TAG, "janela invalida");
    ESP_RETURN_ON_FALSE(x_end <= handle->h_res && y_end <= handle->v_res, ESP_ERR_INVALID_ARG, TAG, "janela fora da resolucao");

    xs = x_start + handle->x_gap;
    xe = (uint16_t)(x_end - 1 + handle->x_gap);
    ys = y_start + handle->y_gap;
    ye = (uint16_t)(y_end - 1 + handle->y_gap);

    caset[0] = (uint8_t)(xs >> 8);
    caset[1] = (uint8_t)(xs & 0xFF);
    caset[2] = (uint8_t)(xe >> 8);
    caset[3] = (uint8_t)(xe & 0xFF);
    raset[0] = (uint8_t)(ys >> 8);
    raset[1] = (uint8_t)(ys & 0xFF);
    raset[2] = (uint8_t)(ye >> 8);
    raset[3] = (uint8_t)(ye & 0xFF);

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(handle->io, LCD_CMD_CASET, caset, sizeof(caset)), TAG, "falha em CASET");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(handle->io, LCD_CMD_RASET, raset, sizeof(raset)), TAG, "falha em RASET");
    return ESP_OK;
}

/**
 * @brief Envia um frame RGB565 de forma assíncrona usando DMA.
 */
esp_err_t st7796_draw_bitmap_async(st7796_handle_t handle, uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, const void *frame_data)
{
    ESP_RETURN_ON_FALSE(handle && frame_data, ESP_ERR_INVALID_ARG, TAG, "argumento invalido");

    size_t data_size = st7796_region_size_bytes(x_start, y_start, x_end, y_end);

    xSemaphoreTake(handle->lock, portMAX_DELAY);
    esp_err_t ret = st7796_set_address_window(handle, x_start, y_start, x_end, y_end);
    if (ret == ESP_OK) {
        handle->pending_dma++;
        ret = esp_lcd_panel_io_tx_color(handle->io, LCD_CMD_RAMWR, frame_data, data_size);
        if (ret != ESP_OK) {
            handle->pending_dma--;
        }
    }
    xSemaphoreGive(handle->lock);

    return ret;
}

/**
 * @brief Aguarda a conclusão de todas as transferências DMA pendentes.
 */
esp_err_t st7796_wait_all_dma_done(st7796_handle_t handle, uint32_t timeout_ms)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "handle nulo");

    TickType_t timeout_ticks = (timeout_ms == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    while (handle->pending_dma > 0) {
        if (xSemaphoreTake(handle->done_sem, timeout_ticks) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
    }

    return ESP_OK;
}

/**
 * @brief Envia um frame RGB565 e aguarda o término da DMA.
 */
esp_err_t st7796_draw_bitmap_blocking(st7796_handle_t handle, uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, const void *frame_data)
{
    ESP_RETURN_ON_ERROR(st7796_draw_bitmap_async(handle, x_start, y_start, x_end, y_end, frame_data), TAG, "falha ao iniciar dma");
    return st7796_wait_all_dma_done(handle, 1000);
}

/**
 * @brief Aloca um buffer adequado para transferências DMA do barramento I80.
 */
void *st7796_alloc_dma_buffer(st7796_handle_t handle, size_t size)
{
    if (!handle) {
        return NULL;
    }
    return esp_lcd_i80_alloc_draw_buffer(handle->io, size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
}

/**
 * @brief Retorna a largura lógica configurada do painel.
 */
uint16_t st7796_get_width(st7796_handle_t handle)
{
    return handle ? handle->h_res : 0;
}

/**
 * @brief Retorna a altura lógica configurada do painel.
 */
uint16_t st7796_get_height(st7796_handle_t handle)
{
    return handle ? handle->v_res : 0;
}

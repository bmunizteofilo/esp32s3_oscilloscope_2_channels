/**
 * @file ft6336u.c
 * @brief Implementação do driver do touch capacitivo FT6336U usando I2C.
 */

#include "ft6336u.h"
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"

#define FT6336U_I2C_ADDR            (0x38)
#define FT6336U_REG_DEV_MODE        (0x00)
#define FT6336U_REG_TD_STATUS       (0x02)
#define FT6336U_REG_P1_XH           (0x03)
#define FT6336U_REG_CHIP_ID         (0xA3)
#define FT6336U_REG_G_MODE          (0xA4)
#define FT6336U_REG_FIRMWARE_ID     (0xA6)

#define FT6336U_TOUCH_POINTS_MAX    (2U)
#define FT6336U_READ_FRAME_SIZE     (11U)
#define FT6336U_I2C_TIMEOUT_MS      (100)

static const char *TAG = "ft6336u";

/** @brief Estrutura interna do driver FT6336U. */
struct ft6336u_t {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    int int_gpio_num;
    int reset_gpio_num;
    bool own_bus;
    bool skip_reset;
    bool reset_active_low;
    uint16_t x_max;
    uint16_t y_max;
    bool swap_xy;
    bool mirror_x;
    bool mirror_y;
};

/**
 * @brief Aplica transformações de orientação em uma coordenada.
 */
static void ft6336u_transform_point(ft6336u_handle_t handle, uint16_t *x, uint16_t *y)
{
    uint16_t tx = *x;
    uint16_t ty = *y;

    if (handle->swap_xy) {
        uint16_t tmp = tx;
        tx = ty;
        ty = tmp;
    }
    if (handle->mirror_x && tx < handle->x_max) {
        tx = (uint16_t)(handle->x_max - 1U - tx);
    }
    if (handle->mirror_y && ty < handle->y_max) {
        ty = (uint16_t)(handle->y_max - 1U - ty);
    }

    *x = tx;
    *y = ty;
}

/**
 * @brief Valida a configuração de criação do driver.
 */
static esp_err_t ft6336u_validate_config(const ft6336u_config_t *config)
{
    ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, TAG, "config nula");
    ESP_RETURN_ON_FALSE(config->sda_gpio_num >= 0, ESP_ERR_INVALID_ARG, TAG, "sda invalido");
    ESP_RETURN_ON_FALSE(config->scl_gpio_num >= 0, ESP_ERR_INVALID_ARG, TAG, "scl invalido");
    ESP_RETURN_ON_FALSE(config->i2c_clk_speed_hz > 0, ESP_ERR_INVALID_ARG, TAG, "clock i2c invalido");
    ESP_RETURN_ON_FALSE(config->x_max > 0 && config->y_max > 0, ESP_ERR_INVALID_ARG, TAG, "limites invalidos");
    return ESP_OK;
}

/**
 * @brief Executa reset de hardware quando disponível.
 */
esp_err_t ft6336u_reset(ft6336u_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "handle nulo");

    if (handle->reset_gpio_num < 0 || handle->skip_reset) {
        return ESP_OK;
    }

    gpio_set_level(handle->reset_gpio_num, handle->reset_active_low ? 0 : 1);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(handle->reset_gpio_num, handle->reset_active_low ? 1 : 0);
    vTaskDelay(pdMS_TO_TICKS(200));

    return ESP_OK;
}

/**
 * @brief Cria e inicializa a instância do driver FT6336U.
 */
esp_err_t ft6336u_new(const ft6336u_config_t *config, ft6336u_handle_t *out_handle)
{
    esp_err_t ret = ESP_OK;
    ft6336u_handle_t handle = NULL;

    ESP_RETURN_ON_ERROR(ft6336u_validate_config(config), TAG, "config invalida");
    ESP_RETURN_ON_FALSE(out_handle, ESP_ERR_INVALID_ARG, TAG, "out_handle nulo");

    handle = calloc(1, sizeof(*handle));
    ESP_GOTO_ON_FALSE(handle, ESP_ERR_NO_MEM, err, TAG, "sem memoria");

    handle->int_gpio_num = config->int_gpio_num;
    handle->reset_gpio_num = config->reset_gpio_num;
    handle->skip_reset = config->skip_reset;
    handle->reset_active_low = config->reset_active_low;
    handle->x_max = config->x_max;
    handle->y_max = config->y_max;
    handle->swap_xy = config->swap_xy;
    handle->mirror_x = config->mirror_x;
    handle->mirror_y = config->mirror_y;

    if (handle->reset_gpio_num >= 0) {
        gpio_config_t rst_cfg = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << handle->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&rst_cfg), err, TAG, "falha gpio reset");
    }

    if (handle->int_gpio_num >= 0) {
        gpio_config_t int_cfg = {
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = 1ULL << handle->int_gpio_num,
            .pull_up_en = GPIO_PULLUP_ENABLE,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&int_cfg), err, TAG, "falha gpio int");
    }

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = config->i2c_port,
        .sda_io_num = config->sda_gpio_num,
        .scl_io_num = config->scl_gpio_num,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = config->enable_internal_pullup,
    };
    ESP_GOTO_ON_ERROR(i2c_new_master_bus(&bus_cfg, &handle->bus), err, TAG, "falha criar bus i2c");
    handle->own_bus = true;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = FT6336U_I2C_ADDR,
        .scl_speed_hz = config->i2c_clk_speed_hz,
        .scl_wait_us = 0,
    };
    ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(handle->bus, &dev_cfg, &handle->dev), err, TAG, "falha add device i2c");

    *out_handle = handle;
    return ESP_OK;

err:
    ft6336u_del(handle);
    return ret;
}

/**
 * @brief Libera os recursos da instância do FT6336U.
 */
esp_err_t ft6336u_del(ft6336u_handle_t handle)
{
    if (!handle) {
        return ESP_OK;
    }

    if (handle->dev) {
        i2c_master_bus_rm_device(handle->dev);
    }
    if (handle->bus && handle->own_bus) {
        i2c_del_master_bus(handle->bus);
    }
    if (handle->int_gpio_num >= 0) {
        gpio_reset_pin(handle->int_gpio_num);
    }
    if (handle->reset_gpio_num >= 0 && !handle->skip_reset) {
        gpio_reset_pin(handle->reset_gpio_num);
    }

    free(handle);
    return ESP_OK;
}

/**
 * @brief Lê um registrador de 8 bits do FT6336U.
 */
esp_err_t ft6336u_read_reg(ft6336u_handle_t handle, uint8_t reg_addr, uint8_t *value)
{
    ESP_RETURN_ON_FALSE(handle && value, ESP_ERR_INVALID_ARG, TAG, "argumento invalido");
    return i2c_master_transmit_receive(handle->dev, &reg_addr, 1, value, 1, FT6336U_I2C_TIMEOUT_MS);
}

/**
 * @brief Escreve um registrador de 8 bits do FT6336U.
 */
esp_err_t ft6336u_write_reg(ft6336u_handle_t handle, uint8_t reg_addr, uint8_t value)
{
    uint8_t payload[2] = {reg_addr, value};
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "handle nulo");
    return i2c_master_transmit(handle->dev, payload, sizeof(payload), FT6336U_I2C_TIMEOUT_MS);
}

/**
 * @brief Retorna a versão de firmware reportada pelo controlador.
 */
esp_err_t ft6336u_get_firmware_id(ft6336u_handle_t handle, uint8_t *firmware_id)
{
    return ft6336u_read_reg(handle, FT6336U_REG_FIRMWARE_ID, firmware_id);
}

/**
 * @brief Retorna o chip ID reportado pelo controlador.
 */
esp_err_t ft6336u_get_chip_id(ft6336u_handle_t handle, uint8_t *chip_id)
{
    return ft6336u_read_reg(handle, FT6336U_REG_CHIP_ID, chip_id);
}

/**
 * @brief Inicializa o controlador touch e valida a comunicação básica.
 */
esp_err_t ft6336u_init(ft6336u_handle_t handle)
{
    uint8_t chip_id = 0;

    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "handle nulo");
    ESP_RETURN_ON_ERROR(ft6336u_reset(handle), TAG, "falha no reset");
    ESP_RETURN_ON_ERROR(ft6336u_write_reg(handle, FT6336U_REG_DEV_MODE, 0x00), TAG, "falha no modo normal");
    ESP_RETURN_ON_ERROR(ft6336u_write_reg(handle, FT6336U_REG_G_MODE, 0x00), TAG, "falha no modo de interrupcao");
    ESP_RETURN_ON_ERROR(ft6336u_get_chip_id(handle, &chip_id), TAG, "falha na leitura do chip id");
    ESP_LOGI(TAG, "FT6336U detectado, chip_id=0x%02X", chip_id);

    return ESP_OK;
}

/**
 * @brief Lê o quadro atual de toque com até dois pontos.
 */
esp_err_t ft6336u_read_touch_data(ft6336u_handle_t handle, ft6336u_touch_data_t *touch_data)
{
    uint8_t reg = FT6336U_REG_TD_STATUS;
    uint8_t buf[FT6336U_READ_FRAME_SIZE] = {0};

    ESP_RETURN_ON_FALSE(handle && touch_data, ESP_ERR_INVALID_ARG, TAG, "argumento invalido");
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(handle->dev, &reg, 1, buf, sizeof(buf), FT6336U_I2C_TIMEOUT_MS), TAG, "falha leitura touch");

    touch_data->touch_count = buf[0] & 0x0F;
    if (touch_data->touch_count > FT6336U_TOUCH_POINTS_MAX) {
        touch_data->touch_count = FT6336U_TOUCH_POINTS_MAX;
    }

    for (size_t i = 0; i < FT6336U_TOUCH_POINTS_MAX; i++) {
        touch_data->points[i].pressed = false;
        touch_data->points[i].x = 0;
        touch_data->points[i].y = 0;
        touch_data->points[i].id = 0;
        touch_data->points[i].event = 0;
        touch_data->points[i].weight = 0;
        touch_data->points[i].area = 0;
    }

    for (size_t i = 0; i < touch_data->touch_count; i++) {
        const size_t base = 1 + (i * 6);
        uint16_t x = (uint16_t)(((buf[base + 0] & 0x0F) << 8) | buf[base + 1]);
        uint16_t y = (uint16_t)(((buf[base + 2] & 0x0F) << 8) | buf[base + 3]);

        ft6336u_transform_point(handle, &x, &y);

        touch_data->points[i].pressed = true;
        touch_data->points[i].event = (uint8_t)(buf[base + 0] >> 6);
        touch_data->points[i].id = (uint8_t)(buf[base + 2] >> 4);
        touch_data->points[i].x = x;
        touch_data->points[i].y = y;
        touch_data->points[i].weight = buf[base + 4];
        touch_data->points[i].area = (uint8_t)(buf[base + 5] >> 4);
    }

    return ESP_OK;
}

/**
 * @brief Retorna `true` quando a linha de interrupção indica atividade de toque.
 */
bool ft6336u_is_interrupt_active(ft6336u_handle_t handle)
{
    if (!handle || handle->int_gpio_num < 0) {
        return false;
    }
    return gpio_get_level(handle->int_gpio_num) == 0;
}

/**
 * @brief Atualiza a transformação das coordenadas lidas.
 */
esp_err_t ft6336u_set_transform(ft6336u_handle_t handle, bool swap_xy, bool mirror_x, bool mirror_y)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "handle nulo");
    handle->swap_xy = swap_xy;
    handle->mirror_x = mirror_x;
    handle->mirror_y = mirror_y;
    return ESP_OK;
}

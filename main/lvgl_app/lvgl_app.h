#pragma once

/**
 * @file lvgl_app.h
 * @brief Módulo de integração do LVGL com o display ST7796 e o touch FT6336U.
 */

#include "st7796.h"
#include "ft6336u.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Inicializa o LVGL, a interface de osciloscópio e a task dedicada da UI. */
void lvgl_app_start(st7796_handle_t lcd, ft6336u_handle_t touch);

#ifdef __cplusplus
}
#endif

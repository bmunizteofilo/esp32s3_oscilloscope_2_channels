idf_component_register(SRCS "main.c"
                            "adc/adc_scope.c"
                            "touch/ft6336u.c"
                            "st7796/st7796.c"
                            "lvgl_app/lvgl_app.c"
                            "web/scope_web.c"
                       REQUIRES esp_lcd lvgl esp_timer esp_driver_i2c esp_adc
                                esp_wifi esp_event esp_netif nvs_flash esp_http_server
                       INCLUDE_DIRS "."
                                    "adc"
                                    "touch"
                                    "st7796"
                                    "lvgl_app"
                                    "web")

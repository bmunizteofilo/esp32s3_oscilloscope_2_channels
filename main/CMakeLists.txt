idf_component_register(SRCS "main.c"
                            "adc/adc_scope.c"
                            "touch/ft6336u.c"
                            "st7796/st7796.c"
                            "lvgl_app/lvgl_app.c"
                       REQUIRES esp_lcd lvgl esp_timer esp_driver_i2c esp_adc
                       INCLUDE_DIRS "."
                                    "adc"
                                    "touch"
                                    "st7796"
                                    "lvgl_app")

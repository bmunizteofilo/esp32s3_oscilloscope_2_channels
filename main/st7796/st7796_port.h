#pragma once

/**
 * @file st7796_port.h
 * @brief Defines de hardware e parâmetros do painel ST7796 para ajuste do projeto.
 */

#define ST7796_PIN_RESET            (4)         /**< GPIO do sinal RESET do display. Use `-1` se não estiver ligado. */
#define ST7796_PIN_BACKLIGHT        (45)         /**< GPIO de controle do backlight. Use `-1` se não for controlado pelo firmware. */
#define ST7796_PIN_DC               (0)         /**< GPIO do sinal D/C ou RS do barramento 8080. */
#define ST7796_PIN_WR               (47)         /**< GPIO do sinal WR ou PCLK do barramento 8080. */
#define ST7796_PIN_CS               (-1)         /**< GPIO do chip select. Use `-1` para uso exclusivo do barramento. */
#define ST7796_PIN_D0               (9)         /**< GPIO da linha de dados D0. */
#define ST7796_PIN_D1               (46)         /**< GPIO da linha de dados D1. */
#define ST7796_PIN_D2               (3)         /**< GPIO da linha de dados D2. */
#define ST7796_PIN_D3               (8)         /**< GPIO da linha de dados D3. */
#define ST7796_PIN_D4               (18)         /**< GPIO da linha de dados D4. */
#define ST7796_PIN_D5               (17)         /**< GPIO da linha de dados D5. */
#define ST7796_PIN_D6               (16)         /**< GPIO da linha de dados D6. */
#define ST7796_PIN_D7               (15)         /**< GPIO da linha de dados D7. */

#define ST7796_BACKLIGHT_ON_LEVEL   (1)          /**< Nível lógico que liga o backlight configurado em `ST7796_PIN_BACKLIGHT`. */
#define ST7796_PIXEL_CLOCK_HZ       (40000000UL) /**< Frequência do sinal WR do barramento I80 em hertz. */
#define ST7796_DMA_QUEUE_DEPTH      (4U)         /**< Quantidade de transferências DMA que podem ficar enfileiradas. */
#define ST7796_DMA_BURST_SIZE       (16U)        /**< Tamanho do burst DMA em bytes. */
#define ST7796_H_RES                (480U)       /**< Resolução horizontal lógica do painel em pixels em modo paisagem. */
#define ST7796_V_RES                (320U)       /**< Resolução vertical lógica do painel em pixels em modo paisagem. */
#define ST7796_DRAW_BUFFER_LINES    (40U)        /**< Quantidade de linhas por bloco no buffer de desenho em RAM interna. */
#define ST7796_LVGL_TASK_STACK_SIZE (8192U)      /**< Tamanho da stack da task dedicada ao LVGL em bytes. */
#define ST7796_LVGL_TASK_PRIORITY   (4U)         /**< Prioridade da task dedicada ao LVGL. */
#define ST7796_X_GAP                (0U)         /**< Offset horizontal aplicado ao programar a janela de escrita. */
#define ST7796_Y_GAP                (0U)         /**< Offset vertical aplicado ao programar a janela de escrita. */
#define ST7796_SWAP_XY              (1)          /**< Troca os eixos X e Y via registrador MADCTL para usar o painel em paisagem. */
#define ST7796_MIRROR_X             (1)          /**< Espelha a imagem no eixo X para fechar a rotação em paisagem. */
#define ST7796_MIRROR_Y             (1)          /**< Espelha a imagem no eixo Y para fechar a rotação em paisagem. */
#define ST7796_INVERT_COLORS        (1)          /**< Habilita inversão de cores do display quando diferente de zero. */
#define ST7796_BGR_ORDER            (1)          /**< Usa ordem de cor BGR quando diferente de zero, ou RGB quando zero. */
#define ST7796_SWAP_COLOR_BYTES     (1)          /**< Troca os bytes de cada pixel RGB565 no periférico I80 quando diferente de zero. */
#define ST7796_PCLK_ACTIVE_NEG      (0)          /**< Usa borda de descida do WR para amostragem quando diferente de zero. */
#define ST7796_PCLK_IDLE_LOW        (0)          /**< Mantém o WR em nível baixo quando o barramento está ocioso. */

#define FT6336U_PIN_INT             (7)          /**< GPIO da interrupção do touch FT6336U. */
#define FT6336U_PIN_SDA             (6)          /**< GPIO da linha SDA do touch FT6336U. */
#define FT6336U_PIN_SCL             (5)          /**< GPIO da linha SCL do touch FT6336U. */
#define FT6336U_PIN_RST             (4)          /**< GPIO do reset do touch FT6336U, compartilhado com o reset do LCD. */
#define FT6336U_I2C_CLOCK_HZ        (400000UL)   /**< Clock do barramento I2C do touch em hertz. */
#define FT6336U_SWAP_XY             (1)          /**< Troca os eixos do touch para acompanhar a rotação do display em paisagem. */
#define FT6336U_MIRROR_X            (1)          /**< Espelha o eixo X do touch para alinhar com o display quando diferente de zero. */
#define FT6336U_MIRROR_Y            (0)          /**< Mantém o eixo Y do touch no sentido natural para a navegação vertical em paisagem. */
#define FT6336U_SKIP_RESET          (1)          /**< Não aplica reset dedicado no touch quando diferente de zero. Útil quando o reset é compartilhado com o LCD. */

#define APP_ADC_SAMPLE_FREQ_HZ      (20000UL)      /**< Frequência alvo do ADC contínuo em hertz por canal. */
#define APP_ADC_UNIT               ADC_UNIT_1      /**< Unidade ADC usada pelo exemplo. */
#define APP_ADC_CHANNEL_1          ADC_CHANNEL_9   /**< Canal 1 do exemplo, mapeado no ESP32-S3 para GPIO10. */
#define APP_ADC_CHANNEL_2          ADC_CHANNEL_0   /**< Canal 2 do exemplo, mapeado no ESP32-S3 para GPIO01. */
#define APP_ADC_ATTENUATION_1      ADC_ATTEN_DB_12 /**< Atenuação do canal 1 usada na conversão. */
#define APP_ADC_ATTENUATION_2      ADC_ATTEN_DB_12 /**< Atenuação do canal 2 usada na conversão. */
#define APP_ADC_HISTORY_SAMPLES     (20000U)      /**< Quantidade de amostras retidas no histórico circular para a base de tempo. */
#define APP_ADC_CHART_POINTS        (240U)       /**< Quantidade de pontos exibidos no gráfico do osciloscópio. */
#define APP_ADC_BLOCK_SAMPLES       (240U)       /**< Quantidade padrão de amostras em uma captura por bloco. */
#define APP_ADC_DMA_FRAME_BYTES     (256U)       /**< Tamanho do frame DMA do ADC em bytes. */
#define APP_ADC_STORE_BUFFER_BYTES  (2048U)      /**< Tamanho do pool interno do driver contínuo do ADC em bytes. */
#define APP_ADC_TASK_STACK_SIZE     (4096U)      /**< Tamanho da stack da task de aquisição ADC em bytes. */
#define APP_ADC_TASK_PRIORITY       (5U)         /**< Prioridade da task dedicada à captura ADC. */

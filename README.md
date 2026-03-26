# Mini Osciloscopio 2 Canais

Osciloscópio embarcado para `ESP32-S3` com interface gráfica em `LVGL`, display `ST7796` em barramento paralelo `I80`, touch `FT6336U` e aquisição contínua de `ADC + DMA`.

O projeto foi pensado para entregar uma experiência de bancada compacta: visual tipo osciloscópio, dois canais analógicos, trigger, cursores, pausa com navegação no histórico, autoajuste de escala e interface touch otimizada para uso direto no display.

## Destaques

- Interface gráfica em `LVGL 9` com layout em paisagem para display `480x320`
- Aquisição contínua com `ADC1 + DMA` no `ESP32-S3`
- Dois canais analógicos simultâneos
- Trigger por borda de subida e descida
- Modos de trigger `Auto`, `Normal` e `Single`
- Nível de trigger ajustável por gesto vertical no gráfico
- Histórico circular para navegação temporal
- Modo `Pause` com exploração do buffer congelado
- Cursores de tempo e tensão com leitura individual e delta
- Escala vertical configurável
- Base de tempo configurável
- Auto Set para escolher automaticamente `Div/Volts` e `Div/Time`
- Tela splash de inicialização
- Modo foco em tela cheia por triple-tap no chart

## Hardware

### Plataforma principal

- `ESP32-S3`
- Display `ST7796`
- Touch `FT6336U`
- PSRAM habilitada

### Resolução e orientação

- Resolução lógica: `480 x 320`
- Orientação: paisagem

### Mapeamento atual do display

Configurado em [main/st7796/st7796_port.h](/c:/esp/Projects_553/LVGL_ESP32S3/main/st7796/st7796_port.h).

- `RESET`: `GPIO4`
- `BACKLIGHT`: `GPIO45`
- `DC`: `GPIO0`
- `WR`: `GPIO47`
- `D0..D7`: `GPIO9, 46, 3, 8, 18, 17, 16, 15`

### Mapeamento atual do touch

- `INT`: `GPIO7`
- `SDA`: `GPIO6`
- `SCL`: `GPIO5`
- `RST`: `GPIO4`

### Canais ADC atuais

- `Canal 1`: `GPIO10 / ADC1_CH9`
- `Canal 2`: `GPIO01 / ADC1_CH0`

## Stack usada

- `ESP-IDF 5.5.3`
- `LVGL 9`
- `esp_lcd`
- `esp_adc`

## Funcionalidades do osciloscópio

### Aquisição

- Aquisição contínua com DMA
- Dois canais adquiridos simultaneamente
- Conversão para milivolts com suporte à calibração do ADC
- Buffer circular de histórico por canal
- Buffer em bloco para capturas finitas

### Visualização

- Chart com tema de osciloscópio
- Fundo preto
- Grade com linhas centrais destacadas
- Linha do canal 1 em verde fluorescente
- Linha do canal 2 em vermelho
- Labels de `T/div` e `V/div`
- Indicador de `Buffer %` dentro do chart

### Canais

Seleção no topo da interface:

- `Ch 1`
- `Ch 2`
- `Ch 1 e 2`

Quando apenas um canal está ativo:

- O chart ocupa mais altura
- A barra inferior mostra só as métricas do canal visível

Quando os dois canais estão ativos:

- O chart reduz altura
- A barra inferior mostra duas linhas de métricas, uma por canal

### Trigger

Recursos atuais:

- `Off`
- `Subida`
- `Descida`
- `Auto`
- `Normal`
- `Single`
- Canal de trigger selecionável
- Nível de trigger ajustável por toque vertical no gráfico
- Linha horizontal de trigger
- Ocultação automática da linha visual após alguns segundos sem interação

Observação importante:

- Na base de tempo de `1 s`, o trigger é desativado automaticamente porque essa janela encosta no limite do histórico atual

### Pause e histórico

- `Capture`: aquisição contínua e atualização ao vivo
- `Pause`: interrupção da aquisição e navegação no histórico congelado
- Arraste horizontal no chart para caminhar pelo buffer pausado
- `Buffer %` mostra:
  - em `Capture`: ocupação do histórico
  - em `Pause`: posição da janela atual dentro do histórico congelado

### Cursores

Dois modos:

- `Tempo`
- `Tensao`

Comportamento:

- Duas linhas aparecem no chart
- Seleção de `Linha 1` ou `Linha 2`
- Quando o cursor está ativo, o gesto passa a priorizar o cursor

Leituras exibidas:

- Em `Tempo`: `T1`, `T2`, `Dt`
- Em `Tensao`: `V1`, `V2`, `DV`

### Escalas

#### Div/Volts

Escalas disponíveis:

- `100 mV`
- `200 mV`
- `500 mV`
- `1 V`
- `2 V`
- `3.3 V`

Comportamento:

- Sempre mostra de `0 V` até o valor escolhido
- Se o sinal ultrapassar a escala, `Pk+` indica `Overflow`

#### Div/Time

Bases disponíveis:

- `5 ms`
- `10 ms`
- `15 ms`
- `20 ms`
- `25 ms`
- `50 ms`
- `100 ms`
- `250 ms`
- `500 ms`
- `1 s`

### Auto Set

O `Auto Set` tenta escolher automaticamente:

- `Div/Volts`
- `Div/Time`

Critério:

- Usa o canal atualmente selecionado na UI
- Se `Ch 1 e 2` estiver ativo, usa `Canal 1` como referência
- Analisa amplitude e periodicidade do sinal
- Exibe um aviso central temporário enquanto processa

### Métricas exibidas

Na barra inferior:

- `RMS`
- `Pk+`
- `Pk-`
- `Freq`
- `Duty`

Regras:

- `Freq` e `Duty` dependem de trigger válido
- A barra inferior é rolável horizontalmente quando há mais métricas do que slots visíveis

### Gestos e atalhos

- Arraste vertical com trigger ativo: ajusta o nível do trigger
- Arraste horizontal em `Pause`: navega no histórico
- Triple-tap no chart: alterna entre modo normal e modo foco em tela cheia

## Interface

### Barra superior

A barra superior é rolável horizontalmente e mostra até 4 grupos por vez.

Ordem atual:

- Canal
- Canal Trig
- Div/Volts
- Div/Time
- Trigger
- Trig Mode
- Status
- Cursor
- Linha
- Auto Set

### Modo foco

No modo foco:

- O chart ocupa praticamente toda a tela
- As barras superior e inferior são escondidas
- Um novo triple-tap restaura a UI completa

## Estrutura do projeto

### Arquivos principais

- [main/main.c](/c:/esp/Projects_553/LVGL_ESP32S3/main/main.c)
  Bootstrap da aplicação, inicialização do LCD, touch, ADC e LVGL

- [main/lvgl_app/lvgl_app.c](/c:/esp/Projects_553/LVGL_ESP32S3/main/lvgl_app/lvgl_app.c)
  Interface gráfica do osciloscópio, splash screen, chart, trigger, cursores, métricas e gestos

- [main/adc/adc_scope.c](/c:/esp/Projects_553/LVGL_ESP32S3/main/adc/adc_scope.c)
  Captura ADC contínua com DMA, histórico circular, trigger e composição das janelas

- [main/adc/adc_scope.h](/c:/esp/Projects_553/LVGL_ESP32S3/main/adc/adc_scope.h)
  API pública do módulo de aquisição

- [main/st7796/st7796.c](/c:/esp/Projects_553/LVGL_ESP32S3/main/st7796/st7796.c)
  Driver do display ST7796 usando `esp_lcd` sobre barramento `I80`

- [main/st7796/st7796_port.h](/c:/esp/Projects_553/LVGL_ESP32S3/main/st7796/st7796_port.h)
  Configuração central de pinos, orientação, cor, touch e parâmetros de aquisição

## Configuração atual do painel

Combinação validada no hardware atual:

- `ST7796_INVERT_COLORS = 1`
- `ST7796_BGR_ORDER = 1`
- `ST7796_SWAP_COLOR_BYTES = 1`

Essas definições ficam em [main/st7796/st7796_port.h](/c:/esp/Projects_553/LVGL_ESP32S3/main/st7796/st7796_port.h).

## Build

### Pré-requisitos

- `ESP-IDF 5.5.3`
- Ferramentas do `ESP32-S3` instaladas
- Ambiente exportado com `export.ps1`

### Compilar

```powershell
idf.py build
```

### Gravar

```powershell
idf.py -p COMx flash
```

### Monitor serial

```powershell
idf.py -p COMx monitor
```

## Comportamento de memória

- Flash configurada para `8 MB`
- PSRAM habilitada
- LVGL configurado para usar `malloc` do sistema, permitindo uso da heap geral
- Histórico atual do ADC: `20000` amostras por canal

Com `20 kSa/s`, isso representa cerca de:

- `1 segundo` de histórico por canal

## Limitações atuais

- A base máxima útil hoje é `1 s`, limitada pelo tamanho do histórico atual
- O trigger é desabilitado automaticamente em `1 s`
- O projeto trabalha com sinais positivos de `0 a 3.3 V`
- Os pinos ADC do `ESP32-S3` não são tolerantes a `5 V`

## Próximas evoluções possíveis

- Aumentar o histórico para bases de tempo mais longas
- Ajuste explícito de nível de trigger por valor numérico
- Trigger mais avançado com histerese configurável
- Mais refinamentos visuais no modo foco
- Exportação de captura ou snapshot
- Mais canais ou front-end analógico dedicado

## Autor

**Bruno Muniz**  
Engenheiro eletrônico e engenheiro de software

## Licença

Defina aqui a licença escolhida para o repositório antes de publicar no GitHub.

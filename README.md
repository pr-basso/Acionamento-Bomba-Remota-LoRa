# Acionamento Bomba Remota LoRa

Firmware Arduino para acionamento remoto de bomba d'água via rádio LoRa. Um nó sensor de boia (**Node A**) lê o nível de água e envia comandos por LoRa para um ou mais nós controladores de bomba (**Node B/C/D**), que acionam módulos de relé.

O hardware utilizado é o **Heltec WiFi LoRa 32** (V2 ou V3) — uma placa ESP32 com rádio LoRa integrado e display OLED 128×64.

## Como funciona

- **Boia** (`Boia.ino`): lê o nível de água por um sensor de boia em um pino digital, envia mensagens `CMD` (`PUMP=ON` / `PUMP=OFF`) para todos os nós de bomba cadastrados e aguarda a confirmação (`ACK`) de cada um.
- **Bomba** (`Bomba.ino`): escuta mensagens `CMD`, aciona o relé fisicamente e responde com `ACK`, reportando o estado real do relé.

Cada versão (`V1.0` a `V8.0`) contém as duas sketches, `Boia/Boia.ino` e `Bomba/Bomba.ino`, de forma independente. **A V8.0 é a versão atual/mais recente**; as anteriores foram mantidas como referência histórica do desenvolvimento.

## Estrutura do repositório

```
V1.0/ .. V8.0/
├── Boia/Boia.ino    # Node A — sensor de boia
└── Bomba/Bomba.ino  # Node B/C/D — controlador de bomba
```

## Protocolo de comunicação

Mensagens em formato texto, separadas por `|`, com 6 campos:

```
SRC|DST|SEQ|TYPE|PAYLOAD|CRC
```

| Campo     | Descrição                                                            |
|-----------|-----------------------------------------------------------------------|
| `SRC/DST` | Endereço de origem/destino do nó                                     |
| `SEQ`     | 4 dígitos hex, incrementado a cada `CMD`                              |
| `TYPE`    | `CMD` (Boia → Bomba) ou `ACK` (Bomba → Boia)                          |
| `PAYLOAD` | `PUMP=ON` / `PUMP=OFF` (CMD) ou `RELAY=1;PUMP=ON` (ACK, estado real)  |
| `CRC`     | 2 dígitos hex, XOR de todos os bytes de `SRC\|DST\|SEQ\|TYPE\|PAYLOAD` |

## Parâmetros de rádio LoRa (V8.0)

Todos os nós devem usar exatamente a mesma configuração. Na V8.0 ela fica em um único arquivo, `V8.0/common/LoraConfig.h`, que as duas sketches incluem por link simbólico (`Boia/LoraConfig.h` e `Bomba/LoraConfig.h`):

| Parâmetro        | Valor      |
|-------------------|-----------|
| Frequência         | 915 MHz   |
| Spreading Factor   | SF9       |
| Bandwidth          | 125 kHz   |
| Coding Rate        | 4/5       |
| Preâmbulo          | 8         |
| Potência TX        | 14 dBm    |
| IQ Inversion       | desligado |

## Hardware

- `RELAY_PIN = 26`, lógica **invertida**: `RELAY_ON = LOW`, `RELAY_OFF = HIGH` (módulos de relé azul padrão)
- `BOIA_PIN = 2`, `INPUT_PULLUP`: `LOW` = caixa vazia (boia aberta), `HIGH` = caixa cheia (boia fechada)
- Display OLED SSD1306 128×64, I2C endereço `0x3C`, via `HT_SSD1306Wire`
- O pino `Vext` deve ficar em nível baixo para energizar o OLED nas placas Heltec

## Arquitetura (V8.0)

- **Multi-bomba com fan-out sequencial**: a Boia mantém uma tabela `peers[]` e envia os comandos um nó por vez (`INTER_PEER_MS = 2000`), evitando colisões de RF.
- **ISR de rádio disciplinada**: os callbacks (`OnTxDone`, `OnRxDone`, `OnRxError`) só alteram flags `volatile bool`; as chamadas reais de rádio acontecem em `loop()`.
- **Guarda de tempo TX→RX**: a Bomba aguarda `TX_GUARD_MS = 100ms` após receber um `CMD` antes de enviar o `ACK`, garantindo que a Boia já esteja escutando.
- **Persistência em EEPROM**: o estado do relé é salvo a cada mudança e restaurado no boot. Se restaurado como ligado, o watchdog fica suspenso até chegar o primeiro `CMD` real.
- **Watchdog**: desliga a bomba após `WATCHDOG_MS = 600000ms` (10 min) sem `CMD` válido. Antes de desligar, relê a EEPROM — se o último `CMD` conhecido foi ligar, reinicia o watchdog e mantém a bomba ligada (link apenas temporariamente fora do ar).

## Compilar e gravar

Use **Arduino IDE** ou **arduino-cli**. Não há Makefile — é um projeto Arduino padrão.

Placa necessária: **Heltec ESP32** (pacote oficial, fornece `LoRaWan_APP.h` e `HT_SSD1306Wire.h`).

```bash
# Instalar o board via arduino-cli
arduino-cli core install Heltec-esp32:esp32

# Compilar (exemplo para o nó Bomba)
arduino-cli compile --fqbn Heltec-esp32:esp32:WIFI_LoRa_32_V2 "V8.0/Bomba"

# Gravar
arduino-cli upload -p /dev/cu.usbserial-XXXX --fqbn Heltec-esp32:esp32:WIFI_LoRa_32_V2 "V8.0/Bomba"

# Monitor serial (115200 baud)
arduino-cli monitor -p /dev/cu.usbserial-XXXX -b 115200
```

> **V1.0** usa a biblioteca `heltec-unofficial` (`heltec.h`). Todas as versões seguintes usam `LoRaWan_APP.h`.

## Adicionar um novo nó de bomba

Em `V8.0/Boia/Boia.ino`, adicione o endereço do novo nó em `PEER_LIST` e grave a nova placa Bomba com `MY_ADDRESS` igual a esse endereço. Nenhuma outra alteração é necessária.

## Telemetria (LoRa → MQTT)

Rede **separada** da rede Boia/Bomba (920 MHz em vez de 915 MHz), com placas **Heltec WiFi LoRa 32 V4**. Sensores enviam leituras por LoRa a um gateway, que publica no broker Mosquitto via Ethernet.

```
Telemetria/
├── common/TelemetriaConfig.h   # rádio + protocolo (fonte única, via link simbólico)
├── Gateway/Gateway.ino         # LoRa -> MQTT, Ethernet W5500
└── SensorNivel/SensorNivel.ino # nível da caixa (JSN-SR04T), bateria + deep sleep
```

**Protocolo** (mesmo formato da V8.0): `N1|GW|SEQ|TEL|pct=62;dist=87;vbat=3.98;err=ok|CRC`, respondido com `GW|N1|SEQ|ACK|OK|CRC`. O sensor tenta até 3 vezes; o gateway descarta reenvios do mesmo `SEQ` (só repete o ACK).

**Tópicos MQTT:**

| Tópico                  | Conteúdo                                                      |
|-------------------------|---------------------------------------------------------------|
| `lora/<SRC>/<chave>`    | Um tópico por campo do payload, só o valor, retido. Ex. `lora/N1/pct` = `62`, `lora/N1/vbat` = `3.98` |
| `lora/gateway/status`   | `online` / `offline` (LWT), retido                            |
| `lora/gateway/info`     | JSON com uptime, IP e contadores, a cada 60 s                 |

**Ligações (ajuste nos `#define` se a sua placa diferir):**

| Gateway — W5500 | GPIO |   | Sensor — JSN-SR04T | GPIO |
|-----------------|------|---|--------------------|------|
| SCK             | 47   |   | TRIG               | 47   |
| MISO            | 48   |   | ECHO               | 48   |
| MOSI            | 33   |   | VCC                | Vext (3,3 V) |
| CS              | 34   |   | GND                | GND  |
| INT             | 4    |   |                    |      |
| RST             | 6    |   |                    |      |

**Requisitos:** ESP32 Arduino core **3.x** (para `ETH.h` com W5500), biblioteca Heltec ESP32 Dev-Boards e **PubSubClient**. Antes de compilar o gateway, copie `Gateway/secrets.example.h` para `Gateway/secrets.h` e preencha o broker (o `secrets.h` fica fora do git).

**Adicionar um sensor:** grave com outro `MY_ADDRESS` (ex. `N2`); o gateway publica em `lora/N2/<chave>` sem nenhuma alteração. Sensores de outro tipo seguem o mesmo formato `chave=valor`, ex. `temp=24.5;vbat=4.01` ou `va=220.1;vb=219.8;vc=221.0`. Para o display do gateway mostrar a unidade, inclua a chave na tabela `UNITS` do `Gateway.ino` (o MQTT sempre leva o número puro).

**Display do gateway:** uma tela por sensor, em rodízio a cada 5 s, com RSSI, idade da última leitura, valores com unidade e `vbat` (útil para acompanhar a carga pelo painel solar da V4).

## Histórico de versões

| Versão      | Principal mudança                                                                                      |
|-------------|-----------------------------------------------------------------------------------------------------------|
| V1.0        | Biblioteca `heltec-unofficial`, classe `String` do Arduino, peer único, mensagens PING/PONG/STATUS       |
| V2.0 – V3.0 | Migração para `LoRaWan_APP.h` oficial, arrays de char em estilo C, padrão de flags em callback            |
| V4.0 – V5.0 | Adição de watchdog, correção de timing com `TX_GUARD_MS`                                                  |
| V6.0 – V7.0 | Struct de estado por peer, fila de envio por bitmask, SF7 (mais rápido, menor alcance que o SF9 da V1)    |
| V8.0        | `PEER_LIST` multi-bomba, persistência em EEPROM na Bomba, watchdog ciente da EEPROM                       |

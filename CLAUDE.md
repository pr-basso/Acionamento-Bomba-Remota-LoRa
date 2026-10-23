# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Arduino firmware for a LoRa-based remote water pump control system. A float sensor node (Node A) reads water level and sends commands via LoRa radio to one or more pump controller nodes (Nodes B/C/D) that operate relay modules. Hardware is Heltec WiFi LoRa 32 (V2 or V3) — an ESP32 board with integrated LoRa radio and 128×64 OLED display.

## Build and Flash

Use **Arduino IDE** or **arduino-cli**. No Makefile or build scripts exist — this is a standard Arduino project.

Required board package: **Heltec ESP32** (official, adds `LoRaWan_APP.h` and `HT_SSD1306Wire.h`).

```bash
# Install board via arduino-cli
arduino-cli core install Heltec-esp32:esp32

# Compile (example for Bomba node)
arduino-cli compile --fqbn Heltec-esp32:esp32:WIFI_LoRa_32_V2 "V8.0/Bomba"

# Upload
arduino-cli upload -p /dev/cu.usbserial-XXXX --fqbn Heltec-esp32:esp32:WIFI_LoRa_32_V2 "V8.0/Bomba"

# Monitor serial output (115200 baud)
arduino-cli monitor -p /dev/cu.usbserial-XXXX -b 115200
```

**V1.0 only** uses the `heltec-unofficial` library (`heltec.h`) instead of the official Heltec library. All later versions use `LoRaWan_APP.h`.

## Repository Structure

Each version folder (`V1.0` through `V8.0`) contains two independent firmware sketches:

- `Boia/Boia.ino` — **Node A**: Float sensor. Reads water level via digital pin, sends `CMD` messages to all pump nodes, waits for `ACK`.
- `Bomba/Bomba.ino` — **Node B/C/D**: Pump controller. Listens for `CMD`, actuates relay, sends `ACK` with relay status.

**V8.0 is the current/latest version.** Earlier versions are kept for reference.

## Protocol

Message format (pipe-separated, 6 fields):

```
SRC|DST|SEQ|TYPE|PAYLOAD|CRC
```

- `SEQ`: 4 hex digits, increments per CMD
- `TYPE`: `CMD` (A→pump nodes) or `ACK` (pump node→A)
- `CRC`: 2 hex digits, XOR of all bytes in `SRC|DST|SEQ|TYPE|PAYLOAD`
- CMD payload: `PUMP=ON` or `PUMP=OFF`
- ACK payload: `RELAY=1;PUMP=ON` (reports physical relay state)

## LoRa Parameters (V8.0)

All nodes must use identical radio settings. They live in one place, `V8.0/common/LoraConfig.h`, included by both sketches via symlinks (`Boia/LoraConfig.h`, `Bomba/LoraConfig.h`) because Arduino only compiles files inside the sketch folder. Change radio params only there.
- Frequency: 915 MHz
- SF9, BW 125 kHz, CR 4/5, Preamble 8 (~250 ms airtime per packet)
- TX power: 14 dBm
- `LORA_IQ_INVERSION_ON = false`

## Key Architecture Points (V8.0)

**Multi-pump fan-out** (`Boia.ino`): Node A maintains a `peers[]` table and a bitmask send queue. CMDs are sent sequentially (one peer at a time, not simultaneously) to avoid RF collisions. `INTER_PEER_MS = 2000` ensures the previous ACK has been received before the next CMD goes out.

**Radio ISR discipline**: All LoRa callbacks (`OnTxDone`, `OnRxDone`, `OnRxError`) only set `volatile bool` flags. All actual radio calls (`Radio.Rx(0)`, `Radio.Sleep()`) happen in `loop()`. This avoids calling radio APIs from interrupt context.

**TX→RX timing** (`Bomba.ino`): After receiving a CMD, the pump node waits `TX_GUARD_MS = 100ms` before sending ACK. This gives Node A time to finish its TX and open RX after `OnTxDone`. Without this guard, the ACK would arrive before Node A is listening.

**EEPROM persistence** (`Bomba.ino`, V8.0+): Relay state is saved to EEPROM on every change. On reboot, last state is restored. If restored as ON, the watchdog is suspended until the first real CMD arrives (to avoid a spurious watchdog shutdown after a brief power interruption).

**Watchdog** (`Bomba.ino`): Fires after `WATCHDOG_MS = 600000ms` (10 min) without a valid CMD while pump is ON. Before shutting down, it re-reads EEPROM: if EEPROM says ON (last CMD was ON and link is just temporarily down), it resets the watchdog and keeps the pump running.

**Adding a new pump node**: In `V8.0/Boia/Boia.ino`, add the node's address string to `PEER_LIST`. Flash the new Bomba board with `MY_ADDRESS` set to that address. No other changes needed.

## Telemetria (LoRa → MQTT)

Separate subsystem in `Telemetria/`, independent of the pump network, on **Heltec WiFi LoRa 32 V4** boards (ESP32-S3 + SX1262 + FEM; `WIFI_LORA_32_V4` and `USE_GC1109_PA`/`USE_KCT8103L_PA` come from the board selection). **Tools > LoRa FEM Type must match the board's FEM chip** (fqbn `...:heltec_wifi_lora_32_V4:LORA_FEM_TYPE=2` for GC1109, `=1` for KCT8103L, plus `LORA_RX_LNA=1` = Enabled); the default `USE_NONE_PA` never powers the FEM and range drops to ~10 m, so `TelemetriaConfig.h` `#error`s on it. `TX_OUTPUT_POWER` is dBm at the FEM output (the lib converts).

- `common/TelemetriaConfig.h` — radio params (920 MHz, SF9, BW125, CR4/5) and shared protocol helpers (`calcCRC`, `buildMsg`, `parseMsg`), symlinked into each sketch like V8.0's `LoraConfig.h`.
- `Gateway/Gateway.ino` — receives `TEL`, replies `ACK` after `TX_GUARD_MS`, dedups by (SRC, SEQ), publishes each payload field as its own retained topic with the bare value (`lora/<SRC>/<key>`, e.g. `lora/N1/pct` = `62`), so new sensor types (temp, phase voltages) need no gateway change. OLED rotates one screen per known sensor (values with units from the `UNITS` table, plus `vbat`). W5500 on a **second SPI bus (SPI3_HOST)** via core 3.x `ETH.h`, because the LoRa SPI (GPIO 9–11) is not on the V4 headers (SCK 47, MISO 48, MOSI 41, CS 34, INT 4, RST 6; `W5500Teste/` diagnoses wiring and PHY). `ETH_FORCE_10M` forces 10M half (current module would not hold a 100M link). `#define USE_WIFI` at the top of the sketch swaps to the board's WiFi (`WIFI_SSID`/`WIFI_PASS` in `secrets.h`) for bench testing. Broker config in gitignored `secrets.h` (template: `secrets.example.h`). Publishes queue in RAM (16) while MQTT is down. A DS18B20 on GPIO 3 (OneWire, 4.7k pull-up, non-blocking conversion) publishes the gateway's own temperature every 60 s to the fixed retained topic `geral/temperatura` (outside `MQTT_TOPIC_BASE`, not queued) and shows it on the OLED header.
- `SensorNivel/SensorNivel.ino` — whole cycle runs in `setup()`: power JSN-SR04T via Vext, median of 5 readings, read VBAT (GPIO1, ADC_CTRL GPIO37 HIGH), send with up to 3 retries, deep sleep `SLEEP_MINUTES`. Payload `pct=..;dist=..;vbat=..;err=ok` (`err=eco;vbat=..` on failed reading; `err` always sent so the retained topic clears). `seq` lives in `RTC_DATA_ATTR`. Vext is held off during sleep with `gpio_hold_en` + `gpio_deep_sleep_hold_en`.
- `Display/Display.ino` — read-only MQTT dashboard on a **Waveshare ESP32-S3-Touch-LCD-4.3B** (800×480 RGB, GT911, CH422G), WiFi only. Uses the **Espressif `esp32` core 3.x, not Heltec's**: fqbn `esp32:esp32:waveshare_esp32_s3_touch_lcd_43B:PSRAM=enabled,PartitionScheme=app3M_fat9M_16MB,USBMode=hwcdc,CDCOnBoot=cdc` (PSRAM=enabled is OPI; without CDC On Boot, Serial goes to UART0 = the board's RS485). The lib logs via printf, so `setup()` redirects stdout to USB with `fwopen`. The global `ESP32_Display_Panel/esp_panel_drivers_conf.h` must enable RGB/I2C bus, ST7262, GT911, CH422G and SWITCH_EXPANDER backlight by name (`*_USE_ALL` is ignored in Arduino builds), otherwise boot fails with "Unknown controller: ST7262". ESP32_Display_Panel 1.x with board pins from `BoardConfig.cpp` (copy of the lib's `board_dynamic_config` example pointing at the 4.3B header, so the global lib stays untouched) + LVGL 9 with an in-sketch port (flush/touch/tick in `loop()`, no LVGL task). The global `~/Documents/Arduino/libraries/lv_conf.h` is the v8.4 template; LVGL 9 fills the rest with defaults (48 KB builtin heap). Subscribes `<BASE>/#` + `geral/temperatura`; one card per SRC (SRC may contain `/`, key = after the last `/`), built dynamically. LVGL built-in fonts are ASCII + `°` only, so UI strings have no accents. Messages within `RETAINED_WIN_MS` of subscribe count as retained (age unknown) since PubSubClient hides the retain flag. Own gitignored `secrets.h` (client id `lora-display`).
- `common/TelemetriaUnits.h` — `UNITS` table + `unitFor()`, symlinked into Gateway and Display (kept out of `TelemetriaConfig.h`, which `#error`s off the V4).
- V4 reserved GPIOs: 1 (VBAT), 2/5/7/46 (FEM), 8–14 (LoRa), 17/18/21 (OLED), 19/20 (USB), 26–32 (flash/PSRAM), 36 (Vext), 37 (ADC_Ctrl).

## Version History Summary

| Version | Key change |
|---------|-----------|
| V1.0 | `heltec-unofficial` lib, Arduino `String` class, single peer, PING/PONG/STATUS messages |
| V2.0–V3.0 | Switched to official `LoRaWan_APP.h`, C-style char arrays, callback-flag pattern |
| V4.0–V5.0 | Added watchdog, TX_GUARD_MS timing fix |
| V6.0–V7.0 | Per-peer state struct, bitmask send queue, SF7 (faster, lower range than V1's SF9) |
| V8.0 | Multi-pump PEER_LIST, EEPROM persistence in Bomba, EEPROM-aware watchdog |

## Hardware Notes

- `RELAY_PIN = 26`, logic **inverted**: `RELAY_ON = LOW`, `RELAY_OFF = HIGH` (standard blue relay modules)
- `BOIA_PIN = 2`, `INPUT_PULLUP`: `LOW` = tank empty (float open), `HIGH` = tank full (float closed)
- Display: SSD1306 128×64 OLED, I2C at 0x3C, controlled via `HT_SSD1306Wire`
- `Vext` pin must be pulled LOW to power the OLED on Heltec boards

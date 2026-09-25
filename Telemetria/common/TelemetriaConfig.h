/**
 * ============================================================
 *  REDE DE TELEMETRIA LoRa — CONFIGURAÇÃO COMPARTILHADA
 *  Hardware : Heltec WiFi LoRa 32 V4 (ESP32-S3 + SX1262 + FEM)
 *
 *  Fonte única dos parâmetros de rádio e do protocolo para o
 *  Gateway e todos os sensores. Cada sketch acessa este arquivo
 *  por link simbólico (<Sketch>/TelemetriaConfig.h ->
 *  ../common/TelemetriaConfig.h).
 *
 *  REDE SEPARADA da rede Boia/Bomba (V8.0, 915 MHz): usa outra
 *  frequência, então um sistema não escuta o outro.
 *
 *  PROTOCOLO (mesmo formato da V8.0, separador "|"):
 *    TEL : <SRC>|GW|<SEQ>|TEL|k=v;k=v;...|CRC   (sensor -> gateway)
 *    ACK : GW|<SRC>|<SEQ>|ACK|OK|CRC            (gateway -> sensor)
 *  CRC = XOR de todos os bytes de "SRC|DST|SEQ|TYPE|PAYLOAD".
 *  Chaves do payload viram tópicos MQTT (lora/<SRC>/<chave>): use
 *  nomes curtos e estáveis — pct, dist, temp, umid, va/vb/vc, vbat, err.
 * ============================================================
 */

#ifndef TELEMETRIA_CONFIG_H
#define TELEMETRIA_CONFIG_H

#include <Arduino.h>

#if !defined(WIFI_LORA_32_V4)
#error "Selecione a placa Heltec WiFi LoRa 32 V4"
#endif
#if !defined(USE_GC1109_PA) && !defined(USE_KCT8103L_PA)
#warning "FEM da V4 nao definido (USE_GC1109_PA ou USE_KCT8103L_PA) - confira a opcao da placa"
#endif

// ── Parâmetros LoRa ────────────────────────────────────────
// Faixa 915–928 MHz (ANATEL). 920 MHz fica longe dos 915 MHz
// da rede da bomba.
#define RF_FREQUENCY          920000000
// Potência entregue pelo SX1262. O FEM da V4 soma ganho a isso,
// então a saída real na antena é maior. Comece baixo e suba só
// se o link precisar (limite ANATEL: 30 dBm EIRP).
#define TX_OUTPUT_POWER               10   // dBm no SX1262
#define LORA_BANDWIDTH                0    // 0 = 125 kHz
#define LORA_SPREADING_FACTOR         9    // ~350ms no ar para ~55 bytes
#define LORA_CODINGRATE               1    // 1 = 4/5
#define LORA_PREAMBLE_LENGTH          8
#define LORA_SYMBOL_TIMEOUT           0
#define LORA_IQ_INVERSION_ON      false

// ── Protocolo ─────────────────────────────────────────────
#define GATEWAY_ADDRESS  "GW"
#define BUFFER_SIZE      128
#define TOTAL_FIELDS       6
#define FIELD_SIZE        80   // PAYLOAD de telemetria pode ter ~60 chars

// Tempo de guarda antes do gateway responder o ACK — o sensor
// abre RX no OnTxDone (mesmo raciocínio do TX_GUARD_MS da V8.0).
#define TX_GUARD_MS      100

static uint8_t calcCRC(const char *s) {
  uint8_t c = 0;
  while (*s) c ^= (uint8_t)(*s++);
  return c;
}

static void buildMsg(char *out, const char *src, const char *dst,
                     uint16_t seq, const char *type, const char *pay) {
  char body[BUFFER_SIZE];
  snprintf(body, BUFFER_SIZE, "%s|%s|%04X|%s|%s", src, dst, seq, type, pay);
  snprintf(out, BUFFER_SIZE, "%s|%02X", body, calcCRC(body));
}

// Separa os 6 campos e valida o CRC. Não filtra destino.
static bool parseMsg(const char *raw, char fields[][FIELD_SIZE]) {
  char tmp[BUFFER_SIZE];
  strncpy(tmp, raw, BUFFER_SIZE - 1); tmp[BUFFER_SIZE - 1] = '\0';
  char *ptr = tmp;
  for (int i = 0; i < TOTAL_FIELDS; i++) {
    char *sep = (i < TOTAL_FIELDS - 1) ? strchr(ptr, '|') : nullptr;
    if (i < TOTAL_FIELDS - 1 && !sep) return false;
    if (sep) *sep = '\0';
    strncpy(fields[i], ptr, FIELD_SIZE - 1); fields[i][FIELD_SIZE - 1] = '\0';
    if (sep) ptr = sep + 1;
  }
  char body[BUFFER_SIZE];
  snprintf(body, BUFFER_SIZE, "%s|%s|%s|%s|%s",
           fields[0], fields[1], fields[2], fields[3], fields[4]);
  if (calcCRC(body) != (uint8_t)strtol(fields[5], nullptr, 16)) {
    Serial.println("[PARSE] CRC erro");
    return false;
  }
  return true;
}

#endif

/**
 * ============================================================
 *  NODE B — CONTROLADOR DE BOMBA D'ÁGUA (Relé + Contatora)
 *  Hardware : Heltec WiFi LoRa 32 (V2 ou V3)
 *  Biblioteca: Heltec ESP32 (instalar via Arduino Library Manager)
 *              https://github.com/HelTecAutomation/Heltec_ESP32
 *
 *  ⚠️  NÃO usa heltec.h — usa LoRaWan_APP.h (driver nativo Heltec)
 *
 *  Função: Recebe comandos CMD do Node A e aciona o relé que
 *          controla a contatora da bomba d'água.
 *
 *  HARDWARE DO RELÉ:
 *    GPIO RELAY_PIN → módulo relé → bobina da contatora → motor
 *    RELAY_ON  = LOW  (módulo de relé com lógica ativa em LOW — padrão)
 *    RELAY_OFF = HIGH
 *    Se o seu módulo for ativo em HIGH, inverta RELAY_ON e RELAY_OFF.
 *
 *  PROTEÇÕES:
 *    • Watchdog: sem comunicação por WATCHDOG_MS → desliga bomba
 *    • Tempo mínimo ligado para proteger o motor
 *    • Reporta STATUS ao Node A após cada CMD e periodicamente
 *
 *  PROTOCOLO LoRa — string com separador "|"
 *  ┌──────┬──────┬──────┬────────┬────────────────────────┬─────┐
 *  │ SRC  │ DST  │ SEQ  │  TYPE  │        PAYLOAD          │ CRC │
 *  ├──────┼──────┼──────┼────────┼────────────────────────┼─────┤
 *  │  A   │  B   │ 0001 │ CMD    │ PUMP=ON;BOIA=BAIXO      │ XX  │
 *  │  B   │  A   │ 0001 │ ACK    │                        │ XX  │
 *  │  B   │  A   │ 0010 │ STATUS │ PUMP=ON;RELAY=1         │ XX  │
 *  │  A   │  B   │ 0003 │ PING   │                        │ XX  │
 *  │  B   │  A   │ 0003 │ PONG   │                        │ XX  │
 *  └──────┴──────┴──────┴────────┴────────────────────────┴─────┘
 * ============================================================
 */

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"   // display OLED nativo Heltec

// ── Identificação ──────────────────────────────────────────
#define MY_ADDRESS   "B"
#define PEER_ADDRESS "A"

// ── Hardware ───────────────────────────────────────────────
#define RELAY_PIN    26      // GPIO para o módulo relé
                             // Pinos livres no Heltec V2: 13,26,34,35,36,39
                             // Pinos livres no Heltec V3: 1,2,3,4,5,6,7,46
#define RELAY_ON     LOW     // LOW ativa o relé (lógica invertida — padrão)
#define RELAY_OFF    HIGH    // HIGH desativa

// ── Parâmetros LoRa (IDÊNTICOS ao Node A) ─────────────────
#define RF_FREQUENCY          915000000
#define TX_OUTPUT_POWER       14
#define LORA_BANDWIDTH        0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE       1
#define LORA_PREAMBLE_LENGTH  8
#define LORA_SYMBOL_TIMEOUT   0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON  false
#define RX_TIMEOUT_VALUE      1000

// ── Buffer ─────────────────────────────────────────────────
#define BUFFER_SIZE  128
char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

// ── Display OLED ───────────────────────────────────────────
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED,
                           GEOMETRY_128_64, RST_OLED);

// ── Protocolo ─────────────────────────────────────────────
#define SEP           "|"
#define FIELD_SRC     0
#define FIELD_DST     1
#define FIELD_SEQ     2
#define FIELD_TYPE    3
#define FIELD_PAYLOAD 4
#define FIELD_CRC     5
#define TOTAL_FIELDS  6

// ── Máquina de estados do rádio (nome próprio para evitar conflito) ───
typedef enum {
  APP_STATE_TX,
  APP_STATE_RX,
  APP_STATE_TX_TIMEOUT,
  APP_STATE_RX_TIMEOUT,
  APP_STATE_RX_ERROR,
  APP_STATE_LOWPOWER
} AppRadioState_t;
static volatile AppRadioState_t radioState = APP_STATE_LOWPOWER;
static RadioEvents_t RadioEvents;


// ── Estado da aplicação ───────────────────────────────────
uint16_t      txSeq        = 0;
uint16_t      lastRxSeq    = 0xFFFF;
bool          waitingAck   = false;
char          pendingMsg[BUFFER_SIZE];
uint8_t       retries      = 0;
unsigned long lastAckReq   = 0;
unsigned long lastStatusTx = 0;
unsigned long lastRxTime   = 0;   // watchdog

bool          pumpState    = false;
unsigned long pumpOnTime   = 0;

// Temporização
#define ACK_TIMEOUT_MS     3000
#define MAX_RETRIES        3
#define STATUS_INTERVAL_MS 20000
#define WATCHDOG_MS        60000
#define MIN_PUMP_ON_MS     8000

// ══════════════════════════════════════════════════════════
//  DISPLAY OLED
//  Layout (128×64):
//   linha 0  (y= 0): "NODE B | BOMBA"      — título fixo
//   linha 1  (y=16): estado do relé / bomba
//   linha 2  (y=28): tempo ligada (se ON) ou "Relé: OFF"
//   linha 3  (y=40): último evento recebido
//   linha 4  (y=52): RSSI / watchdog / retry
// ══════════════════════════════════════════════════════════

void updateDisplay(const char *evento = "", int rssi = 0) {
  display.clear();
  display.setFont(ArialMT_Plain_10);

  // ── Título ──────────────────────────────────────────────
  display.drawString(0, 0, "NODE B | BOMBA");
  display.drawHorizontalLine(0, 12, 128);

  // ── Estado da bomba ─────────────────────────────────────
  display.drawString(0, 16,
    pumpState ? "Bomba: LIGADA  [ON]" : "Bomba: DESLIGADA");

  // ── Tempo ligada ────────────────────────────────────────
  if (pumpState) {
    char buf[28];
    unsigned long sec = (millis() - pumpOnTime) / 1000;
    snprintf(buf, sizeof(buf), "Tempo ON: %lus", sec);
    display.drawString(0, 28, buf);
  } else {
    display.drawString(0, 28, "Rele: OFF");
  }

  // ── Último evento ───────────────────────────────────────
  if (strlen(evento) > 0) {
    display.drawString(0, 40, evento);
  }

  // ── Rodapé ──────────────────────────────────────────────
  if (rssi != 0) {
    char buf[24];
    snprintf(buf, sizeof(buf), "RSSI: %d dBm", rssi);
    display.drawString(0, 52, buf);
  } else if (waitingAck) {
    char buf[28];
    snprintf(buf, sizeof(buf), "Aguard. ACK  retry %d/%d",
             retries, MAX_RETRIES);
    display.drawString(0, 52, buf);
  }

  display.display();
}

// ══════════════════════════════════════════════════════════
//  CONTROLE DO RELÉ
// ══════════════════════════════════════════════════════════

void pumpOn() {
  if (pumpState) return;
  digitalWrite(RELAY_PIN, RELAY_ON);
  pumpState = true;
  pumpOnTime = millis();
  Serial.println("[RELE] LIGADO — bomba acionada.");
  updateDisplay("CMD recebido: LIGAR");
}

void pumpOff() {
  if (pumpState && (millis() - pumpOnTime < MIN_PUMP_ON_MS)) {
    Serial.println("[RELE] Aguardando tempo mínimo para desligar...");
    updateDisplay("Aguard. t.min p/desligar");
    return;
  }
  digitalWrite(RELAY_PIN, RELAY_OFF);
  pumpState = false;
  Serial.println("[RELE] DESLIGADO — bomba parada.");
  updateDisplay("CMD recebido: DESLIGAR");
}

// ══════════════════════════════════════════════════════════
//  CALLBACKS DO RÁDIO
// ══════════════════════════════════════════════════════════

void OnTxDone(void) {
  radioState = APP_STATE_TX;
}

void OnTxTimeout(void) {
  Radio.Sleep();
  radioState = APP_STATE_TX_TIMEOUT;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  memset(rxpacket, 0, BUFFER_SIZE);
  memcpy(rxpacket, payload, size < BUFFER_SIZE - 1 ? size : BUFFER_SIZE - 1);
  Radio.Sleep();
  Serial.printf("[RX] \"%s\"  RSSI=%d SNR=%d\n", rxpacket, rssi, snr);
  lastRxTime = millis();
  radioState = APP_STATE_RX;
}

void OnRxTimeout(void) {
  Radio.Sleep();
  radioState = APP_STATE_RX_TIMEOUT;
}

void OnRxError(void) {
  Radio.Sleep();
  radioState = APP_STATE_RX_ERROR;
}

// ══════════════════════════════════════════════════════════
//  UTILITÁRIOS DE PROTOCOLO
// ══════════════════════════════════════════════════════════

uint8_t calcCRC(const char *s, int len) {
  uint8_t crc = 0;
  for (int i = 0; i < len; i++) crc ^= (uint8_t)s[i];
  return crc;
}

void buildMsg(char *dest, const char *type, const char *payload, uint16_t seq) {
  char body[BUFFER_SIZE];
  snprintf(body, BUFFER_SIZE, "%s|%s|%04X|%s|%s",
           MY_ADDRESS, PEER_ADDRESS, seq, type, payload);
  uint8_t crc = calcCRC(body, strlen(body));
  snprintf(dest, BUFFER_SIZE, "%s|%02X", body, crc);
}

bool parseMsg(const char *raw, char fields[][32]) {
  char tmp[BUFFER_SIZE];
  strncpy(tmp, raw, BUFFER_SIZE - 1);
  tmp[BUFFER_SIZE - 1] = '\0';

  char *ptr = tmp;
  for (int i = 0; i < TOTAL_FIELDS; i++) {
    char *sep = (i < TOTAL_FIELDS - 1) ? strchr(ptr, '|') : nullptr;
    if (i < TOTAL_FIELDS - 1 && sep == nullptr) return false;
    if (sep) *sep = '\0';
    strncpy(fields[i], ptr, 31);
    fields[i][31] = '\0';
    if (sep) ptr = sep + 1;
  }

  char body[BUFFER_SIZE];
  snprintf(body, BUFFER_SIZE, "%s|%s|%s|%s|%s",
           fields[0], fields[1], fields[2], fields[3], fields[4]);
  uint8_t expected = calcCRC(body, strlen(body));
  uint8_t received = (uint8_t)strtol(fields[FIELD_CRC], nullptr, 16);
  if (expected != received) {
    Serial.printf("[ERRO] CRC esperado=%02X recebido=%02X\n", expected, received);
    return false;
  }
  if (strcmp(fields[FIELD_DST], MY_ADDRESS) != 0) return false;
  return true;
}

// ══════════════════════════════════════════════════════════
//  ENVIO LoRa
// ══════════════════════════════════════════════════════════

void sendRaw(const char *msg) {
  Serial.printf("[TX] \"%s\"\n", msg);
  Radio.Send((uint8_t *)msg, strlen(msg));
}

void sendAck(uint16_t seq) {
  char msg[BUFFER_SIZE];
  buildMsg(msg, "ACK", "", seq);
  sendRaw(msg);
}

void sendPong(uint16_t seq) {
  char msg[BUFFER_SIZE];
  buildMsg(msg, "PONG", "", seq);
  sendRaw(msg);
}

void sendStatus() {
  char payload[32];
  snprintf(payload, sizeof(payload), "PUMP=%s;RELAY=%d",
           pumpState ? "ON" : "OFF", pumpState ? 1 : 0);
  buildMsg(pendingMsg, "STATUS", payload, txSeq++);
  waitingAck = true;
  retries    = 0;
  lastAckReq = millis();
  sendRaw(pendingMsg);
}

// ══════════════════════════════════════════════════════════
//  PROCESSAMENTO DE MENSAGEM RECEBIDA
// ══════════════════════════════════════════════════════════

void handleReceived() {
  char fields[TOTAL_FIELDS][32];
  if (!parseMsg(rxpacket, fields)) {
    updateDisplay("ERRO: CRC invalido");
    return;
  }

  uint16_t seq     = (uint16_t)strtol(fields[FIELD_SEQ], nullptr, 16);
  const char *type = fields[FIELD_TYPE];
  const char *pay  = fields[FIELD_PAYLOAD];

  // ── ACK (confirmação do nosso STATUS) ─────────────────
  if (strcmp(type, "ACK") == 0) {
    if (waitingAck) {
      waitingAck = false;
      Serial.println("[ACK] STATUS confirmado pelo Node A.");
      updateDisplay("ACK: STATUS confirmado");
    }
    return;
  }

  // ── PONG ──────────────────────────────────────────────
  if (strcmp(type, "PONG") == 0) {
    Serial.println("[PONG] Link OK");
    updateDisplay("PONG: link OK");
    return;
  }

  // Descarta duplicatas
  if (seq == lastRxSeq) {
    sendAck(seq);
    return;
  }
  lastRxSeq = seq;

  // ── PING ──────────────────────────────────────────────
  if (strcmp(type, "PING") == 0) {
    sendPong(seq);
    updateDisplay("PING recebido > PONG");
    return;
  }

  // ── CMD (comando principal do Node A) ─────────────────
  if (strcmp(type, "CMD") == 0) {
    sendAck(seq);   // ACK imediato

    bool cmdOn = (strstr(pay, "PUMP=ON") != nullptr);
    const char *boiaInfo = strstr(pay, "BOIA=");
    Serial.printf("[CMD] %s bomba  (%s)\n",
                  cmdOn ? "LIGAR" : "DESLIGAR",
                  boiaInfo ? boiaInfo : "");

    // pumpOn()/pumpOff() já chamam updateDisplay internamente
    if (cmdOn) pumpOn();
    else       pumpOff();

    delay(50);
    if (!waitingAck) sendStatus();
    return;
  }
}

// ══════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
  Serial.println("\nNODE B — Controlador de Bomba  [LoRaWan_APP]");

  // ── Inicializa display OLED ─────────────────────────────
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
  delay(100);
  display.init();
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0,  "NODE B | iniciando...");
  display.drawString(0, 16, "LoRa 915 MHz  SF9");
  display.drawString(0, 28, "Rele: OFF");
  display.display();

  // ── Relé começa DESLIGADO por segurança ────────────────
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF);
  pumpState  = false;
  lastRxTime = millis();

  // ── Rádio ──────────────────────────────────────────────
  RadioEvents.TxDone    = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone    = OnRxDone;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError   = OnRxError;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);

  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0,
                    LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, LORA_PREAMBLE_LENGTH,
                    LORA_FIX_LENGTH_PAYLOAD_ON, true,
                    0, 0, LORA_IQ_INVERSION_ON, 3000);

  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true,
                    0, 0, LORA_IQ_INVERSION_ON, true);

  Radio.SetMaxPayloadLength(MODEM_LORA, BUFFER_SIZE);

  Radio.Rx(0);
  radioState = APP_STATE_LOWPOWER;

  Serial.println("Rele: OFF — aguardando CMD do Node A...");
  updateDisplay("Aguardando CMD...");
}

// ══════════════════════════════════════════════════════════
//  LOOP
// ══════════════════════════════════════════════════════════

void loop() {
  unsigned long now = millis();

  // ── Máquina de estados do rádio ──────────────────────
  switch (radioState) {

    case APP_STATE_TX:
      Radio.Rx(0);
      radioState = APP_STATE_LOWPOWER;
      break;

    case APP_STATE_RX:
      handleReceived();   // updateDisplay() chamado internamente
      Radio.Rx(0);
      radioState = APP_STATE_LOWPOWER;
      break;

    case APP_STATE_TX_TIMEOUT:
      Serial.println("[WARN] TX timeout");
      updateDisplay("WARN: TX timeout");
      Radio.Rx(0);
      radioState = APP_STATE_LOWPOWER;
      break;

    case APP_STATE_RX_TIMEOUT:
    case APP_STATE_RX_ERROR:
      Radio.Rx(0);
      radioState = APP_STATE_LOWPOWER;
      break;

    case APP_STATE_LOWPOWER:
    default:
      break;
  }

  // ── Watchdog de comunicação ───────────────────────────
  if (pumpState && (now - lastRxTime >= WATCHDOG_MS)) {
    Serial.printf("[WATCHDOG] Sem comunicacao por %lus — DESLIGANDO!\n",
                  WATCHDOG_MS / 1000);
    digitalWrite(RELAY_PIN, RELAY_OFF);
    pumpState = false;
    lastRxTime = now;
    updateDisplay("WATCHDOG: sem sinal!");
  }

  // ── Retransmissão de STATUS sem ACK ──────────────────
  if (waitingAck && (now - lastAckReq >= ACK_TIMEOUT_MS)) {
    if (radioState == APP_STATE_LOWPOWER) {
      if (retries < MAX_RETRIES) {
        retries++;
        lastAckReq = now;
        updateDisplay();   // mostra "Aguard. ACK retry N/M"
        Radio.Sleep();
        sendRaw(pendingMsg);
      } else {
        waitingAck = false;
        retries    = 0;
        Serial.println("[STATUS] Node A nao confirmou STATUS.");
        updateDisplay("Node A sem resposta");
        Radio.Rx(0);
        radioState = APP_STATE_LOWPOWER;
      }
    }
  }

  // ── Reporte periódico de STATUS ───────────────────────
  if (!waitingAck && (now - lastStatusTx >= STATUS_INTERVAL_MS)) {
    if (radioState == APP_STATE_LOWPOWER) {
      lastStatusTx = now;
      Radio.Sleep();
      sendStatus();
      updateDisplay("TX STATUS periodico");
    }
  }

  // ── Atualiza tempo ligado no display a cada 5 s ───────
  static unsigned long lastDisplayRefresh = 0;
  if (pumpState && (now - lastDisplayRefresh >= 5000)) {
    lastDisplayRefresh = now;
    updateDisplay();
  }

  Radio.IrqProcess();
}

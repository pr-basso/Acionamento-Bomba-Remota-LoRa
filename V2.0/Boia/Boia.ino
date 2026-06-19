/**
 * ============================================================
 *  NODE A — SENSOR DE BOIA (Caixa d'água)
 *  Hardware : Heltec WiFi LoRa 32 (V2 ou V3)
 *  Biblioteca: Heltec ESP32 (instalar via Arduino Library Manager)
 *              https://github.com/HelTecAutomation/Heltec_ESP32
 *
 *  ⚠️  NÃO usa heltec.h — usa LoRaWan_APP.h (driver nativo Heltec)
 *
 *  Função: Lê o estado da boia no pino GPIO 2 e envia comandos
 *          ao Node B para ligar/desligar a bomba d'água via LoRa.
 *
 *  LÓGICA DE CONTROLE:
 *    Boia ABERTA  (pino LOW  com pull-up) = tanque VAZIO  → liga bomba
 *    Boia FECHADA (pino HIGH com pull-up) = tanque CHEIO  → desliga bomba
 *
 *  PROTOCOLO LoRa — string com separador "|"
 *  ┌──────┬──────┬──────┬────────┬────────────────────────┬─────┐
 *  │ SRC  │ DST  │ SEQ  │  TYPE  │        PAYLOAD          │ CRC │
 *  ├──────┼──────┼──────┼────────┼────────────────────────┼─────┤
 *  │  A   │  B   │ 0001 │ CMD    │ PUMP=ON;BOIA=BAIXO      │ XX  │
 *  │  A   │  B   │ 0002 │ CMD    │ PUMP=OFF;BOIA=CHEIO     │ XX  │
 *  │  A   │  B   │ 0003 │ PING   │                        │ XX  │
 *  │  B   │  A   │ 0001 │ ACK    │                        │ XX  │
 *  │  B   │  A   │ 0003 │ PONG   │                        │ XX  │
 *  │  B   │  A   │ 0010 │ STATUS │ PUMP=ON;RELAY=1         │ XX  │
 *  └──────┴──────┴──────┴────────┴────────────────────────┴─────┘
 *
 *  Tipos de mensagem:
 *    CMD    → Node A ordena ligar/desligar a bomba
 *    ACK    → Confirmação de recebimento (sem payload)
 *    PING   → Verifica link
 *    PONG   → Resposta ao PING
 *    STATUS → Node B reporta estado atual do relé
 * ============================================================
 */

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"   // display OLED nativo Heltec

// ── Identificação ──────────────────────────────────────────
#define MY_ADDRESS   "A"
#define PEER_ADDRESS "B"

// ── Hardware ───────────────────────────────────────────────
#define BOIA_PIN     2       // GPIO2 = D2; boia com pull-up interno
//  Boia normalmente fechada (NC) — padrão de mercado:
//    LOW  (boia aberta)  = tanque VAZIO  → ligar bomba
//    HIGH (boia fechada) = tanque CHEIO  → desligar bomba
#define BOIA_VAZIO   LOW
#define BOIA_CHEIO   HIGH

// ── Parâmetros LoRa ────────────────────────────────────────
#define RF_FREQUENCY          915000000   // 915 MHz (Brasil/EUA)
#define TX_OUTPUT_POWER       14          // dBm
#define LORA_BANDWIDTH        0           // 0=125kHz 1=250kHz 2=500kHz
#define LORA_SPREADING_FACTOR 7           // SF9
#define LORA_CODINGRATE       1           // 1=4/5
#define LORA_PREAMBLE_LENGTH  8
#define LORA_SYMBOL_TIMEOUT   0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON  false
#define RX_TIMEOUT_VALUE      1000        // ms — retorna ao loop após 1s sem receber

// ── Tamanho do buffer ──────────────────────────────────────
#define BUFFER_SIZE  128
char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

// ── Display OLED ───────────────────────────────────────────
// Parâmetros fixos da placa Heltec WiFi LoRa 32 (V2 e V3)
// SDA_OLED, SCL_OLED, RST_OLED são definidos pela biblioteca
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
uint16_t      txSeq          = 0;
uint16_t      lastRxSeq      = 0xFFFF;
bool          waitingAck     = false;
char          pendingMsg[BUFFER_SIZE];
uint8_t       retries        = 0;
unsigned long lastAckRequest = 0;
unsigned long lastPing       = 0;

// Boia
int           boiaState      = BOIA_CHEIO;
int           boiaPrevState  = BOIA_CHEIO;
bool          boiaPending    = false;
unsigned long boiaChangedAt  = 0;

// Bomba
bool          pumpDesired    = false;
bool          pumpConfirmed  = false;
unsigned long lastPumpOn     = 0;
unsigned long lastPumpOff    = 0;

// Temporização
#define ACK_TIMEOUT_MS      3000
#define MAX_RETRIES         5
#define PING_INTERVAL_MS    15000
#define DEBOUNCE_MS         2000
#define MIN_PUMP_ON_MS      10000
#define MIN_PUMP_OFF_MS     5000

// ══════════════════════════════════════════════════════════
//  CALLBACKS DO RÁDIO (chamados por interrupção)
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

/**
 * Monta mensagem no buffer dest.
 * Formato: SRC|DST|SEQ|TYPE|PAYLOAD|CRC
 */
void buildMsg(char *dest, const char *type, const char *payload, uint16_t seq) {
  // Monta corpo sem CRC
  char body[BUFFER_SIZE];
  snprintf(body, BUFFER_SIZE, "%s|%s|%04X|%s|%s",
           MY_ADDRESS, PEER_ADDRESS, seq, type, payload);
  uint8_t crc = calcCRC(body, strlen(body));
  snprintf(dest, BUFFER_SIZE, "%s|%02X", body, crc);
}

/**
 * Parse da mensagem recebida.
 * Preenche fields[] e retorna true se CRC OK e destino correto.
 */
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

  // Verifica CRC: recalcula sobre SRC|DST|SEQ|TYPE|PAYLOAD
  char body[BUFFER_SIZE];
  snprintf(body, BUFFER_SIZE, "%s|%s|%s|%s|%s",
           fields[0], fields[1], fields[2], fields[3], fields[4]);
  uint8_t expected = calcCRC(body, strlen(body));
  uint8_t received = (uint8_t)strtol(fields[FIELD_CRC], nullptr, 16);
  if (expected != received) {
    Serial.printf("[ERRO] CRC esperado=%02X recebido=%02X\n", expected, received);
    return false;
  }

  // Verifica destino
  if (strcmp(fields[FIELD_DST], MY_ADDRESS) != 0) return false;

  return true;
}

// ══════════════════════════════════════════════════════════
//  DISPLAY OLED
//  Layout (128×64):
//   linha 0  (y= 0): "NODE A | BOIA"       — título fixo
//   linha 1  (y=16): estado da boia
//   linha 2  (y=28): comando desejado para a bomba
//   linha 3  (y=40): estado confirmado da bomba (pelo Node B)
//   linha 4  (y=52): último evento / RSSI / retry
// ══════════════════════════════════════════════════════════

void updateDisplay(const char *evento = "", int rssi = 0) {
  display.clear();
  display.setFont(ArialMT_Plain_10);

  // ── Título ──────────────────────────────────────────────
  display.drawString(0, 0, "NODE A | BOIA");
  display.drawHorizontalLine(0, 12, 128);

  // ── Estado da boia ──────────────────────────────────────
  display.drawString(0, 16,
    boiaState == BOIA_VAZIO ? "Boia: VAZIO  (tank low)" : "Boia: CHEIO  (tank full)");

  // ── Comando desejado ────────────────────────────────────
  display.drawString(0, 28,
    pumpDesired ? "CMD: LIGAR bomba" : "CMD: DESLIGAR bomba");

  // ── Estado confirmado ───────────────────────────────────
  display.drawString(0, 40,
    pumpConfirmed ? "Bomba: LIGADA  [OK]" : "Bomba: DESLIGADA [OK]");

  // ── Rodapé: evento ou RSSI ──────────────────────────────
  if (strlen(evento) > 0) {
    display.drawString(0, 52, evento);
  } else if (rssi != 0) {
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

void sendPumpCmd(bool pumpOn) {
  char boiaLabel[8];
  strcpy(boiaLabel, (boiaState == BOIA_VAZIO) ? "BAIXO" : "CHEIO");
  char payload[40];
  snprintf(payload, sizeof(payload), "PUMP=%s;BOIA=%s",
           pumpOn ? "ON" : "OFF", boiaLabel);
  buildMsg(pendingMsg, "CMD", payload, txSeq++);
  waitingAck     = true;
  retries        = 0;
  lastAckRequest = millis();
  sendRaw(pendingMsg);
  Serial.printf("[CMD] Enviando: %s\n", payload);
  char buf[32];
  snprintf(buf, sizeof(buf), "TX CMD: PUMP=%s", pumpOn ? "ON" : "OFF");
  updateDisplay(buf);
}

void sendPing() {
  char msg[BUFFER_SIZE];
  buildMsg(msg, "PING", "", txSeq++);
  sendRaw(msg);
}

// ══════════════════════════════════════════════════════════
//  LÓGICA DA BOIA
// ══════════════════════════════════════════════════════════

bool readBoia() {
  int current = digitalRead(BOIA_PIN);
  unsigned long now = millis();
  if (current != boiaPrevState && !boiaPending) {
    boiaPending   = true;
    boiaChangedAt = now;
    boiaState     = current;
  }
  if (boiaPending && (now - boiaChangedAt >= DEBOUNCE_MS)) {
    boiaPending = false;
    if (current == boiaPrevState) return false;
    boiaPrevState = current;
    boiaState     = current;
    return true;
  }
  return false;
}

void evaluatePump() {
  unsigned long now = millis();
  if (boiaState == BOIA_VAZIO) {
    if (!pumpDesired && (now - lastPumpOff >= MIN_PUMP_OFF_MS)) {
      pumpDesired = true;
      lastPumpOn  = now;
      Serial.println("[BOIA] VAZIO → solicitando LIGAR bomba");
      sendPumpCmd(true);
    }
  } else {
    if (pumpDesired && (now - lastPumpOn >= MIN_PUMP_ON_MS)) {
      pumpDesired = false;
      lastPumpOff = now;
      Serial.println("[BOIA] CHEIO → solicitando DESLIGAR bomba");
      sendPumpCmd(false);
    }
  }
}

// ══════════════════════════════════════════════════════════
//  PROCESSAMENTO DE MENSAGEM RECEBIDA
// ══════════════════════════════════════════════════════════

void handleReceived() {
  char fields[TOTAL_FIELDS][32];
  if (!parseMsg(rxpacket, fields)) return;

  uint16_t seq = (uint16_t)strtol(fields[FIELD_SEQ], nullptr, 16);
  const char *type = fields[FIELD_TYPE];
  const char *pay  = fields[FIELD_PAYLOAD];

  // ── ACK ───────────────────────────────────────────────
  if (strcmp(type, "ACK") == 0) {
    if (waitingAck) {
      waitingAck = false;
      Serial.printf("[ACK] CMD confirmado pelo Node B (seq=%s)\n",
                    fields[FIELD_SEQ]);
      updateDisplay("ACK recebido!");
    }
    return;
  }

  // ── PONG ──────────────────────────────────────────────
  if (strcmp(type, "PONG") == 0) {
    Serial.println("[PONG] Link OK");
    char buf[24];
    snprintf(buf, sizeof(buf), "PONG: link OK");
    updateDisplay(buf);
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
    return;
  }

  // ── STATUS (Node B → Node A) ──────────────────────────
  if (strcmp(type, "STATUS") == 0) {
    sendAck(seq);
    bool pumpOn = (strstr(pay, "PUMP=ON") != nullptr);
    pumpConfirmed = pumpOn;
    Serial.printf("[STATUS] Bomba Node B: %s\n", pumpOn ? "LIGADA" : "DESLIGADA");
    if (!waitingAck && pumpOn != pumpDesired) {
      Serial.println("[ALERTA] Estado divergente! Reenviando CMD...");
      sendPumpCmd(pumpDesired);
    }
    char buf[28];
    snprintf(buf, sizeof(buf), "STATUS B: %s", pumpOn ? "LIGADA" : "DESLIGADA");
    updateDisplay(buf);
    return;
  }
}

// ══════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
  Serial.println("\nNODE A — Sensor de Boia  [LoRaWan_APP]");

  // ── Inicializa display OLED ─────────────────────────────
  // Vext (pino de alimentação do display) é ativo em LOW
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
  delay(100);
  display.init();
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0,  "NODE A | iniciando...");
  display.drawString(0, 16, "LoRa 915 MHz  SF9");
  display.display();

  // ── Boia ───────────────────────────────────────────────
  pinMode(BOIA_PIN, INPUT_PULLUP);
  boiaState     = digitalRead(BOIA_PIN);
  boiaPrevState = boiaState;
  lastPumpOff   = millis();

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

  Serial.printf("Boia inicial: %s\n",
                boiaState == BOIA_VAZIO ? "VAZIO" : "CHEIO");

  updateDisplay("Pronto. Aguardando...");
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
      handleReceived();   // updateDisplay() é chamado dentro
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

  // ── Leitura da boia com debounce ─────────────────────
  if (readBoia()) {
    Serial.printf("[BOIA] Mudança: %s\n",
                  boiaState == BOIA_VAZIO ? "VAZIO" : "CHEIO");
    evaluatePump();   // sendPumpCmd() → updateDisplay() chamado lá dentro
  }

  // ── Retransmissão por timeout de ACK ─────────────────
  if (waitingAck && (now - lastAckRequest >= ACK_TIMEOUT_MS)) {
    if (radioState == APP_STATE_LOWPOWER) {
      if (retries < MAX_RETRIES) {
        retries++;
        lastAckRequest = now;
        Serial.printf("[RETRY] tentativa %d/%d\n", retries, MAX_RETRIES);
        updateDisplay();   // mostra "Aguard. ACK  retry N/M"
        Radio.Sleep();
        sendRaw(pendingMsg);
      } else {
        waitingAck = false;
        retries    = 0;
        Serial.println("[FALHA] Node B sem resposta! CMD perdido.");
        updateDisplay("FALHA: sem resposta!");
        Radio.Rx(0);
        radioState = APP_STATE_LOWPOWER;
      }
    }
  }

  // ── PING periódico ────────────────────────────────────
  if (!waitingAck && (now - lastPing >= PING_INTERVAL_MS)) {
    if (radioState == APP_STATE_LOWPOWER) {
      lastPing = now;
      Radio.Sleep();
      sendPing();
    }
  }

  Radio.IrqProcess();
}

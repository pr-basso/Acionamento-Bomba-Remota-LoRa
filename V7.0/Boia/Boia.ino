/**
 * ============================================================
 *  NODE A — SENSOR DE BOIA
 *  Hardware : Heltec WiFi LoRa 32 (V2 ou V3)
 *  Biblioteca: Heltec ESP32 — LoRaWan_APP.h
 *
 *  PROTOCOLO (separador "|"):
 *    CMD : A|B|<SEQ>|CMD|PUMP=ON|CRC    (envia)
 *    ACK : B|A|<SEQ>|ACK|RELAY=1;PUMP=ON|CRC  (recebe)
 *
 *  RÁDIO: permanece em RX contínuo.
 *    Para TX: Sleep → Send → OnTxDone → Rx(0)
 * ============================================================
 */

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"

// ── Identificação ──────────────────────────────────────────
#define MY_ADDRESS   "A"
#define PEER_ADDRESS "B"

// ── Hardware ───────────────────────────────────────────────
#define BOIA_PIN    2
#define BOIA_VAZIO  LOW
#define BOIA_CHEIO  HIGH

// ── Parâmetros LoRa ────────────────────────────────────────
#define RF_FREQUENCY          915000000
#define TX_OUTPUT_POWER               2   // baixo para teste sem antena
#define LORA_BANDWIDTH                0   // 125 kHz
#define LORA_SPREADING_FACTOR         7   // SF7 — rápido para teste
#define LORA_CODINGRATE               1   // 4/5
#define LORA_PREAMBLE_LENGTH          8
#define LORA_SYMBOL_TIMEOUT           0
#define LORA_IQ_INVERSION_ON      false

// ── Temporização ──────────────────────────────────────────
#define ACK_TIMEOUT_MS   6000   // espera ACK por 6 s
#define MAX_RETRIES          5
#define DEBOUNCE_MS       2000
#define MIN_PUMP_ON_MS   10000
#define MIN_PUMP_OFF_MS   5000

// ── Protocolo ─────────────────────────────────────────────
#define BUFFER_SIZE  128
#define TOTAL_FIELDS   6

static char rxBuf[BUFFER_SIZE];
static char pendingMsg[BUFFER_SIZE];
static RadioEvents_t RadioEvents;

// ── Display ────────────────────────────────────────────────
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED,
                           GEOMETRY_128_64, RST_OLED);

// ── Flags de ISR (voláteis) ───────────────────────────────
static volatile bool evTxDone  = false;
static volatile bool evRxDone  = false;
static volatile bool evRxError = false;
static volatile int  lastRssi  = 0;

// ── Estado da aplicação ───────────────────────────────────
uint16_t      txSeq      = 0;
bool          txBusy     = false;   // true enquanto rádio está em TX
bool          waitingAck = false;
uint8_t       retries    = 0;
unsigned long lastAckReq = 0;

int           boiaState  = BOIA_CHEIO;
int           boiaPrev   = BOIA_CHEIO;
bool          boiaPend   = false;
unsigned long boiaAt     = 0;

bool          pumpDesired = false;
bool          relayRemote = false;
bool          pumpRemote  = false;
unsigned long lastPumpOn  = 0;
unsigned long lastPumpOff = 0;

// ══════════════════════════════════════════════════════════
//  CALLBACKS — só setam flags, zero Radio.* aqui
// ══════════════════════════════════════════════════════════

void OnTxDone(void) {
  evTxDone = true;
}
void OnTxTimeout(void) {
  evTxDone = true;
  Serial.println("[TX TIMEOUT]");
}
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  memset(rxBuf, 0, BUFFER_SIZE);
  memcpy(rxBuf, payload, min((int)size, BUFFER_SIZE - 1));
  lastRssi = rssi;
  evRxDone = true;
}
void OnRxTimeout(void) { /* continua em RX — normal */ }
void OnRxError(void)   { evRxError = true; }

// ══════════════════════════════════════════════════════════
//  PROTOCOLO
// ══════════════════════════════════════════════════════════

uint8_t calcCRC(const char *s) {
  uint8_t c = 0;
  while (*s) c ^= (uint8_t)(*s++);
  return c;
}

void buildMsg(char *out, const char *type, const char *pay, uint16_t seq) {
  char body[BUFFER_SIZE];
  snprintf(body, BUFFER_SIZE, "%s|%s|%04X|%s|%s",
           MY_ADDRESS, PEER_ADDRESS, seq, type, pay);
  snprintf(out, BUFFER_SIZE, "%s|%02X", body, calcCRC(body));
}

bool parseMsg(const char *raw, char fields[][32]) {
  char tmp[BUFFER_SIZE];
  strncpy(tmp, raw, BUFFER_SIZE - 1); tmp[BUFFER_SIZE - 1] = '\0';
  char *ptr = tmp;
  for (int i = 0; i < TOTAL_FIELDS; i++) {
    char *sep = (i < TOTAL_FIELDS - 1) ? strchr(ptr, '|') : nullptr;
    if (i < TOTAL_FIELDS - 1 && !sep) return false;
    if (sep) *sep = '\0';
    strncpy(fields[i], ptr, 31); fields[i][31] = '\0';
    if (sep) ptr = sep + 1;
  }
  char body[BUFFER_SIZE];
  snprintf(body, BUFFER_SIZE, "%s|%s|%s|%s|%s",
           fields[0], fields[1], fields[2], fields[3], fields[4]);
  if (calcCRC(body) != (uint8_t)strtol(fields[5], nullptr, 16)) {
    Serial.println("[PARSE] CRC erro");
    return false;
  }
  if (strcmp(fields[1], MY_ADDRESS) != 0) return false;
  return true;
}

// ══════════════════════════════════════════════════════════
//  DISPLAY
// ══════════════════════════════════════════════════════════

void updateDisplay(const char *ev = "") {
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0,  0, "NODE A | BOIA");
  display.drawHorizontalLine(0, 12, 128);
  display.drawString(0, 14, boiaState == BOIA_VAZIO
                            ? "Boia : VAZIO" : "Boia : CHEIO");
  display.drawString(0, 26, pumpDesired
                            ? "Cmd  : PUMP=ON" : "Cmd  : PUMP=OFF");
  char ln[22];
  snprintf(ln, sizeof(ln), "Remoto: R=%d P=%s",
           relayRemote, pumpRemote ? "ON" : "OFF");
  display.drawString(0, 38, ln);
  if (strlen(ev)) {
    display.drawString(0, 52, ev);
  } else if (waitingAck) {
    snprintf(ln, sizeof(ln), "Aguard.ACK %d/%d", retries, MAX_RETRIES);
    display.drawString(0, 52, ln);
  } else {
    snprintf(ln, sizeof(ln), "RSSI:%d", lastRssi);
    display.drawString(0, 52, ln);
  }
  display.display();
}

// ══════════════════════════════════════════════════════════
//  ENVIO — Sleep → Send; OnTxDone → Rx(0)
// ══════════════════════════════════════════════════════════

void doSend(const char *msg) {
  txBusy = true;
  Radio.Sleep();                                    // sai do RX
  Radio.Send((uint8_t *)msg, strlen(msg));          // entra em TX
  Serial.printf("[TX] \"%s\"\n", msg);
}

void sendCmd(bool pumpOn) {
  buildMsg(pendingMsg, "CMD", pumpOn ? "PUMP=ON" : "PUMP=OFF", txSeq++);
  waitingAck  = true;
  retries     = 0;
  lastAckReq  = millis();
  doSend(pendingMsg);
  updateDisplay(pumpOn ? "TX: PUMP=ON" : "TX: PUMP=OFF");
}

void retrySend() {
  retries++;
  lastAckReq = millis();
  Serial.printf("[RETRY] %d/%d \"%s\"\n", retries, MAX_RETRIES, pendingMsg);
  updateDisplay();
  doSend(pendingMsg);
}

// ══════════════════════════════════════════════════════════
//  BOIA
// ══════════════════════════════════════════════════════════

bool readBoia() {
  int cur = digitalRead(BOIA_PIN);
  unsigned long now = millis();
  if (cur != boiaPrev && !boiaPend) { boiaPend = true; boiaAt = now; }
  if (boiaPend && now - boiaAt >= DEBOUNCE_MS) {
    boiaPend = false;
    if (cur == boiaPrev) return false;
    boiaPrev = boiaState = cur;
    return true;
  }
  return false;
}

void evaluatePump() {
  unsigned long now = millis();
  if (boiaState == BOIA_VAZIO && !pumpDesired
      && now - lastPumpOff >= MIN_PUMP_OFF_MS) {
    pumpDesired = true; lastPumpOn = now;
    Serial.println("[BOIA] VAZIO -> PUMP=ON");
    sendCmd(true);
  } else if (boiaState == BOIA_CHEIO && pumpDesired
             && now - lastPumpOn >= MIN_PUMP_ON_MS) {
    pumpDesired = false; lastPumpOff = now;
    Serial.println("[BOIA] CHEIO -> PUMP=OFF");
    sendCmd(false);
  }
}

// ══════════════════════════════════════════════════════════
//  RECEPÇÃO
// ══════════════════════════════════════════════════════════

void handleReceived() {
  Serial.printf("[RX] \"%s\" RSSI=%d\n", rxBuf, lastRssi);
  char fields[TOTAL_FIELDS][32];
  if (!parseMsg(rxBuf, fields)) return;
  if (strcmp(fields[3], "ACK") != 0) return;

  uint16_t ackSeq = (uint16_t)strtol(fields[2], nullptr, 16);
  uint16_t cmdSeq = (txSeq - 1) & 0xFFFF;
  if (ackSeq != cmdSeq) {
    Serial.printf("[ACK] SEQ %04X ignorado (esperava %04X)\n", ackSeq, cmdSeq);
    return;
  }
  waitingAck  = false;
  relayRemote = (strstr(fields[4], "RELAY=1") != nullptr);
  pumpRemote  = (strstr(fields[4], "PUMP=ON") != nullptr);
  Serial.printf("[ACK] OK RELAY=%d PUMP=%s\n",
                relayRemote, pumpRemote ? "ON" : "OFF");
  if (pumpRemote != pumpDesired) {
    Serial.println("[ALERTA] divergente -> reenviar");
    sendCmd(pumpDesired);
  } else {
    updateDisplay("ACK OK");
  }
}

// ══════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
  Serial.println("\nNODE A — Sensor de Boia");

  pinMode(Vext, OUTPUT); digitalWrite(Vext, LOW); delay(100);
  display.init(); display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "NODE A iniciando...");
  display.display();

  pinMode(BOIA_PIN, INPUT_PULLUP);
  boiaState = boiaPrev = digitalRead(BOIA_PIN);
  lastPumpOff = millis();

  // Se ao ligar a boia já está em VAZIO, agenda envio do CMD
  // após o rádio inicializar — evita perder estado após falta de energia
  if (boiaState == BOIA_VAZIO) {
    pumpDesired = true;
    lastPumpOn  = millis();
    Serial.println("[BOOT] boia VAZIO detectado — CMD PUMP=ON sera enviado");
  } else {
    Serial.println("[BOOT] boia CHEIO");
  }

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
                    false, true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, false,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
  Radio.SetMaxPayloadLength(MODEM_LORA, BUFFER_SIZE);

  Radio.Rx(0);   // RX contínuo desde o início
  Serial.println("Pronto — RX contínuo.");
  updateDisplay("Aguardando boia...");
}

// ══════════════════════════════════════════════════════════
//  LOOP
// ══════════════════════════════════════════════════════════

void loop() {
  unsigned long now = millis();
  Radio.IrqProcess();

  // ── CMD inicial de boot (boia já estava em VAZIO ao ligar) ───
  static bool bootCmdSent = false;
  if (!bootCmdSent) {
    bootCmdSent = true;
    if (boiaState == BOIA_VAZIO) {
      Serial.println("[BOOT] enviando CMD PUMP=ON inicial");
      sendCmd(true);
    }
  }

  // ── TX concluído → volta ao RX imediatamente ─────────
  if (evTxDone) {
    evTxDone = false;
    txBusy   = false;
    Radio.Rx(0);
    Serial.println("[RX] modo RX reaberto");
  }

  // ── Pacote recebido ───────────────────────────────────
  if (evRxDone) {
    evRxDone = false;
    handleReceived();
    Radio.Rx(0);   // mantém RX após receber
  }

  // ── Erro de RX → volta ao RX ─────────────────────────
  if (evRxError) {
    evRxError = false;
    Radio.Rx(0);
  }

  // ── Boia (só avalia se rádio livre) ──────────────────
  if (readBoia()) {
    Serial.printf("[BOIA] %s\n",
                  boiaState == BOIA_VAZIO ? "VAZIO" : "CHEIO");
    if (!txBusy && !waitingAck) evaluatePump();
    else updateDisplay();
  }

  // ── Retry (só quando rádio não está em TX) ───────────
  if (waitingAck && !txBusy && now - lastAckReq >= ACK_TIMEOUT_MS) {
    if (retries < MAX_RETRIES) {
      retrySend();
    } else {
      waitingAck = false; retries = 0;
      Serial.println("[FALHA] sem ACK");
      updateDisplay("FALHA: sem resposta!");
    }
  }

  // ── Diagnóstico a cada 15 s ───────────────────────────
  static unsigned long lastDiag = 0;
  if (now - lastDiag >= 15000) {
    lastDiag = now;
    Serial.printf("[DIAG] boia=%s pump=%d ack=%d tx=%d uptime=%lus\n",
                  boiaState == BOIA_VAZIO ? "VAZIO" : "CHEIO",
                  pumpDesired, waitingAck, txBusy, now / 1000);
  }
}

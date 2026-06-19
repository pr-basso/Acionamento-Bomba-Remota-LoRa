/**
 * ============================================================
 *  NODE A — SENSOR DE BOIA
 *  Hardware : Heltec WiFi LoRa 32 (V2 ou V3)
 *  Biblioteca: Heltec ESP32 — LoRaWan_APP.h
 *
 *  PROTOCOLO (separador "|"):
 *    CMD : A|B|<SEQ>|CMD|PUMP=ON  |CRC   (Node A → B)
 *    ACK : B|A|<SEQ>|ACK|RELAY=1;PUMP=ON|CRC  (Node B → A)
 *
 *  BOIA (GPIO2, pull-up interno, boia NC):
 *    LOW  = boia aberta  = tanque VAZIO  → PUMP=ON
 *    HIGH = boia fechada = tanque CHEIO  → PUMP=OFF
 * ============================================================
 */

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"

// ── Identificação ──────────────────────────────────────────
#define MY_ADDRESS   "A"
#define PEER_ADDRESS "B"

// ── Hardware ───────────────────────────────────────────────
#define BOIA_PIN   2
#define BOIA_VAZIO LOW
#define BOIA_CHEIO HIGH

// ── Parâmetros LoRa ────────────────────────────────────────
#define RF_FREQUENCY          915000000
#define TX_OUTPUT_POWER       14
#define LORA_BANDWIDTH        0
#define LORA_SPREADING_FACTOR 9
#define LORA_CODINGRATE       1
#define LORA_PREAMBLE_LENGTH  8
#define LORA_SYMBOL_TIMEOUT   0
#define LORA_IQ_INVERSION_ON  false

// ── Temporização ──────────────────────────────────────────
#define ACK_TIMEOUT_MS   5000   // espera ACK por 5 s
#define MAX_RETRIES         5   // retransmissões máximas
#define DEBOUNCE_MS      2000   // debounce da boia
#define MIN_PUMP_ON_MS  10000   // mínimo ligada
#define MIN_PUMP_OFF_MS  5000   // mínimo desligada

// ── Protocolo ─────────────────────────────────────────────
#define BUFFER_SIZE  128
#define TOTAL_FIELDS   6

static char rxBuf[BUFFER_SIZE];
static char pendingMsg[BUFFER_SIZE];
static RadioEvents_t RadioEvents;

// ── Display ────────────────────────────────────────────────
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED,
                           GEOMETRY_128_64, RST_OLED);

// ── Flags de rádio (voláteis — setadas em callbacks ISR) ───
static volatile bool rxReady = false;
static volatile bool rxError = false;
static volatile int  lastRssi = 0;

// ── Estado da aplicação ───────────────────────────────────
uint16_t      txSeq       = 0;
bool          waitingAck  = false;
uint8_t       retries     = 0;
unsigned long lastAckReq  = 0;

int           boiaState   = BOIA_CHEIO;
int           boiaPrev    = BOIA_CHEIO;
bool          boiaPending = false;
unsigned long boiaAt      = 0;

bool          pumpDesired  = false;
bool          relayRemote  = false;
bool          pumpRemote   = false;
unsigned long lastPumpOn   = 0;
unsigned long lastPumpOff  = 0;

// ══════════════════════════════════════════════════════════
//  CALLBACKS DO RÁDIO
// ══════════════════════════════════════════════════════════

void OnTxDone(void) {
  // Abre RX imediatamente na ISR — sem depender do loop()
  Radio.Rx(0);
}

void OnTxTimeout(void) {
  Radio.Sleep();
  Radio.Rx(0);
  Serial.println("[TX TIMEOUT]");
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  memset(rxBuf, 0, BUFFER_SIZE);
  memcpy(rxBuf, payload, min((int)size, BUFFER_SIZE - 1));
  lastRssi = rssi;
  Radio.Sleep();
  rxReady = true;
  Serial.printf("[RX] \"%s\" RSSI=%d\n", rxBuf, rssi);
}

void OnRxTimeout(void) { Radio.Sleep(); }
void OnRxError(void)   { Radio.Sleep(); rxError = true; }

// ══════════════════════════════════════════════════════════
//  PROTOCOLO
// ══════════════════════════════════════════════════════════

uint8_t calcCRC(const char *s) {
  uint8_t c = 0;
  while (*s) c ^= (uint8_t)(*s++);
  return c;
}

void buildMsg(char *out, const char *type, const char *payload, uint16_t seq) {
  char body[BUFFER_SIZE];
  snprintf(body, BUFFER_SIZE, "%s|%s|%04X|%s|%s",
           MY_ADDRESS, PEER_ADDRESS, seq, type, payload);
  snprintf(out, BUFFER_SIZE, "%s|%02X", body, calcCRC(body));
}

// Devolve true se mensagem válida e destinada a mim
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
  // CRC
  char body[BUFFER_SIZE];
  snprintf(body, BUFFER_SIZE, "%s|%s|%s|%s|%s",
           fields[0], fields[1], fields[2], fields[3], fields[4]);
  if (calcCRC(body) != (uint8_t)strtol(fields[5], nullptr, 16)) {
    Serial.println("[PARSE] CRC erro");
    return false;
  }
  // Destino
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
//  ENVIO
// ══════════════════════════════════════════════════════════

void sendCmd(bool pumpOn) {
  buildMsg(pendingMsg, "CMD", pumpOn ? "PUMP=ON" : "PUMP=OFF", txSeq++);
  waitingAck  = true;
  retries     = 0;
  lastAckReq  = millis();
  Radio.Sleep();
  Radio.Send((uint8_t *)pendingMsg, strlen(pendingMsg));
  Serial.printf("[TX] \"%s\"\n", pendingMsg);
  updateDisplay(pumpOn ? "TX: PUMP=ON" : "TX: PUMP=OFF");
}

// ══════════════════════════════════════════════════════════
//  BOIA
// ══════════════════════════════════════════════════════════

bool readBoia() {
  int cur = digitalRead(BOIA_PIN);
  unsigned long now = millis();
  if (cur != boiaPrev && !boiaPending) {
    boiaPending = true; boiaAt = now; boiaState = cur;
  }
  if (boiaPending && now - boiaAt >= DEBOUNCE_MS) {
    boiaPending = false;
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
  char fields[TOTAL_FIELDS][32];
  if (!parseMsg(rxBuf, fields)) return;

  // Só trata ACK
  if (strcmp(fields[3], "ACK") != 0) return;

  // SEQ deve corresponder ao último CMD enviado
  uint16_t ackSeq = (uint16_t)strtol(fields[2], nullptr, 16);
  uint16_t cmdSeq = (txSeq - 1) & 0xFFFF;   // SEQ do último CMD
  if (ackSeq != cmdSeq) {
    Serial.printf("[ACK] SEQ %04X ignorado (esperava %04X)\n", ackSeq, cmdSeq);
    return;
  }

  waitingAck  = false;
  relayRemote = (strstr(fields[4], "RELAY=1") != nullptr);
  pumpRemote  = (strstr(fields[4], "PUMP=ON") != nullptr);
  Serial.printf("[ACK] confirmado RELAY=%d PUMP=%s\n",
                relayRemote, pumpRemote ? "ON" : "OFF");

  // Segurança: se estado remoto diverge, reenviar
  if (pumpRemote != pumpDesired) {
    Serial.println("[ALERTA] estado divergente -> reenviar CMD");
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

  // Display
  pinMode(Vext, OUTPUT); digitalWrite(Vext, LOW); delay(100);
  display.init(); display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "NODE A iniciando...");
  display.display();

  // Boia
  pinMode(BOIA_PIN, INPUT_PULLUP);
  boiaState = boiaPrev = digitalRead(BOIA_PIN);
  lastPumpOff = millis();

  // Rádio
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
  Radio.Rx(0);

  Serial.println("Pronto.");
  updateDisplay("Aguardando boia...");
}

// ══════════════════════════════════════════════════════════
//  LOOP
// ══════════════════════════════════════════════════════════

void loop() {
  unsigned long now = millis();
  Radio.IrqProcess();

  // Recepção
  if (rxReady) {
    rxReady = false;
    handleReceived();
    Radio.Rx(0);   // garante RX após processar
  }
  if (rxError) {
    rxError = false;
    Radio.Rx(0);
  }

  // Boia
  if (readBoia()) {
    Serial.printf("[BOIA] %s\n",
                  boiaState == BOIA_VAZIO ? "VAZIO" : "CHEIO");
    if (!waitingAck) evaluatePump();
    updateDisplay();
  }

  // Retry — retransmite CMD se não receber ACK
  if (waitingAck && now - lastAckReq >= ACK_TIMEOUT_MS) {
    if (retries < MAX_RETRIES) {
      retries++;
      lastAckReq = now;
      Serial.printf("[RETRY] %d/%d \"%s\"\n", retries, MAX_RETRIES, pendingMsg);
      updateDisplay();
      Radio.Sleep();
      Radio.Send((uint8_t *)pendingMsg, strlen(pendingMsg));
    } else {
      waitingAck = false; retries = 0;
      Serial.println("[FALHA] sem ACK");
      updateDisplay("FALHA: sem resposta!");
      Radio.Rx(0);
    }
  }

  // Diagnóstico a cada 15 s
  static unsigned long lastDiag = 0;
  if (now - lastDiag >= 15000) {
    lastDiag = now;
    Serial.printf("[DIAG] boia=%s pumpDesired=%d waitingAck=%d uptime=%lus\n",
                  boiaState == BOIA_VAZIO ? "VAZIO" : "CHEIO",
                  pumpDesired, waitingAck, now / 1000);
  }
}

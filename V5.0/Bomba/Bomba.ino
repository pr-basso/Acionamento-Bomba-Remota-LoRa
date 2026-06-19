/**
 * ============================================================
 *  NODE B — CONTROLADOR DE BOMBA D'ÁGUA
 *  Hardware : Heltec WiFi LoRa 32 (V2 ou V3)
 *  Biblioteca: Heltec ESP32 — LoRaWan_APP.h
 *
 *  PROTOCOLO (separador "|"):
 *    CMD : A|B|<SEQ>|CMD|PUMP=ON           (recebe)
 *    ACK : B|A|<SEQ>|ACK|RELAY=1;PUMP=ON  (envia)
 *
 *  RELAY_PIN: saída para módulo relé
 *    RELAY_ON  = LOW  (lógica invertida — padrão módulos azuis)
 *    RELAY_OFF = HIGH
 *
 *  PROTEÇÕES:
 *    Watchdog: sem CMD por WATCHDOG_MS → desliga bomba
 *    Tempo mínimo ligado (MIN_PUMP_ON_MS)
 * ============================================================
 */

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"

// ── Identificação ──────────────────────────────────────────
#define MY_ADDRESS   "B"
#define PEER_ADDRESS "A"

// ── Hardware ───────────────────────────────────────────────
#define RELAY_PIN  26
#define RELAY_ON   LOW
#define RELAY_OFF  HIGH

// ── Parâmetros LoRa (idênticos ao Node A) ─────────────────
#define RF_FREQUENCY          915000000
#define TX_OUTPUT_POWER       14
#define LORA_BANDWIDTH        0
#define LORA_SPREADING_FACTOR 9
#define LORA_CODINGRATE       1
#define LORA_PREAMBLE_LENGTH  8
#define LORA_SYMBOL_TIMEOUT   0
#define LORA_IQ_INVERSION_ON  false

// ── Temporização ──────────────────────────────────────────
#define WATCHDOG_MS     60000
#define MIN_PUMP_ON_MS   8000
// Guarda antes de responder: aguarda Node A terminar TX e abrir RX.
// SF9/BW125kHz/~30 bytes ≈ 120ms de transmissão no ar.
#define TX_GUARD_MS       500

// ── Protocolo ─────────────────────────────────────────────
#define BUFFER_SIZE  128
#define TOTAL_FIELDS   6

static char rxBuf[BUFFER_SIZE];
static RadioEvents_t RadioEvents;


// ── Display ────────────────────────────────────────────────
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED,
                           GEOMETRY_128_64, RST_OLED);

// ── Flags de rádio ────────────────────────────────────────
static volatile bool rxReady  = false;
static volatile bool rxError  = false;
static volatile int  lastRssi = 0;

// ── Estado da aplicação ───────────────────────────────────
uint16_t      txSeq      = 0;
unsigned long lastRxTime = 0;
bool          pumpState  = false;
unsigned long pumpOnTime = 0;

// ══════════════════════════════════════════════════════════
//  CALLBACKS DO RÁDIO
// ══════════════════════════════════════════════════════════

void OnTxDone(void) {
  // Volta ao RX imediatamente após enviar o ACK
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
  lastRssi   = rssi;
  lastRxTime = millis();
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
  display.drawString(0,  0, "NODE B | BOMBA");
  display.drawHorizontalLine(0, 12, 128);
  display.drawString(0, 14, pumpState
                            ? "Bomba: LIGADA  [ON]"
                            : "Bomba: DESLIGADA");
  if (pumpState) {
    char buf[24];
    snprintf(buf, sizeof(buf), "Tempo ON: %lus",
             (millis() - pumpOnTime) / 1000);
    display.drawString(0, 26, buf);
  } else {
    display.drawString(0, 26, "Rele: OFF");
  }
  if (strlen(ev)) display.drawString(0, 40, ev);
  char buf[22];
  snprintf(buf, sizeof(buf), "RSSI: %d", lastRssi);
  display.drawString(0, 52, buf);
  display.display();
}

// ══════════════════════════════════════════════════════════
//  CONTROLE DO RELÉ
// ══════════════════════════════════════════════════════════

void pumpOn() {
  if (pumpState) return;
  digitalWrite(RELAY_PIN, RELAY_ON);
  pumpState = true; pumpOnTime = millis();
  Serial.println("[RELE] LIGADO");
  updateDisplay("CMD: LIGAR");
}

void pumpOff() {
  if (pumpState && millis() - pumpOnTime < MIN_PUMP_ON_MS) {
    Serial.println("[RELE] aguardando tempo minimo");
    updateDisplay("Aguard. t.min...");
    return;
  }
  digitalWrite(RELAY_PIN, RELAY_OFF);
  pumpState = false;
  Serial.println("[RELE] DESLIGADO");
  updateDisplay("CMD: DESLIGAR");
}

// ══════════════════════════════════════════════════════════
//  RECEPÇÃO E RESPOSTA
// ══════════════════════════════════════════════════════════

void handleReceived() {
  char fields[TOTAL_FIELDS][32];
  if (!parseMsg(rxBuf, fields)) return;

  // Só trata CMD
  if (strcmp(fields[3], "CMD") != 0) return;

  uint16_t seq   = (uint16_t)strtol(fields[2], nullptr, 16);
  bool     cmdOn = (strstr(fields[4], "PUMP=ON") != nullptr);

  Serial.printf("[CMD] seq=%04X PUMP=%s\n", seq, cmdOn ? "ON" : "OFF");

  // Executa o relé
  if (cmdOn) pumpOn();
  else       pumpOff();

  // Aguarda Node A terminar TX e abrir RX antes de responder
  delay(TX_GUARD_MS);

  // Monta ACK com estado real do relé
  char payload[32], msg[BUFFER_SIZE];
  snprintf(payload, sizeof(payload), "RELAY=%d;PUMP=%s",
           pumpState ? 1 : 0, pumpState ? "ON" : "OFF");
  buildMsg(msg, "ACK", payload, seq);   // mesmo SEQ do CMD

  Radio.Sleep();
  Radio.Send((uint8_t *)msg, strlen(msg));
  Serial.printf("[TX] \"%s\"\n", msg);
}

// ══════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
  Serial.println("\nNODE B — Controlador de Bomba");

  // Display
  pinMode(Vext, OUTPUT); digitalWrite(Vext, LOW); delay(100);
  display.init(); display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "NODE B iniciando...");
  display.display();

  // Relé começa DESLIGADO
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF);
  pumpState  = false;
  lastRxTime = millis();

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

  Serial.println("Pronto. Aguardando CMD...");
  updateDisplay("Aguardando CMD...");
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

  // Watchdog — sem CMD por 60s desliga bomba
  if (pumpState && now - lastRxTime >= WATCHDOG_MS) {
    Serial.printf("[WATCHDOG] sem CMD por %ds — desligando\n",
                  WATCHDOG_MS / 1000);
    digitalWrite(RELAY_PIN, RELAY_OFF);
    pumpState  = false;
    lastRxTime = now;
    updateDisplay("WATCHDOG: desligado!");
  }

  // Atualiza tempo ligado no display a cada 5 s
  static unsigned long lastDisp = 0;
  if (pumpState && now - lastDisp >= 5000) {
    lastDisp = now;
    updateDisplay();
  }

  // Diagnóstico a cada 15 s
  static unsigned long lastDiag = 0;
  if (now - lastDiag >= 15000) {
    lastDiag = now;
    Serial.printf("[DIAG] pump=%d lastRx=%lus uptime=%lus\n",
                  pumpState, (now - lastRxTime) / 1000, now / 1000);
  }
}

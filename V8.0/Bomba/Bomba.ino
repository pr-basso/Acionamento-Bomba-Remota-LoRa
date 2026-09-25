/**
 * ============================================================
 *  NODE B — CONTROLADOR DE BOMBA D'ÁGUA
 *  Hardware : Heltec WiFi LoRa 32 (V2 ou V3)
 *  Biblioteca: Heltec ESP32 — LoRaWan_APP.h
 *
 *  PROTOCOLO (separador "|"):
 *    CMD : A|B|<SEQ>|CMD|PUMP=ON|CRC   (recebe)
 *    ACK : B|A|<SEQ>|ACK|RELAY=1;PUMP=ON|CRC  (envia)
 *
 *  RÁDIO: permanece em RX contínuo.
 *    Para TX: Sleep → Send → OnTxDone → Rx(0)
 *
 *  EEPROM — persiste o último comando recebido:
 *    Endereço 0 : assinatura (0xAB) — valida dados gravados
 *    Endereço 1 : último estado da bomba (1=ON, 0=OFF)
 *    Ao reiniciar: restaura o estado anterior automaticamente
 *
 *  WATCHDOG — protege contra falha de comunicação:
 *    Só conta enquanto a bomba está LIGADA
 *    Reseta a cada CMD válido recebido
 *    Dispara após WATCHDOG_MS sem CMD → desliga bomba
 * ============================================================
 */

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"
#include "LoraConfig.h"   // parâmetros de rádio compartilhados (../common)
#include <EEPROM.h>

// ── Identificação ──────────────────────────────────────────
#define MY_ADDRESS   "B"
#define PEER_ADDRESS "A"

// ── Hardware ───────────────────────────────────────────────
#define RELAY_PIN   26
#define RELAY_ON   LOW    // lógica invertida — módulos azuis
#define RELAY_OFF  HIGH

// Tempo de guarda antes de enviar ACK.
// O Node A abre RX no OnTxDone — que dispara ao fim do TX do CMD.
// Com SF9/BW125 um pacote de ~35 bytes leva ~250ms no ar.
// O Radio.Rx(0) leva ~1ms para estabilizar.
// Aguardamos 100ms como margem segura antes de transmitir o ACK.
#define TX_GUARD_MS  100
#define WATCHDOG_MS    600000   // 10 minutos
#define MIN_PUMP_ON_MS   8000   // tempo mínimo ligada

// ── EEPROM ────────────────────────────────────────────────
#define EEPROM_SIZE      2
#define EEPROM_ADDR      0   // endereço base
#define EEPROM_SIG    0xAB   // assinatura para validar dados
// Layout:
//   EEPROM[EEPROM_ADDR + 0] = assinatura (0xAB)
//   EEPROM[EEPROM_ADDR + 1] = último estado (1=ON, 0=OFF)

// ── Protocolo ─────────────────────────────────────────────
#define BUFFER_SIZE  128
#define TOTAL_FIELDS   6

static char rxBuf[BUFFER_SIZE];
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
bool          txBusy        = false;
bool          pumpState     = false;
unsigned long pumpOnTime    = 0;
unsigned long lastCmdTime   = 0;   // watchdog — momento do último CMD válido

// ══════════════════════════════════════════════════════════
//  EEPROM
// ══════════════════════════════════════════════════════════

void eepromSave(bool state) {
  EEPROM.write(EEPROM_ADDR,     EEPROM_SIG);
  EEPROM.write(EEPROM_ADDR + 1, state ? 1 : 0);
  EEPROM.commit();
  Serial.printf("[EEPROM] salvo: PUMP=%s\n", state ? "ON" : "OFF");
}

// Retorna true se havia dados válidos e preenche *state
bool eepromLoad(bool *state) {
  if (EEPROM.read(EEPROM_ADDR) != EEPROM_SIG) return false;
  *state = (EEPROM.read(EEPROM_ADDR + 1) == 1);
  return true;
}

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
//  RELÉ — salva estado na EEPROM a cada mudança
// ══════════════════════════════════════════════════════════

void applyRelay(bool on) {
  if (on) {
    digitalWrite(RELAY_PIN, RELAY_ON);
    pumpState = true;
    pumpOnTime = millis();
    Serial.println("[RELE] LIGADO");
    updateDisplay("CMD: LIGAR");
  } else {
    digitalWrite(RELAY_PIN, RELAY_OFF);
    pumpState = false;
    Serial.println("[RELE] DESLIGADO");
    updateDisplay("CMD: DESLIGAR");
  }
  eepromSave(pumpState);   // persiste sempre que o relé muda
}

void pumpOn() {
  if (pumpState) return;
  applyRelay(true);
}

void pumpOff() {
  if (pumpState && millis() - pumpOnTime < MIN_PUMP_ON_MS) {
    Serial.println("[RELE] aguardando tempo minimo");
    updateDisplay("Aguard. t.min...");
    return;
  }
  applyRelay(false);
}

// ══════════════════════════════════════════════════════════
//  ENVIO
// ══════════════════════════════════════════════════════════

void sendAck(uint16_t seq) {
  char payload[32], msg[BUFFER_SIZE];
  snprintf(payload, sizeof(payload), "RELAY=%d;PUMP=%s",
           pumpState ? 1 : 0, pumpState ? "ON" : "OFF");
  buildMsg(msg, "ACK", payload, seq);
  txBusy = true;
  Radio.Sleep();
  Radio.Send((uint8_t *)msg, strlen(msg));
  Serial.printf("[TX] \"%s\"\n", msg);
}

// ══════════════════════════════════════════════════════════
//  RECEPÇÃO
// ══════════════════════════════════════════════════════════

void handleReceived() {
  Serial.printf("[RX] \"%s\" RSSI=%d\n", rxBuf, lastRssi);
  char fields[TOTAL_FIELDS][32];
  if (!parseMsg(rxBuf, fields)) return;
  if (strcmp(fields[3], "CMD") != 0) return;

  // Reseta watchdog a cada CMD válido recebido
  lastCmdTime = millis();

  uint16_t seq = (uint16_t)strtol(fields[2], nullptr, 16);
  bool cmdOn   = (strstr(fields[4], "PUMP=ON") != nullptr);
  Serial.printf("[CMD] seq=%04X PUMP=%s\n", seq, cmdOn ? "ON" : "OFF");

  if (cmdOn) pumpOn();
  else       pumpOff();

  // Aguarda Node A terminar TX e estabilizar RX antes de responder
  delay(TX_GUARD_MS);
  sendAck(seq);
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

  // Relé começa desligado — será sobrescrito pela EEPROM se houver dado válido
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF);

  // EEPROM — restaura último estado
  EEPROM.begin(EEPROM_SIZE);
  bool savedState = false;
  if (eepromLoad(&savedState)) {
    Serial.printf("[EEPROM] restaurado: PUMP=%s\n", savedState ? "ON" : "OFF");
    pumpState = savedState;
    if (savedState) {
      digitalWrite(RELAY_PIN, RELAY_ON);
      pumpOnTime = millis();
      // Watchdog desabilitado até primeiro CMD real chegar:
      // seta lastCmdTime no futuro para que (now - lastCmdTime) seja negativo
      lastCmdTime = millis() + 0xFFFFFFFF / 2;
      updateDisplay("Restaurado: ON");
      Serial.println("[EEPROM] watchdog suspenso ate primeiro CMD");
    } else {
      lastCmdTime = millis();
      updateDisplay("Restaurado: OFF");
    }
  } else {
    Serial.println("[EEPROM] sem dados — estado inicial OFF");
    pumpState   = false;
    lastCmdTime = millis();
    eepromSave(false);
    updateDisplay("Aguardando CMD...");
  }

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
  Serial.println("Pronto — RX contínuo.");
}

// ══════════════════════════════════════════════════════════
//  LOOP
// ══════════════════════════════════════════════════════════

void loop() {
  unsigned long now = millis();
  Radio.IrqProcess();

  // ── TX concluído → volta ao RX ───────────────────────
  if (evTxDone) {
    evTxDone = false;
    txBusy   = false;
    Radio.Rx(0);
    Serial.println("[RX] RX reaberto");
  }

  // ── Pacote recebido ───────────────────────────────────
  if (evRxDone) {
    evRxDone = false;
    Radio.Rx(0);                    // reabre RX antes de processar
    if (!txBusy) handleReceived();
  }

  // ── Erro de RX → volta ao RX ─────────────────────────
  if (evRxError) {
    evRxError = false;
    if (!txBusy) Radio.Rx(0);
  }

  // ── Watchdog ──────────────────────────────────────────
  // Só age se bomba ligada e sem CMD por WATCHDOG_MS
  // Consulta EEPROM antes de desligar — se EEPROM diz ON,
  // mantém ligado e reseta o contador (link pode ter caído
  // mas o último comando válido era ligar)
  if (pumpState && now - lastCmdTime >= WATCHDOG_MS) {
    bool savedState = false;
    bool valid = eepromLoad(&savedState);
    if (valid && savedState) {
      // EEPROM confirma ON — mantém ligado, reseta watchdog
      Serial.println("[WATCHDOG] EEPROM=ON — mantendo bomba ligada");
      lastCmdTime = now;
    } else {
      // EEPROM diz OFF (ou inválida) — desliga
      Serial.printf("[WATCHDOG] EEPROM=OFF — desligando apos %ds sem CMD\n",
                    WATCHDOG_MS / 1000);
      applyRelay(false);
      lastCmdTime = now;
      updateDisplay("WATCHDOG: desligado!");
    }
  }

  // ── Display: atualiza tempo ligado a cada 5 s ─────────
  static unsigned long lastDisp = 0;
  if (pumpState && now - lastDisp >= 5000) {
    lastDisp = now;
    updateDisplay();
  }

  // ── Diagnóstico a cada 15 s ───────────────────────────
  static unsigned long lastDiag = 0;
  if (now - lastDiag >= 15000) {
    lastDiag = now;
    unsigned long semCmd = (now - lastCmdTime) / 1000;
    Serial.printf("[DIAG] pump=%d tempoON=%lus semCmd=%lus uptime=%lus\n",
                  pumpState,
                  pumpState ? (now - pumpOnTime) / 1000 : 0,
                  semCmd,
                  now / 1000);
  }
}

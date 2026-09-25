/**
 * ============================================================
 *  NODE A — SENSOR DE BOIA (multi-bomba)
 *  Hardware : Heltec WiFi LoRa 32 (V2 ou V3)
 *  Biblioteca: Heltec ESP32 — LoRaWan_APP.h
 *
 *  PROTOCOLO (separador "|"):
 *    CMD : A|<DST>|<SEQ>|CMD|PUMP=ON|CRC   (envia)
 *    ACK : <DST>|A|<SEQ>|ACK|RELAY=1;PUMP=ON|CRC  (recebe)
 *
 *  MÚLTIPLOS NODES:
 *    Defina os endereços dos nodes em PEER_LIST.
 *    Para adicionar um novo node, basta incluir seu endereço.
 *    O Node A envia CMD para cada peer e aguarda ACK individual.
 *    Retry independente por peer — se um não responde, os
 *    outros não são afetados.
 *
 *  ADICIONANDO UM NOVO NODE:
 *    1. Inclua o endereço em PEER_LIST abaixo
 *    2. Grave o firmware do node com MY_ADDRESS = novo endereço
 *    Sem mais alterações necessárias.
 * ============================================================
 */

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"
#include "LoraConfig.h"   // parâmetros de rádio compartilhados (../common)

// ── Identificação ──────────────────────────────────────────
#define MY_ADDRESS "A"

// ── Lista de peers — adicione novos nodes aqui ─────────────
const char *PEER_LIST[] = { "B" };
#define PEER_COUNT  (sizeof(PEER_LIST) / sizeof(PEER_LIST[0]))

// ── Hardware ───────────────────────────────────────────────
#define BOIA_PIN    2
#define BOIA_VAZIO  LOW
#define BOIA_CHEIO  HIGH

// ── Temporização ──────────────────────────────────────────
#define ACK_TIMEOUT_MS   6000
#define MAX_RETRIES          5
#define DEBOUNCE_MS       2000
#define MIN_PUMP_ON_MS   10000
#define MIN_PUMP_OFF_MS   5000
// Intervalo entre envios para peers diferentes.
// Deve ser maior que: tempo TX do CMD + TX_GUARD do peer + tempo TX do ACK
// SF9/BW125: ~250ms por pacote (~600ms no total). Margem generosa para garantir recepção.
#define INTER_PEER_MS     2000

// ── Protocolo ─────────────────────────────────────────────
#define BUFFER_SIZE  128
#define TOTAL_FIELDS   6

static char rxBuf[BUFFER_SIZE];
static RadioEvents_t RadioEvents;

// ── Display ────────────────────────────────────────────────
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED,
                           GEOMETRY_128_64, RST_OLED);

// ── Flags de ISR ─────────────────────────────────────────
static volatile bool evTxDone  = false;
static volatile bool evRxDone  = false;
static volatile bool evRxError = false;
static volatile int  lastRssi  = 0;

// ── Estado por peer ───────────────────────────────────────
struct Peer {
  const char   *addr;          // endereço do node
  uint16_t      seq;           // SEQ do último CMD enviado
  bool          ackPending;    // aguardando ACK
  uint8_t       retries;       // tentativas do CMD atual
  unsigned long lastCmdTime;   // quando foi enviado o último CMD
  bool          confirmed;     // último estado confirmado pelo ACK
  bool          relayState;    // estado do relé reportado
};

static Peer peers[PEER_COUNT];

// ── Estado global ─────────────────────────────────────────
bool          txBusy      = false;
uint8_t       txPeerIdx   = 0;    // qual peer está sendo transmitido agora
bool          pumpDesired = false;

int           boiaState   = BOIA_CHEIO;
int           boiaPrev    = BOIA_CHEIO;
bool          boiaPend    = false;
unsigned long boiaAt      = 0;
unsigned long lastPumpOn  = 0;
unsigned long lastPumpOff = 0;

// Fila de envio: qual peer precisa receber CMD agora
// Implementado como bitmask — bit i = peer i precisa de CMD
static uint32_t sendQueue  = 0;
static uint32_t retryQueue = 0;   // peers em retry (timeout)

// ══════════════════════════════════════════════════════════
//  CALLBACKS — só setam flags
// ══════════════════════════════════════════════════════════

void OnTxDone(void)    { evTxDone = true; }
void OnTxTimeout(void) { evTxDone = true; Serial.println("[TX TIMEOUT]"); }

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  memset(rxBuf, 0, BUFFER_SIZE);
  memcpy(rxBuf, payload, min((int)size, BUFFER_SIZE - 1));
  lastRssi = rssi;
  evRxDone = true;
}
void OnRxTimeout(void) {}
void OnRxError(void)   { evRxError = true; }

// ══════════════════════════════════════════════════════════
//  PROTOCOLO
// ══════════════════════════════════════════════════════════

uint8_t calcCRC(const char *s) {
  uint8_t c = 0;
  while (*s) c ^= (uint8_t)(*s++);
  return c;
}

void buildMsg(char *out, const char *dst, const char *type,
              const char *pay, uint16_t seq) {
  char body[BUFFER_SIZE];
  snprintf(body, BUFFER_SIZE, "%s|%s|%04X|%s|%s",
           MY_ADDRESS, dst, seq, type, pay);
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
  // DST deve ser MY_ADDRESS
  if (strcmp(fields[1], MY_ADDRESS) != 0) return false;
  return true;
}

// ══════════════════════════════════════════════════════════
//  DISPLAY
// ══════════════════════════════════════════════════════════

void updateDisplay() {
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "NODE A | BOIA");
  display.drawHorizontalLine(0, 12, 128);
  display.drawString(0, 14, boiaState == BOIA_VAZIO
                            ? "Boia: VAZIO" : "Boia: CHEIO");
  display.drawString(0, 26, pumpDesired
                            ? "Cmd : PUMP=ON" : "Cmd : PUMP=OFF");
  // Status de cada peer em uma linha
  char line[22] = "";
  for (uint8_t i = 0; i < PEER_COUNT; i++) {
    char buf[8];
    snprintf(buf, sizeof(buf), "%s:%s ",
             peers[i].addr,
             peers[i].ackPending ? "..." :
             peers[i].confirmed  ? "OK"  : "--");
    strncat(line, buf, sizeof(line) - strlen(line) - 1);
  }
  display.drawString(0, 38, line);
  char rssiStr[16];
  snprintf(rssiStr, sizeof(rssiStr), "RSSI:%d", lastRssi);
  display.drawString(0, 52, rssiStr);
  display.display();
}

// ══════════════════════════════════════════════════════════
//  ENVIO
// ══════════════════════════════════════════════════════════

void doSendToPeer(uint8_t idx, bool pumpOn) {
  char msg[BUFFER_SIZE];
  peers[idx].seq++;
  buildMsg(msg, peers[idx].addr, "CMD",
           pumpOn ? "PUMP=ON" : "PUMP=OFF", peers[idx].seq);
  peers[idx].ackPending  = true;
  peers[idx].lastCmdTime = millis();
  txBusy    = true;
  txPeerIdx = idx;
  Radio.Sleep();
  Radio.Send((uint8_t *)msg, strlen(msg));
  Serial.printf("[TX->%s] \"%s\"\n", peers[idx].addr, msg);
  updateDisplay();
}

// Procura o próximo peer na fila e transmite.
// Só envia para um peer por vez — aguarda ACK ou timeout antes do próximo.
bool processQueue() {
  if (txBusy) return false;

  // Se algum peer ainda está aguardando ACK, não avança na fila
  // (mas verifica timeout para colocar em retry)
  unsigned long now = millis();
  bool anyPending = false;
  for (uint8_t i = 0; i < PEER_COUNT; i++) {
    if (peers[i].ackPending) {
      anyPending = true;
      // Verifica timeout deste peer
      if (now - peers[i].lastCmdTime >= ACK_TIMEOUT_MS) {
        if (peers[i].retries < MAX_RETRIES) {
          peers[i].retries++;
          peers[i].lastCmdTime = now;
          retryQueue |= (1 << i);
          Serial.printf("[RETRY->%s] %d/%d\n",
                        peers[i].addr, peers[i].retries, MAX_RETRIES);
        } else {
          peers[i].ackPending = false;
          peers[i].retries    = 0;
          anyPending = false;   // este peer desistiu, pode avançar
          Serial.printf("[FALHA->%s] sem ACK\n", peers[i].addr);
          updateDisplay();
        }
      }
      // Se ainda pendente, aguarda — não envia para próximo
      if (peers[i].ackPending) return false;
    }
  }

  // Nenhum peer pendente — envia para o próximo na fila
  uint32_t queue = sendQueue | retryQueue;
  if (queue == 0) return false;

  for (uint8_t i = 0; i < PEER_COUNT; i++) {
    if (queue & (1 << i)) {
      sendQueue  &= ~(1 << i);
      retryQueue &= ~(1 << i);
      doSendToPeer(i, pumpDesired);
      return true;
    }
  }
  return false;
}

// Enfileira CMD para todos os peers
void sendCmdAll(bool pumpOn) {
  pumpDesired = pumpOn;
  for (uint8_t i = 0; i < PEER_COUNT; i++) {
    peers[i].ackPending = false;
    peers[i].retries    = 0;
    peers[i].confirmed  = false;
    sendQueue |= (1 << i);
  }
  Serial.printf("[CMD] PUMP=%s para %d peers\n",
                pumpOn ? "ON" : "OFF", PEER_COUNT);
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
    lastPumpOn = now;
    Serial.println("[BOIA] VAZIO -> PUMP=ON para todos");
    sendCmdAll(true);
  } else if (boiaState == BOIA_CHEIO && pumpDesired
             && now - lastPumpOn >= MIN_PUMP_ON_MS) {
    lastPumpOff = now;
    Serial.println("[BOIA] CHEIO -> PUMP=OFF para todos");
    sendCmdAll(false);
  }
}

// ══════════════════════════════════════════════════════════
//  RECEPÇÃO
// ══════════════════════════════════════════════════════════

void handleReceived() {
  Serial.printf("[RX] \"%s\" RSSI=%d\n", rxBuf, lastRssi);
  char fields[TOTAL_FIELDS][32];
  if (!parseMsg(rxBuf, fields)) {
    Serial.println("[RX] descartado: parse falhou");
    return;
  }
  Serial.printf("[RX] SRC=%s DST=%s SEQ=%s TYPE=%s PAY=%s\n",
                fields[0], fields[1], fields[2], fields[3], fields[4]);
  if (strcmp(fields[3], "ACK") != 0) {
    Serial.printf("[RX] descartado: TYPE=%s (esperava ACK)\n", fields[3]);
    return;
  }

  const char *src    = fields[0];
  uint16_t   ackSeq  = (uint16_t)strtol(fields[2], nullptr, 16);

  for (uint8_t i = 0; i < PEER_COUNT; i++) {
    if (strcmp(peers[i].addr, src) == 0) {
      Serial.printf("[ACK %s] recebido SEQ=%04X esperava SEQ=%04X\n",
                    src, ackSeq, peers[i].seq);
      if (ackSeq != peers[i].seq) {
        Serial.printf("[ACK %s] SEQ ignorado\n", src);
        return;
      }
      peers[i].ackPending = false;
      peers[i].retries    = 0;
      peers[i].confirmed  = true;
      peers[i].relayState = (strstr(fields[4], "RELAY=1") != nullptr);
      Serial.printf("[ACK %s] OK RELAY=%d PUMP=%s\n",
                    src, peers[i].relayState,
                    strstr(fields[4], "PUMP=ON") ? "ON" : "OFF");
      updateDisplay();
      bool pumpOn = (strstr(fields[4], "PUMP=ON") != nullptr);
      if (pumpOn != pumpDesired) {
        Serial.printf("[ALERTA %s] divergente -> reenviar\n", src);
        sendQueue |= (1 << i);
      }
      return;
    }
  }
  Serial.printf("[ACK] SRC=%s nao encontrado na lista de peers\n", src);
}

// ══════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
  Serial.printf("\nNODE A — Sensor de Boia (%d peers)\n", PEER_COUNT);

  pinMode(Vext, OUTPUT); digitalWrite(Vext, LOW); delay(100);
  display.init(); display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "NODE A iniciando...");
  display.display();

  // Inicializa tabela de peers
  for (uint8_t i = 0; i < PEER_COUNT; i++) {
    peers[i].addr        = PEER_LIST[i];
    peers[i].seq         = 0;
    peers[i].ackPending  = false;
    peers[i].retries     = 0;
    peers[i].lastCmdTime = 0;
    peers[i].confirmed   = false;
    peers[i].relayState  = false;
  }

  // Boia
  pinMode(BOIA_PIN, INPUT_PULLUP);
  boiaState = boiaPrev = digitalRead(BOIA_PIN);
  lastPumpOff = millis();

  if (boiaState == BOIA_VAZIO) {
    pumpDesired = true;
    lastPumpOn  = millis();
    Serial.println("[BOOT] boia VAZIO — CMD PUMP=ON sera enviado");
  } else {
    Serial.println("[BOOT] boia CHEIO");
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
  updateDisplay();
}

// ══════════════════════════════════════════════════════════
//  LOOP
// ══════════════════════════════════════════════════════════

void loop() {
  unsigned long now = millis();
  Radio.IrqProcess();

  // ── CMD inicial de boot ───────────────────────────────
  static bool bootDone = false;
  if (!bootDone) {
    bootDone = true;
    if (boiaState == BOIA_VAZIO) {
      sendCmdAll(true);
    }
  }

  // ── TX concluído → volta ao RX ───────────────────────
  if (evTxDone) {
    evTxDone = false;
    txBusy   = false;
    Radio.Rx(0);                           // abre RX ANTES de qualquer outra coisa
    Serial.println("[RX] RX reaberto");    // print depois — não bloqueia o RX
  }

  // ── Pacote recebido ───────────────────────────────────
  if (evRxDone) {
    evRxDone = false;
    Radio.Rx(0);        // reabre RX imediatamente — antes de processar
    handleReceived();   // processamento pode demorar, RX já está aberto
  }

  // ── Erro de RX ───────────────────────────────────────
  if (evRxError) {
    evRxError = false;
    Radio.Rx(0);
  }

  // ── Boia ─────────────────────────────────────────────
  if (readBoia()) {
    Serial.printf("[BOIA] %s\n",
                  boiaState == BOIA_VAZIO ? "VAZIO" : "CHEIO");
    evaluatePump();
    updateDisplay();
  }

  // ── Processa fila de envio ───────────────────────────
  processQueue();

  // ── Diagnóstico a cada 15 s ───────────────────────────
  static unsigned long lastDiag = 0;
  if (now - lastDiag >= 15000) {
    lastDiag = now;
    Serial.printf("[DIAG] boia=%s pump=%d queue=%lu uptime=%lus\n",
                  boiaState == BOIA_VAZIO ? "VAZIO" : "CHEIO",
                  pumpDesired, sendQueue | retryQueue, now / 1000);
    for (uint8_t i = 0; i < PEER_COUNT; i++) {
      Serial.printf("  peer %s: ack=%d retry=%d confirmed=%d relay=%d\n",
                    peers[i].addr, peers[i].ackPending,
                    peers[i].retries, peers[i].confirmed,
                    peers[i].relayState);
    }
  }
}

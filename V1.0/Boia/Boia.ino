
/**
 * ============================================================
 *  NODE A — SENSOR DE BOIA (Caixa d'água)
 *  Hardware : Heltec WiFi LoRa 32 (V2 / V3)
 *  Biblioteca: heltec-unofficial
 *
 *  Função: Lê o estado da boia no pino D2 e envia comandos
 *          ao Node B para ligar/desligar a bomba d'água.
 *
 *  LÓGICA DE CONTROLE:
 *    Boia ABERTA  (nível BAIXO)  → tanque vazio → liga bomba
 *    Boia FECHADA (nível CHEIO)  → tanque cheio → desliga bomba
 *    Proteção contra liga/desliga rápido (debounce + tempo mínimo)
 *
 *  PROTOCOLO LoRa (separador "|")
 *  ┌──────┬──────┬──────┬──────────┬─────────────────────────┬─────┐
 *  │ SRC  │ DST  │ SEQ  │  TYPE    │        PAYLOAD           │ CRC │
 *  ├──────┼──────┼──────┼──────────┼─────────────────────────┼─────┤
 *  │  A   │  B   │ 0001 │ CMD      │ PUMP=ON;BOIA=BAIXO       │ XX  │
 *  │  A   │  B   │ 0002 │ CMD      │ PUMP=OFF;BOIA=CHEIO      │ XX  │
 *  │  A   │  B   │ 0003 │ PING     │                          │ XX  │
 *  │  B   │  A   │ 0001 │ ACK      │                          │ XX  │
 *  │  B   │  A   │ 0003 │ PONG     │                          │ XX  │
 *  │  B   │  A   │ 0010 │ STATUS   │ PUMP=ON;RELAY=1          │ XX  │
 *  └──────┴──────┴──────┴──────────┴─────────────────────────┴─────┘
 *
 *  Tipos de mensagem:
 *    CMD    → Node A ordena ao Node B ligar ou desligar a bomba
 *    ACK    → Confirmação de recebimento (sem payload)
 *    PING   → Verifica se o link está ativo
 *    PONG   → Resposta ao PING
 *    STATUS → Node B reporta estado atual do relé/bomba
 *
 *  Payload do CMD:
 *    PUMP=ON  ou PUMP=OFF  — ação solicitada
 *    BOIA=BAIXO ou BOIA=CHEIO — estado atual da boia
 *
 *  Payload do STATUS (enviado pelo Node B):
 *    PUMP=ON/OFF  — estado da bomba
 *    RELAY=1/0    — estado físico do pino de relé
 * ============================================================
 */

#include "heltec.h"


// ── Identificação ──────────────────────────────────────────
#define MY_ADDRESS   "A"
#define PEER_ADDRESS "B"

// ── Hardware ───────────────────────────────────────────────
#define BOIA_PIN       2      // D2 — entrada da boia (pull-up interno)
//  Boia normalmente fechada (NC):
//    LOW  = boia aberta  = tanque VAZIO  → ligar bomba
//    HIGH = boia fechada = tanque CHEIO  → desligar bomba
#define BOIA_VAZIO     LOW
#define BOIA_CHEIO     HIGH

// ── Parâmetros LoRa ────────────────────────────────────────
#define LORA_FREQUENCY    915E6
#define LORA_BANDWIDTH    125E3
#define LORA_SF           9
#define LORA_CR           5
#define LORA_TX_POWER     14
#define LORA_PREAMBLE     8
#define LORA_SYNC_WORD    0x12

// ── Temporização ──────────────────────────────────────────
#define ACK_TIMEOUT_MS        3000    // tempo máximo aguardando ACK
#define MAX_RETRIES           5       // tentativas antes de desistir (crítico: mais que antes)
#define PING_INTERVAL_MS      15000   // keepalive a cada 15 s
#define DEBOUNCE_MS           2000    // debounce da boia (2 s)
#define STATUS_REQUEST_MS     30000   // pede status ao Node B a cada 30 s
#define MIN_PUMP_ON_MS        10000   // bomba fica ligada no mínimo 10 s
#define MIN_PUMP_OFF_MS       5000    // bomba fica desligada no mínimo 5 s

// ── Protocolo ─────────────────────────────────────────────
#define SEP          "|"
#define FIELD_SRC     0
#define FIELD_DST     1
#define FIELD_SEQ     2
#define FIELD_TYPE    3
#define FIELD_PAYLOAD 4
#define FIELD_CRC     5
#define TOTAL_FIELDS  6

// ── Estado global ─────────────────────────────────────────
uint16_t      txSeq           = 0;
uint16_t      lastRxSeq       = 0xFFFF;
bool          waitingAck      = false;
String        pendingMsg      = "";
uint8_t       retries         = 0;
unsigned long lastAckRequest  = 0;
unsigned long lastPing        = 0;
unsigned long lastStatusReq   = 0;

// Estado da boia e da bomba
int           boiaState       = BOIA_CHEIO;   // estado atual lido
int           boiaPrevState   = BOIA_CHEIO;   // estado anterior confirmado
unsigned long boiaChangedAt   = 0;            // quando mudou (debounce)
bool          boiaPending     = false;        // mudança aguardando debounce

bool          pumpDesired     = false;        // o que Node A quer da bomba
bool          pumpConfirmed   = false;        // último estado confirmado pelo Node B
unsigned long lastPumpOn      = 0;
unsigned long lastPumpOff     = 0;

// ══════════════════════════════════════════════════════════
//  UTILITÁRIOS DE PROTOCOLO
// ══════════════════════════════════════════════════════════

uint8_t calcCRC(const String &s) {
  uint8_t crc = 0;
  for (size_t i = 0; i < s.length(); i++) crc ^= (uint8_t)s[i];
  return crc;
}

String toHex(uint8_t v) {
  char buf[3];
  snprintf(buf, sizeof(buf), "%02X", v);
  return String(buf);
}

String buildMsg(const String &type, const String &payload, uint16_t seq) {
  char seqBuf[6];
  snprintf(seqBuf, sizeof(seqBuf), "%04X", seq);
  String body = String(MY_ADDRESS) + SEP +
                String(PEER_ADDRESS) + SEP +
                String(seqBuf) + SEP +
                type + SEP +
                payload;
  return body + SEP + toHex(calcCRC(body));
}

bool parseMsg(const String &raw, String fields[]) {
  String tmp = raw;
  int count = 0;
  while (count < TOTAL_FIELDS - 1) {
    int idx = tmp.indexOf('|');
    if (idx < 0) return false;
    fields[count++] = tmp.substring(0, idx);
    tmp = tmp.substring(idx + 1);
  }
  fields[TOTAL_FIELDS - 1] = tmp;
  if (count < TOTAL_FIELDS - 1) return false;
  String body = "";
  for (int i = 0; i < FIELD_CRC; i++) {
    if (i > 0) body += SEP;
    body += fields[i];
  }
  uint8_t expected = calcCRC(body);
  uint8_t received = (uint8_t)strtol(fields[FIELD_CRC].c_str(), nullptr, 16);
  return expected == received;
}

// ══════════════════════════════════════════════════════════
//  ENVIO LoRa
// ══════════════════════════════════════════════════════════

void sendRaw(const String &msg) {
  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.endPacket();
  Serial.println("[TX] " + msg);
}

void sendAck(uint16_t seq) {
  char seqBuf[6];
  snprintf(seqBuf, sizeof(seqBuf), "%04X", seq);
  String body = String(MY_ADDRESS) + SEP +
                String(PEER_ADDRESS) + SEP +
                String(seqBuf) + SEP +
                "ACK" + SEP + "";
  sendRaw(body + SEP + toHex(calcCRC(body)));
}

void sendPong(uint16_t seq) {
  char seqBuf[6];
  snprintf(seqBuf, sizeof(seqBuf), "%04X", seq);
  String body = String(MY_ADDRESS) + SEP +
                String(PEER_ADDRESS) + SEP +
                String(seqBuf) + SEP +
                "PONG" + SEP + "";
  sendRaw(body + SEP + toHex(calcCRC(body)));
}

/**
 * Envia comando CMD para o Node B com confirmação (ARQ).
 * pumpOn = true  → PUMP=ON
 * pumpOn = false → PUMP=OFF
 */
void sendPumpCmd(bool pumpOn) {
  String boiaLabel = (boiaState == BOIA_VAZIO) ? "BAIXO" : "CHEIO";
  String payload   = String("PUMP=") + (pumpOn ? "ON" : "OFF") +
                     ";BOIA=" + boiaLabel;
  pendingMsg     = buildMsg("CMD", payload, txSeq++);
  waitingAck     = true;
  retries        = 0;
  lastAckRequest = millis();
  sendRaw(pendingMsg);

  Serial.println("[CMD] Enviando: " + payload);
}

void sendPing() {
  sendRaw(buildMsg("PING", "", txSeq++));
}

// ══════════════════════════════════════════════════════════
//  DISPLAY OLED
// ══════════════════════════════════════════════════════════

void updateDisplay(int rssi = 0) {
  Heltec.display->clear();
  Heltec.display->setFont(ArialMT_Plain_10);

  // Cabeçalho
  Heltec.display->drawString(0, 0, "NODE A | BOIA");

  // Estado da boia
  String boiaStr = (boiaState == BOIA_VAZIO) ? "BOIA: VAZIO" : "BOIA: CHEIO";
  Heltec.display->drawString(0, 14, boiaStr);

  // Estado desejado da bomba
  String pumpStr = pumpDesired ? "CMD: LIGAR BOMBA" : "CMD: DESLIGAR";
  Heltec.display->drawString(0, 26, pumpStr);

  // Estado confirmado
  String confStr = pumpConfirmed ? "BOMBA: LIGADA" : "BOMBA: DESLIGADA";
  Heltec.display->drawString(0, 38, confStr);

  // RSSI / link
  if (rssi != 0)
    Heltec.display->drawString(0, 52, "RSSI: " + String(rssi) + " dBm");
  else if (waitingAck)
    Heltec.display->drawString(0, 52, "Aguardando ACK...");

  Heltec.display->display();
}

// ══════════════════════════════════════════════════════════
//  LÓGICA DA BOIA
// ══════════════════════════════════════════════════════════

/**
 * Lê a boia com debounce.
 * Retorna true se houve mudança de estado confirmada.
 */
bool readBoia() {
  int current = digitalRead(BOIA_PIN);
  unsigned long now = millis();

  if (current != boiaPrevState && !boiaPending) {
    // Início de possível mudança
    boiaPending   = true;
    boiaChangedAt = now;
    boiaState     = current;   // atualiza para display imediato
  }

  if (boiaPending && (now - boiaChangedAt >= DEBOUNCE_MS)) {
    // Confirma leitura após debounce
    boiaPending   = false;
    if (current == boiaPrevState) return false;   // ruído, ignorar
    boiaPrevState = current;
    boiaState     = current;
    return true;   // mudança real confirmada
  }
  return false;
}

/**
 * Decide se deve ligar ou desligar a bomba com base na boia,
 * respeitando os tempos mínimos de operação.
 */
void evaluatePump() {
  unsigned long now = millis();

  if (boiaState == BOIA_VAZIO) {
    // Tanque vazio → quer ligar, respeita tempo mínimo desligado
    if (!pumpDesired && (now - lastPumpOff >= MIN_PUMP_OFF_MS)) {
      pumpDesired = true;
      lastPumpOn  = now;
      Serial.println("[BOIA] VAZIO detectado → solicitando LIGAR bomba");
      sendPumpCmd(true);
    }
  } else {
    // Tanque cheio → quer desligar, respeita tempo mínimo ligado
    if (pumpDesired && (now - lastPumpOn >= MIN_PUMP_ON_MS)) {
      pumpDesired  = false;
      lastPumpOff  = now;
      Serial.println("[BOIA] CHEIO detectado → solicitando DESLIGAR bomba");
      sendPumpCmd(false);
    }
  }
}

// ══════════════════════════════════════════════════════════
//  PROCESSAMENTO DE MENSAGEM RECEBIDA
// ══════════════════════════════════════════════════════════

void handleReceived(const String &raw, int rssi) {
  Serial.println("[RX] " + raw + "  RSSI=" + String(rssi));

  String fields[TOTAL_FIELDS];
  if (!parseMsg(raw, fields)) {
    Serial.println("[ERRO] CRC inválido.");
    return;
  }
  if (fields[FIELD_DST] != MY_ADDRESS) return;

  uint16_t seq  = (uint16_t)strtol(fields[FIELD_SEQ].c_str(), nullptr, 16);
  String   type = fields[FIELD_TYPE];
  String   pay  = fields[FIELD_PAYLOAD];

  // ── ACK ───────────────────────────────────────────────
  if (type == "ACK") {
    if (waitingAck) {
      waitingAck = false;
      Serial.println("[ACK] CMD confirmado pelo Node B.");
    }
    updateDisplay(rssi);
    return;
  }

  // ── PONG ──────────────────────────────────────────────
  if (type == "PONG") {
    Serial.println("[PONG] Link OK  RSSI=" + String(rssi));
    updateDisplay(rssi);
    return;
  }

  // Duplicata
  if (seq == lastRxSeq) { sendAck(seq); return; }
  lastRxSeq = seq;

  // ── PING ──────────────────────────────────────────────
  if (type == "PING") {
    sendPong(seq);
    return;
  }

  // ── STATUS (enviado espontaneamente pelo Node B) ───────
  if (type == "STATUS") {
    sendAck(seq);
    // Parse: "PUMP=ON;RELAY=1"  ou  "PUMP=OFF;RELAY=0"
    bool pumpOn = (pay.indexOf("PUMP=ON") >= 0);
    pumpConfirmed = pumpOn;
    Serial.println("[STATUS] Bomba no Node B: " + String(pumpOn ? "LIGADA" : "DESLIGADA"));

    // Segurança: se Node B relata estado diferente do desejado, re-envia CMD
    if (!waitingAck && pumpOn != pumpDesired) {
      Serial.println("[ALERTA] Estado divergente! Reenviando CMD...");
      sendPumpCmd(pumpDesired);
    }
    updateDisplay(rssi);
    return;
  }
}

// ══════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════

void setup() {
  Heltec.begin(true, true, true, true, LORA_FREQUENCY);

  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth((long)LORA_BANDWIDTH);
  LoRa.setCodingRate4(LORA_CR);
  // LoRa.setTxPower(LORA_TX_POWER);
  LoRa.setPreambleLength(LORA_PREAMBLE);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.enableCrc();

  pinMode(BOIA_PIN, INPUT_PULLUP);

  // Leitura inicial da boia
  boiaState     = digitalRead(BOIA_PIN);
  boiaPrevState = boiaState;
  lastPumpOff   = millis();   // garante tempo mínimo antes de ligar

  Serial.println("NODE A — Sensor de Boia pronto.");
  updateDisplay();
}

// ══════════════════════════════════════════════════════════
//  LOOP
// ══════════════════════════════════════════════════════════

void loop() {
  unsigned long now = millis();

  // ── Recepção ─────────────────────────────────────────
  int pktSize = LoRa.parsePacket();
  if (pktSize > 0) {
    String raw = "";
    while (LoRa.available()) raw += (char)LoRa.read();
    handleReceived(raw, LoRa.packetRssi());
  }

  // ── Leitura da boia com debounce ─────────────────────
  if (readBoia()) {
    Serial.println("[BOIA] Mudança confirmada: " +
                   String(boiaState == BOIA_VAZIO ? "VAZIO" : "CHEIO"));
    evaluatePump();
    updateDisplay();
  }

  // ── Retransmissão por timeout de ACK ─────────────────
  if (waitingAck && (now - lastAckRequest >= ACK_TIMEOUT_MS)) {
    if (retries < MAX_RETRIES) {
      retries++;
      lastAckRequest = now;
      Serial.println("[RETRY] tentativa " + String(retries) + "/" + String(MAX_RETRIES));
      sendRaw(pendingMsg);
      updateDisplay();
    } else {
      waitingAck = false;
      retries    = 0;
      Serial.println("[FALHA] Node B sem resposta! CMD perdido.");
      // Alerta visual de falha de comunicação
      Heltec.display->clear();
      Heltec.display->drawString(0, 0,  "!!! FALHA DE LINK !!!");
      Heltec.display->drawString(0, 16, "Node B nao responde");
      Heltec.display->drawString(0, 30, pumpDesired ? "BOMBA DEVERIA LIGAR" : "BOMBA DEVERIA DESLIGAR");
      Heltec.display->display();
    }
  }

  // ── PING periódico (keepalive) ────────────────────────
  if (!waitingAck && (now - lastPing >= PING_INTERVAL_MS)) {
    lastPing = now;
    sendPing();
  }
}

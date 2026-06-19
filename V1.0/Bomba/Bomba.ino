/**
 * ============================================================
 *  NODE B — CONTROLADOR DE BOMBA D'ÁGUA (Relé + Contatora)
 *  Hardware : Heltec WiFi LoRa 32 (V2 / V3)
 *  Biblioteca: heltec-unofficial
 *
 *  Função: Recebe comandos do Node A via LoRa e aciona um
 *          relé que controla a contatora da bomba d'água.
 *
 *  HARDWARE DO RELÉ:
 *    Pino RELAY_PIN → módulo relé → bobina da contatora → motor
 *    RELAY_ON  = LOW  (relé de módulo ativo em LOW — padrão de mercado)
 *    RELAY_OFF = HIGH
 *    Ajuste RELAY_ON/RELAY_OFF se seu módulo for ativo em HIGH.
 *
 *  PROTEÇÕES IMPLEMENTADAS:
 *    • Watchdog: se perder comunicação por WATCHDOG_MS, desliga bomba
 *    • Tempo mínimo ligado: evita ciclos rápidos que danificam o motor
 *    • Confirmação de ACK antes de executar (segurança)
 *    • Reporte periódico de STATUS ao Node A
 *
 *  PROTOCOLO LoRa (separador "|")
 *  ┌──────┬──────┬──────┬──────────┬─────────────────────────┬─────┐
 *  │ SRC  │ DST  │ SEQ  │  TYPE    │        PAYLOAD           │ CRC │
 *  ├──────┼──────┼──────┼──────────┼─────────────────────────┼─────┤
 *  │  A   │  B   │ 0001 │ CMD      │ PUMP=ON;BOIA=BAIXO       │ XX  │
 *  │  A   │  B   │ 0002 │ CMD      │ PUMP=OFF;BOIA=CHEIO      │ XX  │
 *  │  B   │  A   │ 0001 │ ACK      │                          │ XX  │
 *  │  B   │  A   │ 0010 │ STATUS   │ PUMP=ON;RELAY=1          │ XX  │
 *  │  A   │  B   │ 0003 │ PING     │                          │ XX  │
 *  │  B   │  A   │ 0003 │ PONG     │                          │ XX  │
 *  └──────┴──────┴──────┴──────────┴─────────────────────────┴─────┘
 * ============================================================
 */

#include "heltec.h"

// ── Identificação ──────────────────────────────────────────
#define MY_ADDRESS   "B"
#define PEER_ADDRESS "A"

// ── Hardware ───────────────────────────────────────────────
#define RELAY_PIN     26      // Pino de saída para o módulo relé
                              // (use um pino livre do Heltec, ex: 26)
#define RELAY_ON      LOW     // LOW ativa o relé (módulo com lógica invertida)
#define RELAY_OFF     HIGH    // HIGH desativa
//  Se seu módulo de relé é ativo em HIGH, troque:
//    RELAY_ON  HIGH
//    RELAY_OFF LOW

// ── Parâmetros LoRa (IDÊNTICOS ao Node A) ─────────────────
#define LORA_FREQUENCY    915E6
#define LORA_BANDWIDTH    125E3
#define LORA_SF           9
#define LORA_CR           5
#define LORA_TX_POWER     14
#define LORA_PREAMBLE     8
#define LORA_SYNC_WORD    0x12

// ── Temporização ──────────────────────────────────────────
#define ACK_TIMEOUT_MS      3000
#define MAX_RETRIES         3
#define STATUS_INTERVAL_MS  20000   // envia STATUS ao Node A a cada 20 s
#define WATCHDOG_MS         60000   // desliga bomba se não receber nada em 60 s
#define MIN_PUMP_ON_MS      8000    // bomba fica ligada no mínimo 8 s após CMD ON

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
uint16_t      txSeq          = 0;
uint16_t      lastRxSeq      = 0xFFFF;
bool          waitingAck     = false;
String        pendingMsg     = "";
uint8_t       retries        = 0;
unsigned long lastAckReq     = 0;
unsigned long lastStatusTx   = 0;
unsigned long lastRxTime     = 0;   // para watchdog

// Estado da bomba/relé
bool          pumpState      = false;    // true = ligada
unsigned long pumpOnTime     = 0;        // quando ligou

// ══════════════════════════════════════════════════════════
//  CONTROLE DO RELÉ
// ══════════════════════════════════════════════════════════

void pumpOn() {
  if (pumpState) return;   // já ligada
  digitalWrite(RELAY_PIN, RELAY_ON);
  pumpState = true;
  pumpOnTime = millis();
  Serial.println("[RELE] LIGADO — bomba acionada.");
}

void pumpOff() {
  // Respeita tempo mínimo para não ciclar o motor
  if (pumpState && (millis() - pumpOnTime < MIN_PUMP_ON_MS)) {
    Serial.println("[RELE] Aguardando tempo mínimo para desligar...");
    return;
  }
  digitalWrite(RELAY_PIN, RELAY_OFF);
  pumpState = false;
  Serial.println("[RELE] DESLIGADO — bomba parada.");
}

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

/** Envia STATUS ao Node A: informa estado atual da bomba */
void sendStatus() {
  String relay  = pumpState ? "1" : "0";
  String pump   = pumpState ? "ON" : "OFF";
  String payload = "PUMP=" + pump + ";RELAY=" + relay;
  pendingMsg   = buildMsg("STATUS", payload, txSeq++);
  waitingAck   = true;
  retries      = 0;
  lastAckReq   = millis();
  sendRaw(pendingMsg);
}

// ══════════════════════════════════════════════════════════
//  DISPLAY OLED
// ══════════════════════════════════════════════════════════

void updateDisplay(const String &event = "", int rssi = 0) {
  Heltec.display->clear();
  Heltec.display->setFont(ArialMT_Plain_10);

  Heltec.display->drawString(0, 0, "NODE B | BOMBA");

  // Estado do relé / bomba
  String pumpStr = pumpState ? "BOMBA: LIGADA  [ON]" : "BOMBA: DESLIGADA";
  Heltec.display->drawString(0, 14, pumpStr);

  // Tempo ligada
  if (pumpState) {
    unsigned long sec = (millis() - pumpOnTime) / 1000;
    Heltec.display->drawString(0, 26, "Tempo ON: " + String(sec) + "s");
  } else {
    Heltec.display->drawString(0, 26, "Relé: OFF");
  }

  // Último evento
  if (event.length() > 0)
    Heltec.display->drawString(0, 38, event.substring(0, 21));

  // RSSI ou watchdog
  if (rssi != 0)
    Heltec.display->drawString(0, 52, "RSSI: " + String(rssi) + " dBm");

  Heltec.display->display();
}

// ══════════════════════════════════════════════════════════
//  PROCESSAMENTO DE MENSAGEM RECEBIDA
// ══════════════════════════════════════════════════════════

void handleReceived(const String &raw, int rssi) {
  Serial.println("[RX] " + raw + "  RSSI=" + String(rssi));
  lastRxTime = millis();   // resetar watchdog a cada recepção válida (antes do parse)

  String fields[TOTAL_FIELDS];
  if (!parseMsg(raw, fields)) {
    Serial.println("[ERRO] CRC inválido.");
    return;
  }
  if (fields[FIELD_DST] != MY_ADDRESS) return;

  lastRxTime = millis();   // confirma após parse bem-sucedido

  uint16_t seq  = (uint16_t)strtol(fields[FIELD_SEQ].c_str(), nullptr, 16);
  String   type = fields[FIELD_TYPE];
  String   pay  = fields[FIELD_PAYLOAD];

  // ── ACK (confirmação do STATUS que enviamos) ────────────
  if (type == "ACK") {
    if (waitingAck) {
      waitingAck = false;
      Serial.println("[ACK] STATUS confirmado pelo Node A.");
    }
    return;
  }

  // ── PONG ───────────────────────────────────────────────
  if (type == "PONG") {
    Serial.println("[PONG] Link OK");
    return;
  }

  // Duplicata
  if (seq == lastRxSeq) {
    Serial.println("[DUP] Reenviando ACK seq=" + fields[FIELD_SEQ]);
    sendAck(seq);
    return;
  }
  lastRxSeq = seq;

  // ── PING ───────────────────────────────────────────────
  if (type == "PING") {
    sendPong(seq);
    updateDisplay("PING recebido", rssi);
    return;
  }

  // ── CMD ────────────────────────────────────────────────
  if (type == "CMD") {
    // Responde ACK IMEDIATAMENTE antes de qualquer processamento
    sendAck(seq);

    // Parse do payload: "PUMP=ON;BOIA=BAIXO" ou "PUMP=OFF;BOIA=CHEIO"
    bool cmdOn = (pay.indexOf("PUMP=ON") >= 0);

    // Extrai estado da boia para log
    String boiaInfo = "";
    int bi = pay.indexOf("BOIA=");
    if (bi >= 0) boiaInfo = pay.substring(bi);   // "BOIA=BAIXO" ou "BOIA=CHEIO"

    Serial.println("[CMD] " + String(cmdOn ? "LIGAR" : "DESLIGAR") +
                   " bomba  (" + boiaInfo + ")");

    if (cmdOn) {
      pumpOn();
      updateDisplay("CMD: LIGAR  " + boiaInfo, rssi);
    } else {
      pumpOff();
      updateDisplay("CMD: DESLIGAR " + boiaInfo, rssi);
    }

    // Envia STATUS logo após executar o comando
    if (!waitingAck) sendStatus();
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
  LoRa.setTxPower(LORA_TX_POWER);
  LoRa.setPreambleLength(LORA_PREAMBLE);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.enableCrc();

  // Relé começa DESLIGADO (segurança)
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF);
  pumpState = false;

  lastRxTime = millis();   // inicializa watchdog

  Serial.println("NODE B — Controlador de Bomba pronto. Relé: OFF.");
  updateDisplay("Aguardando CMD...");
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

  // ── Watchdog de comunicação ───────────────────────────
  // Se ficou muito tempo sem receber nada do Node A, desliga por segurança
  if (pumpState && (now - lastRxTime >= WATCHDOG_MS)) {
    Serial.println("[WATCHDOG] Sem comunicacao por " +
                   String(WATCHDOG_MS / 1000) + "s — DESLIGANDO BOMBA!");
    digitalWrite(RELAY_PIN, RELAY_OFF);
    pumpState = false;

    Heltec.display->clear();
    Heltec.display->drawString(0, 0,  "!!! WATCHDOG !!!");
    Heltec.display->drawString(0, 14, "Sem sinal do Node A");
    Heltec.display->drawString(0, 28, "BOMBA DESLIGADA");
    Heltec.display->drawString(0, 42, "por seguranca");
    Heltec.display->display();

    lastRxTime = now;   // reseta para não ficar em loop de alerta
  }

  // ── Retransmissão de STATUS sem ACK ──────────────────
  if (waitingAck && (now - lastAckReq >= ACK_TIMEOUT_MS)) {
    if (retries < MAX_RETRIES) {
      retries++;
      lastAckReq = now;
      sendRaw(pendingMsg);
    } else {
      waitingAck = false;
      retries    = 0;
      Serial.println("[STATUS] Node A não confirmou STATUS.");
    }
  }

  // ── Reporte periódico de STATUS ───────────────────────
  if (!waitingAck && (now - lastStatusTx >= STATUS_INTERVAL_MS)) {
    lastStatusTx = now;
    sendStatus();
    updateDisplay("STATUS enviado");
  }

  // ── Atualiza display com tempo ligado ─────────────────
  if (pumpState && (now % 5000 < 50)) {   // atualiza display a cada ~5 s
    updateDisplay();
  }
}

/**
 * ============================================================
 *  GATEWAY DE TELEMETRIA — LoRa -> MQTT (Ethernet W5500)
 *  Hardware : Heltec WiFi LoRa 32 V4 + módulo W5500
 *  Biblioteca: Heltec ESP32 Dev-Boards (LoRaWan_APP.h)
 *              ESP32 Arduino core 3.x (ETH.h com W5500)
 *              PubSubClient (knolleary)
 *
 *  FLUXO:
 *    Sensor --TEL--> Gateway --ACK--> Sensor
 *                    Gateway --MQTT--> Mosquitto
 *
 *  MQTT:
 *    <BASE>/<SRC>/<chave>  um tópico por campo do payload, só com o
 *                          valor, retido. Ex. "N1|GW|..|TEL|pct=62;vbat=3.98"
 *                          -> lora/N1/pct = 62, lora/N1/vbat = 3.98.
 *                          Sensores novos (temp, tensão de fase...) não
 *                          exigem mudança no gateway.
 *    <BASE>/gateway/status "online" / "offline" (LWT), retido
 *    <BASE>/gateway/info   JSON com uptime, IP e contadores (60 s)
 *
 *  DISPLAY:
 *    Uma tela por sensor, em rodízio a cada 5 s: RSSI, idade da
 *    última leitura, valores com unidade (tabela UNITS) e bateria.
 *
 *  W5500:
 *    O SPI do LoRa (GPIO 9/10/11) não sai no conector da V4, então
 *    o W5500 usa um segundo barramento (SPI3_HOST) em pinos livres.
 *    O LoRa fica sozinho no SPI global (FSPI).
 *
 *  CONFIGURAÇÃO:
 *    Copie secrets.example.h para secrets.h e preencha o broker.
 * ============================================================
 */

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"
#include <ETH.h>
#include <SPI.h>
#include <PubSubClient.h>
#include "TelemetriaConfig.h"   // rádio + protocolo compartilhados (../common)
#include "secrets.h"            // broker MQTT (não versionado)

// ── Identificação ──────────────────────────────────────────
#define MY_ADDRESS GATEWAY_ADDRESS

// ── W5500 (SPI3_HOST) — confira na serigrafia da sua V4 ────
// Evite: 1 (VBAT), 2/5/7/46 (FEM), 8–14 (LoRa), 17/18/21 (OLED),
// 19/20 (USB), 26–32 (flash/PSRAM), 36 (Vext), 37 (ADC_Ctrl).
#define ETH_SCK    47
#define ETH_MISO   48
#define ETH_MOSI   33
#define ETH_CS     34
#define ETH_INT     4
#define ETH_RST     6
#define ETH_SPI_MHZ 20

// ── Temporização ──────────────────────────────────────────
#define MQTT_RETRY_MS    5000
#define INFO_EVERY_MS   60000

// ── Fila de publicação (enquanto o MQTT está fora) ─────────
#define QUEUE_LEN   32   // uma mensagem por campo
#define TOPIC_SIZE  64
#define VALUE_SIZE  12
#define JSON_SIZE  128   // gateway/info

// ── Sensores conhecidos (dedup + display) ─────────────────
#define MAX_SOURCES 16

// Unidade de cada chave, só para o display (o MQTT leva o número puro).
// Chave fora da tabela aparece sem unidade.
struct Unit { const char *key; const char *unit; };
static const Unit UNITS[] = {
  { "pct",   "%"  }, { "dist", "cm" }, { "nivel", "cm" },
  { "temp",  "°C" }, { "umid", "%"  },
  { "va",    "V"  }, { "vb",   "V"  }, { "vc",    "V"  },
  { "vbat",  "V"  },
};

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
static volatile int  lastSnr   = 0;

// ── Rede ──────────────────────────────────────────────────
static volatile bool ethUp = false;
static NetworkClient netClient;
static PubSubClient  mqtt(netClient);
static char topicStatus[TOPIC_SIZE];

// ── Estado da aplicação ───────────────────────────────────
struct QueuedMsg { char topic[TOPIC_SIZE]; char value[VALUE_SIZE]; };
static QueuedMsg queue[QUEUE_LEN];
static uint8_t   qHead = 0, qCount = 0;

struct Source {
  char          src[8];
  uint16_t      seq;
  char          pay[FIELD_SIZE];   // último payload aceito
  int           rssi;
  unsigned long rxMs;
  bool          used;
};
static Source sources[MAX_SOURCES];
static uint8_t dispIdx = 0;        // sensor mostrado no display

bool          txBusy       = false;
unsigned long lastMqttTry  = 0;
uint32_t      rxCount      = 0;
uint32_t      pubCount     = 0;
uint32_t      dupCount     = 0;

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
  lastSnr  = snr;
  evRxDone = true;
}
void OnRxTimeout(void) { /* continua em RX — normal */ }
void OnRxError(void)   { evRxError = true; }

void onNetEvent(arduino_event_id_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      ETH.setHostname(MQTT_CLIENT_ID);
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      ethUp = true;
      Serial.printf("[ETH] IP %s\n", ETH.localIP().toString().c_str());
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
    case ARDUINO_EVENT_ETH_STOP:
      ethUp = false;
      Serial.println("[ETH] link caiu");
      break;
    default:
      break;
  }
}

// ══════════════════════════════════════════════════════════
//  DISPLAY
// ══════════════════════════════════════════════════════════

const char *unitFor(const char *key) {
  for (const Unit &u : UNITS) if (strcmp(u.key, key) == 0) return u.unit;
  return "";
}

// "pct=62;dist=87;vbat=3.98" -> vals "62% 87cm", bat "3.98V".
// err=<motivo> vira "ERRO <motivo>"; err=ok é omitido.
void formatPayload(const char *pay, char *vals, size_t valsSize,
                   char *bat, size_t batSize) {
  char tmp[FIELD_SIZE];
  strncpy(tmp, pay, FIELD_SIZE - 1); tmp[FIELD_SIZE - 1] = '\0';
  size_t len = 0;
  vals[0] = '\0';
  snprintf(bat, batSize, "--");
  char *save = nullptr;
  for (char *kv = strtok_r(tmp, ";", &save); kv; kv = strtok_r(nullptr, ";", &save)) {
    char *eq = strchr(kv, '=');
    if (!eq) continue;
    *eq = '\0';
    const char *val = eq + 1;
    if (strcmp(kv, "vbat") == 0) { snprintf(bat, batSize, "%sV", val); continue; }
    if (strcmp(kv, "err") == 0 && strcmp(val, "ok") == 0) continue;
    if (len >= valsSize) break;
    if (strcmp(kv, "err") == 0)
      len += snprintf(vals + len, valsSize - len, "%sERRO %s", len ? " " : "", val);
    else
      len += snprintf(vals + len, valsSize - len, "%s%s%s", len ? " " : "", val, unitFor(kv));
  }
}

void updateDisplay() {
  char buf[40], bat[12];
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0,  0, "GATEWAY | TELEMETRIA");
  display.drawHorizontalLine(0, 12, 128);
  display.drawString(0, 14, ethUp ? ETH.localIP().toString() : "ETH: sem link");
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(128, 14, mqtt.connected() ? "MQTT ok" : "MQTT --");

  // Próximo sensor conhecido a partir de dispIdx
  Source *s = nullptr;
  for (int i = 0; i < MAX_SOURCES && !s; i++) {
    uint8_t k = (dispIdx + i) % MAX_SOURCES;
    if (sources[k].used) { s = &sources[k]; dispIdx = k; }
  }

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  if (!s) {
    display.drawString(0, 28, "Aguardando sensores...");
  } else {
    unsigned long age = (millis() - s->rxMs) / 1000;
    snprintf(buf, sizeof(buf), "%s  %ddBm", s->src, s->rssi);
    display.drawString(0, 26, buf);
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    if (age < 120) snprintf(buf, sizeof(buf), "%lus", age);
    else           snprintf(buf, sizeof(buf), "%lum", age / 60);
    display.drawString(128, 26, buf);

    display.setTextAlignment(TEXT_ALIGN_LEFT);
    formatPayload(s->pay, buf, sizeof(buf), bat, sizeof(bat));
    display.drawString(0, 38, buf);
    snprintf(buf, sizeof(buf), "Bat: %s", bat);
    display.drawString(0, 50, buf);
  }
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  snprintf(buf, sizeof(buf), "RX:%lu F:%u", (unsigned long)rxCount, qCount);
  display.drawString(128, 50, buf);
  display.display();
}

// ══════════════════════════════════════════════════════════
//  MQTT
// ══════════════════════════════════════════════════════════

void enqueue(const char *topic, const char *value) {
  uint8_t idx = (qHead + qCount) % QUEUE_LEN;
  if (qCount == QUEUE_LEN) {            // cheia: descarta a mais antiga
    qHead = (qHead + 1) % QUEUE_LEN;
    qCount--;
    Serial.println("[FILA] cheia — descartando a mais antiga");
  }
  strncpy(queue[idx].topic, topic, TOPIC_SIZE - 1); queue[idx].topic[TOPIC_SIZE - 1] = '\0';
  strncpy(queue[idx].value, value, VALUE_SIZE - 1); queue[idx].value[VALUE_SIZE - 1] = '\0';
  qCount++;
}

void flushQueue() {
  while (qCount && mqtt.connected()) {
    QueuedMsg &m = queue[qHead];
    if (!mqtt.publish(m.topic, m.value, true)) return;   // tenta de novo depois
    Serial.printf("[MQTT] %s %s\n", m.topic, m.value);
    pubCount++;
    qHead = (qHead + 1) % QUEUE_LEN;
    qCount--;
  }
}

void publishInfo() {
  char topic[TOPIC_SIZE], json[JSON_SIZE];
  snprintf(topic, sizeof(topic), "%s/gateway/info", MQTT_TOPIC_BASE);
  snprintf(json, sizeof(json),
           "{\"uptime\":%lu,\"ip\":\"%s\",\"rx\":%lu,\"pub\":%lu,\"dup\":%lu,\"fila\":%u}",
           millis() / 1000, ETH.localIP().toString().c_str(),
           (unsigned long)rxCount, (unsigned long)pubCount,
           (unsigned long)dupCount, qCount);
  mqtt.publish(topic, json, true);
}

void mqttMaintain(unsigned long now) {
  if (!ethUp) return;
  if (mqtt.connected()) { mqtt.loop(); return; }
  if (now - lastMqttTry < MQTT_RETRY_MS) return;
  lastMqttTry = now;

  Serial.printf("[MQTT] conectando em %s:%d...\n", MQTT_HOST, MQTT_PORT);
  const char *user = strlen(MQTT_USER) ? MQTT_USER : nullptr;
  const char *pass = strlen(MQTT_PASS) ? MQTT_PASS : nullptr;
  if (mqtt.connect(MQTT_CLIENT_ID, user, pass, topicStatus, 1, true, "offline")) {
    Serial.println("[MQTT] conectado");
    mqtt.publish(topicStatus, "online", true);
    publishInfo();
    flushQueue();
  } else {
    Serial.printf("[MQTT] falhou, rc=%d\n", mqtt.state());
  }
  updateDisplay();
}

// ══════════════════════════════════════════════════════════
//  TELEMETRIA
// ══════════════════════════════════════════════════════════

// Registro do sensor (cria se for novo). nullptr se a tabela encheu.
Source *findSource(const char *src, bool *isNew) {
  int freeSlot = -1;
  *isNew = false;
  for (int i = 0; i < MAX_SOURCES; i++) {
    if (!sources[i].used) { if (freeSlot < 0) freeSlot = i; continue; }
    if (strcmp(sources[i].src, src) == 0) return &sources[i];
  }
  if (freeSlot < 0) return nullptr;
  Source &s = sources[freeSlot];
  memset(&s, 0, sizeof(s));
  strncpy(s.src, src, sizeof(s.src) - 1);
  s.used = true;
  *isNew = true;
  return &s;
}

// "pct=62;vbat=3.98" -> <BASE>/<src>/pct = 62, <BASE>/<src>/vbat = 3.98
void publishFields(const char *src, const char *pay) {
  char tmp[FIELD_SIZE], topic[TOPIC_SIZE];
  strncpy(tmp, pay, FIELD_SIZE - 1); tmp[FIELD_SIZE - 1] = '\0';
  char *save = nullptr;
  for (char *kv = strtok_r(tmp, ";", &save); kv; kv = strtok_r(nullptr, ";", &save)) {
    char *eq = strchr(kv, '=');
    if (!eq || eq == kv) continue;
    *eq = '\0';
    snprintf(topic, sizeof(topic), "%s/%s/%s", MQTT_TOPIC_BASE, src, kv);
    enqueue(topic, eq + 1);
  }
}

void sendAck(const char *dst, uint16_t seq) {
  char msg[BUFFER_SIZE];
  buildMsg(msg, MY_ADDRESS, dst, seq, "ACK", "OK");
  txBusy = true;
  Radio.Sleep();
  Radio.Send((uint8_t *)msg, strlen(msg));
  Serial.printf("[TX] \"%s\"\n", msg);
}

void handleReceived() {
  Serial.printf("[RX] \"%s\" RSSI=%d SNR=%d\n", rxBuf, lastRssi, lastSnr);
  char fields[TOTAL_FIELDS][FIELD_SIZE];
  if (!parseMsg(rxBuf, fields)) return;
  if (strcmp(fields[1], MY_ADDRESS) != 0) return;
  if (strcmp(fields[3], "TEL") != 0) return;

  const char *src = fields[0];
  uint16_t seq = (uint16_t)strtol(fields[2], nullptr, 16);
  rxCount++;

  // Duplicado = mesmo SEQ do último aceito: o sensor reenviou porque
  // o ACK se perdeu; só respondemos o ACK de novo.
  bool isNew;
  Source *s = findSource(src, &isNew);
  bool dup = s && !isNew && s->seq == seq;

  // ACK primeiro — o sensor está com o RX aberto esperando
  delay(TX_GUARD_MS);
  sendAck(src, seq);

  if (dup) {
    dupCount++;
    Serial.printf("[TEL] %s seq=%04X duplicado — só ACK\n", src, seq);
    updateDisplay();
    return;
  }

  if (s) {
    s->seq  = seq;
    s->rssi = lastRssi;
    s->rxMs = millis();
    strncpy(s->pay, fields[4], FIELD_SIZE - 1); s->pay[FIELD_SIZE - 1] = '\0';
    dispIdx = s - sources;           // mostra quem acabou de chegar
  } else {
    Serial.println("[TEL] tabela de sensores cheia — sem dedup/display");
  }

  publishFields(src, fields[4]);
  flushQueue();
  updateDisplay();
}

// ══════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  pinMode(Vext, OUTPUT); digitalWrite(Vext, LOW); delay(100);
  display.init(); display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "GATEWAY iniciando...");
  display.display();

  // ── Ethernet ──────────────────────────────────────────
  Network.onEvent(onNetEvent);
  if (!ETH.begin(ETH_PHY_W5500, 1, ETH_CS, ETH_INT, ETH_RST,
                 SPI3_HOST, ETH_SCK, ETH_MISO, ETH_MOSI, ETH_SPI_MHZ)) {
    Serial.println("[ETH] W5500 nao encontrado — confira a fiacao");
  }
#ifdef USE_STATIC_IP
  ETH.config(IPAddress(STATIC_IP), IPAddress(STATIC_GATEWAY),
             IPAddress(STATIC_SUBNET), IPAddress(STATIC_DNS));
#endif

  // ── MQTT ──────────────────────────────────────────────
  snprintf(topicStatus, sizeof(topicStatus), "%s/gateway/status", MQTT_TOPIC_BASE);
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(JSON_SIZE + TOPIC_SIZE + 16);
  mqtt.setSocketTimeout(3);

  // ── LoRa ──────────────────────────────────────────────
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

  Serial.printf("[GATEWAY] pronto — %.1f MHz SF%d\n",
                RF_FREQUENCY / 1e6, LORA_SPREADING_FACTOR);
  updateDisplay();
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

  mqttMaintain(now);

  // ── Info periódica ────────────────────────────────────
  static unsigned long lastInfo = 0;
  if (mqtt.connected() && now - lastInfo >= INFO_EVERY_MS) {
    lastInfo = now;
    publishInfo();
  }

  // ── Display a cada 5 s ────────────────────────────────
  static unsigned long lastDisp = 0;
  if (now - lastDisp >= 5000) {
    lastDisp = now;
    updateDisplay();
    dispIdx = (dispIdx + 1) % MAX_SOURCES;   // próximo sensor
  }
}

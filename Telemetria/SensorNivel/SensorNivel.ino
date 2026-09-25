/**
 * ============================================================
 *  SENSOR DE NÍVEL DA CAIXA D'ÁGUA — bateria + deep sleep
 *  Hardware : Heltec WiFi LoRa 32 V4 + JSN-SR04T (ultrassônico)
 *  Biblioteca: Heltec ESP32 Dev-Boards (LoRaWan_APP.h)
 *
 *  CICLO (tudo roda no setup, loop() nunca é alcançado):
 *    1. Liga Vext -> alimenta o JSN-SR04T
 *    2. Faz N leituras e usa a mediana
 *    3. Desliga Vext, lê a bateria
 *    4. Envia TEL ao gateway e espera ACK (com retry)
 *    5. Rádio em sleep, deep sleep por SLEEP_MINUTES
 *
 *  PAYLOAD:
 *    pct=<0-100>;dist=<cm até a água>;vbat=<V>;err=ok
 *    Leitura inválida: err=eco;vbat=<V>  (pct/dist ficam com o último
 *    valor retido no broker)
 *    O gateway publica cada campo em lora/<MY_ADDRESS>/<chave>.
 *
 *  MONTAGEM:
 *    Sensor na tampa, apontado para baixo. O JSN-SR04T tem zona
 *    cega de ~25 cm: a água cheia precisa ficar abaixo disso.
 *    Alimentado por Vext (3,3 V) — use a versão 3.0 do módulo,
 *    que funciona de 3 a 5,5 V. Com 3,3 V o eco também é 3,3 V,
 *    sem divisor de tensão.
 * ============================================================
 */

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "driver/gpio.h"
#include "TelemetriaConfig.h"   // rádio + protocolo compartilhados (../common)

// ── Identificação — um endereço único por sensor ───────────
#define MY_ADDRESS "N1"

// ── Hardware — confira na serigrafia da sua V4 ─────────────
#define TRIG_PIN      47
#define ECHO_PIN      48
#define VBAT_PIN       1    // ADC da bateria (divisor 390k/100k)
#define ADC_CTRL_PIN  37    // HIGH habilita o divisor na V4

// ── Geometria da caixa (cm) ───────────────────────────────
#define DIST_FUNDO_CM   180  // sensor -> fundo (caixa vazia)
#define DIST_CHEIO_CM    30  // sensor -> água com a caixa cheia

// ── Medição ───────────────────────────────────────────────
#define N_LEITURAS          5
#define SENSOR_WARMUP_MS  300  // JSN-SR04T após ligar o Vext
#define ECHO_TIMEOUT_US 30000  // ~5 m
#define DIST_MIN_CM        20
#define DIST_MAX_CM       450

// ── Temporização ──────────────────────────────────────────
#define SLEEP_MINUTES     10
#define ACK_TIMEOUT_MS  1500   // TEL ~350ms + guard 100ms + ACK ~200ms
#define MAX_TENTATIVAS     3

// Sobrevive ao deep sleep
RTC_DATA_ATTR static uint16_t seq     = 0;
RTC_DATA_ATTR static uint32_t semAck  = 0;   // ciclos sem ACK seguidos

static char rxBuf[BUFFER_SIZE];
static RadioEvents_t RadioEvents;

// ── Flags de ISR (voláteis) ───────────────────────────────
static volatile bool evTxDone  = false;
static volatile bool evRxDone  = false;
static volatile bool evRxError = false;

// ══════════════════════════════════════════════════════════
//  CALLBACKS — só setam flags, zero Radio.* aqui
// ══════════════════════════════════════════════════════════

void OnTxDone(void)    { evTxDone = true; }
void OnTxTimeout(void) { evTxDone = true; Serial.println("[TX TIMEOUT]"); }
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  memset(rxBuf, 0, BUFFER_SIZE);
  memcpy(rxBuf, payload, min((int)size, BUFFER_SIZE - 1));
  evRxDone = true;
}
void OnRxTimeout(void) { evRxError = true; }
void OnRxError(void)   { evRxError = true; }

// ══════════════════════════════════════════════════════════
//  MEDIÇÃO
// ══════════════════════════════════════════════════════════

// Uma leitura em cm, ou -1 se não houve eco válido
int readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(20);                 // JSN-SR04T pede >= 10us
  digitalWrite(TRIG_PIN, LOW);
  unsigned long us = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (us == 0) return -1;
  int cm = (int)(us * 0.0343f / 2.0f + 0.5f);
  if (cm < DIST_MIN_CM || cm > DIST_MAX_CM) return -1;
  return cm;
}

// Mediana das leituras válidas, ou -1 se nenhuma
int measureDistanceCm() {
  int vals[N_LEITURAS];
  int n = 0;
  for (int i = 0; i < N_LEITURAS; i++) {
    int cm = readDistanceCm();
    Serial.printf("[SENSOR] leitura %d: %d cm\n", i + 1, cm);
    if (cm > 0) vals[n++] = cm;
    delay(60);                           // deixa o eco anterior morrer
  }
  if (n == 0) return -1;
  for (int i = 1; i < n; i++)            // insertion sort — n é pequeno
    for (int j = i; j > 0 && vals[j - 1] > vals[j]; j--) {
      int t = vals[j]; vals[j] = vals[j - 1]; vals[j - 1] = t;
    }
  return vals[n / 2];
}

float readBatteryV() {
  pinMode(ADC_CTRL_PIN, OUTPUT);
  digitalWrite(ADC_CTRL_PIN, HIGH);
  delay(10);
  uint32_t mv = analogReadMilliVolts(VBAT_PIN);
  digitalWrite(ADC_CTRL_PIN, LOW);
  return mv * (390.0f + 100.0f) / 100.0f / 1000.0f;
}

void setVext(bool on) {
  gpio_hold_dis((gpio_num_t)Vext);
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, on ? LOW : HIGH);   // Vext_Ctrl ativo em LOW
}

// ══════════════════════════════════════════════════════════
//  RÁDIO
// ══════════════════════════════════════════════════════════

void radioInit() {
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
}

// Espera uma flag de evento processando as IRQs do rádio
bool waitFlag(volatile bool &flag, unsigned long timeoutMs) {
  unsigned long t0 = millis();
  while (millis() - t0 < timeoutMs) {
    Radio.IrqProcess();
    if (flag) { flag = false; return true; }
    delay(1);
  }
  return false;
}

// Envia o TEL e espera o ACK do gateway com o mesmo SEQ
bool sendWithAck(const char *payload) {
  char msg[BUFFER_SIZE];
  buildMsg(msg, MY_ADDRESS, GATEWAY_ADDRESS, seq, "TEL", payload);

  for (int t = 1; t <= MAX_TENTATIVAS; t++) {
    evTxDone = evRxDone = evRxError = false;
    Radio.Sleep();
    Radio.Send((uint8_t *)msg, strlen(msg));
    Serial.printf("[TX] tentativa %d: \"%s\"\n", t, msg);
    if (!waitFlag(evTxDone, 3000)) { Serial.println("[TX] sem TxDone"); continue; }

    Radio.Rx(ACK_TIMEOUT_MS);
    unsigned long t0 = millis();
    while (millis() - t0 < ACK_TIMEOUT_MS + 100) {
      Radio.IrqProcess();
      if (evRxError) break;              // timeout ou erro de RX
      if (evRxDone) {
        evRxDone = false;
        char f[TOTAL_FIELDS][FIELD_SIZE];
        if (parseMsg(rxBuf, f) &&
            strcmp(f[1], MY_ADDRESS) == 0 &&
            strcmp(f[3], "ACK") == 0 &&
            (uint16_t)strtol(f[2], nullptr, 16) == seq) {
          Serial.println("[ACK] recebido");
          return true;
        }
        unsigned long elapsed = millis() - t0;       // pacote de outro, continua ouvindo
        if (elapsed >= ACK_TIMEOUT_MS) break;
        Radio.Rx(ACK_TIMEOUT_MS - elapsed);
      }
      delay(1);
    }
    Serial.println("[ACK] timeout");
    delay(random(200, 700));             // evita colidir de novo com outro sensor
  }
  return false;
}

// ══════════════════════════════════════════════════════════
//  SETUP — um ciclo completo e deep sleep
// ══════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  seq++;

  // ── Medição ───────────────────────────────────────────
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  setVext(true);
  delay(SENSOR_WARMUP_MS);
  int dist = measureDistanceCm();
  setVext(false);
  float vbat = readBatteryV();

  char payload[FIELD_SIZE];
  if (dist < 0) {
    snprintf(payload, sizeof(payload), "err=eco;vbat=%.2f", vbat);
  } else {
    int pct = (DIST_FUNDO_CM - dist) * 100 / (DIST_FUNDO_CM - DIST_CHEIO_CM);
    pct = constrain(pct, 0, 100);
    snprintf(payload, sizeof(payload), "pct=%d;dist=%d;vbat=%.2f;err=ok",
             pct, dist, vbat);
  }
  Serial.printf("[SENSOR] %s\n", payload);

  // ── Envio ─────────────────────────────────────────────
  radioInit();
  if (sendWithAck(payload)) semAck = 0;
  else Serial.printf("[TEL] sem ACK (%lu ciclos seguidos)\n", (unsigned long)++semAck);

  // ── Deep sleep ────────────────────────────────────────
  Radio.Sleep();
  setVext(false);
  gpio_hold_en((gpio_num_t)Vext);        // mantém o Vext desligado dormindo
  gpio_deep_sleep_hold_en();
  Serial.printf("[SLEEP] %d min\n", SLEEP_MINUTES);
  Serial.flush();
  esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_MINUTES * 60ULL * 1000000ULL);
  esp_deep_sleep_start();
}

void loop() {
  // nunca alcançado — o ciclo roda no setup e termina em deep sleep
}

/**
 * ============================================================
 *  DISPLAY DE TELEMETRIA — MQTT -> LCD 4,3" (só leitura)
 *  Hardware : Waveshare ESP32-S3-Touch-LCD-4.3B
 *             (800x480 RGB, touch GT911, expansor CH422G)
 *  Pacote   : esp32 da Espressif 3.x (NÃO o da Heltec)
 *  Biblioteca: ESP32_Display_Panel 1.x (+ ESP32_IO_Expander,
 *              esp-lib-utils), LVGL 9.x, PubSubClient (knolleary)
 *
 *  FERRAMENTAS (Arduino IDE):
 *    Placa "Waveshare ESP32-S3-Touch-LCD-4.3B", Flash 16MB,
 *    PSRAM "OPI PSRAM" (sem isso a tela não liga),
 *    Partition "16M Flash (3MB APP/9.9MB FATFS)".
 *    Fonte de 5 V/1 A ou 7–36 V no borne — USB fraco = tela vermelha.
 *
 *  FLUXO:
 *    Gateway --MQTT--> Mosquitto --MQTT--> Display
 *    Só assina, nunca publica: não interfere no gateway.
 *
 *  TÓPICOS ASSINADOS:
 *    <BASE>/#             <BASE>/<SRC>/<chave> = valor (retido). O SRC
 *                         pode ter "/" (ex. geral/reservatorio): a chave
 *                         é o que vem depois da última "/".
 *    <BASE>/gateway/status "online"/"offline" -> cabeçalho
 *    geral/temperatura    DS18B20 do gateway -> cabeçalho
 *
 *  TELA:
 *    Um cartão por SRC, criado na primeira mensagem, em ordem
 *    alfabética. Sensor ou chave nova aparece sem mudar o código.
 *    pct vira número grande + barra; vbat vira V e %; err diferente
 *    de "ok" pinta o cartão de vermelho; leitura mais velha que
 *    STALE_MS pinta de âmbar. Toque no cartão = lista dos tópicos.
 *
 *  IDADE DAS LEITURAS:
 *    O tópico retido não diz quando foi publicado. Mensagens que
 *    chegam logo após o subscribe (RETAINED_WIN_MS) são tratadas
 *    como retidas: idade "--" até chegar uma leitura nova.
 *
 *  FONTES:
 *    As embutidas da LVGL só têm ASCII + "°" — textos sem acento.
 *
 *  CONFIGURAÇÃO:
 *    Copie secrets.example.h para secrets.h (WiFi + broker).
 * ============================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_display_panel.hpp>
#include <lvgl.h>
#include <time.h>
#include "BoardConfig.hpp"       // pinos/timings da 4.3B
#include "TelemetriaUnits.h"     // unidades compartilhadas (../common)
#include "secrets.h"             // WiFi + broker (não versionado)

using namespace esp_panel::board;
using namespace esp_panel::drivers;

// ── Tópicos ────────────────────────────────────────────────
#define TEMP_TOPIC  "geral/temperatura"

// ── Temporização ──────────────────────────────────────────
#define MQTT_RETRY_MS     5000
#define RETAINED_WIN_MS   2000              // após o subscribe = retido
#define STALE_MS   (25UL * 60 * 1000)       // 2,5x o SLEEP_MINUTES do SensorNivel
#define UI_EVERY_MS        500              // junta as mensagens de uma leitura
#define AGE_EVERY_MS     10000              // atualiza "x min" e relógio
#define TOUCH_RELEASE_MS    80              // sem ponto por mais que isso = soltou

// ── Bateria Li 1S (mesma escala do Controle Lotes) ────────
#define VBAT_EMPTY  3.3f
#define VBAT_FULL   4.2f

// ── Relógio ───────────────────────────────────────────────
#define NTP_SERVER  "pool.ntp.org"
#define TZ_INFO     "<-03>3"                // Brasília, sem horário de verão

// ── Tabelas ───────────────────────────────────────────────
#define MAX_SOURCES  12
#define MAX_FIELDS   10
#define SRC_SIZE     32
#define KEY_SIZE     12
#define VAL_SIZE     16
#define MQTT_BUF    512                     // gateway/info chega inteiro

// ── Layout (800x480) ──────────────────────────────────────
#define SCR_W        800
#define SCR_H        480
#define HDR_H         48
#define CARD_W       250
#define CARD_H       200
#define GAP           10
#define LV_BUF_LINES  20                    // 800x20x2 = 32 KB na SRAM (bounce buffers usam 64 KB)

#define C_BG       lv_color_hex(0x101418)
#define C_CARD     lv_color_hex(0x1C232B)
#define C_BORDER   lv_color_hex(0x2C3640)
#define C_TEXT     lv_color_hex(0xE6EAEE)
#define C_MUTED    lv_color_hex(0x8A96A3)
#define C_OK       lv_color_hex(0x3FB950)
#define C_WARN     lv_color_hex(0xD29922)
#define C_ERR      lv_color_hex(0xF85149)
#define C_ACCENT   lv_color_hex(0x2F81F7)

// ── Hardware ──────────────────────────────────────────────
static Board *board = nullptr;
static LCD   *lcd   = nullptr;
static Touch *touch = nullptr;

// ── Rede ──────────────────────────────────────────────────
static WiFiClient   netClient;
static PubSubClient mqtt(netClient);

// ── Estado ────────────────────────────────────────────────
struct Field { char key[KEY_SIZE]; char val[VAL_SIZE]; };

struct Source {
  bool          used;
  char          src[SRC_SIZE];
  Field         f[MAX_FIELDS];
  uint8_t       nf;
  unsigned long rxMs;     // 0 = só valores retidos, idade desconhecida
  bool          dirty;
  lv_obj_t     *card, *title, *big, *bar, *err, *vals, *bat, *age;
};
static Source sources[MAX_SOURCES];

static float         gwTemp     = NAN;
static int8_t        gwOnline   = -1;   // -1 = ainda não sabe
static bool          hdrDirty   = true;
static unsigned long subMs      = 0;    // instante do último subscribe
static unsigned long lastMqttTry = 0;

// ── LVGL ──────────────────────────────────────────────────
static lv_obj_t *grid, *emptyLbl;
static lv_obj_t *hdrWifi, *hdrMqtt, *hdrGw, *hdrTemp, *hdrClock;
static lv_obj_t *detail = nullptr, *detailLbl = nullptr;
static Source   *detailSrc = nullptr;

// ══════════════════════════════════════════════════════════
//  DADOS
// ══════════════════════════════════════════════════════════

const char *fieldVal(const Source &s, const char *key) {
  for (uint8_t i = 0; i < s.nf; i++)
    if (strcmp(s.f[i].key, key) == 0) return s.f[i].val;
  return nullptr;
}

// Payload vazio = tópico retido apagado no broker -> remove o campo.
void setField(Source &s, const char *key, const char *val) {
  for (uint8_t i = 0; i < s.nf; i++) {
    if (strcmp(s.f[i].key, key) != 0) continue;
    if (*val) { strlcpy(s.f[i].val, val, VAL_SIZE); return; }
    s.f[i] = s.f[--s.nf];
    return;
  }
  if (!*val || s.nf == MAX_FIELDS) return;
  strlcpy(s.f[s.nf].key, key, KEY_SIZE);
  strlcpy(s.f[s.nf].val, val, VAL_SIZE);
  s.nf++;
}

Source *findSource(const char *src) {
  int freeSlot = -1;
  for (int i = 0; i < MAX_SOURCES; i++) {
    if (!sources[i].used) { if (freeSlot < 0) freeSlot = i; continue; }
    if (strcmp(sources[i].src, src) == 0) return &sources[i];
  }
  if (freeSlot < 0) { Serial.println("[MQTT] tabela de sensores cheia"); return nullptr; }
  Source &s = sources[freeSlot];
  memset(&s, 0, sizeof(s));
  strlcpy(s.src, src, SRC_SIZE);
  s.used = true;
  return &s;
}

// ══════════════════════════════════════════════════════════
//  MQTT
// ══════════════════════════════════════════════════════════

void onMqtt(char *topic, byte *payload, unsigned int len) {
  char val[VAL_SIZE];
  unsigned int n = min(len, (unsigned int)(VAL_SIZE - 1));
  memcpy(val, payload, n); val[n] = '\0';
  bool retained = millis() - subMs < RETAINED_WIN_MS;

  if (strcmp(topic, TEMP_TOPIC) == 0) {
    gwTemp = n ? atof(val) : NAN;
    hdrDirty = true;
    return;
  }

  // <BASE>/<SRC>/<chave>, com "/" permitida dentro do SRC
  size_t bl = strlen(MQTT_TOPIC_BASE);
  if (strncmp(topic, MQTT_TOPIC_BASE, bl) != 0 || topic[bl] != '/') return;
  const char *rest  = topic + bl + 1;
  const char *slash = strrchr(rest, '/');
  if (!slash || slash == rest) return;
  size_t sl = slash - rest;
  const char *key = slash + 1;
  if (sl >= SRC_SIZE || !*key || strlen(key) >= KEY_SIZE) return;
  char src[SRC_SIZE];
  memcpy(src, rest, sl); src[sl] = '\0';

  if (strcmp(src, "gateway") == 0) {
    if (strcmp(key, "status") == 0) {
      gwOnline = strcmp(val, "online") == 0;
      hdrDirty = true;
    }
    return;                          // gateway/info não vira cartão
  }

  Source *s = findSource(src);
  if (!s) return;
  setField(*s, key, val);
  if (!retained && n) s->rxMs = millis();
  s->dirty = true;
  Serial.printf("[MQTT] %s = %s%s\n", topic, val, retained ? " (retido)" : "");
}

void mqttMaintain(unsigned long now) {
  if (WiFi.status() != WL_CONNECTED) return;
  if (mqtt.connected()) { mqtt.loop(); return; }
  if (now - lastMqttTry < MQTT_RETRY_MS) return;
  lastMqttTry = now;

  Serial.printf("[MQTT] conectando em %s:%d...\n", MQTT_HOST, MQTT_PORT);
  const char *user = strlen(MQTT_USER) ? MQTT_USER : nullptr;
  const char *pass = strlen(MQTT_PASS) ? MQTT_PASS : nullptr;
  if (mqtt.connect(MQTT_CLIENT_ID, user, pass)) {
    mqtt.subscribe(MQTT_TOPIC_BASE "/#");
    mqtt.subscribe(TEMP_TOPIC);
    subMs = millis();
    Serial.println("[MQTT] conectado, assinando " MQTT_TOPIC_BASE "/# e " TEMP_TOPIC);
  } else {
    Serial.printf("[MQTT] falhou, rc=%d\n", mqtt.state());
  }
  hdrDirty = true;
}

// ══════════════════════════════════════════════════════════
//  LVGL — porta (flush, touch, tick)
// ══════════════════════════════════════════════════════════

void flushCb(lv_display_t *disp, const lv_area_t *a, uint8_t *px) {
  lcd->drawBitmap(a->x1, a->y1, lv_area_get_width(a), lv_area_get_height(a), px);
  lv_display_flush_ready(disp);
}

// O GT911 responde "0 pontos" quando ainda não tem amostra nova, mesmo
// com o dedo na tela. Sem segurar o último ponto por TOUCH_RELEASE_MS,
// um arrasto vira vários toques curtos (abre o detalhe em vez de rolar).
void touchCb(lv_indev_t *, lv_indev_data_t *data) {
  static lv_point_t    last;
  static unsigned long lastSeen = 0;
  static bool          down = false;
  TouchPoint p;
  if (touch && touch->readPoints(&p, 1, 0) > 0) {
    last.x = p.x; last.y = p.y;
    lastSeen = millis();
    down = true;
  } else if (down && millis() - lastSeen > TOUCH_RELEASE_MS) {
    down = false;
  }
  data->point = last;
  data->state = down ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

uint32_t tickCb() { return millis(); }

bool lvglInit() {
  lv_init();
  lv_tick_set_cb(tickCb);
  size_t bufSize = SCR_W * LV_BUF_LINES * sizeof(uint16_t);
  void *buf = heap_caps_malloc(bufSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!buf) buf = heap_caps_malloc(bufSize, MALLOC_CAP_SPIRAM);
  if (!buf) return false;
  lv_display_t *disp = lv_display_create(SCR_W, SCR_H);
  lv_display_set_flush_cb(disp, flushCb);
  lv_display_set_buffers(disp, buf, nullptr, bufSize, LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, touchCb);
  return true;
}

// ══════════════════════════════════════════════════════════
//  TELA
// ══════════════════════════════════════════════════════════

lv_obj_t *mkLabel(lv_obj_t *parent, const lv_font_t *font, lv_color_t color) {
  lv_obj_t *l = lv_label_create(parent);
  lv_obj_set_style_text_font(l, font, 0);
  lv_obj_set_style_text_color(l, color, 0);
  lv_label_set_text(l, "");
  return l;
}

// Caixa transparente sem borda, só para layout
lv_obj_t *mkBox(lv_obj_t *parent) {
  lv_obj_t *o = lv_obj_create(parent);
  lv_obj_remove_style_all(o);
  lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_remove_flag(o, LV_OBJ_FLAG_CLICKABLE);
  return o;
}

void buildScreen() {
  lv_obj_t *scr = lv_screen_active();
  lv_obj_set_style_bg_color(scr, C_BG, 0);
  lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

  // ── Cabeçalho ──
  lv_obj_t *hdr = mkBox(scr);
  lv_obj_set_size(hdr, SCR_W, HDR_H);
  lv_obj_set_style_bg_color(hdr, C_CARD, 0);
  lv_obj_set_style_bg_opa(hdr, LV_OPA_COVER, 0);
  lv_obj_set_style_pad_hor(hdr, 14, 0);
  lv_obj_set_style_pad_column(hdr, 18, 0);
  lv_obj_set_flex_flow(hdr, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(hdr, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

  lv_obj_t *t = mkLabel(hdr, &lv_font_montserrat_26, C_TEXT);
  lv_label_set_text(t, "Telemetria");
  hdrWifi = mkLabel(hdr, &lv_font_montserrat_16, C_MUTED);
  hdrMqtt = mkLabel(hdr, &lv_font_montserrat_16, C_MUTED);
  hdrGw   = mkLabel(hdr, &lv_font_montserrat_16, C_MUTED);
  lv_obj_t *spacer = mkBox(hdr);
  lv_obj_set_height(spacer, 1);
  lv_obj_set_flex_grow(spacer, 1);
  hdrTemp  = mkLabel(hdr, &lv_font_montserrat_26, C_TEXT);
  hdrClock = mkLabel(hdr, &lv_font_montserrat_26, C_MUTED);

  // ── Grade de cartões (rola na vertical se passar de 6) ──
  grid = mkBox(scr);
  lv_obj_add_flag(grid, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_scroll_dir(grid, LV_DIR_VER);
  lv_obj_set_size(grid, SCR_W, SCR_H - HDR_H);
  lv_obj_set_y(grid, HDR_H);
  lv_obj_set_style_pad_all(grid, GAP + 3, 0);
  lv_obj_set_style_pad_gap(grid, GAP, 0);
  lv_obj_set_flex_flow(grid, LV_FLEX_FLOW_ROW_WRAP);

  emptyLbl = mkLabel(scr, &lv_font_montserrat_26, C_MUTED);
  lv_label_set_text(emptyLbl, "Aguardando topicos...");
  lv_obj_align(emptyLbl, LV_ALIGN_CENTER, 0, HDR_H / 2);
}

void onDetailClose(lv_event_t *) {
  lv_obj_delete_async(detail);
  detail = detailLbl = nullptr;
  detailSrc = nullptr;
}

void fillDetail() {
  static char buf[MAX_FIELDS * (SRC_SIZE + KEY_SIZE + VAL_SIZE + 16)];
  size_t len = 0;
  for (uint8_t i = 0; i < detailSrc->nf && len < sizeof(buf); i++)
    len += snprintf(buf + len, sizeof(buf) - len, "%s" MQTT_TOPIC_BASE "/%s/%s  =  %s",
                    i ? "\n" : "", detailSrc->src, detailSrc->f[i].key, detailSrc->f[i].val);
  lv_label_set_text(detailLbl, len ? buf : "(sem topicos)");
}

// Lista crua dos tópicos do sensor; toque em qualquer lugar fecha.
void onCardClick(lv_event_t *e) {
  if (detail) return;
  detailSrc = (Source *)lv_event_get_user_data(e);
  detail = lv_obj_create(lv_layer_top());
  lv_obj_remove_style_all(detail);
  lv_obj_set_size(detail, SCR_W, SCR_H);
  lv_obj_set_style_bg_color(detail, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(detail, LV_OPA_70, 0);
  lv_obj_add_flag(detail, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(detail, onDetailClose, LV_EVENT_CLICKED, nullptr);

  lv_obj_t *panel = lv_obj_create(detail);
  lv_obj_set_size(panel, 640, LV_SIZE_CONTENT);
  lv_obj_center(panel);
  lv_obj_set_style_bg_color(panel, C_CARD, 0);
  lv_obj_set_style_border_color(panel, C_ACCENT, 0);
  lv_obj_set_style_pad_all(panel, 20, 0);
  lv_obj_set_style_pad_row(panel, 12, 0);
  lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_COLUMN);
  lv_obj_remove_flag(panel, LV_OBJ_FLAG_CLICKABLE);   // toque passa para o fundo

  lv_obj_t *t = mkLabel(panel, &lv_font_montserrat_26, C_TEXT);
  lv_label_set_text(t, detailSrc->src);
  detailLbl = mkLabel(panel, &lv_font_montserrat_16, C_TEXT);
  lv_obj_set_style_text_line_space(detailLbl, 8, 0);
  lv_obj_t *hint = mkLabel(panel, &lv_font_montserrat_14, C_MUTED);
  lv_label_set_text(hint, "Toque para fechar");
  fillDetail();
}

void buildCard(Source &s) {
  lv_obj_t *c = lv_obj_create(grid);
  s.card = c;
  lv_obj_set_size(c, CARD_W, CARD_H);
  lv_obj_set_style_bg_color(c, C_CARD, 0);
  lv_obj_set_style_border_color(c, C_BORDER, 0);
  lv_obj_set_style_border_width(c, 2, 0);
  lv_obj_set_style_radius(c, 10, 0);
  lv_obj_set_style_pad_all(c, 12, 0);
  lv_obj_set_style_pad_row(c, 6, 0);
  lv_obj_set_flex_flow(c, LV_FLEX_FLOW_COLUMN);
  lv_obj_remove_flag(c, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_event_cb(c, onCardClick, LV_EVENT_CLICKED, &s);

  s.title = mkLabel(c, &lv_font_montserrat_16, C_MUTED);
  lv_label_set_long_mode(s.title, LV_LABEL_LONG_MODE_DOTS);
  lv_obj_set_width(s.title, lv_pct(100));
  lv_label_set_text(s.title, s.src);

  s.big = mkLabel(c, &lv_font_montserrat_30, C_TEXT);
  s.bar = lv_bar_create(c);
  lv_obj_set_size(s.bar, lv_pct(100), 14);
  lv_bar_set_range(s.bar, 0, 100);
  lv_obj_set_style_bg_color(s.bar, C_BORDER, LV_PART_MAIN);
  lv_obj_set_style_bg_color(s.bar, C_ACCENT, LV_PART_INDICATOR);
  lv_obj_remove_flag(s.bar, LV_OBJ_FLAG_CLICKABLE);

  s.err  = mkLabel(c, &lv_font_montserrat_16, C_ERR);
  s.vals = mkLabel(c, &lv_font_montserrat_16, C_TEXT);
  lv_obj_set_flex_grow(s.vals, 1);                 // empurra o rodapé para baixo

  lv_obj_t *foot = mkBox(c);
  lv_obj_set_size(foot, lv_pct(100), LV_SIZE_CONTENT);
  lv_obj_set_flex_flow(foot, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(foot, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  s.bat = mkLabel(foot, &lv_font_montserrat_14, C_MUTED);
  s.age = mkLabel(foot, &lv_font_montserrat_14, C_MUTED);

  // Ordem alfabética entre os cartões já existentes
  int idx = 0;
  for (const Source &o : sources)
    if (o.used && o.card && &o != &s && strcmp(o.src, s.src) < 0) idx++;
  lv_obj_move_to_index(c, idx);
}

void setHidden(lv_obj_t *o, bool hidden) {
  if (hidden) lv_obj_add_flag(o, LV_OBJ_FLAG_HIDDEN);
  else        lv_obj_remove_flag(o, LV_OBJ_FLAG_HIDDEN);
}

// Idade da última leitura e cor do aviso (cartão + rodapé)
void updateAge(Source &s, unsigned long now) {
  const char *err = fieldVal(s, "err");
  bool hasErr = err && strcmp(err, "ok") != 0;
  bool stale  = s.rxMs && now - s.rxMs > STALE_MS;
  char buf[24];
  if (!s.rxMs) {
    snprintf(buf, sizeof(buf), "--");
  } else {
    unsigned long min = (now - s.rxMs) / 60000;
    if (min == 0)      snprintf(buf, sizeof(buf), "agora");
    else if (min < 60) snprintf(buf, sizeof(buf), "%lu min", min);
    else               snprintf(buf, sizeof(buf), "%lu h", min / 60);
  }
  lv_label_set_text(s.age, buf);
  lv_obj_set_style_text_color(s.age, stale ? C_WARN : C_MUTED, 0);
  lv_obj_set_style_border_color(s.card, hasErr ? C_ERR : stale ? C_WARN : C_BORDER, 0);
}

void updateCard(Source &s, unsigned long now) {
  if (!s.card) buildCard(s);

  const char *pct = fieldVal(s, "pct");
  setHidden(s.big, !pct);
  setHidden(s.bar, !pct);
  if (pct) {
    lv_label_set_text_fmt(s.big, "%s %%", pct);
    lv_bar_set_value(s.bar, constrain(atoi(pct), 0, 100), LV_ANIM_OFF);
  }

  const char *err = fieldVal(s, "err");
  bool hasErr = err && strcmp(err, "ok") != 0;
  setHidden(s.err, !hasErr);
  if (hasErr) lv_label_set_text_fmt(s.err, "ERRO: %s", err);

  // Demais chaves, uma por linha
  char buf[MAX_FIELDS * (KEY_SIZE + VAL_SIZE + 8)];
  size_t len = 0;
  for (uint8_t i = 0; i < s.nf && len < sizeof(buf); i++) {
    const char *k = s.f[i].key;
    if (!strcmp(k, "pct") || !strcmp(k, "err") || !strcmp(k, "vbat")) continue;
    len += snprintf(buf + len, sizeof(buf) - len, "%s%s: %s %s",
                    len ? "\n" : "", k, s.f[i].val, unitFor(k));
  }
  buf[min(len, sizeof(buf) - 1)] = '\0';
  lv_label_set_text(s.vals, buf);

  const char *vbat = fieldVal(s, "vbat");
  if (vbat) {
    float v = atof(vbat);
    int p = constrain((int)lroundf((v - VBAT_EMPTY) * 100 / (VBAT_FULL - VBAT_EMPTY)), 0, 100);
    lv_label_set_text_fmt(s.bat, "Bat %s V  %d%%", vbat, p);
    lv_obj_set_style_text_color(s.bat, p < 20 ? C_WARN : C_MUTED, 0);
  } else {
    lv_label_set_text(s.bat, "");
  }
  updateAge(s, now);
}

void updateHeader() {
  wl_status_t ws = WiFi.status();
  lv_label_set_text(hdrWifi, LV_SYMBOL_WIFI " WiFi");
  lv_obj_set_style_text_color(hdrWifi, ws == WL_CONNECTED ? C_OK : C_ERR, 0);
  lv_label_set_text(hdrMqtt, LV_SYMBOL_BULLET " MQTT");
  lv_obj_set_style_text_color(hdrMqtt, mqtt.connected() ? C_OK : C_ERR, 0);
  lv_label_set_text(hdrGw, LV_SYMBOL_BULLET " Gateway");
  lv_obj_set_style_text_color(hdrGw, gwOnline < 0 ? C_MUTED : gwOnline ? C_OK : C_ERR, 0);

  if (isnan(gwTemp)) lv_label_set_text(hdrTemp, "-- °C");
  else               lv_label_set_text_fmt(hdrTemp, "%.1f °C", gwTemp);

  struct tm tm;
  time_t t = time(nullptr);
  localtime_r(&t, &tm);
  if (tm.tm_year + 1900 < 2024) lv_label_set_text(hdrClock, "--:--");   // sem NTP ainda
  else lv_label_set_text_fmt(hdrClock, "%02d:%02d", tm.tm_hour, tm.tm_min);
}

void uiMaintain(unsigned long now) {
  static unsigned long lastUi = 0, lastAge = 0;
  if (now - lastUi < UI_EVERY_MS) return;
  lastUi = now;
  bool ageTick = now - lastAge >= AGE_EVERY_MS;
  if (ageTick) lastAge = now;

  // Estado de WiFi/MQTT muda sem evento: compara com o último desenhado
  static int8_t lastWifi = -1, lastMqtt = -1;
  int8_t w = WiFi.status() == WL_CONNECTED, m = mqtt.connected();
  if (w != lastWifi || m != lastMqtt || hdrDirty || ageTick) {
    lastWifi = w; lastMqtt = m; hdrDirty = false;
    updateHeader();
  }

  bool any = false;
  for (Source &s : sources) {
    if (!s.used) continue;
    if (s.dirty && s.nf == 0) {                 // todos os tópicos apagados
      if (detailSrc == &s) onDetailClose(nullptr);
      if (s.card) lv_obj_delete(s.card);
      s.used = false;
      continue;
    }
    any = true;
    if (s.dirty) {
      s.dirty = false;
      updateCard(s, now);
      if (detailSrc == &s) fillDetail();
    } else if (ageTick) {
      updateAge(s, now);
    }
  }
  setHidden(emptyLbl, any);
}

// ══════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════

// A ESP32_Display_Panel loga com printf, que sai na UART0 (o RS485
// desta placa). Redireciona para o USB, junto com o Serial.
static int usbWrite(void *, const char *buf, int n) {
  return Serial.write((const uint8_t *)buf, n);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);   // USB CDC: não perde o log de boot
  stdout = fwopen(nullptr, usbWrite);
  setvbuf(stdout, nullptr, _IONBF, 0);

  board = new Board(BOARD_EXTERNAL_CONFIG);
  if (!board->init() || !board->begin()) {
    Serial.println("[LCD] falha ao iniciar a placa — confira Flash 16MB / PSRAM OPI");
    while (true) delay(1000);
  }
  lcd   = board->getLCD();
  touch = board->getTouch();
  if (!lvglInit()) {
    Serial.println("[LVGL] sem memoria para o buffer");
    while (true) delay(1000);
  }
  buildScreen();
  lv_timer_handler();               // mostra a tela já antes do WiFi

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(MQTT_CLIENT_ID);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[WiFi] conectando em \"%s\"...\n", WIFI_SSID);
  configTzTime(TZ_INFO, NTP_SERVER);

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqtt);
  mqtt.setBufferSize(MQTT_BUF);
  mqtt.setSocketTimeout(2);         // trava a tela no máximo isso ao conectar

  Serial.println("[DISPLAY] pronto");
}

// ══════════════════════════════════════════════════════════
//  LOOP
// ══════════════════════════════════════════════════════════

void loop() {
  unsigned long now = millis();
  mqttMaintain(now);
  uiMaintain(now);
  uint32_t idle = lv_timer_handler();
  delay(min(idle, (uint32_t)5));
}

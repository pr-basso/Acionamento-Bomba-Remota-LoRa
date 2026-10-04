/**
 * Diagnóstico do W5500 na Heltec V4 — lê o registrador VERSIONR (0x0039),
 * que deve responder 0x04. Testa várias velocidades e MISO/MOSI trocados.
 * Mesmos pinos do Gateway.ino. Abra o Serial a 115200 e veja o resumo.
 *
 *   0x04          comunicação ok nessa combinação
 *   0x00          MISO sem sinal (fio solto, módulo sem alimentação, CS errado)
 *   0xFF          MISO preso em alto / CS não seleciona / chip em reset
 *   outro valor   ruído: fio longo, mau contato, velocidade alta
 */
#include <SPI.h>

#define ETH_SCK   47
#define ETH_MISO  48
#define ETH_MOSI  41
#define ETH_CS    34
#define ETH_INT    4
#define ETH_RST    6

// Modo do PHY forçado por software (ignora os pinos PMODE). Comente para
// manter a auto-negociação. 0x00 = 10M half, 0x18 = 100M full.
#define PHY_FORCE  0x00

SPIClass spi(HSPI);   // SPI3_HOST no ESP32-S3, igual ao Gateway

uint8_t readVersion(uint32_t hz) {
  spi.beginTransaction(SPISettings(hz, MSBFIRST, SPI_MODE0));
  digitalWrite(ETH_CS, LOW);
  spi.transfer(0x00);        // endereço alto
  spi.transfer(0x39);        // endereço baixo: VERSIONR
  spi.transfer(0x00);        // controle: registradores comuns, leitura, VDM
  uint8_t v = spi.transfer(0x00);
  digitalWrite(ETH_CS, HIGH);
  spi.endTransaction();
  return v;
}

void writeReg(uint16_t addr, uint8_t val, uint32_t hz) {
  spi.beginTransaction(SPISettings(hz, MSBFIRST, SPI_MODE0));
  digitalWrite(ETH_CS, LOW);
  spi.transfer(addr >> 8);
  spi.transfer(addr & 0xFF);
  spi.transfer(0x04);        // controle: registradores comuns, escrita, VDM
  spi.transfer(val);
  digitalWrite(ETH_CS, HIGH);
  spi.endTransaction();
}

uint8_t readReg(uint16_t addr, uint32_t hz) {
  spi.beginTransaction(SPISettings(hz, MSBFIRST, SPI_MODE0));
  digitalWrite(ETH_CS, LOW);
  spi.transfer(addr >> 8);
  spi.transfer(addr & 0xFF);
  spi.transfer(0x00);
  uint8_t v = spi.transfer(0x00);
  digitalWrite(ETH_CS, HIGH);
  spi.endTransaction();
  return v;
}

// Teste aprofundado na fiação "normal": estatística de leituras,
// MISO com CS baixo e escrita/leitura de um registrador (testa MOSI).
void deepTest() {
  spi.end();
  spi.begin(ETH_SCK, ETH_MISO, ETH_MOSI, -1);
  resetChip();

  const uint32_t hz = 1000000;
  int ok = 0, zero = 0, ff = 0, other = 0;
  for (int i = 0; i < 200; i++) {
    uint8_t v = readVersion(hz);
    if (v == 0x04) ok++; else if (v == 0x00) zero++; else if (v == 0xFF) ff++; else other++;
  }
  Serial.printf("\n200 leituras a 1 MHz: 04=%d 00=%d FF=%d outros=%d\n", ok, zero, ff, other);

  // Com CS baixo o W5500 deve dirigir o MISO: pull-up/down não mudam a leitura
  spi.end();
  digitalWrite(ETH_CS, LOW); delay(1);
  pinMode(ETH_MISO, INPUT_PULLDOWN); delay(5);
  int lo = digitalRead(ETH_MISO);
  pinMode(ETH_MISO, INPUT_PULLUP);   delay(5);
  int hi = digitalRead(ETH_MISO);
  digitalWrite(ETH_CS, HIGH);
  Serial.printf("MISO com CS baixo: pulldown=%d pullup=%d (%s)\n", lo, hi,
                lo == hi ? "dirigido pelo modulo — ok" : "FLUTUANDO — modulo nao responde ao CS");

  // Escreve um padrão no MAC (SHAR, 0x0009..0x000E) e lê de volta
  spi.begin(ETH_SCK, ETH_MISO, ETH_MOSI, -1);
  const uint8_t pat[6] = { 0xA5, 0x5A, 0x3C, 0xC3, 0x0F, 0xF0 };
  for (int i = 0; i < 6; i++) writeReg(0x0009 + i, pat[i], hz);
  Serial.print("SHAR escrito A5 5A 3C C3 0F F0, lido:");
  int match = 0;
  for (int i = 0; i < 6; i++) {
    uint8_t v = readReg(0x0009 + i, hz);
    Serial.printf(" %02X", v);
    if (v == pat[i]) match++;
  }
  Serial.printf("  (%d/6 %s)\n", match, match == 6 ? "ok" : "falhou");
}

void resetChip() {
  digitalWrite(ETH_RST, LOW);
  delay(10);
  digitalWrite(ETH_RST, HIGH);
  delay(200);                // PLL do W5500 estabiliza em ~50 ms
}

void testAll(const char *label, int miso, int mosi) {
  spi.end();
  spi.begin(ETH_SCK, miso, mosi, -1);
  resetChip();
  const uint32_t speeds[] = { 1000000, 4000000, 8000000, 14000000, 20000000 };
  for (uint32_t hz : speeds) {
    uint8_t v[5];
    for (int i = 0; i < 5; i++) v[i] = readVersion(hz);
    Serial.printf("  %-14s %2lu MHz: %02X %02X %02X %02X %02X %s\n",
                  label, (unsigned long)(hz / 1000000),
                  v[0], v[1], v[2], v[3], v[4],
                  (v[0] == 0x04 && v[4] == 0x04) ? "<-- OK" : "");
  }
}

void setup() {
  Serial.begin(9600);
  delay(2000);
  pinMode(ETH_CS, OUTPUT);   digitalWrite(ETH_CS, HIGH);
  pinMode(ETH_RST, OUTPUT);  digitalWrite(ETH_RST, HIGH);
  pinMode(ETH_INT, INPUT_PULLUP);

  Serial.println("\n=== Diagnostico W5500 ===");
  Serial.printf("SCK=%d MISO=%d MOSI=%d CS=%d RST=%d INT=%d\n",
                ETH_SCK, ETH_MISO, ETH_MOSI, ETH_CS, ETH_RST, ETH_INT);

  // MISO com pull-down: se ler 0 sempre, o pino não recebe nada do módulo
  pinMode(ETH_MISO, INPUT_PULLDOWN); delay(5);
  int lowPull = digitalRead(ETH_MISO);
  pinMode(ETH_MISO, INPUT_PULLUP);   delay(5);
  int highPull = digitalRead(ETH_MISO);
  Serial.printf("MISO em repouso: pulldown=%d pullup=%d (%s)\n", lowPull, highPull,
                lowPull != highPull ? "flutuando — normal com CS alto" : "forcado pelo modulo");
  Serial.printf("INT em repouso: %d (esperado 1)\n\n", digitalRead(ETH_INT));

  testAll("normal", ETH_MISO, ETH_MOSI);
  testAll("MISO<->MOSI", ETH_MOSI, ETH_MISO);
  deepTest();

#ifdef PHY_FORCE
  // OPMD=1 (config por software) + OPMDC; RST=0 reinicia o PHY, depois RST=1
  writeReg(0x002E, 0x40 | PHY_FORCE, 1000000);
  delay(10);
  writeReg(0x002E, 0xC0 | PHY_FORCE, 1000000);
  Serial.printf("\nPHY forcado: %s\n", PHY_FORCE == 0x18 ? "100M full" : "10M half");
#endif
  Serial.println("\nMonitor do PHY (PHYCFGR) a cada 1 s:");
}

// PHYCFGR (0x002E): bit0 link, bit1 100M, bit2 full duplex,
// bits 3..5 modo configurado pelos pinos PMODE (111 = auto-negociação).
void loop() {
  uint8_t p = readReg(0x002E, 1000000);
  Serial.printf("PHYCFGR=%02X  link=%s  %s  %s  PMODE=%u%u%u\n", p,
                (p & 0x01) ? "SIM" : "nao",
                (p & 0x02) ? "100M" : "10M",
                (p & 0x04) ? "full" : "half",
                (p >> 5) & 1, (p >> 4) & 1, (p >> 3) & 1);
  delay(1000);
}

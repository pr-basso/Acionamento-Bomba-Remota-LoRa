#include "Arduino.h"
#include "LoRaWan_APP.h"
#include "HT_SSD1306Wire.h"

// --- Configurações do Rádio LoRa P2P ---
#define RF_FREQUENCY                                915000000 
#define TX_OUTPUT_POWER                             14        
#define LORA_BANDWIDTH                              0         
#define LORA_SPREADING_FACTOR                       9         
#define LORA_CODINGRATE                             1         
#define LORA_PREAMBLE_LENGTH                        8         

// --- Pinos do Hardware ---
const int PINO_RELE = 22; // GPIO22 conectado ao módulo de relé

// --- Instanciação do Display OLED ---
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

// --- Endereçamento do Protocolo ---
const byte MEU_ID = 0x02;       // Nó B (Bomba)
const byte ID_DESTINO = 0x01;   // Nó A (Sensor)

const byte TIPO_MSG = 0x00;
const byte TIPO_ACK = 0x01;

const byte COMANDO_DESLIGAR = 0x00;
const byte COMANDO_LIGAR    = 0x01;

// --- Máquina de Estados do Rádio (Sem conflitos de escopo) ---
typedef enum {
    LORA_STATE_LOWPOWER,
    LORA_STATE_TX,
    LORA_STATE_RX,
    LORA_STATE_TX_TIMEOUT,
    LORA_STATE_RX_TIMEOUT,
    LORA_STATE_RX_ERROR
} EstadoLora_t;

EstadoLora_t estadoAtualLoRa;
static RadioEvents_t RadioEvents;

// --- Variáveis de Controle ---
uint8_t txBuffer[10];
String statusBomba = "DESLIGADA";

// --- Declaração das Funções ---
void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnRxTimeout(void);
void OnRxError(void);
void enviarACK(byte destino, byte idMensagem);
void atualizarOLED(String status, String rssiStr);

void setup() {
    Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);

    // Inicialização do Display OLED Heltec
    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);
    atualizarOLED(statusBomba, "Aguardando...");

    // Configuração do pino do Atuador
    pinMode(PINO_RELE, OUTPUT);
    digitalWrite(PINO_RELE, LOW); // Inicializa com a bomba desligada

    estadoAtualLoRa = LORA_STATE_LOWPOWER;

    // Associa os eventos do rádio LoRa
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);

    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, false, true, 0, 0, false, 3000);

    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   5, false, 0, true, 0, 0, false, true);

    Serial.println("--- NO B: ATUADOR DA BOMBA INICIADO ---");
    Radio.Rx(0); // Escuta contínua ativa
}

void loop() {
    // Processamento da máquina de estados assíncrona do rádio
    switch (estadoAtualLoRa) {
        case LORA_STATE_TX:
            Radio.Rx(0); // Após transmitir o ACK, volta para o modo escuta permanente
            estadoAtualLoRa = LORA_STATE_LOWPOWER;
            break;

        case LORA_STATE_RX:
            Radio.Rx(0); 
            estadoAtualLoRa = LORA_STATE_LOWPOWER;
            break;

        case LORA_STATE_TX_TIMEOUT:
        case LORA_STATE_RX_TIMEOUT:
        case LORA_STATE_RX_ERROR:
            Radio.Rx(0); // Força reinício da escuta em qualquer erro
            estadoAtualLoRa = LORA_STATE_LOWPOWER;
            break;

        case LORA_STATE_LOWPOWER:
            break;
    }
}

void enviarACK(byte destino, byte idMensagem) {
    txBuffer[0] = MEU_ID;
    txBuffer[1] = destino;
    txBuffer[2] = idMensagem;
    txBuffer[3] = TIPO_ACK;

    Serial.printf("[TX] Enviando ACK para Msg ID %d...\n", idMensagem);
    Radio.Send(txBuffer, 4); // Envia o frame limpo contendo o cabeçalho de ACK
}

// --- Callbacks do Rádio LoRa ---
void OnTxDone(void) {
    estadoAtualLoRa = LORA_STATE_TX;
}

void OnTxTimeout(void) {
    estadoAtualLoRa = LORA_STATE_TX_TIMEOUT;
}

void OnRxTimeout(void) {
    estadoAtualLoRa = LORA_STATE_RX_TIMEOUT;
}

void OnRxError(void) {
    estadoAtualLoRa = LORA_STATE_RX_ERROR;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    estadoAtualLoRa = LORA_STATE_RX;
    if (size < 5) return; // Requer pelo menos cabeçalho (4b) + comando (1b)

    byte remetente       = payload[0];
    byte destinatario    = payload[1];
    byte idMsgRecebida   = payload[2];
    byte tipoMsg         = payload[3];
    byte comando         = payload[4];

    // Valida se o pacote é um comando válido enviado pelo Nó A para este dispositivo
    if (destinatario == MEU_ID && remetente == ID_DESTINO && tipoMsg == TIPO_MSG) {
        
        if (comando == COMANDO_LIGAR) {
            digitalWrite(PINO_RELE, HIGH);
            statusBomba = "LIGADA";
            Serial.println("\n[Acao] Comando LIGAR executado no Rele.");
        } else if (comando == COMANDO_DESLIGAR) {
            digitalWrite(PINO_RELE, LOW);
            statusBomba = "DESLIGADA";
            Serial.println("\n[Acao] Comando DESLIGAR executado no Rele.");
        }

        atualizarOLED(statusBomba, "Sinal RSSI: " + String(rssi) + "dBm");
        
        // Pequena pausa estratégica e responde imediatamente com o ACK de confirmação
        delay(40); 
        enviarACK(remetente, idMsgRecebida);
    }
}

void atualizarOLED(String status, String rssiStr) {
    display.clear();
    display.drawString(0, 0, "NO B - ATUADOR BOMBA");
    display.drawString(0, 16, "---------------------------------");
    display.drawString(0, 28, "Bomba esta: " + status);
    display.drawString(0, 44, rssiStr);
    display.display();
}
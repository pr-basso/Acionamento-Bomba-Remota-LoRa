#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"

// --- Configurações do Rádio LoRa P2P ---
#define RF_FREQUENCY                                915000000 // Hz (915 MHz)
#define TX_OUTPUT_POWER                             14        // dBm
#define LORA_BANDWIDTH                              0         // [0: 125 kHz]
#define LORA_SPREADING_FACTOR                       9         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5]
#define LORA_PREAMBLE_LENGTH                        8         
#define RX_TIMEOUT_VALUE                            3000      // ms

// --- Pinos do Hardware ---
const int PINO_BOIA = 2; // GPIO2 conectado à boia (GND ao pino, usando Pull-up)

// --- Instanciação do Display OLED ---
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

// --- Endereçamento do Protocolo ---
const byte MEU_ID = 0x01;       // Nó A (Caixa d'Água)
const byte ID_DESTINO = 0x02;   // Nó B (Bomba)

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
byte msgContador = 0;
bool aguardandoACK = false;
unsigned long tempoEnvio = 0;
const unsigned long TIMEOUT_ACK = 2000; // 2 segundos de timeout por software

int ultimoEstadoBoia = -1;
byte comandoAtual = 0x00;
String statusDisplay = "Aguardando Alteracao";

// --- Declaração das Funções ---
void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnRxTimeout(void);
void OnRxError(void);
void enviarComandoBomba(byte comando);
void atualizarOLED(String status, String info);

void setup() {
    Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
    
    // Inicialização do Display OLED Heltec
    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);
    atualizarOLED("Sistema Iniciado", "Sensor Nivel");

    // Configuração do pino do sensor
    pinMode(PINO_BOIA, INPUT_PULLUP); 
    
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

    Serial.println("--- NO A: SENSOR DA CAIXA INICIADO ---");
    Radio.Rx(0); // Inicia escuta
}

void loop() {
    // 1. Leitura da Boia por Evento
    if (!aguardandoACK) {
        int leituraAtual = digitalRead(PINO_BOIA);
        
        if (leituraAtual != ultimoEstadoBoia) {
            ultimoEstadoBoia = leituraAtual;
            
            if (leituraAtual == HIGH) { // Caixa vazia -> Ligar bomba
                comandoAtual = COMANDO_LIGAR;
                statusDisplay = "Nivel Baixo -> LIGAR";
            } else { // Caixa cheia -> Desligar bomba
                comandoAtual = COMANDO_DESLIGAR;
                statusDisplay = "Nivel Cheio -> DESLIGAR";
            }
            Serial.printf("\n[Boia] Alteracao detectada: %s\n", statusDisplay.c_str());
            atualizarOLED(statusDisplay, "Enviando comando...");
            enviarComandoBomba(comandoAtual);
        }
    }

    // 2. Tratamento do Timeout do ACK (Garante a retransmissão)
    if (aguardandoACK && (millis() - tempoEnvio > TIMEOUT_ACK)) {
        Serial.println("-> [TIMEOUT] ACK nao recebido. Retransmitindo comando...");
        atualizarOLED(statusDisplay, "Retransmitindo...");
        enviarComandoBomba(comandoAtual); 
    }

    // 3. Execução da Máquina de Estados do Rádio
    switch (estadoAtualLoRa) {
        case LORA_STATE_TX:
            Radio.Rx(RX_TIMEOUT_VALUE); // Abre janela para escutar o ACK
            estadoAtualLoRa = LORA_STATE_LOWPOWER;
            break;

        case LORA_STATE_RX:
            Radio.Rx(0); // Retorna à escuta contínua
            estadoAtualLoRa = LORA_STATE_LOWPOWER;
            break;

        case LORA_STATE_TX_TIMEOUT:
            Serial.println("Erro: Timeout de hardware Tx.");
            Radio.Rx(0);
            estadoAtualLoRa = LORA_STATE_LOWPOWER;
            break;

        case LORA_STATE_RX_TIMEOUT:
        case LORA_STATE_RX_ERROR:
            if (!aguardandoACK) {
                Radio.Rx(0);
            }
            estadoAtualLoRa = LORA_STATE_LOWPOWER;
            break;

        case LORA_STATE_LOWPOWER:
            break;
    }
}

void enviarComandoBomba(byte comando) {
    aguardandoACK = true;
    tempoEnvio = millis();

    // Estruturação do pacote do protocolo
    txBuffer[0] = MEU_ID;
    txBuffer[1] = ID_DESTINO;
    txBuffer[2] = msgContador;
    txBuffer[3] = TIPO_MSG;
    txBuffer[4] = comando; 

    Serial.printf("[TX] Enviando Msg ID %d para 0x%02X...\n", msgContador, ID_DESTINO);
    Radio.Send(txBuffer, 5);
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
    if (size < 4) return;

    // Filtra se a mensagem é uma confirmação (ACK) vinda do Nó B associada à nossa mensagem atual
    if (payload[1] == MEU_ID && payload[0] == ID_DESTINO && payload[3] == TIPO_ACK) {
        if (payload[2] == msgContador) {
            Serial.println("-> [SUCESSO] ACK Confirmado pelo destino.");
            aguardandoACK = false;
            msgContador++; // Atualiza contador para o próximo ciclo de evento
            atualizarOLED(statusDisplay, "ACK Recebido [OK]");
        }
    }
}

void atualizarOLED(String status, String info) {
    display.clear();
    display.drawString(0, 0, "NO A - SENSOR CAIXA");
    display.drawString(0, 16, "---------------------------------");
    display.drawString(0, 28, "Status: " + status);
    display.drawString(0, 44, "Info: " + info);
    display.display();
}
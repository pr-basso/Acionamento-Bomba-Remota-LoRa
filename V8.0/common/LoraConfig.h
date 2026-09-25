/**
 * ============================================================
 *  PARÂMETROS LoRa COMPARTILHADOS — V8.0
 *
 *  Fonte única da configuração de rádio para TODOS os nodes
 *  (Boia e Bomba). Nodes com SF/BW/CR diferentes não conseguem
 *  decodificar os pacotes uns dos outros — por isso ficam aqui.
 *
 *  Cada sketch acessa este arquivo por um link simbólico
 *  (Boia/LoraConfig.h e Bomba/LoraConfig.h -> ../common/LoraConfig.h),
 *  já que o Arduino só compila arquivos dentro da pasta do sketch.
 *
 *  Tempo no ar (SF9/BW125/CR4/5, ~35 bytes): ~250ms por pacote.
 * ============================================================
 */

#ifndef LORA_CONFIG_H
#define LORA_CONFIG_H

#define RF_FREQUENCY          915000000
#define TX_OUTPUT_POWER               14   // dBm
#define LORA_BANDWIDTH                0    // 0 = 125 kHz
#define LORA_SPREADING_FACTOR         9
#define LORA_CODINGRATE               1    // 1 = 4/5
#define LORA_PREAMBLE_LENGTH          8
#define LORA_SYMBOL_TIMEOUT           0
#define LORA_IQ_INVERSION_ON      false

#endif

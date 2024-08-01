#ifndef __CONFIG_H
#define __CONFIG_H

#include <stm32f103xb.h>
#include <stm32f1xx_hal.h>
#include "lora_sx1276.h"

#define TOTAL_NO_OF_NODES 5
#define NO_OF_NODES_TO_MONITOR 3

#define LORA_BASE_FREQUENCY 433000000UL
#define LORA_BANDWIDTH LORA_BANDWIDTH_10_4_KHZ
#define LORA_FREQUENCY_STEP 25000
#define LORA_SPREADING_FACTOR 6
#define LORA_TX_POWER_LEVEL 10
#define LORA_PREAMBLE_LENGTH 12
#define PAYLOAD_LENGTH 6
// Time On Air and symbol length calculated from https://www.semtech.com/design-support/lora-calculator
#define LORA_TOA 225
#define LORA_SYMBOL_LENGTH 36

#endif
#ifndef __CONFIG_H
#define __CONFIG_H

#include <stm32f103xb.h>
#include <stm32f1xx_hal.h>
#include "lora_sx1276.h"

#define THIS_NODE_NUM 3
#define TOTAL_NO_OF_NODES 5

#define NO_OF_NODES_TO_MONITOR 3

#define THIS_NODE_ADDRESS THIS_NODE_NUM

#define LORA_BASE_FREQUENCY 433000000UL
#define LORA_TX_POWER_LEVEL 10
#define LORA_PREAMBLE_LENGTH 12
#define PAYLOAD_LENGTH 10
// Time On Air and symbol length calculated from https://www.semtech.com/design-support/lora-calculator
#define LORA_SYMBOL_LENGTH 45

#endif
#ifndef __NODES_H
#define __NODES_H

#include "config.h"
#include "pinout.h"
#include "ppp_init.h"
#include <stdbool.h>
#include <string.h>

#define SLAVE_NODE 0
#define MASTER_NODE 1

// Total number of nodes is ODD
#if (TOTAL_NO_OF_NODES % 2)

// Odd numbered node
#if (THIS_NODE_NUM % 2)

#define NODE_TYPE SLAVE_NODE

#if (THIS_NODE_NUM == 1)

#define LORA_UP_SF 7
#define LORA_UP_BANDWIDTH LORA_BANDWIDTH_20_8_KHZ
#define LORA_UP_TOA 280
#define LORA_UP_SYMBOL_LENGTH 45

#define LORA_DOWN_SF LORA_UP_SF
#define LORA_DOWN_BANDWIDTH LORA_UP_BANDWIDTH
#define LORA_DOWN_TOA LORA_UP_TOA
#define LORA_DOWN_SYMBOL_LENGTH LORA_UP_SYMBOL_LENGTH

#elif (THIS_NODE_NUM == 3)

#define LORA_UP_SF 8
#define LORA_UP_BANDWIDTH LORA_BANDWIDTH_41_7_KHZ
#define LORA_UP_TOA 245
#define LORA_UP_SYMBOL_LENGTH 40

#define LORA_DOWN_SF 7
#define LORA_DOWN_BANDWIDTH LORA_BANDWIDTH_20_8_KHZ
#define LORA_DOWN_TOA 280
#define LORA_DOWN_SYMBOL_LENGTH 45

#elif (THIS_NODE_NUM == 5)

#define LORA_DOWN_SF 8
#define LORA_DOWN_BANDWIDTH LORA_BANDWIDTH_41_7_KHZ
#define LORA_DOWN_TOA 245
#define LORA_DOWN_SYMBOL_LENGTH 40

#define LORA_UP_SF LORA_DOWN_SF
#define LORA_UP_BANDWIDTH LORA_DOWN_BANDWIDTH
#define LORA_UP_TOA LORA_DOWN_TOA
#define LORA_UP_SYMBOL_LENGTH LORA_DOWN_SYMBOL_LENGTH

#endif

#else

#define NODE_TYPE MASTER_NODE

#if (THIS_NODE_NUM == 2)

#define LORA_UP_SF 7
#define LORA_UP_BANDWIDTH LORA_BANDWIDTH_20_8_KHZ
#define LORA_UP_TOA 280
#define LORA_UP_SYMBOL_LENGTH 45

#define LORA_DOWN_SF 7
#define LORA_DOWN_BANDWIDTH LORA_UP_BANDWIDTH
#define LORA_DOWN_TOA 280
#define LORA_DOWN_SYMBOL_LENGTH LORA_UP_SYMBOL_LENGTH

#elif (THIS_NODE_NUM == 4)

#define LORA_DOWN_SF 8
#define LORA_DOWN_BANDWIDTH LORA_BANDWIDTH_41_7_KHZ
#define LORA_DOWN_TOA 245
#define LORA_DOWN_SYMBOL_LENGTH 40

#define LORA_UP_SF LORA_DOWN_SF
#define LORA_UP_BANDWIDTH LORA_DOWN_BANDWIDTH
#define LORA_UP_TOA LORA_DOWN_TOA
#define LORA_UP_SYMBOL_LENGTH LORA_DOWN_SYMBOL_LENGTH

#endif

#endif

// Total number of nodes is even
#else

#error "Total no of nodes must be odd"

#endif

extern lora_sx1276 lora;

void nodeInitialisation(void);
void postInit(void);

#endif
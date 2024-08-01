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

#define PNF LORA_BASE_FREQUENCY + LORA_FREQUENCY_STEP
#define NNF LORA_BASE_FREQUENCY + LORA_FREQUENCY_STEP

#elif (THIS_NODE_NUM == TOTAL_NO_OF_NODES)

#define NNF ((((TOTAL_NO_OF_NODES - 1) / 2) * LORA_FREQUENCY_STEP) + LORA_BASE_FREQUENCY)
#define PNF ((((TOTAL_NO_OF_NODES - 1) / 2) * LORA_FREQUENCY_STEP) + LORA_BASE_FREQUENCY)

#else

#define PNF (((THIS_NODE_NUM / 2) * LORA_FREQUENCY_STEP) + LORA_BASE_FREQUENCY)
#define NNF ((((THIS_NODE_NUM + 1) / 2) * LORA_FREQUENCY_STEP) + LORA_BASE_FREQUENCY)

#endif

#else

// Even numbered node, i.e, master node

#define NODE_TYPE MASTER_NODE

#define OPERATING_FREQUENCY (((THIS_NODE_NUM / 2) * LORA_FREQUENCY_STEP) + LORA_BASE_FREQUENCY)
#define NNF OPERATING_FREQUENCY
#define PNF OPERATING_FREQUENCY

#endif

// Total number of nodes is even
#else

#error "Total no of nodes must be odd"

#endif

extern lora_sx1276 lora;

void LoRaReset(void);
void nodeInitialisation(void);
void postInit(void);

#endif
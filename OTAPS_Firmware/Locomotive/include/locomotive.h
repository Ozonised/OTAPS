#ifndef __LOCOMOTIVE_H
#define __LOCOMOTIVE_H

#include "pinout.h"
#include "config.h"
#include "ppp_init.h"

typedef struct locomotive
{
    Nodes *commNode;
    TrainDirection dir;
    TrainState state;
    Signal signalData[NO_OF_NODES_TO_MONITOR + 1];
} Locomotive;

typedef struct node
{
    struct node *prev;
    struct node *next;
    uint8_t nodeNo;
    uint8_t nodeAddress;
    unsigned long frequency;
} Nodes;

typedef enum traindirection
{
    TRAIN_DIR_NOT_KNOWN,
    TO_LOWER_NODE,
    TO_HIGHER_NODE
} TrainDirection;

typedef enum signal
{
    SIGNAL_NOT_KNOWN,
    RED,
    YELLOW,
    DOUBLE_YELLOW,
    GREEN
} Signal;

typedef enum trainstate
{
    STOP,
    SLOW_DOWN,
    GO
} TrainState;

extern lora_sx1276 lora;

#endif
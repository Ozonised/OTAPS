#include "locomotive.h"
#include <stdbool.h>
#include <string.h>

Nodes n1, n2, n3, n4, n5;

Locomotive locomotive;

static void nodeInit()
{
    uint8_t i = 1;

    n1.prev = NULL;
    n1.next = &n2;

    n2.prev = &n1;
    n2.next = &n3;

    n3.prev = &n2;
    n3.next = &n4;

    n4.prev = &n3;
    n4.next = &n5;

    n5.prev = &n4;
    n5.next = NULL;

    for (Nodes *n = &n1; i <= TOTAL_NO_OF_NODES; n = n->next, i++)
    {
        n->nodeNo = i;
        n->nodeAddress = i;

        if (n->nodeNo == 1)
        {
            n->frequency = LORA_BASE_FREQUENCY + LORA_FREQUENCY_STEP;
        }
        else if (n->nodeNo == TOTAL_NO_OF_NODES)
        {
            n->frequency = ((((TOTAL_NO_OF_NODES - 1) / 2) * LORA_FREQUENCY_STEP) + LORA_BASE_FREQUENCY);
        }
        else
        {
            n->frequency = (((n->nodeNo / 2) * LORA_FREQUENCY_STEP) + LORA_BASE_FREQUENCY);
        }
    }
}

static void extractPayLoadData(Locomotive *loco, uint8_t *payLoad)
{
    uint8_t temp;

    switch (loco->dir)
    {
    case TO_HIGHER_NODE:
        temp = payLoad[4];
        loco->signalData[0] = (Signal)((temp & 0xF0) >> 4); // signal state of current communicating node
        loco->signalData[1] = (Signal)(temp & 0x0F);        // signal state of current communicating node + 1

        temp = payLoad[5];
        loco->signalData[2] = (Signal)((temp & 0xF0) >> 4); // signal state of current communicating node + 2
        loco->signalData[3] = (Signal)(temp & 0x0F);        // signal state of current communicating node + 3
        break;

    case TO_LOWER_NODE:

        temp = payLoad[4];
        loco->signalData[0] = (Signal)((temp & 0xF0) >> 4); // signal state of prev node

        temp = payLoad[3];
        loco->signalData[1] = (Signal)(temp & 0x0F);        // signal state of prev node -1
        loco->signalData[2] = (Signal)((temp & 0xF0) >> 4); // signal state of prev node - 2

        temp = payLoad[2];
        loco->signalData[3] = (Signal)(temp & 0x0F); // signal state of prev node - 3
        break;

    default:
        break;
    }
}

void updateLocomotiveState(Locomotive *loco)
{
    uint8_t remainingNodes = 0;

    switch (loco->dir)
    {
    case TO_HIGHER_NODE:
        remainingNodes = TOTAL_NO_OF_NODES - loco->commNode->nodeNo;
        break;
    case TO_LOWER_NODE:
        remainingNodes = loco->commNode->nodeNo - 1;
        break;
    default:
        break;
    }

    switch (loco->signalData[0])
    {
    case GREEN:
    case DOUBLE_YELLOW:
        loco->state = GO;
        break;

    case YELLOW:
        loco->state = SLOW_DOWN;

    case RED:
        loco->state = STOP;

    default:
        break;
    }

    // search for RED signal
    uint8_t *ptr = (uint8_t *)memchr(loco->signalData, RED, NO_OF_NODES_TO_MONITOR + 1);
    uint8_t index = 0;
    if (ptr != NULL)
    {
        index = ptr - loco->signalData;

        switch (index)
        {
        case 0:
            loco->state = STOP;
            break;

        case 1:
            // train incoming
            if (loco->signalData[2] != GREEN && loco->signalData[2] != SIGNAL_NOT_KNOWN)
            {
                loco->state = STOP;
            }
            else
            {
                loco->state = SLOW_DOWN;
            }
            break;

        case 2:
            // train incoming
            if (loco->signalData[3] != GREEN && loco->signalData[3] != SIGNAL_NOT_KNOWN)
            {
                loco->state = STOP;
            }
            break;

        default:
            break;
        }
    }
}

void locomotiveInit(void)
{
    nodeInit();
    LoRaReset();

    if (lora_init(&lora, &hLoRaSpi1, LORA_GPIO_PORT, LORA_GPIO_NSS_PIN, locomotive.commNode->frequency) != LORA_OK)
    {
        // LoRa initialisation failed
        while (1)
        {
            HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_PORT, LED_BUILTIN_GPIO_PIN);
            HAL_Delay(250);
        }
    }
    lora.spi_timeout = 20;

    lora_set_signal_bandwidth(&lora, LORA_BANDWIDTH);
    lora_set_tx_power(&lora, LORA_TX_POWER_LEVEL);
    lora_set_preamble_length(&lora, LORA_PREAMBLE_LENGTH);
    lora_set_spreading_factor(&lora, LORA_SPREADING_FACTOR);
    lora_set_rx_symbol_timeout(&lora, LORA_SYMBOL_LENGTH);
    lora_set_implicit_header_mode(&lora);
    lora_set_payload_length(&lora, PAYLOAD_LENGTH);
    lora_set_crc(&lora, 1);

    if (HAL_GPIO_ReadPin(TRAINDIR_SWITCH_GPIO_PORT, TRAINDIR_SWITCH_GPIO_PIN))
    {
        locomotive.dir = TO_HIGHER_NODE;
        locomotive.commNode = &n1;
    }
    else
    {
        locomotive.dir = TO_LOWER_NODE;
        locomotive.commNode = &n5;
    }
}

void postInit(void)
{
    static unsigned long currentMillis, prevMillis;
    static Signal prevSignalState = SIGNAL_NOT_KNOWN;
    const unsigned long TIMEOUT = 750;

    uint8_t error, rxPayLoad[PAYLOAD_LENGTH];
    bool validPayLoad = false, timeout = false;
    Locomotive *tempLoco = &locomotive;

    lora_set_frequency(&lora, locomotive.commNode->frequency);
    lora_mode_receive_single(&lora);

    prevMillis = HAL_GetTick();
    while (!validPayLoad)
    {
        currentMillis = HAL_GetTick();

        if (currentMillis - prevMillis > TIMEOUT)
        {
            timeout = true;
            break;
        }

        if (lora_is_packet_available(&lora))
        {
            lora_receive_packet(&lora, rxPayLoad, PAYLOAD_LENGTH, &error);

            if (error == LORA_OK)
            {
                // if packet received from the correct node
                if (rxPayLoad[0] == locomotive.commNode->nodeAddress)
                {
                    validPayLoad = true;
                }
                else
                {
                    lora_mode_receive_single(&lora);
                }
            }
            else
            {
                lora_mode_receive_single(&lora);
            }
        }
    }

    if (validPayLoad)
    {
        extractPayLoadData(&locomotive, rxPayLoad);
        updateLocomotiveState(&locomotive);

        switch (locomotive.state)
        {
        case GO:
            /* code */
            break;
        case SLOW_DOWN:
            break;

        case STOP:
            break;

        default:
            break;
        }
        
        if (locomotive.signalData[0] != GREEN && prevSignalState == GREEN)
        {
            // switching communication node
            switch (locomotive.dir)
            {
            case TO_HIGHER_NODE:
                locomotive.commNode = locomotive.commNode->next;
                break;

            case TO_LOWER_NODE:
                locomotive.commNode = locomotive.commNode->prev;
                break;

            default:
                break;
            }
        }
        validPayLoad = false;
    }
}
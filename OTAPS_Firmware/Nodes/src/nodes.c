#include "nodes.h"

#if (THIS_NODE_NUM - 1 > NO_OF_NODES_TO_MONITOR)
const uint8_t NO_OF_NODES_BEFORE = NO_OF_NODES_TO_MONITOR;
#else
const uint8_t NO_OF_NODES_BEFORE = THIS_NODE_NUM - 1;
#endif

#if (TOTAL_NO_OF_NODES - THIS_NODE_NUM > NO_OF_NODES_TO_MONITOR)
const uint8_t NO_OF_NODES_AFTER = NO_OF_NODES_TO_MONITOR;
#else
const uint8_t NO_OF_NODES_AFTER = TOTAL_NO_OF_NODES - THIS_NODE_NUM;
#endif

#if defined(DBG)
#define LOG_RX_PAYLOAD() log_rx_payload()
#define LOG_TX_PAYLOAD() log_tx_payload()
#else
#define LOG_RX_PAYLOAD()
#define LOG_TX_PAYLOAD()
#endif

typedef enum
{
    SIGNAL_NOT_KNOWN,
    RED,
    YELLOW,
    DOUBLE_YELLOW,
    GREEN
} Signal;

typedef struct signalstate
{
    Signal state;
    struct signalstate *next;
} SignalState;

typedef enum
{
    TRAIN_DIR_NOT_KNOWN,
    TO_LOWER_NODE,
    TO_HIGHER_NODE
} TrainDirection;

typedef struct node
{
    struct node *prev;
    struct node *next;
    Signal signal;
    uint8_t nodeNo;
    uint8_t axleCount;
    uint8_t nodeReady;
    uint8_t signalData[3];
    uint8_t trainDir;
} Nodes;

typedef struct
{
    uint8_t spreadingFactor;
    uint8_t bandwidth;
    uint8_t nodeNo;
} CommunicatingNode;

unsigned long currentMillis = 0, prevMillis = 0;
const unsigned long TIMEOUT = 320;

static uint8_t rxPayLoad[PAYLOAD_LENGTH], txPayLoad[PAYLOAD_LENGTH];
volatile uint8_t axleCounter;

uint8_t communicatingNodeNum;

lora_sx1276 lora;

Nodes thisNode, prevNode, nextNode;
CommunicatingNode communicatingNode;
SignalState red, doubleYellow, yellow, green;
volatile SignalState *currentSignalState;
volatile TrainDirection trainDir = TRAIN_DIR_NOT_KNOWN;

void LoRaReset(void);

void log_rx_payload()
{
    printf("com node: %d \n", communicatingNode.nodeNo);
    printf("T: %ld Rx Payload: %d %d %d %d %d %d %d %d %d %d\n", HAL_GetTick(), rxPayLoad[0], rxPayLoad[1], rxPayLoad[2], rxPayLoad[3], rxPayLoad[4], rxPayLoad[5], rxPayLoad[6], rxPayLoad[7], rxPayLoad[8], rxPayLoad[9]);
}

void log_tx_payload()
{
    printf("com node: %d \n", communicatingNode.nodeNo);
    printf("T: %ld tx Payload: %d %d %d %d %d %d %d %d %d %d\n", HAL_GetTick(), txPayLoad[0], txPayLoad[1], txPayLoad[2], txPayLoad[3], txPayLoad[4], txPayLoad[5], txPayLoad[6], txPayLoad[7], txPayLoad[8], txPayLoad[9]);
}

static void signalStateInit(void)
{
    red.next = &yellow;
    red.state = RED;

    doubleYellow.next = &green;
    doubleYellow.state = DOUBLE_YELLOW;

    yellow.next = &doubleYellow;
    yellow.state = YELLOW;

    green.next = NULL;
    green.state = GREEN;

    if (axleCounter == 0)
    {
        currentSignalState = &green;
    }

    thisNode.nodeNo = THIS_NODE_NUM;
    thisNode.axleCount = axleCounter;
    thisNode.signal = SIGNAL_NOT_KNOWN;
    thisNode.nodeReady = false;
    thisNode.trainDir = TRAIN_DIR_NOT_KNOWN;

#if (THIS_NODE_NUM == TOTAL_NO_OF_NODES)
    thisNode.next = NULL;
    thisNode.prev = &prevNode;
    prevNode.signal = SIGNAL_NOT_KNOWN;
    prevNode.nodeNo = THIS_NODE_NUM - 1;
    prevNode.axleCount = 0;
    nextNode.nodeReady = false;
    prevNode.trainDir = TRAIN_DIR_NOT_KNOWN;
    prevNode.next = NULL;
    prevNode.prev = NULL;
    memset(prevNode.signalData, SIGNAL_NOT_KNOWN, sizeof(prevNode.signalData));

#elif (THIS_NODE_NUM == 1)
    thisNode.next = &nextNode;
    nextNode.signal = SIGNAL_NOT_KNOWN;
    nextNode.nodeNo = THIS_NODE_NUM + 1;
    nextNode.axleCount = 0;
    nextNode.trainDir = TRAIN_DIR_NOT_KNOWN;
    nextNode.nodeReady = false;
    nextNode.next = NULL;
    nextNode.prev = NULL;
    memset(nextNode.signalData, SIGNAL_NOT_KNOWN, sizeof(nextNode.signalData));

#else
    thisNode.next = &nextNode;
    thisNode.prev = &prevNode;

    nextNode.nodeNo = (THIS_NODE_NUM + 1);
    nextNode.signal = SIGNAL_NOT_KNOWN;
    nextNode.axleCount = 0;
    nextNode.trainDir = TRAIN_DIR_NOT_KNOWN;
    nextNode.nodeReady = false;
    nextNode.next = NULL;
    nextNode.prev = NULL;
    memset(nextNode.signalData, SIGNAL_NOT_KNOWN, sizeof(nextNode.signalData));

    prevNode.signal = SIGNAL_NOT_KNOWN;
    prevNode.nodeNo = (THIS_NODE_NUM - 1);
    prevNode.axleCount = 0;
    nextNode.nodeReady = false;
    prevNode.trainDir = TRAIN_DIR_NOT_KNOWN;
    prevNode.next = NULL;
    prevNode.prev = NULL;
    memset(prevNode.signalData, SIGNAL_NOT_KNOWN, sizeof(prevNode.signalData));
#endif
}

static bool isNodeReady(void)
{
    uint8_t signals[NO_OF_NODES_BEFORE + NO_OF_NODES_AFTER];

    memset(signals, SIGNAL_NOT_KNOWN, sizeof(signals));

    if (thisNode.prev != NULL)
    {
        memcpy(signals, thisNode.prev->signalData, NO_OF_NODES_BEFORE);
    }
    if (thisNode.next != NULL)
    {
        memcpy(&signals[NO_OF_NODES_BEFORE], thisNode.next->signalData, NO_OF_NODES_AFTER);
    }

    // signals is not known then the node is not ready
    for (uint8_t i = 0; i < sizeof(signals); i++)
    {
        if (signals[i] == SIGNAL_NOT_KNOWN)
            return false;
    }

    return true;
}

static bool isPayLoadValid(uint8_t *payload, CommunicatingNode *currentComNode)
{
    if (payload[0] == currentComNode->nodeNo && payload[1] == thisNode.nodeNo)
        return true;

    return false;
}

// static void extractPayLoadData(uint8_t *payLoad, uint8_t len, uint8_t currentComNodeNum)
// {
//     uint8_t temp;

//     // data received from next node
//     if (thisNode.next != NULL && communicatingNode.nodeNo == thisNode.next->nodeNo)
//     {
//         thisNode.next->axleCount = payLoad[2];

//         temp = payLoad[5];
//         thisNode.next->signalData[0] = (temp & 0xF0) >> 4; // signal state of next node
//         thisNode.next->signalData[1] = (temp & 0x0F);      // signal state of next node + 1
//         thisNode.next->signal = thisNode.next->signalData[0];

//         temp = payLoad[6];
//         thisNode.next->signalData[2] = (temp & 0xF0) >> 4; // signal state of next node + 2
//     }
//     // data received from previous node
//     else if (thisNode.prev != NULL && communicatingNode.nodeNo == thisNode.prev->nodeNo)
//     {
//         thisNode.prev->axleCount = payLoad[2];

//         temp = payLoad[5];
//         thisNode.prev->signalData[0] = (temp & 0xF0) >> 4; // signal state of prev node
//         thisNode.prev->signal = thisNode.prev->signalData[0];

//         temp = payLoad[4];
//         thisNode.prev->signalData[1] = (temp & 0x0F);      // signal state of prev node -1
//         thisNode.prev->signalData[2] = (temp & 0xF0) >> 4; // signal state of prev node - 2
//     }
// }
static void extractPayLoadData(uint8_t *payLoad, uint8_t len, uint8_t currentComNodeNum)
{
    uint8_t temp;

    // data received from next node
    if (thisNode.next != NULL && communicatingNode.nodeNo == thisNode.next->nodeNo)
    {
        thisNode.next->axleCount = payLoad[2];

        thisNode.next->signalData[0] = payLoad[6]; // signal state of next node
        thisNode.next->signal = thisNode.next->signalData[0];

        thisNode.next->signalData[1] = payLoad[7]; // signal state of next node + 1

        thisNode.next->signalData[2] = payLoad[8]; // signal state of next node + 2
    }
    // data received from previous node
    else if (thisNode.prev != NULL && communicatingNode.nodeNo == thisNode.prev->nodeNo)
    {
        thisNode.prev->axleCount = payLoad[2];

        thisNode.prev->signalData[0] = payLoad[6]; // signal state of prev node
        thisNode.prev->signal = thisNode.prev->signalData[0];

        thisNode.prev->signalData[1] = payLoad[5]; // signal state of prev node -1
        thisNode.prev->signalData[2] = payLoad[4]; // signal state of prev node - 2
    }
}

// static void updateTxPayload(uint8_t *buffer)
// {
//     memset(buffer, 0, PAYLOAD_LENGTH);

//     thisNode.axleCount = axleCounter;

//     buffer[0] = thisNode.nodeNo;
//     buffer[1] = communicatingNode.nodeNo;
//     buffer[2] = thisNode.axleCount;
//     buffer[5] = currentSignalState->state << 4;

//     if (thisNode.prev != NULL)
//     {
//         buffer[4] = thisNode.prev->signalData[0];
//         buffer[4] |= thisNode.prev->signalData[1] << 4;
//         buffer[3] |= thisNode.prev->signalData[2];
//     }
//     else if (thisNode.next != NULL)
//     {
//         buffer[5] |= thisNode.next->signalData[0];
//         buffer[6] = thisNode.next->signalData[1] << 4;
//         buffer[6] |= thisNode.next->signalData[2];
//     }
// }

static void updateTxPayload(uint8_t *buffer)
{
    memset(buffer, 0, PAYLOAD_LENGTH);

    thisNode.axleCount = axleCounter;

    buffer[0] = thisNode.nodeNo;
    buffer[1] = communicatingNode.nodeNo;
    buffer[2] = thisNode.axleCount;
    buffer[6] = currentSignalState->state;

    if (thisNode.prev != NULL)
    {
        buffer[5] = thisNode.prev->signalData[0];
        buffer[4] = thisNode.prev->signalData[1];
        buffer[3] = thisNode.prev->signalData[2];
    }

    if (thisNode.next != NULL)
    {
        buffer[7] = thisNode.next->signalData[0];
        buffer[8] = thisNode.next->signalData[1];
        buffer[9] = thisNode.next->signalData[2];
    }
}

static void updateSignalState(void)
{
    SignalState temp = *currentSignalState;
    thisNode.axleCount = axleCounter;

    switch (trainDir)
    {
    case TO_HIGHER_NODE:

        switch (temp.state)
        {
        case RED:
            if (thisNode.axleCount == thisNode.next->axleCount)
            {
                // if train is moving towards higher node and the axle counter from the next node is equal to this node
                // than the train has passed this signal and the signal after
                if (temp.next != NULL)
                    currentSignalState = temp.next;
                axleCounter = 0;
                thisNode.axleCount = 0;
            }
            break;

        case YELLOW:
            if (thisNode.next->signal == YELLOW)
            {
                // if next node is yellow than this node should now be double yellow
                if (temp.next != NULL)
                    currentSignalState = temp.next;
            }

            break;

        case DOUBLE_YELLOW:
            if (thisNode.next->signal == DOUBLE_YELLOW)
            {
                // if the next node is double yellow then this node should now be green
                if (temp.next != NULL)
                {
                    currentSignalState = temp.next;
                }   
            }
            break;

        case GREEN:
            trainDir = TRAIN_DIR_NOT_KNOWN;
            break;

        default:
            break;
        }



        break;

    case TO_LOWER_NODE:
        switch (temp.state)
        {
        case RED:
            if (thisNode.axleCount == thisNode.prev->axleCount)
            {
                // if train is moving towards lower node and the axle counter from the prev node is equal to this node
                // than the train has passed this signal and the signal before
                if (temp.next != NULL)
                    currentSignalState = temp.next;
                axleCounter = 0;
                thisNode.axleCount = 0;
            }
            break;

        case YELLOW:
            if (thisNode.prev->signal == YELLOW)
            {
                if (temp.next != NULL)
                    currentSignalState = temp.next;
            }

            break;

        case DOUBLE_YELLOW:
            if (thisNode.prev->signal == DOUBLE_YELLOW)
            {
                if (temp.next != NULL)
                    currentSignalState = temp.next;
            }
            break;

        case GREEN:
            trainDir = TRAIN_DIR_NOT_KNOWN;
            break;

        default:
            break;
        }

        break;

    default:
        break;
    }
}

void setSignalLeds(void)
{
    SignalState temp = *currentSignalState;
    switch (temp.state)
    {
    case GREEN:
        HAL_GPIO_WritePin(LED_SIGNAL_GPIO_PORT, LED_SIGNAL_GREEN_GPIO_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_SIGNAL_GPIO_PORT, LED_SIGNAL_RED_GPIO_PIN | LED_SIGNAL_YELLOW1_GPIO_PIN | LED_SIGNAL_YELLOW2_GPIO_PIN, GPIO_PIN_RESET);
        break;

    case RED:
        HAL_GPIO_WritePin(LED_SIGNAL_GPIO_PORT, LED_SIGNAL_RED_GPIO_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_SIGNAL_GPIO_PORT, LED_SIGNAL_YELLOW1_GPIO_PIN | LED_SIGNAL_GREEN_GPIO_PIN | LED_SIGNAL_YELLOW2_GPIO_PIN, GPIO_PIN_RESET);
        break;

    case YELLOW:
        HAL_GPIO_WritePin(LED_SIGNAL_GPIO_PORT, LED_SIGNAL_YELLOW1_GPIO_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_SIGNAL_GPIO_PORT, LED_SIGNAL_RED_GPIO_PIN | LED_SIGNAL_GREEN_GPIO_PIN | LED_SIGNAL_YELLOW2_GPIO_PIN, GPIO_PIN_RESET);
        break;

    case DOUBLE_YELLOW:
        HAL_GPIO_WritePin(LED_SIGNAL_GPIO_PORT, LED_SIGNAL_YELLOW1_GPIO_PIN | LED_SIGNAL_YELLOW2_GPIO_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_SIGNAL_GPIO_PORT, LED_SIGNAL_RED_GPIO_PIN | LED_SIGNAL_GREEN_GPIO_PIN, GPIO_PIN_RESET);
        break;

    default:
        break;
    }
}

void LoRaReInit(void)
{
    LoRaReset();

#if (NODE_TYPE == SLAVE_NODE)

#if (THIS_NODE_NUM > 1 && THIS_NODE_NUM < TOTAL_NO_OF_NODES)
    communicatingNode.spreadingFactor = LORA_UP_SF;
    communicatingNode.bandwidth = LORA_UP_BANDWIDTH;
    communicatingNode.nodeNo = THIS_NODE_NUM + 1;
#elif (THIS_NODE_NUM == 1)
    communicatingNode.spreadingFactor = LORA_UP_SF;
    communicatingNode.bandwidth = LORA_UP_BANDWIDTH;
    communicatingNode.nodeNo = THIS_NODE_NUM + 1;
#else
    communicatingNode.spreadingFactor = LORA_DOWN_SF;
    communicatingNode.bandwidth = LORA_DOWN_BANDWIDTH;
    communicatingNode.nodeNo = THIS_NODE_NUM - 1;
#endif

#else
    communicatingNode.spreadingFactor = LORA_DOWN_SF;
    communicatingNode.bandwidth = LORA_DOWN_BANDWIDTH;
    communicatingNode.nodeNo = THIS_NODE_NUM - 1;
#endif

    if (lora_init(&lora, &hLoRaSpi1, LORA_GPIO_PORT, LORA_GPIO_NSS_PIN, LORA_BASE_FREQUENCY) != LORA_OK)
    {
        // LoRa initialisation failed
        while (1)
        {
            HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_PORT, LED_BUILTIN_GPIO_PIN);
            HAL_Delay(50);
        }
    }
    lora_set_tx_power(&lora, LORA_TX_POWER_LEVEL);
    lora_set_preamble_length(&lora, LORA_PREAMBLE_LENGTH);
    lora_set_crc(&lora, 1);

    lora_set_signal_bandwidth(&lora, communicatingNode.bandwidth);
    lora_set_spreading_factor(&lora, communicatingNode.spreadingFactor);

}

void nodeInit()
{
    signalStateInit();

    // turn all the signal lights ON
    HAL_GPIO_WritePin(LED_SIGNAL_GPIO_PORT, LED_SIGNAL_RED_GPIO_PIN | LED_SIGNAL_YELLOW1_GPIO_PIN | LED_SIGNAL_GREEN_GPIO_PIN | LED_SIGNAL_YELLOW2_GPIO_PIN, GPIO_PIN_SET);

    LoRaReInit();
}

#if (NODE_TYPE == SLAVE_NODE)

void node(void)
{
    bool validPayLoad = false;
    uint8_t error;
    uint8_t timeout = LORA_UP_TOA + 50;

    // enter receive mode
    // if packet received from master
    // extract the data and check is node is ready
    // enter transmit mode and transmit data to master
    // if starting node or ending node then stay in receive mode
    // else switch communication node no and frequency
    // once node is ready set the signal leds

    lora_mode_receive_continuous(&lora);
    // HAL_Delay(100);
    prevMillis = HAL_GetTick();

    while (!validPayLoad)
    {
        //     currentMillis = HAL_GetTick();

        //     if (currentMillis - prevMillis > TIMEOUT)
        //     {
        //         // timeout = true;
        //         break;
        //     }

        if (lora_is_packet_available(&lora))
        {
            lora_receive_packet(&lora, rxPayLoad, PAYLOAD_LENGTH, &error);

            if (error == LORA_OK)
            {
                // if packet received from the correct node
                if (isPayLoadValid(rxPayLoad, &communicatingNode))
                {
                    LOG_RX_PAYLOAD();
                    lora_mode_standby(&lora);
                    validPayLoad = true;
                }
            }
        }
    }
    if (validPayLoad == true)
    {
        extractPayLoadData(rxPayLoad, PAYLOAD_LENGTH, communicatingNode.nodeNo);
        thisNode.nodeReady = isNodeReady();

#if (THIS_NODE_NUM > 1 && THIS_NODE_NUM < TOTAL_NO_OF_NODES)

#endif
        updateTxPayload(txPayLoad);
        error = lora_send_packet_blocking(&lora, txPayLoad, PAYLOAD_LENGTH, TIMEOUT);
        if (error == LORA_OK)
            HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_PORT, LED_BUILTIN_GPIO_PIN);

        printf("Tx Error: %d\n", error);

        LOG_TX_PAYLOAD();
#if (THIS_NODE_NUM > 1 && THIS_NODE_NUM < TOTAL_NO_OF_NODES)

        if (communicatingNode.nodeNo == THIS_NODE_NUM + 1)
        {
            communicatingNode.nodeNo = THIS_NODE_NUM - 1;
            communicatingNode.spreadingFactor = LORA_DOWN_SF;
            communicatingNode.bandwidth = LORA_DOWN_BANDWIDTH;
        }
        else
        {
            communicatingNode.nodeNo = THIS_NODE_NUM + 1;
            communicatingNode.spreadingFactor = LORA_UP_SF;
            communicatingNode.bandwidth = LORA_UP_BANDWIDTH;
        }
        lora_set_signal_bandwidth(&lora, communicatingNode.bandwidth);
        lora_set_spreading_factor(&lora, communicatingNode.spreadingFactor);

#else
        // HAL_Delay(240);
#endif
        validPayLoad = false;
    }

    // if (timeout == false && error == LORA_OK)
    // {
    //     updateTxPayload(txPayLoad);
    //     error = lora_send_packet_blocking(&lora, txPayLoad, PAYLOAD_LENGTH, TIMEOUT);
    //     if (error == LORA_OK)
    //         HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_PORT, LED_BUILTIN_GPIO_PIN);
    // }

    // timeout = false;
    if (thisNode.nodeReady)
    {
        updateSignalState();
        setSignalLeds();
    }
}

#endif

#if (NODE_TYPE == MASTER_NODE)

void node(void)
{
    bool validPayLoad = false, timeout = false;
    uint8_t error;

    // while node is not ready
    // transmit data
    // enter receive mode and wait for slave reponse
    // extract the data from the slave response and check is node is ready
    // switch communication node no and frequency
    // once node is ready set the signal leds

    updateTxPayload(txPayLoad);
    error = lora_send_packet_blocking(&lora, txPayLoad, PAYLOAD_LENGTH, TIMEOUT);
    printf("TX Error: %d\n", error);
    LOG_TX_PAYLOAD();
    // check for packet if not timeout
    lora_mode_receive_continuous(&lora);
    prevMillis = HAL_GetTick();
    while (!validPayLoad)
    {
        currentMillis = HAL_GetTick();

        if (currentMillis - prevMillis > TIMEOUT)
        {
            lora_mode_standby(&lora);
            printf("timeout\n");
            timeout = true;
            break;
        }

        if (lora_is_packet_available(&lora))
        {
            lora_receive_packet(&lora, rxPayLoad, PAYLOAD_LENGTH, &error);

            if (error == LORA_OK)
            {
                LOG_RX_PAYLOAD();
                // check for valid payload
                if (isPayLoadValid(rxPayLoad, &communicatingNode))
                {
                    lora_mode_standby(&lora);
                    validPayLoad = true;
                }
            }
            else
            {
                printf("Error: %d\n", error);
                LOG_RX_PAYLOAD();
            }
        }
    }

    if (validPayLoad)
    {
        extractPayLoadData(rxPayLoad, PAYLOAD_LENGTH, communicatingNode.nodeNo);
        thisNode.nodeReady = isNodeReady();
        HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_PORT, LED_BUILTIN_GPIO_PIN);
        validPayLoad = false;
    }

    if (communicatingNode.nodeNo == THIS_NODE_NUM + 1)
    {
        communicatingNode.nodeNo = THIS_NODE_NUM - 1;
    }
    else
    {
        communicatingNode.nodeNo = THIS_NODE_NUM + 1;
    }

    if (thisNode.nodeReady)
    {
        updateSignalState();
        setSignalLeds();
    }
}

#endif

/*
 * @brief Resets the LoRa module
 *
 * Pulls the rst pin low for 1mS and then waits for 10mS
 *
 * @param None
 *
 * @return None
 */
void LoRaReset(void)
{
    HAL_GPIO_WritePin(LORA_GPIO_PORT, LORA_GPIO_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(LORA_GPIO_PORT, LORA_GPIO_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == PROXMITY_SNSR_GPIO_PIN)
    {
        if (HAL_GPIO_ReadPin(PROXMITY_SNSR_GPIO_PORT, PROXMITY_SNSR_GPIO_PIN) == GPIO_PIN_RESET)
        {
            axleCounter++;
        }

        if (currentSignalState->state != RED)
        {
            currentSignalState = &red;

            switch (thisNode.nodeNo)
            {
            case 1:
                // for first node
                // check next node axle counter
                // if it is greater than this node than train is moving towards lower node
                // else it is moving towards higher node
                if (thisNode.next->axleCount > axleCounter)
                {
                    trainDir = TO_LOWER_NODE;
                }
                else if (thisNode.next->axleCount < axleCounter)
                {
                    trainDir = TO_HIGHER_NODE;
                }
                else
                {
                    trainDir = TRAIN_DIR_NOT_KNOWN;
                }
                break;

            case TOTAL_NO_OF_NODES:
                // for last node
                // check prev node axle counter
                // if it is greater than this node than train is moving towards higher node
                // else it is moving towards lower node
                if (thisNode.prev->axleCount > axleCounter)
                {
                    trainDir = TO_HIGHER_NODE;
                }
                else if (thisNode.prev->axleCount < axleCounter)
                {
                    trainDir = TO_LOWER_NODE;
                }
                else
                {
                    trainDir = TRAIN_DIR_NOT_KNOWN;
                }
                break;

            default:
                // for  center node
                // check if prev node axle counter is greater than next node axle counter, train is moving towards higher node
                // else if prev node axle counter is less than next node axle counter, train is moving towards lower node
                if (thisNode.prev->axleCount > axleCounter && thisNode.next->axleCount < axleCounter)
                {
                    trainDir = TO_HIGHER_NODE;
                }
                else if (thisNode.prev->axleCount < axleCounter && thisNode.next->axleCount > axleCounter)
                {
                    trainDir = TO_LOWER_NODE;
                }
                else
                {
                    trainDir = TRAIN_DIR_NOT_KNOWN;
                }
                break;
            }
        }
    }
}
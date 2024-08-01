#ifndef __PPP_INIT_H
#define __PPP_INIT_H


#include <stm32f1xx_hal.h>
#include "pinout.h"

extern SPI_HandleTypeDef hLoRaSpi1;

void ClockConfig(void);
void GPIO_Init(void);
void SPI_Init(void);
void NVIC_Init(void);

#endif
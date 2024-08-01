#ifndef __PINOUT_H
#define __PINOUT_H

#include <stm32f1xx_hal.h>

/* LORA */
#define LORA_GPIO_PORT GPIOB
#define LORA_GPIO_NSS_PIN GPIO_PIN_10
#define LORA_GPIO_RST_PIN GPIO_PIN_1
#define LORA_GPIO_DIDO_PIN GPIO_PIN_0

#define LORA_SPI_PORT GPIOA
#define LORA_SPI_INSTANCE SPI1
#define LORA_SPI_MOSI_PIN GPIO_PIN_7
#define LORA_SPI_MISO_PIN GPIO_PIN_6
#define LORA_SPI_SCK_PIN GPIO_PIN_5

/* LEDS */
#define LED_BUILTIN_GPIO_PORT GPIOC
#define LED_BUILTIN_GPIO_PIN GPIO_PIN_13

#define LED_SIGNAL_GPIO_PORT GPIOA
#define LED_SIGNAL_RED_GPIO_PIN GPIO_PIN_3
#define LED_SIGNAL_YELLOW1_GPIO_PIN GPIO_PIN_2
#define LED_SIGNAL_GREEN_GPIO_PIN GPIO_PIN_1
#define LED_SIGNAL_YELLOW2_GPIO_PIN GPIO_PIN_0

/* SENSOR */
#define PROXMITY_SNSR_GPIO_PORT GPIOB
#define PROXMITY_SNSR_GPIO_PIN GPIO_PIN_14

#endif
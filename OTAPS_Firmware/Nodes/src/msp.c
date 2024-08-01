#include "ppp_init.h"

void HAL_MspInit()
{
    ClockConfig();
    SPI_Init();
    GPIO_Init();
    NVIC_Init();
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *spiHandle)
{
    GPIO_InitTypeDef GPIO_LoRa;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();

    /* LoRa SPI MOSI and SCK pin as alternate function output */
    GPIO_LoRa.Pin = LORA_SPI_MOSI_PIN | LORA_SPI_SCK_PIN;
    GPIO_LoRa.Mode = GPIO_MODE_AF_PP;
    GPIO_LoRa.Pull = GPIO_NOPULL;
    GPIO_LoRa.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(LORA_SPI_PORT, &GPIO_LoRa);

    /* LoRa SPI MISO pin as alternate funciton input */
    GPIO_LoRa.Pin = LORA_SPI_MISO_PIN;
    GPIO_LoRa.Mode = GPIO_MODE_AF_INPUT;
    GPIO_LoRa.Pull = GPIO_NOPULL;
    GPIO_LoRa.Speed = GPIO_SPEED_FREQ_MEDIUM;

    HAL_GPIO_Init(LORA_SPI_PORT, &GPIO_LoRa);


    __HAL_SPI_ENABLE(spiHandle);
}

#include "ppp_init.h"


SPI_HandleTypeDef hLoRaSpi1;

/*
 * @brief Configures the system clock frequency
 *
 * Sets the System frequency to 64MHz, configures the PLL to use HSE as input with a multiplier of 8
 *
 * @param None
 *
 * @return None
 */
void ClockConfig(void)
{
    RCC_OscInitTypeDef OSC;
    RCC_ClkInitTypeDef CLK;


    OSC.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    OSC.HSEState = RCC_HSE_ON;
    OSC.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    OSC.PLL.PLLMUL = RCC_PLL_MUL8;
    OSC.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    OSC.PLL.PLLState = RCC_PLL_ON;

    if (HAL_RCC_OscConfig(&OSC) != HAL_OK)
    {
        while (1)
        {
            /* Error */
        }
    }

    CLK.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    CLK.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    CLK.AHBCLKDivider = RCC_SYSCLK_DIV1;
    CLK.APB1CLKDivider = RCC_HCLK_DIV2;
    CLK.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&CLK, FLASH_LATENCY_2) != HAL_OK)
    {
        while (1)
        {
            /* Error */
        }
    }
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

/*
 * @brief Initialises the SPI1 module
 *
 * Initialises the SPI1 pheripheral in full duplex master mode, datasize of 8 bits, MSB first and baudrate of 4Mb/s
 *
 * @param None
 *
 * @return None
 */
void SPI_Init()
{
    hLoRaSpi1.Instance = LORA_SPI_INSTANCE;
    hLoRaSpi1.Init.Mode = SPI_MODE_MASTER;
    hLoRaSpi1.Init.Direction = SPI_DIRECTION_2LINES;
    hLoRaSpi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hLoRaSpi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hLoRaSpi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hLoRaSpi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hLoRaSpi1.Init.NSS = SPI_NSS_SOFT;
    hLoRaSpi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hLoRaSpi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hLoRaSpi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hLoRaSpi1.Init.CRCPolynomial = 10;

    if (HAL_SPI_Init(&hLoRaSpi1) != HAL_OK)
    {
        while (1)
        {
            /* error */
        }
    }
}

void GPIO_Init(void)
{

    GPIO_InitTypeDef GPIO_PortA;
    GPIO_InitTypeDef GPIO_PortB;
    GPIO_InitTypeDef GPIO_PortC;

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* LoRa NSS and RST pin as ouput with pull up */
    GPIO_PortB.Pin = LORA_GPIO_NSS_PIN | LORA_GPIO_RST_PIN;
    GPIO_PortB.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_PortB.Pull = GPIO_PULLUP;
    GPIO_PortB.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_WritePin(LORA_GPIO_PORT, LORA_GPIO_RST_PIN | LORA_GPIO_NSS_PIN, GPIO_PIN_SET);
    HAL_GPIO_Init(LORA_GPIO_PORT, &GPIO_PortB);

    /* Builtin Led pin as output */
    GPIO_PortC.Pin = LED_BUILTIN_GPIO_PIN;
    GPIO_PortC.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_PortC.Pull = GPIO_NOPULL;
    GPIO_PortC.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_WritePin(LED_BUILTIN_GPIO_PORT, LED_BUILTIN_GPIO_PIN, GPIO_PIN_SET);
    HAL_GPIO_Init(LED_BUILTIN_GPIO_PORT, &GPIO_PortC);

    /* Proximity Sensor pin as input with pull up and interrupt on falling edge */
    GPIO_PortB.Pin = TRAINDIR_SWITCH_GPIO_PORT;
    GPIO_PortB.Mode = GPIO_MODE_INPUT;
    GPIO_PortB.Pull = GPIO_PULLUP;

    HAL_GPIO_Init(TRAINDIR_SWITCH_GPIO_PORT, &GPIO_PortB);
}

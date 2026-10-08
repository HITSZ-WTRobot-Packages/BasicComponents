#include "main.h"

// Strong override of the weak HAL_ETH_MspInit/HAL_ETH_MspDeInit symbols
// (Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_eth.c).
extern "C" void HAL_ETH_MspInit(ETH_HandleTypeDef* handle)
{
    if (handle->Instance != ETH)
        return;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_ETH1MAC_CLK_ENABLE();
    __HAL_RCC_ETH1TX_CLK_ENABLE();
    __HAL_RCC_ETH1RX_CLK_ENABLE();

    GPIO_InitTypeDef pins{};
    pins.Mode      = GPIO_MODE_AF_PP;
    pins.Pull      = GPIO_NOPULL;
    pins.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    pins.Alternate = GPIO_AF11_ETH;

    // REF_CLK, MDIO, CRS_DV; MDIO requires the board's external pull-up.
    pins.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &pins);
    // TX_EN, TXD0, TXD1.
    pins.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
    HAL_GPIO_Init(GPIOB, &pins);
    // MDC, RXD0, RXD1.
    pins.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOC, &pins);

    // ETH completion callbacks release RTOS semaphores: priority must be >= 5.
    HAL_NVIC_SetPriority(ETH_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(ETH_IRQn);
}

extern "C" void HAL_ETH_MspDeInit(ETH_HandleTypeDef* handle)
{
    if (handle->Instance != ETH)
        return;

    HAL_NVIC_DisableIRQ(ETH_IRQn);
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13);
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5);
    __HAL_RCC_ETH1RX_CLK_DISABLE();
    __HAL_RCC_ETH1TX_CLK_DISABLE();
    __HAL_RCC_ETH1MAC_CLK_DISABLE();
    // GPIO clocks are shared with other peripherals and remain enabled.
}

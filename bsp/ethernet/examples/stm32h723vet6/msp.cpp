#include "msp.hpp"

#if USE_HAL_ETH_REGISTER_CALLBACKS != 1
#    error "UserCode ETH MSP requires HAL ETH callback registration."
#endif

namespace
{
void eth_msp_init(ETH_HandleTypeDef* handle)
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

void eth_msp_deinit(ETH_HandleTypeDef* handle)
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
} // namespace

bool eth_msp_register(ETH_HandleTypeDef* handle) noexcept
{
    if (handle == nullptr)
        return false;

    if (HAL_ETH_RegisterCallback(handle, HAL_ETH_MSPINIT_CB_ID, eth_msp_init) != HAL_OK)
        return false;

    return HAL_ETH_RegisterCallback(handle, HAL_ETH_MSPDEINIT_CB_ID, eth_msp_deinit) == HAL_OK;
}

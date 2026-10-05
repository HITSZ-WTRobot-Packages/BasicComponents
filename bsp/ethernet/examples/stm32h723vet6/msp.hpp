#pragma once

#include "main.h"

// Register UserCode RMII callbacks before HAL_ETH_Init; false means init must stop.
// Callback registration avoids collisions with CubeMX's generated MSP symbols.
bool eth_msp_register(ETH_HandleTypeDef* handle) noexcept;

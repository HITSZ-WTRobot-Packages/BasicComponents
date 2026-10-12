#!/usr/bin/env python3
"""Focused host regression for the bsp/can_driver2 acceptance filters.

Compiles the three real can_driver2 translation units against the real STM32Cube
HAL/CMSIS headers of one family, links them with a small model of the HAL
hardware boundary, and checks the filter behaviors of interest:

    python3 bsp/can_driver2/can_driver_filter_test.py --family g4 \
        --drivers /path/to/STM32Cube_FW_G4_V1.6.3/Drivers

Generated shims, fixture and objects live in a temporary directory removed on
exit. Simulated boundary: the HAL entry points the driver calls are replaced by
an in-process model that captures the native filter descriptors and interprets
the real Mask and action fields to decide acceptance and FIFO placement; frames
are injected instead of arriving from a bus; the CMSIS core header is shadowed
by an x86 intrinsic shim. Not modelled: bus timing, arbitration, error counters
and TX completion. TX calls are inert stubs and no test path touches a
peripheral register, so there are no board side effects.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
LIB_ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))

# family: (HAL define, callback define, device, core header, Cortex-M, HAL dir,
#          CMSIS device dir, hal_def, hal module)
FAMILIES = {
    "f4": ("HAL_CAN_MODULE_ENABLED", "USE_HAL_CAN_REGISTER_CALLBACKS=1", "STM32F405xx",
           "core_cm4.h", "4", "STM32F4xx_HAL_Driver/Inc",
           "CMSIS/Device/ST/STM32F4xx/Include", "stm32f4xx_hal_def.h", "stm32f4xx_hal_can.h"),
    "g4": ("HAL_FDCAN_MODULE_ENABLED", "USE_HAL_FDCAN_REGISTER_CALLBACKS=1", "STM32G431xx",
           "core_cm4.h", "4", "STM32G4xx_HAL_Driver/Inc",
           "CMSIS/Device/ST/STM32G4xx/Include", "stm32g4xx_hal_def.h", "stm32g4xx_hal_fdcan.h"),
    "h7": ("HAL_FDCAN_MODULE_ENABLED", "USE_HAL_FDCAN_REGISTER_CALLBACKS=1", "STM32H723xx",
           "core_cm7.h", "7", "STM32H7xx_HAL_Driver/Inc",
           "CMSIS/Device/ST/STM32H7xx/Include", "stm32h7xx_hal_def.h", "stm32h7xx_hal_fdcan.h"),
}

HOST_CMSIS = r'''/* Host-only CMSIS core substitute (generated): the register qualifiers and
   non-ARM intrinsic surface the real HAL and device headers need. PRIMASK is a
   memory cell, so ISRGuard keeps its real read/disable/restore control flow. */
#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define __CORTEX_M __CORTEX_M_VALUE__U
#define __NVIC_PRIO_BITS 4U
#define __I volatile const
#define __O volatile
#define __IO volatile
#define __weak __attribute__((weak))
#define __STATIC_INLINE static inline
#define _VAL2FLD(field, value) (((uint32_t)(value) << field##_Pos) & field##_Msk)
#define _FLD2VAL(field, value) (((uint32_t)(value) & field##_Msk) >> field##_Pos)
extern volatile uint32_t host_primask_sim;
__STATIC_INLINE uint32_t __CLZ(uint32_t v) { return v == 0U ? 32U : (uint32_t)__builtin_clz(v); }
__STATIC_INLINE uint32_t __RBIT(uint32_t v) { uint32_t r = 0U; for (uint32_t i = 0U; i < 32U; ++i) { r = (r << 1U) | (v & 1U); v >>= 1U; } return r; }
__STATIC_INLINE uint32_t __get_PRIMASK(void) { return host_primask_sim; }
__STATIC_INLINE void     __set_PRIMASK(uint32_t value) { host_primask_sim = value; }
__STATIC_INLINE void     __disable_irq(void) { host_primask_sim = 1U; }
__STATIC_INLINE void     __enable_irq(void) { host_primask_sim = 0U; }
__STATIC_INLINE uint32_t __get_IPSR(void) { return 0U; }
__STATIC_INLINE uint32_t __get_BASEPRI(void) { return 0U; }
__STATIC_INLINE void     __set_BASEPRI(uint32_t value) { (void)value; }
__STATIC_INLINE void     __DSB(void) {}
__STATIC_INLINE void     __ISB(void) {}
#ifdef __cplusplus
}
#endif
'''

CORE_SHADOW = r'''/* Shadows the real CMSIS core header (ARM intrinsics cannot run here). */
#pragma once
#include "host_cmsis.h"
'''

MAIN_SHIM = r'''/* Host shim for the CubeMX-generated Core/Inc/main.h. */
#pragma once
#include "__HAL_DEF__"
#include "__HAL_MODULE__"
void Error_Handler(void);
'''

HOST_SIM_H = r'''/* Control surface of the host HAL boundary (see hal_sim_host.cpp). */
#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define HOST_SIM_DROPPED      0
#define HOST_SIM_STORED_FIFO0 1
#define HOST_SIM_STORED_FIFO1 2
/* Zero the model and attach it to the native handle. */
void host_sim_reset(void* handle);
/* Inject one standard-ID classic data frame: apply the programmed acceptance
   filtering and raise the matching FIFO interrupt callback on acceptance. */
int host_sim_deliver(uint32_t id, uint32_t length, uint8_t seed);
#ifdef __cplusplus
}
#endif
'''

HAL_SIM_CPP = r'''/* Host HAL boundary for the can_driver2 filter regression (generated).
 * Replaces only the HAL entry points the driver calls. Acceptance filtering is
 * evaluated from the native descriptors captured by HAL_*_ConfigFilter() using
 * the real Mask and action fields, so a filter whose native action does not
 * store (reject, priority-only, disabled) never delivers a frame. */
#ifndef _GNU_SOURCE
#    define _GNU_SOURCE 1
#endif
#include "host_sim.h"
#include "main.h"
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <tuple>

volatile uint32_t host_primask_sim = 0U;

void Error_Handler(void) { std::abort(); }

/* Parameter types come from the real HAL declarations, so a definition matches
   the installed header even where its constness differs. */
namespace hal_decl
{
template <class F> struct sig;
template <class R, class... A> struct sig<R (*)(A...)> { using args = std::tuple<A...>; };
template <std::size_t I, class F> using arg = std::tuple_element_t<I, typename sig<F>::args>;
} // namespace hal_decl

struct HostMsg { uint32_t id = 0U; uint32_t dlc = 0U; uint8_t data[8] = {}; };

#if defined(HAL_FDCAN_MODULE_ENABLED)

using FdcanCb = void (*)(FDCAN_HandleTypeDef*, uint32_t);

namespace
{
constexpr uint32_t kSlots = 16U;
constexpr uint32_t kFifo  = 4U;

struct Sim
{
    FDCAN_HandleTypeDef* handle = nullptr;
    FdcanCb  rx_cb[2] = {};
    FDCAN_FilterTypeDef element[kSlots];
    bool     used[kSlots] = {};
    uint32_t anfs = FDCAN_ACCEPT_IN_RX_FIFO0; /* device reset default */
    HostMsg  fifo[2][kFifo];
    uint32_t count[2] = {};
    uint32_t interrupts = 0U;
};

Sim g;

uint32_t dlc_of(uint32_t length)
{
    static const uint32_t code[9] = {FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
                                     FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
                                     FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8};
    return code[length < 9U ? length : 8U];
}

bool pop(uint32_t fifo, HostMsg& out)
{
    if (g.count[fifo] == 0U)
        return false;
    out = g.fifo[fifo][0];
    for (uint32_t slot = 1U; slot < g.count[fifo]; ++slot)
        g.fifo[fifo][slot - 1U] = g.fifo[fifo][slot];
    --g.count[fifo];
    return true;
}

int store(const HostMsg& message, uint32_t fifo)
{
    if (g.count[fifo] >= kFifo)
        return HOST_SIM_DROPPED;
    g.fifo[fifo][g.count[fifo]++] = message;
    const uint32_t notification = fifo == 0U ? FDCAN_IT_RX_FIFO0_NEW_MESSAGE
                                            : FDCAN_IT_RX_FIFO1_NEW_MESSAGE;
    if ((g.interrupts & notification) != 0U && g.rx_cb[fifo] != nullptr)
        g.rx_cb[fifo](g.handle, notification);
    return fifo == 0U ? HOST_SIM_STORED_FIFO0 : HOST_SIM_STORED_FIFO1;
}
} // namespace

extern "C" void host_sim_reset(void* handle)
{
    g = Sim{};
    g.handle         = static_cast<FDCAN_HandleTypeDef*>(handle);
    host_primask_sim = 0U;
}

extern "C" int host_sim_deliver(uint32_t id, uint32_t length, uint8_t seed)
{
    HostMsg message{};
    message.id  = id;
    message.dlc = dlc_of(length);
    for (uint32_t byte = 0U; byte < length && byte < sizeof(message.data); ++byte)
        message.data[byte] = (uint8_t)(seed + byte);

    for (uint32_t index = 0U; index < kSlots; ++index)
    {
        if (!g.used[index])
            continue;
        const FDCAN_FilterTypeDef& filter = g.element[index];
        if (filter.IdType != FDCAN_STANDARD_ID || filter.FilterConfig == FDCAN_FILTER_DISABLE)
            continue;
        /* Real Mask semantics for the mode the tested cases program; other
           FilterTypes are not exercised by this regression. */
        if (filter.FilterType != FDCAN_FILTER_MASK)
            continue;
        if ((id & filter.FilterID2) != (filter.FilterID1 & filter.FilterID2))
            continue;
        /* Real action semantics: reject and priority-only store nothing. */
        switch (filter.FilterConfig)
        {
            case FDCAN_FILTER_TO_RXFIFO0:
            case FDCAN_FILTER_TO_RXFIFO0_HP: return store(message, 0U);
            case FDCAN_FILTER_TO_RXFIFO1:
            case FDCAN_FILTER_TO_RXFIFO1_HP: return store(message, 1U);
            default: return HOST_SIM_DROPPED;
        }
    }
    if (g.anfs == FDCAN_REJECT)
        return HOST_SIM_DROPPED;
    return store(message, g.anfs == FDCAN_ACCEPT_IN_RX_FIFO1 ? 1U : 0U);
}

extern "C" HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef* hfdcan, const FDCAN_FilterTypeDef* config)
{
    if (hfdcan == nullptr || config == nullptr || config->FilterIndex >= kSlots)
        return HAL_ERROR;
    if (hfdcan->State != HAL_FDCAN_STATE_READY && hfdcan->State != HAL_FDCAN_STATE_BUSY)
        return HAL_ERROR;
    g.handle                       = hfdcan;
    g.element[config->FilterIndex] = *config;
    g.used[config->FilterIndex]    = true;
    return HAL_OK;
}

extern "C" HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef* hfdcan, uint32_t std, uint32_t ext, uint32_t rr_std, uint32_t rr_ext)
{
    if (hfdcan == nullptr)
        return HAL_ERROR;
    g.anfs = std;
    (void)ext; /* injection uses standard identifiers only */
    (void)rr_std;
    (void)rr_ext;
    return HAL_OK;
}

extern "C" HAL_StatusTypeDef HAL_FDCAN_RegisterCallback(FDCAN_HandleTypeDef* hfdcan, HAL_FDCAN_CallbackIDTypeDef id, void (*callback)(FDCAN_HandleTypeDef*))
{
    if (hfdcan == nullptr || callback == nullptr)
        return HAL_ERROR;
    if (id == HAL_FDCAN_TX_FIFO_EMPTY_CB_ID)
        return HAL_OK;
#if defined(FDCAN_FILTER_TO_RXBUFFER)
    if (id == HAL_FDCAN_RX_BUFFER_NEW_MSG_CB_ID)
        return HAL_OK;
#endif
    return HAL_ERROR;
}

extern "C" HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef* hfdcan, uint32_t location, FDCAN_RxHeaderTypeDef* header, uint8_t* data)
{
    if (hfdcan == nullptr || header == nullptr || data == nullptr)
        return HAL_ERROR;
    const uint32_t fifo = location == FDCAN_RX_FIFO0 ? 0U : location == FDCAN_RX_FIFO1 ? 1U : 2U;
    HostMsg message{};
    if (fifo == 2U || !pop(fifo, message))
        return HAL_ERROR;
    *header             = FDCAN_RxHeaderTypeDef{};
    header->Identifier  = message.id;
    header->IdType      = FDCAN_STANDARD_ID;
    header->RxFrameType = FDCAN_DATA_FRAME;
    header->DataLength  = message.dlc;
    header->FDFormat    = FDCAN_CLASSIC_CAN;
    std::memcpy(data, message.data, sizeof(message.data));
    return HAL_OK;
}

/* Remaining boundary surface: FIFO callbacks are recorded, the rest is inert
   (TX and dedicated Rx buffers are outside this regression). */
extern "C" HAL_StatusTypeDef HAL_FDCAN_RegisterRxFifo0Callback(FDCAN_HandleTypeDef* h, FdcanCb cb)
{ if (h == nullptr || cb == nullptr) return HAL_ERROR; g.rx_cb[0] = cb; return HAL_OK; }
extern "C" HAL_StatusTypeDef HAL_FDCAN_RegisterRxFifo1Callback(FDCAN_HandleTypeDef* h, FdcanCb cb)
{ if (h == nullptr || cb == nullptr) return HAL_ERROR; g.rx_cb[1] = cb; return HAL_OK; }
extern "C" HAL_StatusTypeDef HAL_FDCAN_RegisterTxBufferCompleteCallback(FDCAN_HandleTypeDef* h, FdcanCb cb)
{ return h != nullptr && cb != nullptr ? HAL_OK : HAL_ERROR; }
extern "C" HAL_StatusTypeDef HAL_FDCAN_RegisterTxBufferAbortCallback(FDCAN_HandleTypeDef* h, FdcanCb cb)
{ return h != nullptr && cb != nullptr ? HAL_OK : HAL_ERROR; }
extern "C" HAL_StatusTypeDef HAL_FDCAN_ConfigExtendedIdMask(FDCAN_HandleTypeDef* h, uint32_t mask)
{ (void)mask; return h == nullptr ? HAL_ERROR : HAL_OK; }
extern "C" HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef* h)
{ if (h == nullptr || h->State != HAL_FDCAN_STATE_READY) return HAL_ERROR; h->State = HAL_FDCAN_STATE_BUSY; return HAL_OK; }
extern "C" HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef* h)
{ if (h == nullptr) return HAL_ERROR; h->State = HAL_FDCAN_STATE_READY; return HAL_OK; }
extern "C" HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef* h, uint32_t its, uint32_t buffers)
{ (void)buffers; if (h == nullptr || h->State != HAL_FDCAN_STATE_BUSY) return HAL_ERROR; g.interrupts |= its; return HAL_OK; }
extern "C" HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(hal_decl::arg<0, decltype(&HAL_FDCAN_GetProtocolStatus)> h, FDCAN_ProtocolStatusTypeDef* status)
{ if (h == nullptr || status == nullptr) return HAL_ERROR; *status = FDCAN_ProtocolStatusTypeDef{}; return HAL_OK; }
extern "C" uint32_t HAL_FDCAN_GetRxFifoFillLevel(hal_decl::arg<0, decltype(&HAL_FDCAN_GetRxFifoFillLevel)> h, uint32_t fifo)
{ (void)h; return fifo == FDCAN_RX_FIFO0 ? g.count[0] : g.count[1]; }
extern "C" uint32_t HAL_FDCAN_GetTxFifoFreeLevel(hal_decl::arg<0, decltype(&HAL_FDCAN_GetTxFifoFreeLevel)> h)
{ (void)h; return 0U; }
extern "C" HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(hal_decl::arg<0, decltype(&HAL_FDCAN_AddMessageToTxFifoQ)> h, hal_decl::arg<1, decltype(&HAL_FDCAN_AddMessageToTxFifoQ)> header, hal_decl::arg<2, decltype(&HAL_FDCAN_AddMessageToTxFifoQ)> data)
{ (void)h; (void)header; (void)data; return HAL_ERROR; }
#if defined(FDCAN_FILTER_TO_RXBUFFER)
extern "C" uint32_t HAL_FDCAN_IsRxBufferMessageAvailable(FDCAN_HandleTypeDef* h, uint32_t index)
{ (void)h; (void)index; return 0U; }
#endif

#elif defined(HAL_CAN_MODULE_ENABLED)

using CanCb = void (*)(CAN_HandleTypeDef*);

namespace
{
#if defined(CAN2) || defined(CAN3)
constexpr uint32_t kBanks = 28U; /* CAN1/CAN2 shared filter domain */
#else
constexpr uint32_t kBanks = 14U;
#endif
constexpr uint32_t kFifo = 3U;

struct Bank
{
    bool     used    = false;
    bool     enabled = false;
    bool     usable  = false; /* 32-bit mask bank, as IdFilter::Mask programs it */
    uint32_t key     = 0U;
    uint32_t mask    = 0U;
    uint32_t fifo    = 0U;
};

struct Sim
{
    CAN_HandleTypeDef* handle = nullptr;
    CanCb  rx_cb[2] = {};
    Bank   banks[kBanks];
    HostMsg fifo[2][kFifo];
    uint32_t count[2] = {};
    uint32_t interrupts = 0U;
};

Sim g;

int store(const HostMsg& message, uint32_t fifo)
{
    if (g.count[fifo] >= kFifo)
        return HOST_SIM_DROPPED;
    g.fifo[fifo][g.count[fifo]++] = message;
    const uint32_t notification = fifo == 0U ? CAN_IT_RX_FIFO0_MSG_PENDING
                                            : CAN_IT_RX_FIFO1_MSG_PENDING;
    if ((g.interrupts & notification) != 0U && g.rx_cb[fifo] != nullptr)
        g.rx_cb[fifo](g.handle);
    return fifo == 0U ? HOST_SIM_STORED_FIFO0 : HOST_SIM_STORED_FIFO1;
}
} // namespace

extern "C" void host_sim_reset(void* handle)
{
    g = Sim{};
    g.handle         = static_cast<CAN_HandleTypeDef*>(handle);
    host_primask_sim = 0U;
}

extern "C" int host_sim_deliver(uint32_t id, uint32_t length, uint8_t seed)
{
    const uint32_t key = (id & 0x7FFU) << 21U; /* STID at 31:21 of the filter key */
    for (uint32_t index = 0U; index < kBanks; ++index)
    {
        const Bank& bank = g.banks[index];
        if (!bank.used || !bank.enabled || !bank.usable)
            continue;
        if ((key & bank.mask) != (bank.key & bank.mask))
            continue;
        HostMsg message{};
        message.id  = id & 0x7FFU;
        message.dlc = length > 8U ? 8U : length;
        for (uint32_t byte = 0U; byte < length && byte < sizeof(message.data); ++byte)
            message.data[byte] = (uint8_t)(seed + byte);
        return store(message, bank.fifo);
    }
    return HOST_SIM_DROPPED;
}

extern "C" HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef* hcan, const CAN_FilterTypeDef* config)
{
    if (hcan == nullptr || config == nullptr)
        return HAL_ERROR;
    if (hcan->State != HAL_CAN_STATE_READY && hcan->State != HAL_CAN_STATE_LISTENING)
        return HAL_ERROR;
    const uint32_t index = config->FilterBank & 0x1FU;
    if (index >= kBanks)
        return HAL_ERROR;
    g.handle     = hcan;
    Bank& bank   = g.banks[index];
    bank.used    = true;
    bank.enabled = config->FilterActivation == CAN_FILTER_ENABLE;
    bank.fifo    = config->FilterFIFOAssignment == CAN_FILTER_FIFO0 ? 0U : 1U;
    bank.usable  = config->FilterScale == CAN_FILTERSCALE_32BIT &&
                  config->FilterMode == CAN_FILTERMODE_IDMASK;
    bank.key  = ((uint32_t)config->FilterIdHigh << 16U) | config->FilterIdLow;
    bank.mask = ((uint32_t)config->FilterMaskIdHigh << 16U) | config->FilterMaskIdLow;
    return HAL_OK;
}

extern "C" HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef* hcan, uint32_t fifo, CAN_RxHeaderTypeDef* header, uint8_t* data)
{
    if (hcan == nullptr || header == nullptr || data == nullptr)
        return HAL_ERROR;
    const uint32_t channel = fifo == CAN_RX_FIFO0 ? 0U : fifo == CAN_RX_FIFO1 ? 1U : 2U;
    if (channel == 2U || g.count[channel] == 0U)
        return HAL_ERROR;
    const HostMsg message = g.fifo[channel][0];
    for (uint32_t index = 1U; index < g.count[channel]; ++index)
        g.fifo[channel][index - 1U] = g.fifo[channel][index];
    --g.count[channel];
    *header       = CAN_RxHeaderTypeDef{};
    header->StdId = message.id; /* standard-ID data frame only */
    header->IDE   = CAN_ID_STD;
    header->RTR   = CAN_RTR_DATA;
    header->DLC   = message.dlc;
    std::memcpy(data, message.data, sizeof(message.data)); /* HAL writes 8 bytes */
    return HAL_OK;
}

extern "C" HAL_StatusTypeDef HAL_CAN_RegisterCallback(CAN_HandleTypeDef* hcan, HAL_CAN_CallbackIDTypeDef id, CanCb callback)
{
    if (hcan == nullptr || callback == nullptr)
        return HAL_ERROR;
    if (id == HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID) { g.rx_cb[0] = callback; return HAL_OK; }
    if (id == HAL_CAN_RX_FIFO1_MSG_PENDING_CB_ID) { g.rx_cb[1] = callback; return HAL_OK; }
    /* Tx/error notifications only drive the software queue: accepted, unused. */
    return id >= HAL_CAN_TX_MAILBOX0_COMPLETE_CB_ID && id <= HAL_CAN_ERROR_CB_ID ? HAL_OK : HAL_ERROR;
}

extern "C" HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef* h)
{ if (h == nullptr || h->State != HAL_CAN_STATE_READY) return HAL_ERROR; h->State = HAL_CAN_STATE_LISTENING; return HAL_OK; }
extern "C" HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef* h)
{ if (h == nullptr) return HAL_ERROR; h->State = HAL_CAN_STATE_READY; return HAL_OK; }
extern "C" HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef* h, uint32_t its)
{ if (h == nullptr || h->State != HAL_CAN_STATE_LISTENING) return HAL_ERROR; g.interrupts |= its; return HAL_OK; }
extern "C" HAL_StatusTypeDef HAL_CAN_DeactivateNotification(CAN_HandleTypeDef* h, uint32_t its)
{ if (h == nullptr) return HAL_ERROR; g.interrupts &= ~its; return HAL_OK; }
extern "C" uint32_t HAL_CAN_GetRxFifoFillLevel(hal_decl::arg<0, decltype(&HAL_CAN_GetRxFifoFillLevel)> h, uint32_t fifo)
{ (void)h; return fifo == CAN_RX_FIFO0 ? g.count[0] : g.count[1]; }
extern "C" uint32_t HAL_CAN_GetTxMailboxesFreeLevel(hal_decl::arg<0, decltype(&HAL_CAN_GetTxMailboxesFreeLevel)> h)
{ (void)h; return 0U; }
extern "C" HAL_StatusTypeDef HAL_CAN_AddTxMessage(hal_decl::arg<0, decltype(&HAL_CAN_AddTxMessage)> h, hal_decl::arg<1, decltype(&HAL_CAN_AddTxMessage)> header, hal_decl::arg<2, decltype(&HAL_CAN_AddTxMessage)> data, uint32_t* mailbox)
{ (void)h; (void)header; (void)data; (void)mailbox; return HAL_ERROR; }

#else
#    error "can_driver2 filter test needs HAL_CAN_MODULE_ENABLED or HAL_FDCAN_MODULE_ENABLED"
#endif
'''

FIXTURE = r'''/* can_driver2 filter regression fixture (generated; temporary). */
#include "can_driver.hpp"
#include "host_sim.h"
#include <cstddef>
#include <cstdint>
#include <cstdio>

using namespace bsp::can;

static int         g_failures = 0;
static const char* g_case     = "";

#define CHECK(cond) do { if (!(cond)) { std::printf("    FAIL %s: %s (line %d)\n", g_case, #cond, __LINE__); ++g_failures; } } while (0)
#define CHECK_STATUS(expr, want) do { const Status s_ = (expr); if (s_ != (want)) { std::printf("    FAIL %s: %s -> %d want %d (line %d)\n", g_case, #expr, static_cast<int>(s_), static_cast<int>(want), __LINE__); ++g_failures; } } while (0)
#define CHECK_INT(expr, want) do { const long long v_ = static_cast<long long>(expr); if (v_ != static_cast<long long>(want)) { std::printf("    FAIL %s: %s -> %lld want %lld (line %d)\n", g_case, #expr, v_, static_cast<long long>(want), __LINE__); ++g_failures; } } while (0)
#define CONFIGURE(bus, want, ...) CHECK_STATUS((bus).configure_filter(__VA_ARGS__), want)

struct Recorder { int calls = 0; std::uint32_t id = 0U; std::size_t length = 0U; std::uint8_t data[8] = {}; };
static Recorder g_rec;

static void on_frame(const FrameView& frame)
{
    ++g_rec.calls;
    g_rec.id     = frame.header.id;
    g_rec.length = frame.data.size();
    for (std::size_t i = 0; i < frame.data.size() && i < sizeof(g_rec.data); ++i)
        g_rec.data[i] = frame.data[i];
}

static IdFilter id_filter(std::uint32_t index, IdType type, FilterAction action,
                          std::uint32_t id1, std::uint32_t id2,
                          std::uint32_t rx_buffer = 0U, bool calibration = false)
{
    return IdFilter{index, type, FilterMode::Mask, action, id1, id2, rx_buffer, calibration};
}

/* The mask-mode standard-ID filter used by most checks below. */
static IdFilter std_filter(std::uint32_t index, FilterAction action, std::uint32_t id,
                           std::uint32_t rx_buffer = 0U, bool calibration = false)
{
    return id_filter(index, IdType::Standard, action, id, 0x7FFU, rx_buffer, calibration);
}

[[maybe_unused]] static GlobalFilter reject_all()
{
    GlobalFilter filter{};
    filter.standard = NonMatchingAction::Reject;
    filter.extended = NonMatchingAction::Reject;
    return filter;
}

static void case_begin(const char* name)
{
    g_case = name;
    g_rec  = Recorder{};
    std::printf("[case] %s\n", name);
}

static void check_payload(std::uint32_t id)
{
    CHECK_INT(g_rec.calls, 1); CHECK_INT(g_rec.id, id);
    CHECK_INT(g_rec.length, 4U);
    for (std::size_t i = 0; i < 4; ++i)
        CHECK_INT(g_rec.data[i], 0xA5U + i);
}

#if CAN_DRIVER_FDCAN
static FDCAN_HandleTypeDef make_fdcan(std::uint32_t rx0, std::uint32_t rx1)
{
    FDCAN_HandleTypeDef handle{};
    handle.Instance           = FDCAN1;
    handle.State              = HAL_FDCAN_STATE_READY;
    handle.Init.FrameFormat   = FDCAN_FRAME_FD_BRS;
    handle.Init.Mode          = FDCAN_MODE_NORMAL;
    handle.Init.StdFiltersNbr = 2U;
    handle.Init.ExtFiltersNbr = 1U;
#if !defined(STM32G4) /* fixed-RAM families have no FIFO layout in Init */
    handle.Init.RxFifo0ElmtsNbr = rx0;    handle.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
    handle.Init.RxFifo1ElmtsNbr = rx1;    handle.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_64;
    handle.Init.RxBuffersNbr = 0U;        handle.Init.RxBufferSize = FDCAN_DATA_BYTES_64;
    handle.Init.TxFifoQueueElmtsNbr = 3U; handle.Init.TxElmtSize = FDCAN_DATA_BYTES_64;
#else
    (void)rx0; (void)rx1;
#endif
    return handle;
}
#else
static CAN_HandleTypeDef make_can()
{
    CAN_HandleTypeDef handle{};
    handle.Instance = CAN1;
    handle.State    = HAL_CAN_STATE_READY;
    return handle;
}
#endif

#if CAN_DRIVER_FDCAN
/* One element with the given action: the matching frame must reach the selected
   FIFO through its registered ISR callback; other ids follow the reject policy. */
static void run_fdcan_case(const char* name, FilterAction action, int verdict)
{
    case_begin(name);
    FDCAN_HandleTypeDef handle = make_fdcan(2U, 2U);
    host_sim_reset(&handle);
    Can bus(handle);
    CONFIGURE(bus, Status::Ok, std_filter(0U, action, 0x123U));
    CONFIGURE(bus, Status::Ok, reject_all());
    CHECK_STATUS(bus.register_callback(on_frame), Status::Ok);
    CHECK_STATUS(bus.start(), Status::Ok);
    CHECK_INT(host_sim_deliver(0x123U, 4U, 0xA5U), verdict);
    check_payload(0x123U);
    g_rec = Recorder{};
    CHECK_INT(host_sim_deliver(0x456U, 4U, 0x11U), HOST_SIM_DROPPED);
    CHECK_INT(g_rec.calls, 0);
    CHECK(HAL_FDCAN_Stop(&handle) == HAL_OK);
}

static void case_fdcan_disable_stops_delivery()
{
    case_begin("fdcan_disable_stops_delivery");
    FDCAN_HandleTypeDef handle = make_fdcan(2U, 2U);
    host_sim_reset(&handle);
    Can bus(handle);
    CONFIGURE(bus, Status::Ok, std_filter(0U, FilterAction::Fifo0, 0x123U));
    CONFIGURE(bus, Status::Ok, std_filter(0U, FilterAction::Disable, 0x123U));
    CONFIGURE(bus, Status::Ok, reject_all());
    CHECK_STATUS(bus.register_callback(on_frame), Status::Ok);
    CHECK_STATUS(bus.start(), Status::Ok);
    CHECK_INT(host_sim_deliver(0x123U, 4U, 0xA5U), HOST_SIM_DROPPED);
    CHECK_INT(g_rec.calls, 0);
    CHECK(HAL_FDCAN_Stop(&handle) == HAL_OK);
}

#if !defined(STM32G4) /* configurable message RAM */
static void case_fdcan_unallocated_fifo()
{
    case_begin("fdcan_unallocated_fifo");
    FDCAN_HandleTypeDef handle = make_fdcan(/*rx0*/ 0U, /*rx1*/ 2U);
    host_sim_reset(&handle);
    Can bus(handle);
    CONFIGURE(bus, Status::NoCapacity, std_filter(0U, FilterAction::PriorityFifo0, 0x123U));
    CONFIGURE(bus, Status::NoCapacity, std_filter(0U, FilterAction::Fifo0, 0x123U));
    CONFIGURE(bus, Status::Ok, std_filter(0U, FilterAction::Fifo1, 0x123U));
}
#endif
#else /* bxCAN */
/* The identifier-width check applies to every action, including Disable, and a
   rejected request must leave the programmed bank untouched. */
static void case_bxcan_disable()
{
    case_begin("bxcan_disable");
    { /* invalid ids are rejected without disturbing the accepted filter */
        CAN_HandleTypeDef handle = make_can();
        host_sim_reset(&handle);
        Can bus(handle);
        CONFIGURE(bus, Status::Ok, std_filter(0U, FilterAction::Fifo0, 0x123U));
        CONFIGURE(bus, Status::InvalidArgument,
                  id_filter(0U, IdType::Standard, FilterAction::Disable, 0x800U, 0x7FFU));
        CONFIGURE(bus, Status::InvalidArgument,
                  id_filter(0U, IdType::Standard, FilterAction::Disable, 0x7FFU, 0x800U));
        CONFIGURE(bus, Status::InvalidArgument,
                  id_filter(0U, IdType::Extended, FilterAction::Disable, 0x20000000U, 0U));
        CONFIGURE(bus, Status::InvalidArgument,
                  id_filter(0U, IdType::Extended, FilterAction::Disable, 0x1FFFFFFFU, 0x20000000U));
        /* At-limit values stay valid, on banks that do not disturb bank 0. */
        CONFIGURE(bus, Status::Ok,
                  id_filter(3U, IdType::Standard, FilterAction::Disable, 0x7FFU, 0x7FFU));
        CONFIGURE(bus, Status::Ok,
                  id_filter(4U, IdType::Extended, FilterAction::Disable, 0x1FFFFFFFU, 0x1FFFFFFFU));
        CHECK_STATUS(bus.register_callback(on_frame), Status::Ok);
        CHECK_STATUS(bus.start(), Status::Ok);
        CHECK_INT(host_sim_deliver(0x123U, 4U, 0xA5U), HOST_SIM_STORED_FIFO0);
        check_payload(0x123U);
        CHECK(HAL_CAN_Stop(&handle) == HAL_OK);
    }
    { /* a valid Disable really disables the previously accepted filter */
        CAN_HandleTypeDef handle = make_can();
        host_sim_reset(&handle);
        Can bus(handle);
        CONFIGURE(bus, Status::Ok, std_filter(0U, FilterAction::Fifo0, 0x123U));
        CONFIGURE(bus, Status::Ok, std_filter(0U, FilterAction::Disable, 0x123U));
        CHECK_STATUS(bus.register_callback(on_frame), Status::Ok);
        CHECK_STATUS(bus.start(), Status::Ok);
        g_rec = Recorder{};
        CHECK_INT(host_sim_deliver(0x123U, 4U, 0xA5U), HOST_SIM_DROPPED);
        CHECK_INT(g_rec.calls, 0);
        CHECK(HAL_CAN_Stop(&handle) == HAL_OK);
    }
}
#endif /* CAN_DRIVER_FDCAN */

/* Rx-Buffer-only fields must not be silently ignored by other actions. */
static void case_payload_field_errors()
{
    case_begin("payload_field_errors");
#if CAN_DRIVER_FDCAN
    FDCAN_HandleTypeDef handle = make_fdcan(2U, 2U);
#else
    CAN_HandleTypeDef handle = make_can();
#endif
    host_sim_reset(&handle);
    Can bus(handle);
    CONFIGURE(bus, Status::InvalidArgument, std_filter(0U, FilterAction::Fifo0, 0x100U, 1U));
    CONFIGURE(bus, Status::Unsupported, std_filter(0U, FilterAction::Fifo0, 0x100U, 0U, true));
#if !CAN_DRIVER_FDCAN
    CONFIGURE(bus, Status::Unsupported,
              id_filter(0U, IdType::Standard, FilterAction::RxBuffer, 0x100U, 0U));
#endif
}

int main()
{
    std::setvbuf(stdout, nullptr, _IONBF, 0U);
    std::printf("can_driver2 acceptance-filter host regression (%s backend)\n",
                CAN_DRIVER_FDCAN ? "FDCAN" : "bxCAN");

    case_payload_field_errors();
#if CAN_DRIVER_FDCAN
    run_fdcan_case("fdcan_priority_fifo0_delivers", FilterAction::PriorityFifo0,
                   HOST_SIM_STORED_FIFO0);
    run_fdcan_case("fdcan_priority_fifo1_delivers", FilterAction::PriorityFifo1,
                   HOST_SIM_STORED_FIFO1);
    case_fdcan_disable_stops_delivery();
#    if !defined(STM32G4)
    case_fdcan_unallocated_fifo();
#    endif
#else
    case_bxcan_disable();
#endif

    if (g_failures == 0)
    {
        std::printf("RESULT: PASS\n");
        return 0;
    }
    std::printf("RESULT: FAIL (%d checks failed)\n", g_failures);
    return 1;
}
'''


def fail(message):
    sys.stderr.write("error: %s\n" % message)
    raise SystemExit(2)


def main():
    parser = argparse.ArgumentParser(description="Host regression for bsp/can_driver2 filters.")
    parser.add_argument("--family", required=True, choices=sorted(FAMILIES),
                        help="HAL family (f4 = bxCAN, g4/h7 = FDCAN)")
    parser.add_argument("--drivers", required=True,
                        help="Cube 'Drivers' root, e.g. /path/to/STM32Cube_FW_G4_V1.6.3/Drivers")
    parser.add_argument("--device", default=None,
                        help="device define; defaults per family: "
                             + ", ".join("%s=%s" % (name, spec[2])
                                         for name, spec in sorted(FAMILIES.items())))
    parser.add_argument("--cxx", default="g++", help="host C++ compiler (default: g++)")
    parser.add_argument("--validate-tx-frame", type=int, default=1, choices=(0, 1),
                        help="value of CAN_DRIVER_VALIDATE_TX_FRAME (default: 1)")
    parser.add_argument("--tx-queue-size", type=int, default=0, choices=(0, 4),
                        help="value of CAN_DRIVER_TX_QUEUE_SIZE (default: 0)")
    parser.add_argument("--sanitize", action="store_true",
                        help="build with -fsanitize=address,undefined")
    args = parser.parse_args()

    hal, callbacks, default_device, core, cortex, hal_dir, dev_dir, hal_def, hal_module = \
            FAMILIES[args.family]
    device = args.device or default_device
    drivers = os.path.abspath(args.drivers)
    hal_inc = os.path.join(drivers, hal_dir)
    dev_inc = os.path.join(drivers, dev_dir)
    device_header = os.path.join(dev_inc, device.lower() + ".h")

    missing = [path for path in (hal_inc, dev_inc) if not os.path.isdir(path)]
    if missing:
        fail("--drivers is not a STM32Cube 'Drivers' root for family %s; missing:\n  %s\n"
             "expected layout: <drivers>/%s and <drivers>/%s"
             % (args.family, "\n  ".join(missing), hal_dir, dev_dir))
    if not os.path.isfile(device_header):
        fail("device header not found for --device %s: %s\n(pass the matching device, e.g. %s)"
             % (device, device_header, default_device))
    for path, what in ((HERE, "can_driver2 module"),
                       (os.path.join(LIB_ROOT, "utils"), "utils/isr_lock.h"),
                       (os.path.join(LIB_ROOT, "libs", "utils", "ring_buffer"),
                        "libs/utils/ring_buffer")):
        if not os.path.isdir(path):
            fail("cannot locate %s at %s" % (what, path))
    try:
        subprocess.run([args.cxx, "--version"], stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL, check=True)
    except (OSError, subprocess.CalledProcessError):
        fail("host C++ compiler %r not usable; pass one with --cxx" % args.cxx)

    tmp = tempfile.mkdtemp(prefix="can_driver2_filter_%s_" % args.family)
    try:
        platform, sim, objdir = (os.path.join(tmp, part) for part in ("platform", "sim", "obj"))
        for directory in (platform, sim, objdir):
            os.makedirs(directory)

        def emit(path, text):
            with open(path, "w", encoding="utf-8") as handle:
                handle.write(text)

        emit(os.path.join(platform, "host_cmsis.h"),
             HOST_CMSIS.replace("__CORTEX_M_VALUE__", cortex))
        emit(os.path.join(platform, core), CORE_SHADOW)
        emit(os.path.join(platform, "cmsis_compiler.h"), CORE_SHADOW)
        emit(os.path.join(platform, "main.h"),
             MAIN_SHIM.replace("__HAL_DEF__", hal_def).replace("__HAL_MODULE__", hal_module))
        emit(os.path.join(sim, "host_sim.h"), HOST_SIM_H)
        emit(os.path.join(sim, "hal_sim_host.cpp"), HAL_SIM_CPP)
        emit(os.path.join(tmp, "filter_test.cpp"), FIXTURE)

        defines = ["-D" + device, "-D" + hal, "-D" + callbacks,
                   "-DCAN_DRIVER_TX_QUEUE_SIZE=%d" % args.tx_queue_size,
                   "-DCAN_DRIVER_MAX_CALLBACKS=14", "-DCAN_DRIVER_MAX_INSTANCES=3",
                   "-DCAN_DRIVER_VALIDATE_TX_FRAME=%d" % args.validate_tx_frame]
        includes = ["-I" + platform, "-I" + sim, "-I" + HERE,
                    "-I" + os.path.join(LIB_ROOT, "utils"),
                    "-I" + os.path.join(LIB_ROOT, "libs", "utils", "ring_buffer"),
                    "-I" + hal_inc, "-I" + dev_inc]
        flags = ["-std=c++20", "-O1", "-g", "-Wall", "-fno-rtti", "-fno-exceptions", "-fno-pie"]
        if args.sanitize:
            flags += ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"]
        link_flags = ["-fsanitize=address,undefined"] if args.sanitize else []

        units = [(os.path.join(HERE, "can_driver.cpp"), "can_driver.o"),
                 (os.path.join(HERE, "can_driver_bxcan.cpp"), "can_driver_bxcan.o"),
                 (os.path.join(HERE, "can_driver_fdcan.cpp"), "can_driver_fdcan.o"),
                 (os.path.join(sim, "hal_sim_host.cpp"), "hal_sim_host.o"),
                 (os.path.join(tmp, "filter_test.cpp"), "filter_test.o")]

        sys.stdout.write("building %s filter regression (device %s, queue %d, validate %d%s)\n"
                         % (args.family, device, args.tx_queue_size, args.validate_tx_frame,
                            ", sanitized" if args.sanitize else ""))
        for source, obj in units:
            command = [args.cxx] + flags + defines + includes + \
                      ["-c", source, "-o", os.path.join(objdir, obj)]
            result = subprocess.run(command, cwd=tmp, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, text=True)
            if result.returncode != 0:
                sys.stdout.write(result.stdout)
                fail("compiling %s failed\ncommand: %s"
                     % (os.path.basename(source), " ".join(command)))

        executable = os.path.join(tmp, "filter_test")
        link_command = [args.cxx] + link_flags + ["-no-pie"] + \
                       [os.path.join(objdir, name) for _, name in units] + ["-o", executable]
        result = subprocess.run(link_command, cwd=tmp, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True)
        if result.returncode != 0:
            sys.stdout.write(result.stdout)
            fail("linking failed\ncommand: %s" % " ".join(link_command))

        return subprocess.run([executable], cwd=tmp, text=True).returncode
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())

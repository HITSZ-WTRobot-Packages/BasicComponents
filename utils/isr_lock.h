/**
 * @file    isr_lock.h
 * @author  syhanjin
 * @date    2026-02-24
 */
#pragma once
extern "C"
{
#include "cmsis_compiler.h"
static uint32_t isr_lock()
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}
static void isr_unlock(uint32_t primask)
{
    __DSB();
    __ISB();
    __set_PRIMASK(primask);
}
}
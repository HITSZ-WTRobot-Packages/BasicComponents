/**
 * @file    isr_lock.h
 * @author  syhanjin
 * @date    2026-02-24
 * @brief   CMSIS 中断临界区封装。
 *
 * 适用于需要在裸机或 RTOS 环境里短暂屏蔽中断的场景。
 * 这个封装保留并恢复原始 PRIMASK，避免嵌套临界区把外层状态弄乱。
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

#ifdef __cplusplus
class ISRGuard
{
public:
    /**
     * @brief 构造时关中断，析构时恢复。
     *
     * 适合只需要局部保护的临界区，退出作用域即可自动恢复。
     */
    ISRGuard() { primask = isr_lock(); }
    ~ISRGuard() { isr_unlock(primask); }

    ISRGuard(const ISRGuard&)            = delete;
    ISRGuard& operator=(const ISRGuard&) = delete;
    ISRGuard(ISRGuard&&)                 = delete;
    ISRGuard& operator=(ISRGuard&&)      = delete;

private:
    uint32_t primask;
};
#endif

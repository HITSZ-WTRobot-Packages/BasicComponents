/**
 * @file    utils.h
 * @author  syhanjin
 * @date    2026-02-01
 */
#ifndef UTILS_H
#define UTILS_H

#include "main.h"

#ifdef __cplusplus
extern "C"
{
#endif

static void delay_us(const uint32_t us)
{
    uint32_t cycles = SystemCoreClock / 4000000 * us;

    __asm volatile("1: \n"                     // 标签 1
                   "   subs %[n], %[n], #1 \n" // n--
                   "   bne 1b \n"              // 如果 n != 0，跳回标签 1
                   : [n] "+r"(cycles)          // 输出操作数（读写）
                   :                           // 无输入
                   : "cc"                      // 声明修改了条件码寄存器
    );
}

#define static_new(expr)                                                                           \
    (                                                                                              \
            []()                                                                                   \
            {                                                                                      \
                static auto inst = (expr);                                                         \
                return &inst;                                                                      \
            }())

#define static_new_with_vars(vars, expr)                                                           \
    (                                                                                              \
            [vars]()                                                                               \
            {                                                                                      \
                static auto inst = (expr);                                                         \
                return &inst;                                                                      \
            }())

#ifdef __cplusplus
}
#endif

#endif // UTILS_H

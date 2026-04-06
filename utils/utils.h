/**
 * @file    utils.h
 * @author  syhanjin
 * @date    2026-02-01
 * @brief   通用嵌入式小工具。
 *
 * 这个文件里的函数都很小，目的主要是给上层代码补一些 HAL 没有直接提供的基础能力。
 * 当前最常见的是微秒级忙等延时，适合初始化时序或者很短的同步等待，不适合拿来做长期阻塞。
 */
#ifndef UTILS_H
#define UTILS_H

#include "main.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief 微秒级忙等延时。
 *
 * 这个实现依赖 SystemCoreClock 估算循环次数，因此只适合对精度要求不高、但需要很短延时的场景。
 */
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

#ifdef __cplusplus
}
#endif

#endif // UTILS_H

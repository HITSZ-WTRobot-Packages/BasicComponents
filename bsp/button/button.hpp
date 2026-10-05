/**
 * @file button.hpp
 * @brief 基于分装好的gpio的STM32按键二次封装：输入引脚、EXTI 通知、软件消抖及状态变化回调。
 * @author  Wleaf
 * @date    2026-10-5
 *
 * 业务只需在构造时传入一个接收 bool 的函数，并在任务中周期调用 update()。
 * 引脚读取、中断通知和消抖状态机均由 Button 内部处理。
 * 本文件声明接口及成员变量，具体函数实现在 button.cpp 中。
 */

#pragma once

#include "gpio_driver.hpp"
#include <cstdint>

namespace bsp
{
    /**
     * @brief 一个 Button 对应一个 GPIO 输入引脚，继承 GpioPinInput 封装。
     *
     * 工作流程：
     * 1. 构造时注册该引脚的 EXTI 回调。
     * 2. 中断只增加通知计数，不读取电平、不消抖，也不执行业务回调。
     * 3. update() 在任务中读取电平，用 HAL_GetTick() 记录候选状态持续的时间。
     * 4. 采样得到的候选状态保持 stable_ms 后，才确认按下或松开并通知业务。
     *
     * 回调参数 true 表示确认按下，false 表示确认松开。只有确认状态发生变化才回调，
     * 长按不会重复触发；首次读到松开也不会产生松开回调。上电时已按住的按键同样
     * 需要经过消抖确认，不会在第一次采样时立即触发。
     *
     * 使用约定：
     * - GPIO 时钟、输入模式、上下拉及 EXTI/NVIC 仍由 CubeMX 初始化。
     * - 应用层在 app.cpp 中定义统一的 HAL_GPIO_EXTI_Callback，并调用
     *   bsp::gpio::DispatchExtiInterrupt()；按键与其他已注册 GPIO 共用此分发入口。
     * - 至少配置按下对应的中断边沿：高电平有效用上升沿，低电平有效用下降沿。
     *   按住后会持续采样，因此松开检测不依赖松开边沿中断，也支持双边沿配置。
     * - 同一个对象只能由一个任务调用 update() 和 pressed()；回调也在该任务中执行。
     * - 定期调用 update()，例如每 10ms 调用一次，默认消抖时间为 30ms。
     *   实际确认时间受采样周期和任务调度影响，两个采样点之间的短暂变化可能被漏采。
     * - 每条 EXTI 线只能绑定一个对象；相同引脚编号即使位于不同端口也共用 EXTI 线。
     * - 注册后不要修改继承得到的 port、pin 和 user_data，避免破坏引脚及对象绑定。
     *
     * @code
     * void on_button_changed(bool pressed)
     * {
     *     if (pressed)
     *     {
     *         // 处理确认后的按下动作。
     *     }
     *     else
     *     {
     *         // 处理确认后的松开动作。
     *     }
     * }
     *
     * // 在 GPIO 初始化完成后创建，且对象必须存活到任务不再使用它为止。
     * bsp::Button button{ GPIOE, GPIO_PIN_13, on_button_changed };
     * // 在任务循环或者定时器中每 10ms 执行：button.update();
     * @endcode
     */
    class Button final : public bsp::gpio::GpioPinInput
    {
    public:
        /// 业务回调签名：true 为确认按下，false 为确认松开；由 update() 同步调用。
        using Callback = void (*)(bool pressed);

        /**
         * @brief 绑定输入引脚、业务回调和消抖参数，并自动注册 EXTI 回调。
         * @param port GPIO 端口，例如 GPIOE；必须是已经初始化的有效端口。
         * @param pin 单个引脚的位掩码，例如 GPIO_PIN_13；不能组合多个引脚。
         * @param callback 状态变化回调；可传 nullptr，仅通过 pressed() 查询确认状态。
         * @param stable_ms 候选状态需要保持的毫秒数，默认 30ms，按下与松开共用此值。
         * @param active_state 按下时的电平，默认 GPIO_PIN_SET，即高电平有效。
         *
         * 基类构造参数中的 this 保存到 user_data，中断回调据此找到当前 Button。
         * GPIO 的硬件初始化不在此处执行；构造完成后仍需周期调用 update() 才能消抖。
         */
        Button(GPIO_TypeDef *port,
               uint16_t pin,
               Callback callback,
               uint32_t stable_ms = 30,
               GPIO_PinState active_state = GPIO_PIN_SET);

        /**
         * @brief 解除软件中断回调绑定，避免分发器继续保存已销毁对象的地址。
         *
         * 销毁前应保证任务不再访问此对象。注销不修改 GPIO 模式，也不关闭 NVIC 中断。
         */
        ~Button();

        // EXTI 分发表和 user_data 都保存当前对象地址，禁止复制、移动以保持绑定有效。
        Button(const Button &) = delete;
        Button &operator=(const Button &) = delete;
        Button(Button &&) = delete;
        Button &operator=(Button &&) = delete;

        /**
         * @brief 在采样任务中推进消抖状态，并同步执行需要触发的业务回调。
         *
         * 此函数只执行一次采样，不等待消抖时间，也不创建任务或定时器。
         * 业务回调应及时返回，以免影响后续采样和同一任务的其他工作。
         * 空闲松开时暂停采样，新的 EXTI 通知重新启动采样；按住时持续采样以检测松开。
         */
        void update();

        /**
         * @brief 查询最近一次消抖确认的状态，true 为按下，false 为松开。
         *
         * 此函数不读取 GPIO，也不推进消抖；仅允许在调用 update() 的同一个任务中使用。
         */
        bool pressed() const;

    private:
        /**
         * @brief GPIO 分发器调用的静态中断入口，只记录有新的采样通知。
         * @param gpio 注册到分发器的输入引脚句柄，user_data 指向所属 Button。
         *
         * 第二个参数是 GPIO 驱动的中断计数，本类不使用；这里维护独立的通知计数。
         * 运行期间只有中断写 notification_，任务只读且从不清零。若中断恰好出现在
         * update() 读取快照之后，新的计数会在下一次 update() 被发现，不会被任务覆盖。
         */
        static void on_exti(const bsp::gpio::GpioPinInput *gpio, uint32_t);

        const Callback callback_;          ///< 状态变化时调用的业务函数，可为空。
        const uint32_t stable_ms_;         ///< 按下和松开共用的消抖时间，单位 ms。
        const GPIO_PinState active_state_; ///< 表示按下的原始 GPIO 电平。

        // 唯一在中断与采样任务之间共享的状态：中断写入，任务读取。
        // alignas(4) 保证 32 位对齐，依赖 STM32 单核上对齐的 32 位访问不会读到半个值。
        // volatile 保留实际内存访问；这里不把它当作通用的多线程同步或互斥机制。
        alignas(4) volatile uint32_t notification_ = 0;
        // 以下成员只由采样任务访问，不需要与中断同步。
        // 已消费的通知快照，用于判断上次采样后是否又出现了中断。
        uint32_t observed_notification_ = 0;
        // 最近一次候选状态变化时的 HAL 毫秒时间戳。
        uint32_t changed_at_ = 0;
        bool sampling_ = true;   ///< 初始开启采样，保证上电时已按住也能经过消抖确认。
        bool candidate_ = false; ///< 最近采样得到的候选状态，尚未必满足消抖时间。
        bool stable_ = false;    ///< 已确认状态；初始按松开处理，只在确认变化后更新。
    };
}

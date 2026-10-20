/**
 * @file button.cpp
 * @brief Button 的具体实现：输入引脚绑定、中断通知及任务中的采样消抖。
 *
 * 函数实现集中在本文件，接口、参数说明和使用约定见 button.hpp。
 * HAL_GPIO_EXTI_Callback 由业务层统一定义，分发按键与其他 GPIO 的中断。
 */
#include "button.hpp"

namespace bsp
{

    Button::Button(GPIO_TypeDef *port,
                   uint16_t pin,
                   Callback callback,
                   uint32_t stable_ms,
                   GPIO_PinState active_state) : GpioPinInput(port, pin, this),
                                                 callback_(callback),
                                                 stable_ms_(stable_ms),
                                                 active_state_(active_state)
    {
        // 基类将 this 保存到 user_data；每个 Button 共用静态入口，由 user_data 区分实例。
        // 成员完成初始化后再注册，中断到来时即可访问有效的通知计数。
        bsp::gpio::RegisterExtiCallback(this, on_exti);
    }

    Button::~Button()
    {
        // 移除分发表中的对象地址；销毁前应保证任务不再访问此对象。
        // 这里只解除软件回调绑定，不改变 GPIO 模式或 NVIC 配置。
        bsp::gpio::UnregisterExtiCallback(this);
    }

    void Button::update()
    {
        // 先读取通知计数快照。与上次不同，表示期间发生过至少一次 EXTI 中断。
        // 通知只负责启动采样，不直接代表按下，也不会直接改变已确认状态。
        const uint32_t notification = notification_;
        if (notification != observed_notification_)
        {
            observed_notification_ = notification;
            sampling_ = true;
        }
        // 已确认松开且没有新通知时，直接返回，不读取引脚或时钟。
        if (!sampling_)
            return;

        // 将原始电平统一转换成“是否按下”，业务不必关心高、低电平有效的差别。
        const bool pressed = read() == active_state_;
        const uint32_t now_ms = HAL_GetTick();

        // 本次采样与候选状态不同，说明出现新的变化；从现在开始重新计时。
        // 此时只更新候选值，已确认状态 stable_ 保持不变，也不触发业务回调。
        if (pressed != candidate_)
        {
            candidate_ = pressed;
            changed_at_ = now_ms;
            return;
        }
        // 候选状态与已确认状态不同，且采样中已维持足够时间，才接受这次变化。
        // 使用 uint32_t 无符号时间差，允许 HAL 毫秒计数跨越一次回绕后继续计时。
        if (candidate_ != stable_ && uint32_t(now_ms - changed_at_) >= stable_ms_)
        {
            stable_ = candidate_;
            // 松开确认后进入空闲；按下确认后继续采样，以捕获没有中断通知的松开。
            if (!stable_)
                sampling_ = false;
            // 回调发生在调用 update() 的任务中，参数是更新后的确认状态。
            if (callback_ != nullptr)
                callback_(stable_);
            return;
        }
        // 原本就是松开且候选也为松开时，没有状态变化需要确认，可以停止采样。
        // 例如一次中断触发后，任务实际读到的电平仍为松开，此时无需产生回调。
        if (!stable_ && !candidate_)
            sampling_ = false;
    }

    bool Button::pressed() const
    {
        // 查询已确认状态，不额外读取 GPIO 或推进消抖。
        return stable_;
    }

    void Button::on_exti(const bsp::gpio::GpioPinInput *gpio, uint32_t)
    {
        // 恢复构造时保存的对象指针；中断中不调用业务，也不读取引脚或执行消抖。
        // 中断只递增计数，任务只读且不清零，采样期间的新通知会留到后续 update() 处理。
        auto *button = static_cast<Button *>(gpio->user_data);
        button->notification_ = button->notification_ + 1;
    }

}

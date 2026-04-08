/**
 * @file    UartRxSync.hpp
 * @author  syhanjin
 * @date    2026-02-01
 * @brief   带帧头同步的 UART 接收器。
 *
 * 通过“找帧头 + DMA 收剩余帧”来降低丢包风险，适合固定帧格式传感器。
 */
#ifndef UARTRXSYNC_HPP
#define UARTRXSYNC_HPP

#include "main.h"
#include "watchdog.hpp"

#include <array>
#include <cstddef>
#include <cstring>

#ifndef HAL_UART_MODULE_ENABLED
#    error "HAL UART module is not enabled. Enable HAL_UART_MODULE_ENABLED in stm32xxxx_hal_conf.h"
#endif

namespace protocol
{

/**
 * @deprecated 旧式宏包装，建议直接使用类实例的回调注册方式。
 *
 * 这个宏保留主要是为了兼容旧代码，新的代码请直接调用 RegisterCallback 对应的能力。
 */
#define UartRxSync_DefineCallback(__obj__)

#define UartRxSync_RegisterCallback(__obj__, __huart__)                                            \
    HAL_UART_RegisterCallback((__huart__),                                                         \
                              HAL_UART_RX_COMPLETE_CB_ID,                                          \
                              [](UART_HandleTypeDef* huart) { (__obj__)->receiveCallback(); });    \
    HAL_UART_RegisterCallback((__huart__),                                                         \
                              HAL_UART_ERROR_CB_ID,                                                \
                              [](UART_HandleTypeDef* huart) { (__obj__)->errorHandler(); })

/**
 * @brief 带帧头同步功能的串口接收器，基于中断和 DMA。
 *
 * 典型使用场景是固定帧格式传感器：先用中断逐字节找帧头，再交给 DMA 接收整帧剩余部分。
 * 这样既能保持同步，也能减少每个字节都进中断带来的开销。
 */
template <size_t HeaderLen, size_t FrameLen, bool DecodeWithHeader = false> class UartRxSync
{
    static_assert(HeaderLen > 0);
    static_assert(FrameLen > HeaderLen);

public:
    explicit UartRxSync(UART_HandleTypeDef* huart) : huart_(huart) {}
    virtual ~UartRxSync() = default;
    enum class SyncState
    {
        // 未启动或已停止。
        Stopped,
        // 使用 IT 模式逐字节搜索帧头。
        WaitHead,
        // 已匹配帧头，正在用 DMA 收剩余帧。
        Receiving,
        // 正常工作态，DMA 持续接收下一帧。
        DMAActive,
    };

    bool startReceive()
    {
        // 检查 UART 和 DMA 配置是否符合 DMA 循环接收要求。
        if (huart_ == nullptr || huart_->hdmarx == nullptr ||
            huart_->hdmarx->Init.Mode != DMA_CIRCULAR)
            return false;
        state_ = SyncState::WaitHead;
        return HAL_UART_Receive_IT(huart_, rx_buffer_, 1) == HAL_OK;
    }

    void receiveCallback()
    {
        if (state_ == SyncState::DMAActive)
        {
#ifdef DEBUG
            ++data_received_cnt;
#endif
            if (!check_header())
            {
                // 帧头错位，说明流中断或解析失败，重新回到找帧头状态。
#ifdef DEBUG
                ++hdr_error_cnt;
#endif
                HAL_UART_AbortReceive(huart_);
                state_ = SyncState::WaitHead;
                HAL_UART_Receive_IT(huart_, rx_buffer_, 1);
                hdr_idx_ = 0;
                return;
            }
            _decode();
        }
        else if (state_ == SyncState::WaitHead)
        {
            // 逐字节滑动窗口匹配帧头。
            size_t idx_next = hdr_idx_ + 1;
            if (idx_next == HeaderLen)
                idx_next = 0;
            if (rx_buffer_[hdr_idx_] == header()[HeaderLen - 1])
            {
                if (check_header())
                {
#ifdef DEBUG
                    ++hdr_match_cnt;
#endif
                    // 帧头匹配成功后，用 DMA 接收剩余帧数据。
                    HAL_UART_Receive_DMA(huart_, rx_buffer_ + HeaderLen, FrameLen - HeaderLen);
                    state_ = SyncState::Receiving;
                    return;
                }
            }
            // 继续接收下一字节作为滑动窗口的新尾部。
            HAL_UART_Receive_IT(huart_, rx_buffer_ + idx_next, 1);
            hdr_idx_ = idx_next;
        }
        else if (state_ == SyncState::Receiving)
        {
#ifdef DEBUG
            ++data_received_cnt;
#endif
            HAL_UART_AbortReceive(huart_);
            // 这里先重新开启下一帧接收，再在后台解码当前帧，以缩短中断占用时间。
            HAL_UART_Receive_DMA(huart_, rx_buffer_, FrameLen);
            state_ = SyncState::DMAActive;
            _decode();
        }
    }

    void errorHandler()
    {
        if (huart_->ErrorCode == HAL_UART_ERROR_NONE)
        {
            // 不是实际 UART 错误，直接返回。
            return;
        }
#ifdef DEBUG
        ++rx_error_event_cnt;
#endif

        // 清除错误标志，避免错误状态反复触发。
        __HAL_UART_CLEAR_PEFLAG(huart_);
        __HAL_UART_CLEAR_FEFLAG(huart_);
        __HAL_UART_CLEAR_NEFLAG(huart_);
        __HAL_UART_CLEAR_OREFLAG(huart_);

        // 重启接收并回到找帧头状态。
        HAL_UART_AbortReceive(huart_);
        if (state_ != SyncState::WaitHead)
        {
            state_ = SyncState::WaitHead;
        }
        HAL_UART_Receive_IT(huart_, rx_buffer_, 1);
        hdr_idx_ = 0;
    }

    [[nodiscard]] bool isConnected() const
    {
        // 只有状态正常且 watchdog 还在续命时，才认为链路在线。
        return state_ == SyncState::DMAActive && watchdog_.isFed();
    }

    [[nodiscard]] UART_HandleTypeDef* huart() const { return huart_; }

protected:
    virtual const std::array<uint8_t, HeaderLen>& header() const                                = 0;
    virtual bool decode(const uint8_t data[DecodeWithHeader ? FrameLen : FrameLen - HeaderLen]) = 0;

    virtual uint32_t timeout() const { return 10; }

private:
    UART_HandleTypeDef* huart_;

    SyncState state_{ SyncState::Stopped };

    service::Watchdog watchdog_{};

    uint8_t rx_buffer_[FrameLen]{};
    size_t  hdr_idx_{ 0 };

private:
    bool check_header()
    {
        auto& hdr = header();

        // 先检查尾部，再回绕到 buffer 前部，完成滑动窗口比对。
        const size_t first_len = HeaderLen - hdr_idx_ - 1; // tail + 1 到 buffer 末尾
        for (size_t i = 0; i < first_len; ++i)
            if (rx_buffer_[hdr_idx_ + i + 1] != hdr[i])
                return false;

        for (size_t i = 0; i <= hdr_idx_; ++i)
            if (rx_buffer_[i] != hdr[first_len + i])
                return false;

        return true;
    }

    void _decode()
    {
        const uint8_t* data;

        if constexpr (DecodeWithHeader)
        {
            // 如果 decode 需要头部，就把 header 覆盖到缓冲区开头。
            memcpy(rx_buffer_, header().data(), HeaderLen);
            data = &rx_buffer_[0];
        }
        else
        {
            // 默认只把 payload 传给 decode。
            data = &rx_buffer_[HeaderLen];
        }

        if (decode(data))
        {
            // 解码成功后刷新 watchdog。
            watchdog_.feed(timeout());
#ifdef DEBUG
            ++decode_success_cnt;
#endif
        }
        else
        {
            // 此处无须处理，由用户自行丢弃该帧即可
#ifdef DEBUG
            ++decode_fail_cnt;
#endif
        }
    }

#ifdef DEBUG
private:
    uint32_t hdr_match_cnt{ 0 };
    uint32_t hdr_error_cnt{ 0 };
    uint32_t data_received_cnt{ 0 };
    uint32_t decode_success_cnt{ 0 };
    uint32_t decode_fail_cnt{ 0 };
    uint32_t rx_error_event_cnt{ 0 };
#endif
};

} // namespace protocol

#endif // UARTRXSYNC_HPP

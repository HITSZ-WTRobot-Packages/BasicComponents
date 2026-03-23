/**
 * @file    UartRxSync.hpp
 * @author  syhanjin
 * @date    2026-02-01
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
 * @deprecated 无须使用此宏函数，直接调用 RegisterCallback 即可
 * @param __obj__ uart rx sync pointer
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
 * 带帧头同步功能的串口接收器，基于中断和 DMA
 * @tparam HeaderLen 帧头长度
 * @tparam FrameLen 帧长度
 * @tparam DecodeWithHeader decode 时是否带上帧头
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
        Stopped,
        WaitHead,
        Receiving,
        DMAActive,
    };

    bool startReceive()
    {
        // check configurations
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
                // 帧头错误，重新匹配
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
            // 匹配到帧头最后一位就往前进行一次 check
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
                    // 使用 DMA 接收完剩下的内容
                    HAL_UART_Receive_DMA(huart_, rx_buffer_ + HeaderLen, FrameLen - HeaderLen);
                    state_ = SyncState::Receiving;
                    return;
                }
            }
            // 继续接收下一位
            HAL_UART_Receive_IT(huart_, rx_buffer_ + idx_next, 1);
            hdr_idx_ = idx_next;
        }
        else if (state_ == SyncState::Receiving)
        {
#ifdef DEBUG
            ++data_received_cnt;
#endif
            HAL_UART_AbortReceive(huart_);
            /**
             * 由于 decode 抛弃了 head，所以这里可以先开始接收
             * 只要保证 decode 时间 < 1 / bitrate * 10 * header_len 即可
             * 对于 115200 bitrate，1 byte 需要约 86us
             * 对于 2M bitrate, 1 byte 需要约 5us
             * 24 字节的查表法 CRC8 需要约 1.4us，逐位计算需要约 2.3us
             *
             * 其实建议先解算再继续接收( 但是我是犟种
             */
            HAL_UART_Receive_DMA(huart_, rx_buffer_, FrameLen);
            state_ = SyncState::DMAActive;
            _decode();
        }
    }

    void errorHandler()
    {
        if (huart_->ErrorCode == HAL_UART_ERROR_NONE)
        {
            // not a real uart error
            return;
        }
#ifdef DEBUG
        ++rx_error_event_cnt;
#endif

        // clear error flags
        __HAL_UART_CLEAR_PEFLAG(huart_);
        __HAL_UART_CLEAR_FEFLAG(huart_);
        __HAL_UART_CLEAR_NEFLAG(huart_);
        __HAL_UART_CLEAR_OREFLAG(huart_);

        // restart receive
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
            // 整理环形缓冲区里的 header
            memcpy(rx_buffer_, header().data(), HeaderLen);
            data = &rx_buffer_[0];
        }
        else
        {
            data = &rx_buffer_[HeaderLen];
        }

        if (decode(data))
        {
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

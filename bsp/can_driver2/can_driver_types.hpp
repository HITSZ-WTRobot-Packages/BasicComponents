/**
 * @file can_driver_types.hpp
 * @brief HAL-independent CAN frame and filter descriptions.
 *
 * FrameView borrows its payload. send() consumes it before returning or copies
 * it into the software queue. Receive callbacks must not retain the view.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <variant>

namespace bsp::can
{

enum class IdType : std::uint8_t { Standard, Extended };
enum class FrameFormat : std::uint8_t { Classic, Fd };
enum class FrameType : std::uint8_t { Data, Remote };

struct FrameHeader
{
    std::uint32_t id{};
    IdType id_type{IdType::Standard};
    FrameFormat format{FrameFormat::Classic};
    FrameType type{FrameType::Data};
    bool bit_rate_switch{};
    bool error_state_indicator{}; ///< FD ESI: true denotes error passive.
    std::uint8_t remote_length{}; ///< Requested bytes, only for Classic RTR.
};

struct FrameView
{
    FrameHeader header{};
    std::span<const std::uint8_t> data{}; ///< Empty for remote frames.
};

enum class Status : std::uint8_t
{
    Ok,
    Empty,
    Busy,
    InvalidArgument,
    InvalidFrame, ///< A received frame was consumed but cannot be delivered intact.
    InvalidState,
    InvalidConfiguration,
    Unsupported,
    OutOfRange,
    NoCapacity,
    HandleInUse,
    BusOff,
    HardwareError,
};

enum class SendResult : std::uint8_t
{
    Submitted,      ///< Copied into hardware, not necessarily transmitted.
    Queued,         ///< Copied into the software queue.
    ReplacedOldest, ///< Queued after discarding the oldest software entry.
    Busy,           ///< Hardware full and software queue disabled.
    NotStarted,
    InvalidFrame,
    Unsupported,
    BusOff,
    HardwareError,
};

[[nodiscard]] constexpr bool is_fd_length(const std::size_t length) noexcept
{
    return length <= 8 || length == 12 || length == 16 || length == 20 ||
           length == 24 || length == 32 || length == 48 || length == 64;
}

[[nodiscard]] constexpr bool is_valid_id(const IdType type, const std::uint32_t id) noexcept
{
    return (type == IdType::Standard && id <= 0x7ffU) ||
           (type == IdType::Extended && id <= 0x1fffffffU);
}

enum class FilterMode : std::uint8_t { Mask, List, Range, RangeWithoutExtendedMask };
enum class FilterAction : std::uint8_t
{
    Disable,
    Fifo0,
    Fifo1,
    Reject,
    Priority,
    PriorityFifo0,
    PriorityFifo1,
    RxBuffer,
};

/**
 * ID-based filter. index is a bxCAN bank or an FDCAN element index within
 * the selected standard/extended filter array; these are not interchangeable
 * hardware layouts. Filtering is by ID, not by data/remote frame type.
 *
 * Mask: id1 = identifier, id2 = identifier mask (one means compare).
 * List: id1 and id2 are identifiers; repeat an ID for a one-ID list.
 * Range: id1/id2 are inclusive bounds. Ranges are FDCAN-only.
 * RxBuffer: FDCAN exact match on id1; id2 must be zero, mode must be Mask.
 *
 * bxCAN maps Mask to a 32-bit mask bank and a standard-ID List to two
 * 16-bit mask entries, preserving acceptance of both data and remote frames.
 * Two different extended IDs cannot have those semantics in one bxCAN bank:
 * that request returns Unsupported. Use separate banks, or BankFilter when
 * exact control of IDE/RTR bits and native 16/32-bit packing is required.
 */
struct IdFilter
{
    std::uint32_t index{};
    IdType id_type{IdType::Standard};
    FilterMode mode{FilterMode::Mask};
    FilterAction action{FilterAction::Fifo0};
    std::uint32_t id1{};
    std::uint32_t id2{};
    std::uint32_t rx_buffer_index{};
    bool calibration_message{}; ///< FDCAN standard-ID RxBuffer only.
};

enum class FilterScale : std::uint8_t { Bits16, Bits32 };
enum class BankFilterMode : std::uint8_t { Mask, List };
enum class RxFifo : std::uint8_t { Fifo0, Fifo1 };

/**
 * Complete bxCAN bank layout, without HAL types/macros; FDCAN returns
 * Unsupported. The four words are the hardware filter halfwords, so every
 * IDE/RTR combination and partial extended-ID match remains expressible.
 *
 * Bits32/Mask: id_high:id_low and mask_high:mask_low.
 * Bits32/List: two identifiers in those same word pairs.
 * Bits16/Mask: (id_low, mask_low), (id_high, mask_high).
 * Bits16/List: four identifiers in id_low, mask_low, id_high, mask_high.
 * A 32-bit key is StdId<<21 or (ExtId<<3)|IDE; RTR is bit 1, IDE bit 2.
 * A 16-bit key is StdId<<5, or ((ExtId>>18)<<5)|IDE|((ExtId>>15)&7);
 * RTR is bit 4 and IDE bit 3. Extended low 15 bits are not compared in 16-bit mode.
 */
struct BankFilter
{
    std::uint32_t index{};
    FilterScale scale{FilterScale::Bits32};
    BankFilterMode mode{BankFilterMode::Mask};
    RxFifo fifo{RxFifo::Fifo0};
    bool enabled{true};
    std::uint16_t id_high{};
    std::uint16_t id_low{};
    std::uint16_t mask_high{};
    std::uint16_t mask_low{};
};

/** Shared CAN1/CAN2 bank boundary; configure before starting either bus. */
struct FilterBankSplit
{
    std::uint32_t first_can2_bank{14};
};

enum class NonMatchingAction : std::uint8_t { Reject, Fifo0, Fifo1 };

/** FDCAN global acceptance rules; unsupported on bxCAN. */
struct GlobalFilter
{
    NonMatchingAction standard{NonMatchingAction::Reject};
    NonMatchingAction extended{NonMatchingAction::Reject};
    bool reject_standard_remote{};
    bool reject_extended_remote{};
};

/** FDCAN XIDAM; does not replace the mask of an individual filter. */
struct ExtendedIdMask
{
    std::uint32_t mask{0x1fffffffU};
};

/// A single configuration entry point; unsupported alternatives return Unsupported.
using FilterConfig = std::variant<IdFilter, BankFilter, FilterBankSplit,
                                  GlobalFilter, ExtendedIdMask>;

} // namespace bsp::can

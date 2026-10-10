/**
 * @file can_driver_types.hpp
 * @brief 与 HAL 无关的 CAN 帧描述与过滤器配置类型。
 *
 * FrameView 的负载是借用（borrow），不持有所有权。send() 会在返回前消费该
 * 视图或把它复制进软件队列，因此调用方随后即可复用底层缓冲区。接收回调在
 * ISR 上下文同步触发，回调返回后视图立即失效，回调内部必须完成处理，不得
 * 保存视图或其 data 指针供后续使用。
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <variant>

namespace bsp::can
{

/// 标识符类型：Standard 为 11 位标准 ID，Extended 为 29 位扩展 ID；决定 id 的
/// 有效范围与硬件 IDE 位。
enum class IdType : std::uint8_t { Standard, Extended };
/// 帧格式：Classic 为经典 CAN 2.0，Fd 为 CAN FD；仅支持 FD 的后端可发送 Fd。
enum class FrameFormat : std::uint8_t { Classic, Fd };
/// 帧类型：Data 携带数据负载，Remote 为 RTR 远程请求帧（不携带负载）。
enum class FrameType : std::uint8_t { Data, Remote };

/**
 * 帧元数据，不含负载。字段为跨后端的中立描述，与 HAL 枚举无关。
 *
 * 组合约束：Classic 帧的 bit_rate_switch 与 error_state_indicator 必须为
 * false；Remote 帧仅存在于 Classic 格式，remote_length 表示被请求的字节数
 * （0..8）且 data 必须为空；Fd 帧仅允许 Data 类型，data.size() 必须是
 * is_fd_length() 认可的 FD 长度。
 */
struct FrameHeader
{
    std::uint32_t id{};                       ///< 标识符，有效位宽由 id_type 决定。
    IdType id_type{IdType::Standard};         ///< 决定 id 的位宽与硬件 IDE 位。
    FrameFormat format{FrameFormat::Classic}; ///< 经典帧或 FD 帧。
    FrameType type{FrameType::Data};          ///< 数据帧或远程请求帧。
    bool bit_rate_switch{};                   ///< FD BRS：数据段切换到更高比特率。
    bool error_state_indicator{}; ///< FD ESI：true 表示节点处于 error passive。
    std::uint8_t remote_length{}; ///< 被请求的字节数，仅用于 Classic RTR。
};

/// 一帧的只读视图：header 加借用的数据负载 span。
struct FrameView
{
    FrameHeader header{};
    std::span<const std::uint8_t> data{}; ///< 远程帧为空；span 存活期由调用方/后端保证。
};

/**
 * 统一状态码；各 API 只返回与其相关的子集。
 */
enum class Status : std::uint8_t
{
    Ok,                   ///< 操作成功。
    Empty,                ///< 请求的位置当前无可用数据（如 RX FIFO/Buffer 为空）。
    Busy,                 ///< 硬件暂时无法受理（邮箱/FIFO/队列已满）。
    InvalidArgument,      ///< 参数非法：ID 越界、长度非法、字段组合不成立或指针为空。
    InvalidFrame, ///< 接收帧已被消费并确认，但内容无法完整交付（截断或非法的 FD 远程帧）。
    InvalidState,         ///< 当前状态不允许该操作（未 start、重复 start、非线程模式启动等）。
    InvalidConfiguration, ///< HAL 句柄或 Init 配置与操作不匹配。
    Unsupported,          ///< 该后端/硬件不支持所请求的功能或过滤器模式。
    OutOfRange,           ///< 资源 index 超出已配置或硬件的合法范围。
    NoCapacity,           ///< 所需资源数量为 0（无邮箱、无 FIFO element 或 filter element）。
    HandleInUse,          ///< 该外设已被另一个 Can 实例占用。
    BusOff,               ///< 总线处于 bus-off，发送被拒。
    HardwareError,        ///< HAL 或硬件返回未分类的失败。
};

/**
 * send() 的结果。Submitted/Queued/ReplacedOldest 只表示帧已被受理，均不保证
 * 真正发送成功，也不包含 CAN 仲裁顺序信息。
 */
enum class SendResult : std::uint8_t
{
    Submitted,      ///< 已写入硬件邮箱/FIFO，不保证已被发送。
    Queued,         ///< 已复制进软件队列，待硬件空位后重放。
    ReplacedOldest, ///< 软件队列已满，丢弃最旧一项后入队。
    Busy,           ///< 硬件满且软件队列被禁用（CAN_DRIVER_TX_QUEUE_SIZE == 0）。
    NotStarted,     ///< 尚未 start()。
    InvalidFrame,   ///< 帧未通过校验。
    Unsupported,    ///< 当前后端/配置不支持该帧。
    BusOff,         ///< 总线处于 bus-off。
    HardwareError,  ///< 硬件提交失败。
};

/// DLC 到字节数映射中允许的 FD 长度；经典长度 0..8 也在此集合内。
[[nodiscard]] constexpr bool is_fd_length(const std::size_t length) noexcept
{
    return length <= 8 || length == 12 || length == 16 || length == 20 ||
           length == 24 || length == 32 || length == 48 || length == 64;
}

/// 校验标识符是否落在该类型可表示的位宽内（Standard ≤ 0x7FF，Extended ≤ 0x1FFFFFFF）。
[[nodiscard]] constexpr bool is_valid_id(const IdType type, const std::uint32_t id) noexcept
{
    return (type == IdType::Standard && id <= 0x7ffU) ||
           (type == IdType::Extended && id <= 0x1fffffffU);
}

/// ID 匹配方式。Mask/List 两种后端都支持；Range 与 RangeWithoutExtendedMask
/// 为 FDCAN 专属，bxCAN 返回 Unsupported。
enum class FilterMode : std::uint8_t { Mask, List, Range, RangeWithoutExtendedMask };
/// 匹配后的处理动作。Disable/Fifo0/Fifo1 两种后端可用（Fifo0/1 要求对应 FIFO
/// 已分配 element，否则 NoCapacity）；Reject/Priority/PriorityFifo0/
/// PriorityFifo1/RxBuffer 仅 FDCAN 支持，bxCAN 返回 Unsupported。
enum class FilterAction : std::uint8_t
{
    Disable,       ///< 禁用该过滤器 element。
    Fifo0,         ///< 命中帧存入 Rx FIFO0。
    Fifo1,         ///< 命中帧存入 Rx FIFO1。
    Reject,        ///< 命中帧被丢弃（FDCAN）。
    Priority,      ///< 命中帧以高优先级存储（FDCAN）。
    PriorityFifo0, ///< 以高优先级存入 Rx FIFO0（FDCAN）。
    PriorityFifo1, ///< 以高优先级存入 Rx FIFO1（FDCAN）。
    RxBuffer,      ///< 精确匹配存入专用 Rx Buffer（FDCAN）。
};

/**
 * 基于 ID 的过滤器。index 含义随后端而定：bxCAN 为 filter bank 编号，FDCAN
 * 为所选标准/扩展过滤器数组内的 element 下标；两者硬件布局不同，索引不可
 * 互换。过滤只比较 ID，不区分数据帧与远程帧（RTR）类型。
 *
 * Mask：id1 为标识符，id2 为标识符掩码（掩码位为 1 表示比较该位）。
 * List：id1 与 id2 为两个标识符；只列一个 ID 时令 id1 == id2。
 * Range：id1/id2 为闭区间下界/上界，要求 id2 >= id1，仅 FDCAN 支持。
 * RangeWithoutExtendedMask：语义同 Range 但旁路 EIDM，仅 FDCAN 扩展 ID 列表可用。
 * RxBuffer：FDCAN 对 id1 精确匹配，要求 mode == Mask 且 id2 == 0，命中帧写入
 *   rx_buffer_index 指定的专用缓冲区。
 *
 * bxCAN：Mask 映射为一个 32 位掩码 bank，标准 ID 的 List 映射为同一 bank 内
 * 两个 16 位掩码项，且都保持同时接受该 ID 的数据帧与远程帧。一个 bxCAN bank
 * 无法表达两个不同扩展 ID 的精确 List 语义，此类请求返回 Unsupported；如需
 * 精确控制 IDE/RTR 位与原生 16/32 位打包，请使用不同 bank 或 BankFilter。
 *
 * 取值约束：id1/id2 不得超过对应 id_type 的位宽；Range 类模式下 id2 < id1
 * 返回 InvalidArgument；RxBuffer 且 calibration_message 为真时 id_type 必须
 * 为 Standard；RxBuffer 之外 rx_buffer_index 必须为 0（否则 InvalidArgument），
 * 且 calibration_message 必须为假（否则 Unsupported）。
 */
struct IdFilter
{
    std::uint32_t index{};                     ///< bxCAN bank 号或 FDCAN element 下标。
    IdType id_type{IdType::Standard};          ///< 选择标准/扩展过滤器数组。
    FilterMode mode{FilterMode::Mask};         ///< 匹配方式。
    FilterAction action{FilterAction::Fifo0};  ///< 命中后的处理动作。
    std::uint32_t id1{};                       ///< 标识符或区间下界。
    std::uint32_t id2{};                       ///< 掩码、第二标识符或区间上界。
    std::uint32_t rx_buffer_index{};           ///< FDCAN RxBuffer 目标缓冲区下标。
    bool calibration_message{}; ///< FDCAN 标准 ID RxBuffer 专用：标记校准报文。
};

/// bxCAN filter bank 位宽：16 位两项或 32 位一项。
enum class FilterScale : std::uint8_t { Bits16, Bits32 };
/// bxCAN bank 匹配方式：掩码或列表。
enum class BankFilterMode : std::uint8_t { Mask, List };
/// 命中帧写入的 Rx FIFO。
enum class RxFifo : std::uint8_t { Fifo0, Fifo1 };

/**
 * 完整的 bxCAN filter bank 布局，不含 HAL 类型/宏；FDCAN 返回 Unsupported。
 * 四个字即硬件 filter 半字，因此任何 IDE/RTR 组合与部分扩展 ID 匹配都能精确
 * 表达。index 为 bank 编号（越界返回 OutOfRange），enabled 为假时写为禁用 bank。
 *
 * Bits32/Mask：id_high:id_low 为键，mask_high:mask_low 为掩码。
 * Bits32/List：id_high:id_low 依次为两个标识符。
 * Bits16/Mask：(id_low, mask_low) 与 (id_high, mask_high) 为两组键/掩码。
 * Bits16/List：id_low、mask_low、id_high、mask_high 为四个标识符。
 * 32 位键为 StdId<<21 或 (ExtId<<3)|IDE；RTR 为 bit 1，IDE 为 bit 2。
 * 16 位键为 StdId<<5，或 ((ExtId>>18)<<5)|IDE|((ExtId>>15)&7)；
 * RTR 为 bit 4，IDE 为 bit 3；16 位模式不比较扩展 ID 的低 15 位。
 */
struct BankFilter
{
    std::uint32_t index{};               ///< bank 编号。
    FilterScale scale{FilterScale::Bits32};       ///< 半字位宽。
    BankFilterMode mode{BankFilterMode::Mask};    ///< 掩码或列表匹配。
    RxFifo fifo{RxFifo::Fifo0};          ///< 命中帧写入的 FIFO。
    bool enabled{true};                  ///< 是否使能该 bank。
    std::uint16_t id_high{};             ///< 高位半字（键/标识符）。
    std::uint16_t id_low{};              ///< 低位半字（键/标识符）。
    std::uint16_t mask_high{};           ///< 高位半字掩码或第二个标识符。
    std::uint16_t mask_low{};            ///< 低位半字掩码或第二个标识符。
};

/// FDCAN 全局过滤器对未命中帧的处理：拒收，或存入指定 FIFO（该 FIFO 未分配
/// element 时返回 NoCapacity）。
enum class NonMatchingAction : std::uint8_t { Reject, Fifo0, Fifo1 };

/**
 * FDCAN 全局接受规则；bxCAN 不支持（返回 Unsupported）。未显式配置时 FDCAN
 * 复位默认可能接受未匹配帧：需要拒收时请显式提交 GlobalFilter{}。
 */
struct GlobalFilter
{
    NonMatchingAction standard{NonMatchingAction::Reject}; ///< 未命中标准 ID 帧的处理。
    NonMatchingAction extended{NonMatchingAction::Reject}; ///< 未命中扩展 ID 帧的处理。
    bool reject_standard_remote{}; ///< 是否额外拒收标准 ID 远程帧。
    bool reject_extended_remote{}; ///< 是否额外拒收扩展 ID 远程帧。
};

/**
 * FDCAN XIDAM：扩展 ID 的全局掩码，按位作用于所有扩展帧；它不会替换某个具体
 * 过滤器的掩码，而是与其共同生效。默认 0x1FFFFFFF 表示比较全部 29 位；
 * mask 超过 0x1FFFFFFF 返回 InvalidArgument。
 */
struct ExtendedIdMask
{
    std::uint32_t mask{0x1fffffffU};
};

/// 过滤器配置统一入口；后端不支持的备选项返回 Unsupported。
using FilterConfig = std::variant<IdFilter, BankFilter, GlobalFilter, ExtendedIdMask>;

} // 命名空间 bsp::can

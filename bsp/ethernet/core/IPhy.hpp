/**
 * @file    IPhy.hpp
 * @brief   不依赖 HAL、协议栈或 RTOS 的以太网 PHY 接口。
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#pragma once

#include <cstdint>

namespace bsp::ethernet_phy
{

/** @brief PHY 操作结果；具体驱动负责转换底层状态码，不与其数值绑定。 */
enum class PhyResult : std::uint8_t
{
    Ok,               ///< 操作成功；输出参数此时才有效。
    NotInitialized,   ///< 操作因驱动未就绪而被拒绝；启动或写入失败的具体原因由 status() 返回。
    InvalidArgument,  ///< 参数非法，且未产生任何总线访问。
    Unsupported,      ///< 功能/模式在器件上或当前模式下不可用（如已强制模式时重启协商）。
    NotFound,         ///< 在给定地址或扫描范围内未找到匹配型号的器件。
    AmbiguousAddress, ///< 自动探测发现多个匹配器件，必须指定固定地址。
    ReadError,        ///< 寄存器读取失败，不代表正常断链。
    WriteError,       ///< 寄存器写入失败；硬件配置状态不确定，对象已失效。
    ResetTimeout,     ///< 器件复位未在规定时间内完成。
};

/**
 * @brief 成功读取的链路状态；访问错误通过 PhyResult 表达。
 *
 * Down 只表示当前没有链路，并不证明自动协商未在进行；Negotiating 表示已存在链路指示，
 * 但协商出的速率/双工尚无效。该枚举不是完整的自动协商状态机。
 */
enum class PhyLinkState : std::uint8_t
{
    Down,
    Negotiating,
    Up10Half,
    Up10Full,
    Up100Half,
    Up100Full,
};

/** @brief 10/100 Mbps 铜缆链路模式；位掩码可组合为“允许通告的模式集合”。 */
enum class PhyLinkMode : std::uint8_t
{
    None        = 0U,
    Base10Half  = 1U << 0,
    Base10Full  = 1U << 1,
    Base100Half = 1U << 2,
    Base100Full = 1U << 3,
    All10_100   = 0x0FU,
};

/** @brief 位或：组合模式集合。 */
constexpr PhyLinkMode operator|(const PhyLinkMode lhs, const PhyLinkMode rhs) noexcept
{
    return static_cast<PhyLinkMode>(static_cast<std::uint8_t>(lhs) |
                                    static_cast<std::uint8_t>(rhs));
}

/** @brief 位与：从集合中提取模式位。 */
constexpr PhyLinkMode operator&(const PhyLinkMode lhs, const PhyLinkMode rhs) noexcept
{
    return static_cast<PhyLinkMode>(static_cast<std::uint8_t>(lhs) &
                                    static_cast<std::uint8_t>(rhs));
}

/**
 * @brief 链路配置。
 *
 * 自动协商模式下 modes 为非空的“允许通告模式”掩码；强制模式下必须恰好只含一种模式。
 * 掩码为空、含未知位或强制模式含多位均为 InvalidArgument；器件不支持的组合为 Unsupported。
 */
struct PhyLinkConfig
{
    bool        autoNegotiation = true;               ///< true 自动协商；false 强制固定模式。
    PhyLinkMode modes           = PhyLinkMode::All10_100; ///< 见结构体说明。
};

/**
 * @brief 器件实际支持的链路能力（非与 MAC 能力的交集）。
 *
 * 调用方需自行把该能力限制到 MAC 支持的范围内使用。
 */
struct PhyCapabilities
{
    PhyLinkMode modes           = PhyLinkMode::None; ///< 器件支持的 10/100 模式集合。
    bool        autoNegotiation = false;             ///< 器件是否支持自动协商。
};

/**
 * @brief 链路事件硬件回调：由具体 PHY 在触发 INT 的 ISR 上下文中调用。
 *
 * @param[in] context 注册时传入的上下文指针（见 setLinkEventCallback）。
 *
 * 回调只表示“需要处理一个事件”，不承诺已确认链路变化，也不保证事件仍在 pending。回调
 * 可能在 ISR 中执行，因此实现必须保持极短且不得访问 MDIO；可调用该上下文允许的通知 API
 * 唤醒线程，真正的读取、确认与状态转换在线程上下文中完成。
 */
using LinkEventCallback = void (*)(void* context) noexcept;

/**
 * @brief 调用方持有的 PHY 对象接口。
 *
 * 上层仅借用 IPhy&，不得通过基类指针销毁对象；具体对象由调用方按其具体类型管理，
 * 上层不拥有其生命周期。接口不接管 ETH MAC、协议栈或外设生命周期，不需要动态分配、
 * 异常、RTTI 或运行时驱动注册。
 *
 * 线程归属：本接口不创建、拥有或管理任何线程/任务，也不提供线程 stop/join 操作，不依赖
 * RTOS；生命周期由调用方控制。接口只提供可选的“链路事件”硬件通知（setLinkEventCallback
 * 与 interruptAsserted/acknowledgeInterrupt），是否真正接线到某条中断由具体实现决定。
 *
 * 生命周期：
 *  - 构造函数只保存配置（借用引用与地址），不访问硬件、不产生总线流量、不会失败；
 *    因此对象可安全地声明为静态存储期，构造顺序不受底层外设初始化影响。
 *  - 会失败的启动集中在 start(config)：必须在底层外设就绪后调用；具体前置条件由实现
 *    定义。start() 是阻塞的，可能中断既有链路并丢弃此前配置的协商/强制模式；成功只表示
 *    驱动就绪，不保证链路已建立。
 *  - start() 可重复调用，每次都会丢弃此前配置并按传入 config 完整重建；运行期写失败后
 *    再次调用 start(config) 即可恢复，无需销毁重建。
 *
 * 所有总线操作均由调用方串行化；接口不提供锁、后台线程、定时器或自动重试。唯一允许的
 * 异步入口是 setLinkEventCallback 注册的事件回调，它可能从 ISR 触发，只应唤醒调用方的
 * 线程。status() != Ok 时，总线操作方法一律返回 NotInitialized，不访问总线、不改动输出。
 */
class IPhy
{
public:
    /**
     * @brief 启动驱动并按 config 配置链路；每次调用都完整重建。
     *
     * 必须先完成纯参数校验：地址或 config 非法时返回 InvalidArgument，且不产生任何总线
     * 访问。config 会被原样应用，不会被默认协商配置或模式交集静默替换；自动协商模式下
     * modes 为非空通告掩码，强制模式下必须恰含一种模式。
     *
     * 每次调用都会丢弃此前配置并完整重建，可能中断既有链路。实现可能在流程中复位器件，
     * 因此 Unsupported（请求的模式或自动协商不受器件支持）不保证完全没有寄存器写入；
     * 该判定只能来自启动过程中读取到的器件能力。
     *
     * 阻塞调用；任何失败都使对象不可用。成功后 status() == Ok，且仅表示驱动就绪，不表示
     * 链路已建立（链路需经 readLink() 查询）。失败后再次调用 start(config) 即可重试/恢复。
     *
     * @param[in] config 期望的链路配置，必须合法。
     * @return Ok 执行成功且对象就绪；其它值为本次启动失败的原因。
     */
    [[nodiscard]] virtual PhyResult start(const PhyLinkConfig& config) noexcept = 0;

    /**
     * @brief 查询对象状态，返回最近一次 start() 的粘性结果。
     *
     * 未成功执行 start() 前为 NotInitialized；Ok 表示已就绪；其它值表示最近一次 start()
     * 失败的原因，或后续写失败（WriteError）导致对象失效后的原因。该结果不因其它调用改变。
     */
    [[nodiscard]] virtual PhyResult status() const noexcept = 0;

    /**
     * @brief 查询当前链路状态。
     * @param[out] state 仅在返回 Ok 时更新；任何失败均保持调用前的值。
     * @return status() != Ok 时返回 NotInitialized；正常断链返回 Ok 并写入 Down。
     *
     * 调用方必须检查返回值，不能将访问错误当作正常链路状态。
     */
    [[nodiscard]] virtual PhyResult readLink(PhyLinkState& state) noexcept = 0;

    /**
     * @brief 读取 start() 时缓存的器件能力，不做额外总线读取。
     * @param[out] capabilities 仅在返回 Ok 时更新。
     * @return status() != Ok 时返回 NotInitialized，且不改动输出。
     */
    [[nodiscard]] virtual PhyResult getCapabilities(PhyCapabilities& capabilities) noexcept = 0;

    /**
     * @brief 配置链路模式：自动协商通告掩码，或强制单一模式。
     * @param[in] config 非法或器件不支持时返回错误且不写寄存器。
     * @return Ok 成功；NotInitialized status() != Ok；InvalidArgument/Unsupported 配置无效；
     *         ReadError 读取现状失败（对象仍就绪）；WriteError 写入失败且对象转为未就绪。
     *
     * 相同有效配置不会被重复写入，也不会重复触发自动协商重启；调用方可显式调用
     * restartAutoNegotiation()。本方法不等待链路建立。
     */
    [[nodiscard]] virtual PhyResult configureLink(const PhyLinkConfig& config) noexcept = 0;

    /**
     * @brief 显式重启自动协商，返回时不等待协商完成。
     * @return Ok 已提交；NotInitialized status() != Ok；Unsupported 当前为强制模式（无写入）；
     *         ReadError 读取 BMCR 失败（对象仍就绪）；WriteError 写入失败且对象转为未就绪。
     */
    [[nodiscard]] virtual PhyResult restartAutoNegotiation() noexcept = 0;

    /**
     * @brief 注册/更新链路事件回调；可在 start() 之前调用。
     *
     * 具体 PHY 在提供 INT 引脚时，把该回调挂到引脚事件源（例如 EXTI）上；回调可能在 ISR
     * 中触发，实现只通知“有事件需要处理”，不访问 MDIO。传入 nullptr 表示关闭通知；
     * callback/context 的更新不改变 PHY 配置、不产生总线流量、不会失败。
     *
     * 更新 callback 与 context 必须是原子的（要么观察到旧组合、要么观察新组合），以便 ISR
     * 不会读到半配置状态。未提供 INT 的实现可以只保存参数而永不触发。
     *
     * @param[in] callback 事件回调；nullptr 关闭通知。
     * @param[in] context  回调上下文，原样回传给 callback。
     */
    virtual void setLinkEventCallback(LinkEventCallback callback, void* context) noexcept = 0;

    /**
     * @brief 配置是否包含可用的中断事件源（INT 引脚）。
     * @return true 表示本对象配置了 INT，事件回调与 acknowledgeInterrupt() 可用；否则 false。
     * @note 只反映配置，不访问硬件。
     */
    [[nodiscard]] virtual bool usesInterrupt() const noexcept = 0;

    /**
     * @brief 采样中断线电平，判断事件是否仍处于 pending。
     * @return 配置了 INT 时为 true 表示中断线仍有效（active-low 为低）；未配置时为 false。
     * @note 只读取引脚，不访问 MDIO，可在任意上下文调用。
     */
    [[nodiscard]] virtual bool interruptAsserted() const noexcept = 0;

    /**
     * @brief 确认（读取并清除）PHY 事件源，报告是否有链路相关事件。
     * @param[out] link_changed 仅在返回 Ok 时更新；失败时保持原值。
     * @return NotInitialized 表示尚未成功 start()；Unsupported 表示未配置 INT，无事件可确认；
     *         其它值为确认过程中的读取失败原因。
     * @note 可能访问 MDIO，只能在线程上下文调用，不得在 ISR 中调用。
     */
    [[nodiscard]] virtual PhyResult acknowledgeInterrupt(bool& link_changed) noexcept = 0;

protected:
    ~IPhy() = default;
};

} // namespace bsp::ethernet_phy

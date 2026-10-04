/**
 * @file    IPhy.hpp
 * @brief   不依赖 HAL、协议栈或 RTOS 的以太网 PHY 接口。
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#pragma once

#include <cstdint>

/** @brief PHY 操作结果；具体驱动负责转换底层状态码，不与其数值绑定。 */
enum class PhyResult : std::uint8_t
{
    Ok,               ///< 操作成功；readLink() 的输出此时才有效。
    NotInitialized,   ///< 尚未完整初始化，或最近一次初始化失败。
    AddressError,     ///< 地址超出器件范围，或未找到匹配型号的器件。
    AmbiguousAddress, ///< 自动探测发现多个匹配器件，必须指定固定地址。
    ReadError,        ///< 寄存器读取失败，不代表正常断链。
    WriteError,       ///< 寄存器写入失败。
    ResetTimeout,     ///< 器件复位未在规定时间内完成。
    Error,            ///< 其他错误或无法识别的底层结果。
};

/** @brief 成功读取的链路状态；访问错误通过 PhyResult 表达。 */
enum class PhyLinkState : std::uint8_t
{
    Down,
    Negotiating,
    Up10Half,
    Up10Full,
    Up100Half,
    Up100Full,
};

/**
 * @brief 调用方持有的 PHY 对象接口。
 *
 * 上层仅借用 IPhy&，不得通过基类指针销毁对象；具体对象由调用方按其具体类型管理，
 * 可使用静态存储期。接口不接管 ETH MAC、协议栈或外设生命周期，不需要动态分配、
 * 异常、RTTI 或运行时驱动注册。
 *
 * 所有操作均由调用方串行化；接口不提供锁、线程、定时器、中断或后台重试。
 */
class IPhy
{
public:
    /**
     * @brief 显式初始化 PHY。
     *
     * 调用方必须先满足具体实现的底层外设初始化前提。
     * 只有返回 Ok 才可查询链路；初始化失败撤销就绪状态，允许之后显式重试。
     */
    [[nodiscard]] virtual PhyResult init() noexcept = 0;

    /**
     * @brief 查询当前链路状态。
     * @param[out] state 仅在返回 Ok 时更新；任何失败均保持调用前的值。
     * @return 未初始化返回 NotInitialized；正常断链返回 Ok 并写入 Down。
     *
     * 调用方必须检查返回值，不能将访问错误当作正常链路状态。
     */
    [[nodiscard]] virtual PhyResult readLink(PhyLinkState& state) noexcept = 0;

protected:
    ~IPhy() = default;
};

/**
 * @file    static_object.hpp
 * @brief   C++17 固定存储、显式构造的永久对象槽。
 */
#pragma once
#include <cassert>
#include <cstddef>
#include <new>
#include <type_traits>

namespace memory
{
namespace detail
{
struct StaticObjectInitializer;
}

/**
 * @brief 为一个永久对象预留固定地址的存储，不分配堆内存，也不自动析构 T。
 * @tparam T 完整、非数组、非 cv 限定的具体对象类型；不要求默认构造或复制/移动。
 *
 * 使用约定：
 * - 对象槽应具有静态存储期；先用 INIT_OBJECT 构造，再访问对象。
 * - 每个槽只能成功构造一次。重复初始化在 Debug 下断言失败；Release 由调用方保证。
 * - ->、* 和指针转换均不检查状态；未构造时调用这些操作违反前置条件。
 * - 状态查询使用 is_initialized()、if (slot)、!slot 或与 nullptr 的比较。
 *   bool ready = slot 会经由隐式指针转换；需要保存状态时使用 static_cast<bool>(slot)。
 * - const 槽视图提供 const T 访问；取得的指针不拥有对象，禁止 delete。
 * - 不提供线程/ISR 同步；应在使用者开始访问前完成构造，或由调用方同步。
 * - sizeof(StaticObject<T>) 包含 T 的存储、状态及对齐填充，不包含 T 自行申请的内存。
 *   内存区域由实例声明和链接脚本决定；本类型不配置 DMA、MPU 或缓存属性。
 */
template <typename T> class StaticObject
{
    static_assert(std::is_object_v<T> && !std::is_array_v<T>,
                  "StaticObject<T> requires a non-array object type");
    static_assert(!std::is_const_v<T> && !std::is_volatile_v<T>,
                  "StaticObject<T> requires an unqualified type; use const views of the slot");
    static_assert(!std::is_abstract_v<T>, "StaticObject<T> requires a concrete type");

    friend struct detail::StaticObjectInitializer;

public:
    constexpr StaticObject() noexcept            = default;
    StaticObject(const StaticObject&)            = delete;
    StaticObject& operator=(const StaticObject&) = delete;
    StaticObject(StaticObject&&)                 = delete;
    StaticObject& operator=(StaticObject&&)      = delete;

    /// 无检查的对象访问；调用前必须已完成构造。
    T*       operator->() noexcept { return pointer(); }
    const T* operator->() const noexcept { return pointer(); }

    T&       operator*() noexcept { return *pointer(); }
    const T& operator*() const noexcept { return *pointer(); }

    operator T*() noexcept { return pointer(); }
    operator const T*() const noexcept { return pointer(); }

    /// 此查询、bool 上下文及 nullptr 比较可在构造前安全使用。
    // [[nodiscard]] constexpr bool is_initialized() const noexcept { return initialized_; }

    friend constexpr bool operator==(const StaticObject& object, std::nullptr_t) noexcept
    {
        return !object.initialized_;
    }

    friend constexpr bool operator!=(const StaticObject& object, std::nullptr_t) noexcept
    {
        return object.initialized_;
    }

    friend constexpr bool operator==(std::nullptr_t, const StaticObject& object) noexcept
    {
        return !object.initialized_;
    }

    friend constexpr bool operator!=(std::nullptr_t, const StaticObject& object) noexcept
    {
        return object.initialized_;
    }

    // 与指针转换保持相同的 cv 重载，避免可变对象的条件表达式选择 T* 转换。
    explicit constexpr operator bool() noexcept { return initialized_; }
    explicit constexpr operator bool() const noexcept { return initialized_; }

private:
    T*       pointer() noexcept { return std::launder(reinterpret_cast<T*>(storage_)); }
    const T* pointer() const noexcept { return std::launder(reinterpret_cast<const T*>(storage_)); }

    T* initialization_storage() noexcept
    {
        assert(!initialized_);
        // T 的生命周期尚未开始；这里只形成可用于 placement new 的存储指针。
        return reinterpret_cast<T*>(storage_);
    }

    T* const debug_ = initialization_storage();
    alignas(T) std::byte storage_[sizeof(T)]{};
    bool initialized_ = false;
};

namespace detail
{
struct StaticObjectInitializer
{
    template <typename T, typename Constructor>
    static T* initialize(StaticObject<T>& object, Constructor&& constructor)
    {
        // 构造异常原样传播；只有构造成功后才发布状态。
        T* result           = constructor(object.initialization_storage());
        object.initialized_ = true;
        return result;
    }
};
} // namespace detail
} // namespace memory

#define STATIC_OBJECT_DETAIL_INIT_IMPL(object, line, ...)                                          \
    ::memory::detail::StaticObjectInitializer::initialize(                                         \
            (object),                                                                              \
            [&](auto* static_object_detail_storage_##line)                                         \
            {                                                                                      \
                return ::new (static_cast<void*>(static_object_detail_storage_##line))::std::      \
                        remove_reference_t<decltype(*(object))>(__VA_ARGS__);                      \
            })

#define STATIC_OBJECT_DETAIL_INIT_EXPAND(object, line, ...)                                        \
    STATIC_OBJECT_DETAIL_INIT_IMPL(object, line, __VA_ARGS__)

/**
 * @brief 在函数体中原地构造对象，返回 T*；不进行堆分配。
 *
 * object 必须是可变槽的左值，只求值一次。构造参数直接出现在 new T(...) 中，
 * 因此支持裸 {...}，不会遇到完美转发的列表推导限制。构造成功后才发布状态；
 * 若 T 的构造函数抛出异常，异常原样传播，槽仍保持未初始化状态。
 *
 * 示例：INIT_OBJECT(motors[index++], config, {1, 2});
 * 严格 C++17 的无参构造写作 INIT_OBJECT(device, );，保留显式的空可变参数。
 * 宏不属于 C++ 命名空间：使用 INIT_OBJECT，而不是 memory::INIT_OBJECT。
 */
#define INIT_OBJECT(object, ...) STATIC_OBJECT_DETAIL_INIT_EXPAND(object, __LINE__, __VA_ARGS__)

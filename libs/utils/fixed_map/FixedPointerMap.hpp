/**
 * @file    FixedPointerMap.hpp
 * @author  syhanjin
 * @date    2026-02-27
 * @brief   固定容量的键到指针映射表。
 *
 * 适合小规模注册表：无动态分配，查找和删除都按线性扫描实现。
 * 这类容器的优势是行为简单、占用可预测，缺点是容量一旦满了就只能靠上层逻辑做处理。
 */
#pragma once
#include <cstddef>
#include <utility>

template <typename K, typename V, size_t N> class FixedPointerMap
{
public:
    /**
     * @brief 插入一个键值对。
     *
     * @return false 可能表示 key 已存在，或者表已经满了。
     */
    bool insert(const K& key, V* value)
    {
        // key 已存在时不覆盖，保持表内键唯一。
        for (size_t i = 0; i < size_; ++i)
        {
            if (data_[i].first == key)
            {
                return false;
            }
        }

        // 容量允许时追加到末尾。
        if (size_ < N)
        {
            data_[size_++] = { key, value };
            return true;
        }

        return false; // 已满
    }

    /**
     * @brief 按 key 查找对应指针。
     *
     * 找不到时返回 nullptr。
     */
    V* find(const K& key)
    {
        for (size_t i = 0; i < size_; ++i)
            if (data_[i].first == key)
                return data_[i].second;
        return nullptr;
    }

    /**
     * @brief 删除指定 key。
     *
     * 删除时用最后一个元素覆盖当前位置，以避免整体搬移。
     */
    bool erase(const K& key)
    {
        for (size_t i = 0; i < size_; ++i)
        {
            if (data_[i].first == key)
            {
                // 用最后一个元素覆盖删除位置，避免整体搬移。
                --size_;
                return true;
            }
        }
        return false;
    }

    /**
     * @brief 当前表中元素个数。
     */
    [[nodiscard]] size_t size() const { return size_; }

private:
    std::pair<K, V*> data_[N];
    size_t           size_ = 0;
};

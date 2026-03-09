/**
 * @file    FixedPointerMap.hpp
 * @author  syhanjin
 * @date    2026-02-27
 */
#pragma once
#include <cstddef>
#include <utility>

template <typename K, typename V, size_t N> class FixedPointerMap
{
public:
    bool insert(const K& key, V* value)
    {
        // if key exists, return false;
        for (size_t i = 0; i < size_; ++i)
        {
            if (data_[i].first == key)
            {
                return false;
            }
        }

        // insert new if space available
        if (size_ < N)
        {
            data_[size_++] = { key, value };
            return true;
        }

        return false; // full
    }

    V* find(const K& key)
    {
        for (size_t i = 0; i < size_; ++i)
            if (data_[i].first == key)
                return data_[i].second;
        return nullptr;
    }

    bool erase(const K& key)
    {
        for (size_t i = 0; i < size_; ++i)
        {
            if (data_[i].first == key)
            {
                data_[i] = data_[size_ - 1]; // replace with last
                --size_;
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] size_t size() const { return size_; }

private:
    std::pair<K, V*> data_[N];
    size_t           size_ = 0;
};

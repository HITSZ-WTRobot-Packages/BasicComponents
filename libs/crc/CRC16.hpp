/**
 * @file    CRC16.hpp
 * @author  syhanjin
 * @date    2026-02-24
 * @brief   Brief description of the file
 *
 * Detailed description (optional).
 *
 */
#pragma once

#include <cstdint>
#include <cstddef>
#include <array>
namespace crc
{

// compile-time bit reverse
constexpr uint16_t bit_reverse(const uint16_t value, const unsigned bits)
{
    uint16_t r = 0;
    for (unsigned i = 0; i < bits; ++i)
        r |= ((value >> i) & 1) << (bits - 1 - i);
    return r;
}

// CRC16 template with only calc public
template <uint16_t poly, uint16_t init, bool rin, bool rout, uint16_t xorout> class CRC16
{
    using value_type = uint16_t;

    // generate single table entry
    static constexpr uint16_t table_entry(const uint8_t index)
    {
        uint16_t c = rin ? bit_reverse(index, 8) : index;
        c <<= 8;
        for (int i = 0; i < 8; ++i)
        {
            if (c & 0x8000)
                c = (c << 1) ^ poly;
            else
                c <<= 1;
        }
        c &= 0xFFFF;
        return rout ? bit_reverse(c, 16) : c;
    }

    // compile-time table
    static constexpr std::array<uint16_t, 256> generate_table()
    {
        std::array<uint16_t, 256> t{};
        for (std::size_t i = 0; i < 256; ++i)
            t[i] = table_entry(static_cast<uint8_t>(i));
        return t;
    }

    static constexpr std::array<uint16_t, 256> table = generate_table();

public:
    // only this function is public
    static value_type calc(const uint8_t* data, size_t len)
    {
        value_type crc = init;
        while (len--)
        {
            crc = table[(crc >> 8) ^ *data++] ^ (crc << 8);
        }
        if constexpr (xorout != 0)
            crc ^= xorout;
        return crc;
    }
};

// define static table
template <uint16_t poly, uint16_t init, bool rin, bool rout, uint16_t xorout>
constexpr std::array<uint16_t, 256> CRC16<poly, init, rin, rout, xorout>::table;

} // namespace crc

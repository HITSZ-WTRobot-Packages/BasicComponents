/**
 * @file    crc.hpp
 * @author  syhanjin
 * @date    2026-03-01
 * @brief   CRC library
 *
 * 常用 CRC
 * // CRC-8
 * using CRC8_ATM = crc::CRCX<8, 0x07, 0x00, false, false, 0x00>;
 *
 * // CRC-8/MAXIM
 * using CRC8_MAXIM = crc::CRCX<8, 0x31, 0x00, true, true, 0x00>;
 *
 * // CRC-16/MODBUS
 * using CRC16_MODBUS = crc::CRCX<16, 0x8005, 0xFFFF, true, true, 0x0000>;
 *
 * // CRC-16/CCITT-FALSE
 * using CRC16_CCITT = crc::CRCX<16, 0x1021, 0xFFFF, false, false, 0x0000>;
 *
 * // CRC-24/OpenPGP
 * using CRC24_PGP = crc::CRCX<24, 0x864CFB, 0xB704CE, false, false, 0x000000>;
 *
 * // CRC-32 (Ethernet)
 * using CRC32 = crc::CRCX<32, 0x04C11DB7, 0xFFFFFFFF, true, true, 0xFFFFFFFF>;
 *
 * // CRC-32C
 * using CRC32C = crc::CRCX< 2, 0x1EDC6F41, 0xFFFFFFFF, true, true, 0xFFFFFFFF>;
 *
 * // CRC-64/ECMA
 * using CRC64_ECMA = crc::CRCX<64,
 *                             0x42F0E1EBA9EA3693ULL,
 *                             0x0000000000000000ULL,
 *                             false,
 *                             false,
 *                             0x0000000000000000ULL>;
 */
#pragma once

#include <cstdint>
#include <cstddef>
#include <array>

namespace crc
{
namespace detail
{
template <typename T> constexpr T bit_reverse(T value, unsigned bits)
{
    T r = 0;
    for (unsigned i = 0; i < bits; ++i)
        r |= ((value >> i) & T{ 1 }) << (bits - 1 - i);
    return r;
}

template <unsigned Bits> struct uint_of_bits;

template <> struct uint_of_bits<8>
{
    using type = uint8_t;
};
template <> struct uint_of_bits<16>
{
    using type = uint16_t;
};
template <> struct uint_of_bits<24>
{
    using type = uint32_t;
};
template <> struct uint_of_bits<32>
{
    using type = uint32_t;
};
template <> struct uint_of_bits<64>
{
    using type = uint64_t;
};

template <unsigned Bits> using uint_of_bits_t = typename uint_of_bits<Bits>::type;

} // namespace detail

// =======================
// generic CRC
// =======================
template <unsigned Bits, auto Poly, auto Init, bool Rin, bool Rout, auto XorOut> class CRCX
{
    static_assert(Bits == 8 || Bits == 16 || Bits == 24 || Bits == 32 || Bits == 64);

    using value_type = detail::uint_of_bits_t<Bits>;

    static constexpr value_type mask = Bits == 64 ? value_type(-1) : (value_type(1) << Bits) - 1;

    static constexpr value_type table_entry(uint8_t index)
    {
        value_type c = Rin ? detail::bit_reverse<value_type>(index, 8) : value_type(index);

        c <<= (Bits - 8);

        for (int i = 0; i < 8; ++i)
        {
            if (c & (value_type(1) << (Bits - 1)))
                c = (c << 1) ^ Poly;
            else
                c <<= 1;
        }

        c &= mask;
        return Rout ? detail::bit_reverse<value_type>(c, Bits) : c;
    }

    static constexpr std::array<value_type, 256> generate_table()
    {
        std::array<value_type, 256> t{};
        for (std::size_t i = 0; i < 256; ++i)
            t[i] = table_entry(static_cast<uint8_t>(i));
        return t;
    }

    static inline constexpr auto table = generate_table();

public:
    static value_type calc(const uint8_t* data, size_t len)
    {
        value_type crc = Init;

        while (len--)
        {
            if constexpr (Bits <= 8)
            {
                crc = table[(crc ^ *data++) & 0xFF];
            }
            else
            {
                crc = table[((crc >> (Bits - 8)) ^ *data++) & 0xFF] ^ (crc << 8);
            }
            crc &= mask;
        }

        if constexpr (XorOut != 0)
            crc ^= XorOut;

        return crc & mask;
    }

    template <std::size_t N> static constexpr value_type calc(const std::array<uint8_t, N>& data)
    {
        value_type crc = Init;

        for (std::size_t i = 0; i < N; ++i)
        {
            if constexpr (Bits <= 8)
            {
                crc = table[(crc ^ data[i]) & 0xFF];
            }
            else
            {
                crc = table[((crc >> (Bits - 8)) ^ data[i]) & 0xFF] ^ (crc << 8);
            }
            crc &= mask;
        }

        if constexpr (XorOut != 0)
            crc ^= XorOut;

        return crc & mask;
    }
};

} // namespace crc

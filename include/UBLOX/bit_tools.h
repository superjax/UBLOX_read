/* Copyright (c) 2019 James Jackson
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <cstdint>

enum
{
    SwitchByteOrder,
    KeepByteOrder,
    Signed,
    Unsigned
};

// clang-format off
template <int Sign, size_t len>
struct TypeHelper {
    typedef
        typename std::conditional<Sign == Unsigned && len <= 8 , uint8_t,
        typename std::conditional<Sign == Unsigned && len <= 16, uint16_t,
        typename std::conditional<Sign == Unsigned && len <= 32, uint32_t,
        typename std::conditional<Sign == Unsigned && len <= 64, uint64_t,
        typename std::conditional<Sign == Signed && len <= 8 , int8_t,
        typename std::conditional<Sign == Signed && len <= 16, int16_t,
        typename std::conditional<Sign == Signed && len <= 32, int32_t,
        typename std::conditional<Sign == Signed && len <= 64, int64_t,
        void>::type>::type>::type>::type>::type>::type>::type>::type type;
};
// clang-format on

template <int len, int Sign = Unsigned, int byteOrder = SwitchByteOrder>
typename TypeHelper<Sign, len>::type getBit(const unsigned char *buf, const int pos)
{
    typedef typename TypeHelper<Unsigned, len + 8>::type tempType;
    typedef typename TypeHelper<Unsigned, len>::type UnsignedRetType;
    typedef typename TypeHelper<Sign, len>::type retType;
    static constexpr tempType mask = len == 64 ? 0xFFFFFFFF : (1ul << len) - 1ul;
    const size_t last_bit = pos + len - 1;
    static_assert(len <= 64 && len > 0);

    const uint8_t *p = buf + (pos / 8);

    tempType tmp = *p;
    UnsignedRetType ret;
    if constexpr (byteOrder == SwitchByteOrder)
    {
        for (int i = 8 - (pos & 7); i < len; i += 8)
        {
            tmp <<= 8;
            tmp |= *(++p);
        }
        ret = ((tmp >> (7 - (last_bit & 7))) & mask);
    }
    else
    {
        for (int i = 0; i < len; i += 8)
        {
            tmp |= (*(++p) << (i + 8));
        }
        ret = ((tmp >> (pos % 8)) & mask);
    }

    // If requested, move the sign bit to the end of the type
    if constexpr (Sign == Signed)
    {
        static constexpr tempType one = 1;
        if (ret & (one << (len - 1)))
        {
            static constexpr tempType zero = 0;
            return (retType)(ret | (~zero << len));
        }
    }
    return ret;
}

template <int Sign = Unsigned, int byteOrder = SwitchByteOrder>
typename TypeHelper<Sign, 32>::type getBit(const unsigned char *buf, const int pos, const int len)
{
    typedef typename TypeHelper<Unsigned, 64>::type tempType;
    typedef typename TypeHelper<Unsigned, 32>::type UnsignedRetType;
    typedef typename TypeHelper<Sign, 32>::type retType;
    tempType mask = len == 64 ? 0xFFFFFFFF : (1ul << len) - 1ul;
    const size_t last_bit = pos + len - 1;
    // assert(len <= 64 && len > 0);

    const uint8_t *p = buf + (pos / 8);

    tempType tmp = *p;
    UnsignedRetType ret;
    if constexpr (byteOrder == SwitchByteOrder)
    {
        for (int i = 8 - (pos & 7); i < len; i += 8)
        {
            tmp <<= 8;
            tmp |= *(++p);
        }
        ret = ((tmp >> (7 - (last_bit & 7))) & mask);
    }
    else
    {
        for (int i = 0; i < len; i += 8)
        {
            tmp |= (*(++p) << (i + 8));
        }
        ret = ((tmp >> (pos % 8)) & mask);
    }

    // If requested, move the sign bit to the end of the type
    if constexpr (Sign == Signed)
    {
        static constexpr tempType one = 1;
        if (ret & (one << (len - 1)))
        {
            static constexpr tempType zero = 0;
            return (retType)(ret | (~zero << len));
        }
    }
    return ret;
}

template <int len, int Sign = Unsigned, int byteOrder = SwitchByteOrder>
void setBit(unsigned char *buff, const int pos, const unsigned data)
{
    typedef typename TypeHelper<Unsigned, len + 8>::type tempType;
    static constexpr tempType mask = len == 64 ? 0xFFFFFFFF : (1ul << len) - 1ul;
    size_t last_bit = pos + len - 1;
    static_assert(len <= 64 && len > 0);
    static_assert(byteOrder == SwitchByteOrder, "KeepByteOrder Not implemented");

    tempType m = data & mask;
    tempType wmask = ~(mask << (7 - (last_bit & 7)));
    m <<= (7 - (last_bit & 7));
    uint8_t *p = buff + last_bit / 8;
    int i = (last_bit & 7) + 1;
    (*p &= wmask) |= m;
    while (i < len)
    {
        m >>= 8;
        wmask >>= 8;
        (*(--p) &= wmask) |= m;
        i += 8;
    }
}

template <int Sign = Unsigned, int byteOrder = SwitchByteOrder>
void setBit(unsigned char *buff, const int pos, const unsigned data, const int len)
{
    typedef typename TypeHelper<Unsigned, 64>::type tempType;
    tempType mask = len == 64 ? 0xFFFFFFFF : (1ul << len) - 1ul;
    size_t last_bit = pos + len - 1;
    // static_assert(len <= 64 && len > 0);
    static_assert(byteOrder == SwitchByteOrder, "KeepByteOrder Not implemented");

    tempType m = data & mask;
    tempType wmask = ~(mask << (7 - (last_bit & 7)));
    m <<= (7 - (last_bit & 7));
    uint8_t *p = buff + last_bit / 8;
    int i = (last_bit & 7) + 1;
    (*p &= wmask) |= m;
    while (i < len)
    {
        m >>= 8;
        wmask >>= 8;
        (*(--p) &= wmask) |= m;
        i += 8;
    }
}

template <int len>
double getBitGlo(const uint8_t *buf, const int pos)
{
    const double val = getBit<len - 1>(buf, pos + 1);
    return getBit<1>(buf, pos) ? -val : val;
}
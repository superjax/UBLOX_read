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

#include <gtest/gtest.h>
#include "UBLOX/bitfield.h"
#include "UBLOX/parsers/rtcm.h"
#include "reference.h"

TEST(BitField, CheckAlignedData)
{
    union TestUnion
    {
        uint32_t buf[2];
        uint16_t byte;
        BitField<0, 8> aligned1;
        BitField<8, 16> aligned2;
        BitField<32, 16> aligned3;
        BitField<48, 16> aligned4;
    };

    TestUnion TestData;
    TestData.buf[0] = 0x01234567;
    TestData.buf[1] = 0x8ABCDEF0;

    uint8_t aligned1 = TestData.aligned1;
    EXPECT_EQ(aligned1, 0x67);

    uint16_t aligned2 = TestData.aligned2;
    EXPECT_EQ(aligned2, 0x2345);

    uint16_t aligned3 = TestData.aligned3;
    EXPECT_EQ(aligned3, 0xDEF0);

    uint16_t aligned4 = TestData.aligned4;
    EXPECT_EQ(aligned4, 0x8ABC);
}

TEST(BitField, CheckUnAlignedData)
{
    union TestUnion
    {
        uint32_t buf[2];
        BitField<0, 5> offset0;
        BitField<1, 7> offset1;
        BitField<33, 6> offset2;
        BitField<61, 3> offset3;
        BitField<4, 15> offset4;
        BitField<41, 13> offset5;
    };

    TestUnion TestData;
    TestData.buf[0] = 0x01234567;
    TestData.buf[1] = 0x8ABCDEF0;

    uint8_t offset0 = TestData.offset0;
    EXPECT_EQ(offset0, 0x7);

    uint8_t offset1 = TestData.offset1;
    EXPECT_EQ(offset1, 0x67 >> 1);

    uint8_t offset2 = TestData.offset2;
    EXPECT_EQ(offset2, (0xF0 >> 1) & 0b111111);

    uint8_t offset3 = TestData.offset3;
    EXPECT_EQ(offset3, 0x8A >> 5);

    uint16_t offset4 = TestData.offset4;
    EXPECT_EQ(offset4, ((0x01234567 >> 4) & 0x7FFF));

    uint16_t offset5 = TestData.offset5;
    EXPECT_EQ(offset5, ((0x8ABCDEF0 >> 9) & 0x1FFF));
}

TEST(BitField, AlignedvsRTKLIB)
{
    union TestUnion
    {
        uint32_t buf[2];
        uint16_t byte;
        BitField<0, 8> aligned1;
        BitField<8, 16> aligned2;
        BitField<32, 16> aligned3;
        BitField<48, 16> aligned4;
    };

    TestUnion TestData;
    TestData.buf[0] = 0x01234567;
    TestData.buf[1] = 0x8ABCDEF0;

    uint8_t offset0 = TestData.offset0;
    EXPECT_EQ(offset0, 0x7);

    uint8_t offset1 = TestData.offset1;
    EXPECT_EQ(offset1, 0x67 >> 1);

    uint8_t offset2 = TestData.offset2;
    EXPECT_EQ(offset2, (0xF0 >> 1) & 0b111111);

    uint8_t offset3 = TestData.offset3;
    EXPECT_EQ(offset3, 0x8A >> 5);

    uint16_t offset4 = TestData.offset4;
    EXPECT_EQ(offset4, ((0x01234567 >> 4) & 0x7FFF));

    uint16_t offset5 = TestData.offset5;
    EXPECT_EQ(offset5, ((0x8ABCDEF0 >> 9) & 0x1FFF));
}

#include <gtest/gtest.h>

#include "UBLOX/bit_tools.h"

TEST(GetBits, Aligned)
{
    uint8_t data[] = {0x12, 0x34, 0x45, 0x56, 0x67, 0x78, 0x89, 0x9A, 0xAB, 0xBC, 0xCD, 0xDE, 0xEF};
    uint16_t bits = getBit<16>(data, 0);
    EXPECT_EQ(bits, 0x1234);
}

TEST(GetBits, HalfAlignedEnd)
{
    uint8_t data[] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xCD, 0xEF};

    uint16_t bits = getBit<12>(data, 0);
    EXPECT_EQ(bits, 0x123);
}

TEST(GetBits, HalfAlignedStart)
{
    uint8_t data[] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xCD, 0xEF};

    uint16_t bits = getBit<12>(data, 4);
    EXPECT_EQ(bits, 0x234);
}

TEST(GetBits, HalfAlignedBoth)
{
    uint8_t data[] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xCD, 0xEF};

    uint16_t bits = getBit<16>(data, 4);
    EXPECT_EQ(bits, 0x2345);
}

TEST(GetBits, MisAlignedEnd)
{
    uint8_t data[] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xCD, 0xEF};

    uint16_t bits = getBit<13>(data, 24);
    EXPECT_EQ(bits, 0x0F13);
}

TEST(GetBits, MisAlignedStart)
{
    uint8_t data[] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xEF};

    uint16_t bits = getBit<17>(data, 39);
    EXPECT_EQ(bits, 0xBCEF);
}

TEST(GetBits, MisAlignedEndLong)
{
    uint8_t data[] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xEF};

    uint16_t bits = getBit<17>(data, 32);
    EXPECT_EQ(bits, 0x3579);
}

TEST(GetBits, MisAlignedStartEnd)
{
    uint8_t data[] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xEF};

    uint16_t bits = getBit<17>(data, 29);
    EXPECT_EQ(bits, 0x26AF);
}

TEST(SetBits, Aligned)
{
    uint8_t buf[8];
    memset(buf, 0, 8);
    uint32_t data = 0xF125;

    setBit<16>(buf, 0, data);

    EXPECT_EQ(*(reinterpret_cast<uint32_t *>(buf)), 0x25F1);
}

TEST(SetBits, HalfAlignedStart)
{
    uint8_t buf[8];
    memset(buf, 0, 8);
    uint32_t data = 0xF125;

    setBit<16>(buf, 4, data);

    EXPECT_EQ(*(reinterpret_cast<uint32_t *>(buf)), 0x50120F);
}

TEST(SetBits, HalfAlignedEnd)
{
    uint8_t buf[8];
    memset(buf, 0, 8);

    uint32_t data = 0xF125B7;
    setBit<20>(buf, 0, data);

    EXPECT_EQ(*(reinterpret_cast<uint32_t *>(buf)), 0x705b12);
}

TEST(SetBits, HalfAlignedBoth)
{
    uint8_t buf[8];
    memset(buf, 0, 8);
    uint32_t data = 0xF125B7CE;

    setBit<32>(buf, 4, data);

    EXPECT_EQ(*(reinterpret_cast<uint32_t *>(buf)), 0x7c5b120f);
}

TEST(SetBits, UnalignedBoth)
{
    uint8_t buf[8];
    memset(buf, 0, 8);
    uint32_t data = 0xF125B7CE;

    setBit<32>(buf, 3, data);

    EXPECT_EQ(*(reinterpret_cast<uint32_t *>(buf)), 0xf9b6241e);
}

TEST(GetBitsSigned, Aligned)
{
    uint8_t data[] = {0x12, 0x34, 0x45, 0x56, 0x67, 0x78, 0x89, 0x9A, 0xAB, 0xBC, 0xCD, 0xDE, 0xEF};
    int16_t bits = getBit<16, Signed>(data, 48);
    EXPECT_EQ(bits, -30310);
}

TEST(GetBitsSigned, HalfAlignedStart)
{
    uint8_t data[] = {0x12, 0x34, 0x45, 0x56, 0x67, 0x78, 0x89, 0x9A, 0xAB, 0xBC, 0xCD, 0xDE, 0xEF};
    int bits = getBit<20, Signed>(data, 44);
    EXPECT_EQ(bits, -489062);
}

TEST(GetBitsSigned, HalfAlignedEnd)
{
    uint8_t data[] = {0x12, 0x34, 0x45, 0x56, 0x67, 0x78, 0x89, 0x9A, 0xAB, 0xBC, 0xCD, 0xDE, 0xEF};
    int bits = getBit<20, Signed>(data, 48);
    EXPECT_EQ(bits, -484950);
}

TEST(GetBitsSigned, MisalignedBoth)
{
    uint8_t data[] = {0x12, 0x34, 0x45, 0x56, 0x67, 0x78, 0x89, 0x9A, 0xAB, 0xBC, 0xCD, 0xDE, 0xEF};
    int bits = getBit<31, Signed>(data, 41);
    EXPECT_EQ(bits, -125199701);
}

TEST(GetBitsSigned, RealData)
{
    uint8_t data[] = {
        0x8b, 0x0,  0x38, 0xc1, 0x2b, 0xaf, 0x0,  0xb7, 0x60, 0x1c, 0x5e, 0x8f, 0xff, 0xca, 0x26,
        0xac, 0x32, 0x32, 0x2a, 0x9f, 0xc5, 0xd3, 0x7a, 0x1,  0xff, 0xa7, 0x4c, 0x4e, 0xfb, 0x68,
    };

    int i = 160;
    int out = getBit<32, Signed>(data, i);

    EXPECT_EQ(out, -975996415);
}
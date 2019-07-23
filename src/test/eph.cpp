#include <gtest/gtest.h>

#include "UBLOX/eph.h"

TEST (Eph, ByteOrder)
{
    union
    {
        uint32_t word;
        uint8_t bytes[4];
    } thing;

    thing.word = 0x01234567;

    EXPECT_EQ(thing.bytes[3], 0x01);
    EXPECT_EQ(thing.bytes[2], 0x23);
    EXPECT_EQ(thing.bytes[1], 0x45);
    EXPECT_EQ(thing.bytes[0], 0x67);
}

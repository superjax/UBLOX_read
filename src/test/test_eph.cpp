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

#include "UBLOX/parsers/nav.h"

TEST(Eph, ByteOrder)
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

TEST(NavParser, ParseGPS)
{
    // clang-format off
    uint32_t msgs[6][10]{
        {0x22c00e31, 0xb04aebf3, 0x2dd80a, 0x8717a3e7, 0x3ff28987, 0x2b0c8c8f, 0xaa7f16f, 0x34de804e, 0xbfe9d329, 0x93beda23},
        {0x22c00e31, 0xb04b0ccb, 0x1e67abc6, 0xb45ffedf, 0xaf3f61b, 0xb3ec995, 0xaf8bdbc5, 0xa8658185, 0x815a783f, 0x14586cdc},
        {0x22c00e31, 0xb04b2d43, 0x1182e14b, 0x9057592, 0xbf518007, 0x2843254d, 0x975fa428, 0x3506dbe1, 0x92868a6e, 0xbfffee80},
        {0x22c00e31, 0xb04b4947, 0xe40028, 0x16fcca86, 0x9a781ada, 0x8ee9581b, 0x3813faaa, 0x93a4798d, 0x80000f52, 0xbb0cf018},
        {0x22c00e31, 0xb04b6a2f, 0x13bf545b, 0xd8fd7b7, 0xa602fe6, 0xbf52c317, 0x232efafd, 0x80d3a86d, 0x83658d1c, 0x24799fe3},
        {0x22c00e31, 0xb04b8b13, 0x2dd80a, 0x8717a3e7, 0x3ff28987, 0x2b0c8c8f, 0xaa7f16f, 0x34de804e, 0xbfe9d329, 0x93beda23}};
    // clang-format on

    Ephemeris eph;
    NavParser conv;
    bool finished = false;
    conv.registerCallback([&finished](const Ephemeris& eph) {
        finished = true;
        EXPECT_EQ(eph.sat, 21);
        EXPECT_EQ(eph.gnssID, ublox::GnssID_GPS);
        EXPECT_EQ(eph.iode, 78);
        EXPECT_EQ(eph.iodc, 78);
        EXPECT_EQ(eph.ura, 0);
        EXPECT_EQ(eph.health, 0);
        EXPECT_EQ(eph.week, 2062);
        EXPECT_EQ(eph.code_on_L2, 1);
        EXPECT_EQ(eph.alert_flag, 0);
        EXPECT_EQ(eph.toe.sec, 1563660018);
        EXPECT_EQ(eph.toe.nsec, 0);
        EXPECT_EQ(eph.toc.sec, 1563660018);
        EXPECT_EQ(eph.toc.nsec, 0);
        EXPECT_NEAR(eph.sqrta, std::sqrt(26560606.593836), 1e-6);
        EXPECT_NEAR(eph.ecc, 0.024511, 1e-6);
        EXPECT_NEAR(eph.i0, 0.949169, 1e-6);
        EXPECT_NEAR(eph.omega0, 2.358914, 1e-6);
        EXPECT_NEAR(eph.w, -1.427803, 1e-6);
        EXPECT_NEAR(eph.w, -1.427803, 1e-6);
        EXPECT_NEAR(eph.m0, 2.311086, 1e-6);
        EXPECT_NEAR(eph.delta_n, 0.000000, 1e-6);
        EXPECT_NEAR(eph.omegadot, -0.000000, 1e-6);
        EXPECT_NEAR(eph.idot, -0.000000, 1e-6);
        EXPECT_NEAR(eph.crc, 340.968750, 1e-6);
        EXPECT_NEAR(eph.crs, -21.468750, 1e-6);
        EXPECT_NEAR(eph.cuc, -0.000001, 1e-6);
        EXPECT_NEAR(eph.cus, 0.000002, 1e-6);
        EXPECT_NEAR(eph.cic, 0.000000, 1e-6);
        EXPECT_NEAR(eph.cis, -0.000000, 1e-6);
        EXPECT_NEAR(eph.toes, 597600.000000, 1e-6);
        EXPECT_EQ(eph.fit_interval_flag, 4);
        EXPECT_NEAR(eph.af0, -0.000151, 1e-6);
        EXPECT_NEAR(eph.af1, 0.000000, 1e-6);
        EXPECT_NEAR(eph.af2, 0.000000, 1e-6);
        EXPECT_NEAR(eph.tgd[0], 0.000000, 1e-6);
        EXPECT_NEAR(eph.tgd[1], 0.000000, 1e-6);
        EXPECT_NEAR(eph.tgd[2], 0.000000, 1e-6);
        EXPECT_NEAR(eph.tgd[3], 0.000000, 1e-6);
    });

    eph.sat = 21;
    for (int i = 0; i < 6; i++)
    {
        conv.decodeGPS((uint8_t*)msgs[i], &eph);
    }
    EXPECT_TRUE(finished);
}

TEST(NavParser, ParseGLONASS)
{
    uint32_t msg[86][5] = {
        {0xd000406, 0x20b04, 0x7a7d18e5, 0x29d01307, 0xac846000},
        {0xd000406, 0x20b04, 0x10df00df, 0x496534e2, 0x7b4d7000},
        {0xd000406, 0x20b04, 0x1c00b17c, 0x54680c74, 0x4d3ce800},
        {0xd000406, 0x20b04, 0x24a4eab3, 0x1334, 0x448da800},
        {0xd000406, 0x20b04, 0x2d110000, 0x1800, 0x2741000},
        {0xd000406, 0x20b04, 0x350c18e4, 0x97006e18, 0xf464000},
        {0xd000406, 0x20b04, 0x3c94730c, 0xe334c05e, 0x1610e000},
        {0xd000406, 0x20b04, 0x4514c6c2, 0xedf098d0, 0x3bca2000},
        {0xd000406, 0x20b04, 0x4ec27bc6, 0x5db4bfdc, 0x15c4a000},
        {0xd000406, 0x20b04, 0x5ea59463, 0xb2b4bfe4, 0x16570800},
        {0xd000406, 0x20b04, 0x6524a589, 0x5b808b44, 0x19287800},
        {0xd000406, 0x20b04, 0x6e75d813, 0x7ab4bfd0, 0xa676800},
        {0xd000406, 0x20b04, 0x752c11a5, 0xd720a7d0, 0x13eaa800},
        {0xd000406, 0x20b04, 0x7ec688a9, 0x73b4c004, 0xa12a000},
        {0xd000406, 0x20b04, 0x85f2c0c, 0x7bb4457c, 0x305e800},
        {0xd000406, 0x20b04, 0x1c00b17c, 0x54680c74, 0x4d3ce800},
        {0xd000406, 0x20b04, 0x2d110000, 0x1800, 0x2741000},
        {0xd000406, 0x20b04, 0x353453c7, 0xfb406c74, 0xfdff800},
        {0xd000406, 0x3e020b04, 0x3b766955, 0x6f74bfd0, 0x13c0c800},
        {0xd000406, 0x20b04, 0x45380de3, 0xa2108a34, 0x32993800},
        {0xd000406, 0x20b04, 0x4e9c11ee, 0xfaf4bfd3, 0x12568000},
        {0xd000406, 0x20b04, 0x554408ff, 0xb4608ae4, 0x40fce000},
        {0xd000406, 0x20b04, 0x5e11e285, 0xb0b4bff7, 0x12611800},
        {0xd000406, 0x20b04, 0x654c2b6e, 0x6f305ef8, 0x33c5c000},
        {0xd000406, 0x20b04, 0x75541d87, 0x5120c164, 0x3e3b2800},
        {0xd000406, 0x20b04, 0x7b99b390, 0x37f4c02b, 0x5962000},
        {0xd000406, 0x20b04, 0x85f340c, 0x7bb4457c, 0x3061800},
        {0xd000406, 0x20b04, 0x10df00df, 0x496534e2, 0x7b4d7000},
        {0xd000406, 0x20b04, 0x1c00b17c, 0x54680c74, 0x4d3ce800},
        {0xd000406, 0x20b04, 0x2d110000, 0x1800, 0x2741000},
        {0xd000406, 0x20b04, 0x355827a7, 0x9a607330, 0x286ca800},
        {0xd000406, 0x94020b04, 0x3f580c34, 0xfef4bfec, 0x4032000},
        {0xd000406, 0x20b04, 0x456417c4, 0x35c0a1ec, 0x24c68000},
        {0xd000406, 0x20b04, 0x4d954cca, 0xab74bfdf, 0x5f72000},
        {0xd000406, 0x20b04, 0x55680c0c, 0xbd509cbc, 0x1123f800},
        {0xd000406, 0x20b04, 0x5a75887b, 0xc934c052, 0xfe4b000},
        {0xd000406, 0x20b04, 0x6574150f, 0xe7508864, 0x14f3a000},
        {0xd000406, 0x20b04, 0x6eceb917, 0xe4b4bff3, 0xd929000},
        {0xd000406, 0x20b04, 0x757c372d, 0x7bb09bb8, 0x1b2d9800},
        {0xd000406, 0x20b04, 0x7e1619b3, 0xa974bfee, 0x8052000},
        {0xd000406, 0x20b04, 0x85f3c0c, 0x7bb4457c, 0x303e000},
        {0xd000406, 0x20b04, 0x1c00b17c, 0x54680c74, 0x4d3ce800},
        {0xd000406, 0x20b04, 0x3584034d, 0x7ce07858, 0x50b08800},
        {0xd000406, 0x20b04, 0x3858ca53, 0x4e74bf86, 0x7f07000},
        {0xd000406, 0x20b04, 0x458c804c, 0x8d70d7c0, 0x1902b000},
        {0xd000406, 0x20b04, 0x4fe1cab4, 0x434bff4, 0x9646d800},
        {0xd000406, 0x20b04, 0x65981614, 0xee510340, 0x7953800},
        {0xd000406, 0x20b04, 0x7c04648e, 0xa8b4c015, 0x9824f000},
        {0xd000406, 0x20b04, 0x85f440c, 0x7bb4457c, 0x3041800},
        {0xd000406, 0x20b04, 0x10df00df, 0x496534e2, 0x7b4d7000},
        {0xd000406, 0x20b04, 0x1800b17c, 0x54680c74, 0x4d3e9000},
        {0xd000406, 0x20b04, 0x35a81ec3, 0xe640e9b4, 0x12801000},
        {0xd000406, 0x20b04, 0x4809f8eb, 0x1bb4bf80, 0x97d58000},
        {0xd000406, 0x20b04, 0x65c05e67, 0xee50fa04, 0x2629a800},
        {0xd000406, 0x20b04, 0x69a90a26, 0x6274bf7e, 0x96204800},
        {0xd000406, 0x20b04, 0x74a90a4b, 0x2fd83037, 0x6fb88800},
        {0xd000406, 0x20b04, 0x7a7d18e5, 0x29d01307, 0xac846000},
        {0xd000406, 0x73020b04, 0x85f4c0c, 0x7bb4457c, 0x301e000},
        {0xd000406, 0x20b04, 0x10df00df, 0x496534e2, 0x7b4d7000},
        {0xd000406, 0x20b04, 0x1c00b17c, 0x54680c74, 0x4d3ce800},
        {0xd000406, 0x20b04, 0x24a4eab3, 0x1334, 0x448da800},
        {0xd000406, 0x20b04, 0x2d110000, 0x1800, 0x2741000},
        {0xd000406, 0x20b04, 0x350c18e4, 0x97006e18, 0xf464000},
        {0xd000406, 0x20b04, 0x3c94730c, 0xe334c05e, 0x1610e000},
        {0xd000406, 0x20b04, 0x4514c6c2, 0xedf098d0, 0x3bca2000},
        {0xd000406, 0x20b04, 0x4ec27bc6, 0x5db4bfdc, 0x15c4a000},
        {0xd000406, 0x20b04, 0x551800a4, 0xbb208980, 0x3a5dd000},
        {0xd000406, 0x20b04, 0x5ea59463, 0xb2b4bfe4, 0x16570800},
        {0xd000406, 0x20b04, 0x6524a589, 0x5b808b44, 0x19287800},
        {0xd000406, 0x20b04, 0x752c11a5, 0xd720a7d0, 0x13eaa800},
        {0xd000406, 0x20b04, 0x7ec688a9, 0x73b4c004, 0xa12a000},
        {0xd000406, 0x20b04, 0x85f540c, 0x7bb4457c, 0x3021000},
        {0xd000406, 0x20b04, 0x10df00df, 0x496534e2, 0x7b4d7000},
        {0xd000406, 0x20b04, 0x1c00b17c, 0x54680c74, 0x4d3ce800},
        {0xd000406, 0x20b04, 0x24a4eab3, 0x1334, 0x448da800},
        {0xd000406, 0x20b04, 0x2d110000, 0x1800, 0x2741000},
        {0xd000406, 0x20b04, 0x353453c7, 0xfb406c74, 0xfdff800},
        {0xd000406, 0x20b04, 0x45380de3, 0xa2108a34, 0x32993800},
        {0xd000406, 0x20b04, 0x4e9c11ee, 0xfaf4bfd3, 0x12568000},
        {0xd000406, 0x20b04, 0x654c2b6e, 0x6f305ef8, 0x33c5c000},
        {0xd000406, 0x20b04, 0x6f8aeb08, 0x8c74bed3, 0x5e59800},
        {0xd000406, 0x20b04, 0x7b99b390, 0x37f4c02b, 0x5962000},
        {0xd000406, 0x20b04, 0x85f5c0c, 0x7bb4457c, 0x307e800},
        {0xd000406, 0x20b04, 0x10df00df, 0x496534e2, 0x7b4d7000},
        {0xd000406, 0x20b04, 0x1c00b17c, 0x54680c74, 0x4d3ce800},
        {0xd000406, 0x20b04, 0x355827a7, 0x9a607330, 0x286ca800},
    };

    GlonassEphemeris geph;
    NavParser conv;
    conv.setGPSTime(UTCTime(1563660018));

    bool finished = false;
    conv.registerCallback([&finished](const GlonassEphemeris& geph) {
        finished = true;
        EXPECT_EQ(geph.sat, 4);
        EXPECT_EQ(geph.gnssID, ublox::GnssID_Glonass);
        EXPECT_EQ(geph.iode, 95);
        EXPECT_EQ(geph.frq, 6);
        EXPECT_EQ(geph.svh, 0);
        EXPECT_EQ(geph.sva, 3);
        EXPECT_EQ(geph.age, 0);
        EXPECT_EQ(geph.toe.sec, 1563655500);
        EXPECT_EQ(geph.toe.nsec, 0);
        EXPECT_EQ(geph.tof.sec, 1563655950);
        EXPECT_EQ(geph.tof.nsec, 0);
        EXPECT_NEAR(geph.pos[0], 5616046.875, 1e-3);
        EXPECT_NEAR(geph.pos[1], -21385926.26953125, 1e-8);
        EXPECT_NEAR(geph.pos[2], 12753206.54296875, 1e-8);
        EXPECT_NEAR(geph.vel[0], -97.52464294433594, 1e-14);
        EXPECT_NEAR(geph.vel[1], 1744.4267272949219, 1e-13);
        EXPECT_NEAR(geph.vel[2], 2971.3258743286133, 1e-13);
        EXPECT_NEAR(geph.acc[0], -9.313225746154785e-07, 1e-21);
        EXPECT_NEAR(geph.acc[1], -3.725290298461914e-06, 1e-20);
        EXPECT_NEAR(geph.acc[2], 0.0, 1e-25);
        EXPECT_NEAR(geph.taun, -0.000314553268254, 1e-15);
        EXPECT_NEAR(geph.gamn, 9.09494701773e-13, 1e-25);
        EXPECT_NEAR(geph.dtaun, -2.79396772385e-09, 1e-20);
    });

    for (int i = 0; i < 61; i++)
    {
        ublox::RXM_SFRBX_t* p = (ublox::RXM_SFRBX_t*)msg[i];
        conv.decodeGlonass(*p, geph);
    }
    EXPECT_TRUE(finished);
}

TEST(NavParser, Galileo)
{
    // clang-format off
    std::array<std::array<uint32_t, 9>, 23> buffer = {{
        {0x050da0ec, 0x08907cff, 0x30083c86, 0xc84a8000, 0xaaaa8000, 0x0000002a, 0xaaaabe26, 0x99ff4000, 0x00000001},
        {0x06000000, 0x01ffffff, 0x12481e89, 0xe2484000, 0xac868000, 0x0000002a, 0xaaaa7377, 0xc83f4000, 0xbfe93aa9},
        {0x09a9ca55, 0x55555555, 0x551e0060, 0x254f8000, 0xb60f8000, 0x0000002a, 0xaaaa9517, 0x66ff4000, 0x00000001},
        {0x09a9ca55, 0x55555555, 0x55420040, 0x1d5e0000, 0xa20f8000, 0x0000002a, 0xaaaa7b85, 0x273f4000, 0xbfe93aa9},
        {0x0aa03c4f, 0xe2f98403, 0xa40000ff, 0xeb008000, 0xb4878000, 0x0000002a, 0xaaaaad73, 0x72bf4000, 0x00000001},
        {0x0aa03c4f, 0xe22b2dfe, 0x120000ff, 0xeb008000, 0xb4878000, 0x0000002a, 0xaaaa7c86, 0x49ff4000, 0xbfe93aa9},
        {0x00955555, 0x55555555, 0x55555555, 0x50790000, 0x8d920000, 0x0000002a, 0xaaaa88af, 0x2aff4000, 0x00000001},
        {0x00955555, 0x55555555, 0x55555555, 0x50790000, 0x8d924000, 0x0000002a, 0xaaaa74d6, 0xd47f4000, 0xbfe93aa9},
        {0x00955555, 0x55555555, 0x55555555, 0x50790000, 0x8d928000, 0x0000002a, 0xaaaa9783, 0x123f4000, 0x00000001},
        {0x00955555, 0x55555555, 0x55555555, 0x50790000, 0x8d92c000, 0x0000002a, 0xaaaa6bfa, 0xecbf4000, 0xbfe93aa9},
        {0x00955555, 0x55555555, 0x55555555, 0x50790000, 0x8d930000, 0x0000002a, 0xaaaab6f7, 0x5b7f4000, 0x00000001},
        {0x00955555, 0x55555555, 0x55555555, 0x50790000, 0x8d934000, 0x0000002a, 0xaaaa4a8e, 0xa5ff4000, 0xbfe93aa9},
        {0x00955555, 0x55555555, 0x55555555, 0x50790000, 0x8d938000, 0x0000002a, 0xaaaaa9db, 0x63bf4000, 0x00000001},
        {0x00955555, 0x55555555, 0x55555555, 0x50790000, 0x8d93c000, 0x0000002a, 0xaaaa55a2, 0x9d3f4000, 0xbfe93aa9},
        {0x00955555, 0x55555555, 0x55555555, 0x50790000, 0x8d940000, 0x0000002a, 0xaaaaafcb, 0x4cbf4000, 0x00000001},
        {0x00955555, 0x55555555, 0x55555555, 0x50790000, 0x8d944000, 0x0000002a, 0xaaaa53b2, 0xb23f4000, 0xbfe93aa9},
        {0x02129669, 0xae0bc9c4, 0x5dd9d3ba, 0xbdf34000, 0x80ff4000, 0x0000002a, 0xaaaaa8ce, 0xe73f4000, 0x00000001},
        {0x011291e4, 0x22f62ac5, 0x000a6328, 0xaa04c000, 0xb613c000, 0x0000002a, 0xaaaa7516, 0xd8bf4000, 0xbfe93aa9},
        {0x04129300, 0x08001b47, 0x93fff99d, 0x08000000, 0x83804000, 0x0000002a, 0xaaaabbfe, 0x02ff4000, 0x00000001},
        {0x0312bff1, 0x0f48273e, 0xe3465882, 0x703e8000, 0x88dac000, 0x0000002a, 0xaaaa6e90, 0x377f4000, 0xbfe93aa9},
        {0x06000000, 0x01ffffff, 0x12481e89, 0xe2484000, 0xacac8000, 0x0000002a, 0xaaaa8d8c, 0x387f4000, 0x00000001},
        {0x050da0ec, 0x08907cff, 0x30083c86, 0xcaea8000, 0xaaaa8000, 0x0000002a, 0xaaaa7e1f, 0x5f7f4000, 0xbfe93aa9},
        {0x00955555, 0x55555555, 0x55555555, 0x50790000, 0x8d960000, 0x0000002a, 0xaaaab2e8, 0x917f4000, 0x00000001}
    }};
    // clang-format on

    Ephemeris eph;
    eph.gnssID = ublox::GnssID_Galileo;
    eph.sat = 19;
    NavParser parser;
    for (const auto& buf : buffer)
    {
        parser.decodeGalileo((uint8_t*)buf.data(), &eph);
    }

    EXPECT_EQ(eph.gnssID, ublox::GnssID_Galileo);
    EXPECT_EQ(eph.sat, 19);

    EXPECT_EQ(eph.toe.sec, 1573014018);  // toe 2019/11/6 4:20:18
    EXPECT_EQ(eph.toc.sec, 1573014018);  // toc 2019/11/6 4:20:18

    EXPECT_EQ(eph.toe.nsec, 0);
    EXPECT_EQ(eph.toc.nsec, 0);

    EXPECT_EQ(eph.iodc, 74);
    EXPECT_EQ(eph.iode, 74);
    EXPECT_EQ(eph.week, 2078);
    EXPECT_EQ(eph.toes, 274800);
    EXPECT_EQ(eph.tocs, 0);
    EXPECT_EQ(eph.health, 0);
    EXPECT_EQ(eph.alert_flag, 0);
    EXPECT_EQ(eph.anti_spoof, 0);
    EXPECT_EQ(eph.code_on_L2, 0);
    EXPECT_EQ(eph.ura, 0);
    EXPECT_EQ(eph.L2_P_data_flag, 0);
    EXPECT_EQ(eph.fit_interval_flag, 0);
    EXPECT_EQ(eph.age_of_data_offset, 0);
    EXPECT_NEAR(eph.tgd[0], -5.82077e-09, 1e-15);
    EXPECT_NEAR(eph.tgd[1], -6.0536e-09, 1e-15);
    EXPECT_NEAR(eph.tgd[2], 0, 1e-15);
    EXPECT_NEAR(eph.tgd[3], 0, 1e-15);
    EXPECT_NEAR(eph.af2, 0, 1e-9);
    EXPECT_NEAR(eph.af1, 1.98952e-13, 1e-17);
    EXPECT_NEAR(eph.af0, -3.04537e-06, 1e-12);
    EXPECT_NEAR(eph.m0, 0.858087, 1e-6);
    EXPECT_NEAR(eph.delta_n, 2.98155e-09, 1e-15);
    EXPECT_NEAR(eph.ecc, 7.9249e-05, 1e-9);
    EXPECT_NEAR(eph.sqrta, 5440.62, 1e-2);
    EXPECT_NEAR(eph.omega0, 2.20037, 1e-5);
    EXPECT_NEAR(eph.i0, 0.958879, 1e-6);
    EXPECT_NEAR(eph.w, 1.93694, 1e-6);
    EXPECT_NEAR(eph.omegadot, -5.46416e-09, 1e-15);
    EXPECT_NEAR(eph.idot, 9.10752e-11, 1e-17);
    EXPECT_NEAR(eph.cuc, -2.12155e-06, 1e-12);
    EXPECT_NEAR(eph.cus, 1.21035e-05, 1e-11);
    EXPECT_NEAR(eph.crc, 78, 1e-4);
    EXPECT_NEAR(eph.crs, -46.9062, 1e-4);
    EXPECT_NEAR(eph.cic, 1.49012e-08, 1e-14);
    EXPECT_NEAR(eph.cis, 5.02914e-08, 1e-14);
}

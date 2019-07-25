#include <vector>
#include <ctime>
#include <cmath>
#include <cstring>

#include "UBLOX/parsers/nav.h"

void NavParser::registerCallback(eph_cb cb)
{
    eph_callbacks.push_back(cb);
}

void NavParser::registerCallback(geph_cb cb)
{
    geph_callbacks.push_back(cb);
}

bool NavParser::convertUBX(const ublox::RXM_SFRBX_t &msg)
{
    using namespace ublox;
    if (msg.gnssId == GnssID_Glonass)
    {
        GlonassEphemeris *geph = nullptr;
        for (int i = 0; i < geph_.size(); i++)
        {
            if (geph_[i].sat == msg.svId)
                geph = &geph_[i];
        }
        if (!geph)
        {
            geph_.emplace_back();
            geph = &geph_.back();
        }
        return decodeGlonass(msg, *geph);
    }
    else
    {
        Ephemeris *eph = nullptr;
        for (int i = 0; i < eph_.size(); i++)
        {
            if (eph_[i].sat == msg.svId)
                eph = &eph_[i];
        }
        if (!eph)
        {
            eph_.emplace_back();
            eph = &eph_.back();
        }

        eph->sat = msg.svId;

        switch (msg.gnssId)
        {
        case GnssID_GPS:
            return decodeGPS((uint8_t *)msg.dwrd, eph);
        case GnssID_Galileo:
            return decodeGalileo((uint8_t *)msg.dwrd, eph);
        case GnssID_Beidou:
            return decodeBeidou((uint8_t*)msg.dwrd, eph);
        }
    }
}

static unsigned int U4(const unsigned char *p)
{
    unsigned int u;
    memcpy(&u, p, 4);
    return u;
}

extern void setbitu(unsigned char *buff, int pos, int len, unsigned int data)
{
    unsigned int mask = 1u << (len - 1);
    int i;
    if (len <= 0 || 32 < len)
        return;
    for (i = pos; i < pos + len; i++, mask >>= 1)
    {
        if (data & mask)
            buff[i / 8] |= 1u << (7 - i % 8);
        else
            buff[i / 8] &= ~(1u << (7 - i % 8));
    }
}
extern void setbits(unsigned char *buff, int pos, int len, int data)
{
    if (data < 0)
        data |= 1 << (len - 1);
    else
        data &= ~(1 << (len - 1)); /* set sign bit */
    setbitu(buff, pos, len, (unsigned int)data);
}

extern unsigned int getbitu(const unsigned char *buff, int pos, int len)
{
    unsigned int bits = 0;
    int i;
    for (i = pos; i < pos + len; i++)
        bits = (bits << 1) + ((buff[i / 8] >> (7 - i % 8)) & 1u);
    return bits;
}
extern int getbits(const unsigned char *buff, int pos, int len)
{
    unsigned int bits = getbitu(buff, pos, len);
    if (len <= 0 || 32 <= len || !(bits & (1u << (len - 1))))
        return (int)bits;
    return (int)(bits | (~0u << len)); /* extend sign */
}

static double getbitg(const unsigned char *buff, int pos, int len)
{
    double value = getbitu(buff, pos + 1, len - 1);
    return getbitu(buff, pos, 1) ? -value : value;
}

bool NavParser::decodeGPSSubframe1(const uint8_t *const buff, Ephemeris *eph)
{
    double tow;
    int i = 48, week, tgd;
    tow = getbitu(buff, 24, 17) * 6.0;
    week = getbitu(buff, i, 10);
    i += 10;
    eph->code_on_L2 = getbitu(buff, i, 2);
    i += 2;
    eph->ura = getbitu(buff, i, 4);
    i += 4;
    eph->health = getbitu(buff, i, 6);
    i += 6;
    uint8_t iodc0 = getbitu(buff, i, 2);
    i += 2;
    eph->L2_P_data_flag = getbitu(buff, i, 1);
    i += 1 + 87;
    tgd = getbits(buff, i, 8);
    i += 8;
    eph->iode = getbitu(buff, i, 8);
    i += 8;
    eph->tocs = getbitu(buff, i, 16) * 16.0;
    i += 16;
    eph->af2 = getbits(buff, i, 8) * P2_55;
    i += 8;
    eph->af1 = getbits(buff, i, 16) * P2_43;
    i += 16;
    eph->af0 = getbits(buff, i, 22) * P2_31;

    eph->tgd[0] = tgd == -128 ? 0.0 : tgd * P2_31; /* ref [4] */
    eph->iodc = (iodc0 << 8) + eph->iode;
    eph->week = week + UTCTime::GPS_WEEK_ROLLOVER;

    eph->iode1 = eph->iode;
    eph->got_subframe1 = true;

    return true;
}

bool NavParser::decodeGPSSubframe2(const unsigned char *buff, Ephemeris *eph)
{
    int i = 48;
    eph->iode = getbitu(buff, i, 8);
    i += 8;
    eph->crs = getbits(buff, i, 16) * P2_5;
    i += 16;
    eph->delta_n = getbits(buff, i, 16) * P2_43 * PI;
    i += 16;
    eph->m0 = getbits(buff, i, 32) * P2_31 * PI;
    i += 32;
    eph->cuc = getbits(buff, i, 16) * P2_29;
    i += 16;
    eph->ecc = getbitu(buff, i, 32) * P2_33;
    i += 32;
    eph->cus = getbits(buff, i, 16) * P2_29;
    i += 16;
    eph->sqrta = getbitu(buff, i, 32) * P2_19;
    i += 32;
    eph->toes = getbitu(buff, i, 16) * 16.0;
    i += 16;
    eph->fit_interval_flag = getbitu(buff, i, 1) ? 0.0 : 4.0; /* 0:4hr,1:>4hr */

    eph->iode2 = eph->iode;
    eph->got_subframe2 = true;

    return true;
}

bool NavParser::decodeGPSSubframe3(const unsigned char *buff, Ephemeris *eph)
{
    int i = 48;
    eph->cic = getbits(buff, i, 16) * P2_29;
    i += 16;
    eph->omega0 = getbits(buff, i, 32) * P2_31 * PI;
    i += 32;
    eph->cis = getbits(buff, i, 16) * P2_29;
    i += 16;
    eph->i0 = getbits(buff, i, 32) * P2_31 * PI;
    i += 32;
    eph->crc = getbits(buff, i, 16) * P2_5;
    i += 16;
    eph->w = getbits(buff, i, 32) * P2_31 * PI;
    i += 32;
    eph->omegadot = getbits(buff, i, 24) * P2_43 * PI;
    i += 24;
    eph->iode3 = getbitu(buff, i, 8);
    i += 8;
    eph->idot = getbits(buff, i, 14) * P2_43 * PI;

    eph->iode3 = eph->iode;
    eph->got_subframe3 = true;

    return true;
}

bool NavParser::decodeGPS(const uint8_t *buf, Ephemeris *eph)
{
    unsigned int words[10];
    int i, id;
    const uint8_t *p = buf;

    for (i = 0; i < 10; i++, p += 4)
        words[i] = U4(p) >> 6; /* 24 bits without parity */

    id = (words[1] >> 2) & 7;
    if (id < 1 || 5 < id)
    {
        return -1;
    }

    uint8_t subfrm[30];
    for (i = 0; i < 10; i++)
    {
        setbitu(subfrm, i * 24, 24, words[i]);
    }
    switch (id)
    {
    case 1:
        decodeGPSSubframe1(subfrm, eph);
        break;
    case 2:
        decodeGPSSubframe2(subfrm, eph);
        break;
    case 3:
        decodeGPSSubframe3(subfrm, eph);
        break;
    default:
        return false;
    }

    // Check that the iodes are the same across the subframes
    if (eph->got_subframe1 && eph->got_subframe2 && eph->got_subframe3 &&
        eph->iode1 == eph->iode2 && eph->iode1 == eph->iode3 &&
        eph->iode == (eph->iodc & 0xFF))
    {
        // Set toe and toc
        eph->toe = UTCTime::fromGPS(eph->week, eph->toes*1000);
        eph->toc = UTCTime::fromGPS(eph->week, eph->tocs*1000);
        GPS_time_ = eph->toe;

        for (auto &cb : eph_callbacks)
            cb(*eph);

        eph->got_subframe1 = false;
        eph->got_subframe2 = false;
        eph->got_subframe3 = false;
    }

    return 0;
}

int test_glostr(const unsigned char *buff)
{
    static const unsigned char xor_8bit[256] = {/* xor of 8 bits */
                                                0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
                                                1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
                                                1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
                                                0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
                                                1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
                                                0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
                                                0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
                                                1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0};
    static const unsigned char mask_hamming[][12] = {/* mask of hamming codes */
                                                     {0x55, 0x55, 0x5A, 0xAA, 0xAA, 0xAA, 0xB5, 0x55, 0x6A, 0xD8, 0x08},
                                                     {0x66, 0x66, 0x6C, 0xCC, 0xCC, 0xCC, 0xD9, 0x99, 0xB3, 0x68, 0x10},
                                                     {0x87, 0x87, 0x8F, 0x0F, 0x0F, 0x0F, 0x1E, 0x1E, 0x3C, 0x70, 0x20},
                                                     {0x07, 0xF8, 0x0F, 0xF0, 0x0F, 0xF0, 0x1F, 0xE0, 0x3F, 0x80, 0x40},
                                                     {0xF8, 0x00, 0x0F, 0xFF, 0xF0, 0x00, 0x1F, 0xFF, 0xC0, 0x00, 0x80},
                                                     {0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x01, 0x00},
                                                     {0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00},
                                                     {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8}};
    unsigned char cs;
    int i, j, n = 0;

    for (i = 0; i < 8; i++)
    {
        for (j = 0, cs = 0; j < 11; j++)
        {
            cs ^= xor_8bit[buff[j] & mask_hamming[i][j]];
        }
        if (cs)
            n++;
    }
    return n == 0 || (n == 2 && cs);
}

bool NavParser::decodeGlonassString(const unsigned char *buff, GlonassEphemeris *geph)
{
    double tow, tod, tof, toe;
    int P, P1, P2, P3, P4, tk_h, tk_m, tk_s, tb, ln, NT, slot, M, week;
    int i = 1, frn1, frn2, frn3, frn4;

    // trace(3, "decode_glostr:\n");

    /* frame 1 */
    frn1 = getbitu(buff, i, 4);
    i += 4 + 2;
    P1 = getbitu(buff, i, 2);
    i += 2;
    tk_h = getbitu(buff, i, 5);
    i += 5;
    tk_m = getbitu(buff, i, 6);
    i += 6;
    tk_s = getbitu(buff, i, 1) * 30;
    i += 1;
    geph->vel[0] = getbitg(buff, i, 24) * P2_20 * 1E3;
    i += 24;
    geph->acc[0] = getbitg(buff, i, 5) * P2_30 * 1E3;
    i += 5;
    geph->pos[0] = getbitg(buff, i, 27) * P2_11 * 1E3;
    i += 27 + 4;

    /* frame 2 */
    frn2 = getbitu(buff, i, 4);
    i += 4;
    geph->svh = getbitu(buff, i, 3);
    i += 3;
    P2 = getbitu(buff, i, 1);
    i += 1;
    tb = getbitu(buff, i, 7);
    geph->iode = tb;
    i += 7 + 5;
    geph->vel[1] = getbitg(buff, i, 24) * P2_20 * 1E3;
    i += 24;
    geph->acc[1] = getbitg(buff, i, 5) * P2_30 * 1E3;
    i += 5;
    geph->pos[1] = getbitg(buff, i, 27) * P2_11 * 1E3;
    i += 27 + 4;

    /* frame 3 */
    frn3 = getbitu(buff, i, 4);
    i += 4;
    P3 = getbitu(buff, i, 1);
    i += 1;
    geph->gamn = getbitg(buff, i, 11) * P2_40;
    i += 11 + 1;
    P = getbitu(buff, i, 2);
    i += 2;
    ln = getbitu(buff, i, 1);
    i += 1;
    geph->vel[2] = getbitg(buff, i, 24) * P2_20 * 1E3;
    i += 24;
    geph->acc[2] = getbitg(buff, i, 5) * P2_30 * 1E3;
    i += 5;
    geph->pos[2] = getbitg(buff, i, 27) * P2_11 * 1E3;
    i += 27 + 4;

    /* frame 4 */
    frn4 = getbitu(buff, i, 4);
    i += 4;
    geph->taun = getbitg(buff, i, 22) * P2_30;
    i += 22;
    geph->dtaun = getbitg(buff, i, 5) * P2_30;
    i += 5;
    geph->age = getbitu(buff, i, 5);
    i += 5 + 14;
    P4 = getbitu(buff, i, 1);
    i += 1;
    geph->sva = getbitu(buff, i, 4);
    i += 4 + 3;
    NT = getbitu(buff, i, 11);
    i += 11;
    slot = getbitu(buff, i, 5);
    i += 5;
    M = getbitu(buff, i, 2);

    if (frn1 != 1 || frn2 != 2 || frn3 != 3 || frn4 != 4)
    {
        // trace(3, "decode_glostr error: frn=%d %d %d %d %d\n", frn1, frn2, frn3, frn4);
        return false;
    }

    // Convert Time into UTC, using the last GPS time stamp to figure
    // out what day it is.
    int frame_tod_ms = (tk_h * 3600 + tk_m * 60 + tk_s)*1000;
    int dbg = frame_tod_ms / UTCTime::SEC_IN_DAY;
    geph->tof = UTCTime::fromGlonassTimeOfDay(GPS_time_, frame_tod_ms);
    uint64_t eph_tod_ms = (tb * 60 * 15) * 1000;
    geph->toe = UTCTime::fromGlonassTimeOfDay(GPS_time_, eph_tod_ms);

    // Initialize prev_gal_...
//    if (prev_gal_tof == UTCTime{0, 0})
//        prev_gal_tof = geph->tof;
//    if (prev_gal_toe == UTCTime{0, 0})
//        prev_gal_toe = geph->toe;

    // Handle The day wrap, because we are comparing GPS with Gal
    // to get the beginning of the day, we might have the wrong
    // day.  This assumes that we get at least two GLONASS ephemeris
    // message per day, which is probably okay
    if ((GPS_time_ - geph->toe).toSec() > UTCTime::SEC_IN_DAY/2)
        geph->toe += (int)(UTCTime::SEC_IN_DAY);
    else if ((GPS_time_ - geph->toe).toSec() < -(UTCTime::SEC_IN_DAY/2))
        geph->toe -= (int)(UTCTime::SEC_IN_DAY);

    if ((GPS_time_ - geph->tof).toSec() > UTCTime::SEC_IN_DAY/2)
        geph->tof += (int)(UTCTime::SEC_IN_DAY);
    else if ((GPS_time_ - geph->tof).toSec() < - (UTCTime::SEC_IN_DAY/2))
        geph->tof -= (int)(UTCTime::SEC_IN_DAY);

//    prev_gal_tof = geph->tof;
//    prev_gal_toe = geph->toe;
    return true;
}

#include <fstream>


bool NavParser::decodeGlonass(const ublox::RXM_SFRBX_t &msg, GlonassEphemeris &geph)
{
    if (msg.svId == 4)
    {
        static std::ofstream file("geph.txt");

        const uint32_t* p = (const uint32_t*)&msg;
        file << "{";
        for (int i = 0; i < 1+msg.numWords; i++)
        {
            file << "0x" << std::hex << *p << ", ";
            p++;
        }
        file << "},\n";
        file.flush();
        int debug = 1;
    }
    int i, j, k, m, prn;
    unsigned char *p = (uint8_t*)msg.dwrd, *fid;
    unsigned char buff[64];

    if (msg.svId == 255)
        return false; // svId==255 means UBLOX doesn't know who this data came from
    if (GPS_time_ == UTCTime{0, 0})
        return false; // We use the GPS week count to calculate the GLONASS time

    prn = msg.svId;
    int sat = msg.svId;
    // satsys(sat, &prn);

    int m1 = getbitu(p, 25, 4);

    // if (raw->len < 24 + off)
    // {
    //     trace(2, "ubx rawsfrbx gnav length error: len=%d\n", raw->len);
    //     return -1;
    // }
    for (i = k = 0; i < 4; i++, p += 4)
        for (j = 0; j < 4; j++)
        {
            buff[k++] = p[3 - j];
        }
    /* test hamming of glonass string */
    if (!test_glostr(buff))
    {
        // trace(2, "ubx rawsfrbx glo string hamming error: sat=%2d\n", sat);
        return -1;
    }
    m = getbitu(buff, 1, 4);
    if (m < 1 || 15 < m)
    {
        // trace(2, "ubx rawsfrbx glo string no error: sat=%2d\n", sat);
        return -1;
    }
    /* flush frame buffer if frame-id changed */
    fid = subfrm_[sat - 1] + 150;
    if (fid[0] != buff[12] || fid[1] != buff[13])
    {
        for (i = 0; i < 4; i++)
            memset(subfrm_[sat - 1] + i * 10, 0, 10);
        memcpy(fid, buff + 12, 2); /* save frame-id */
    }
    memcpy(subfrm_[sat - 1] + (m - 1) * 10, buff, 10);

    if (m != 4)
        return 0;

    // /* decode glonass ephemeris strings */
    // geph.tof = raw->time;
    if (!decodeGlonassString(subfrm_[sat - 1], &geph))
        return 0;
    geph.frq = msg.freqId - 7;
    geph.sat = msg.svId;
    geph.gnssID = msg.gnssId;

    // if (!strstr(raw->opt, "-EPHALL"))
    // {
    //     if (geph.iode == raw->nav.geph[prn - 1].iode)
    //         return 0; /* unchanged */
    // }
    // raw->nav.geph[prn - 1] = geph;
    // raw->ephsat = sat;
    for (auto& cb : geph_callbacks)
        cb(geph);

    return true;
}

bool NavParser::decodeBeidou(const uint8_t *const buf, Ephemeris *eph)
{
    // TODO
}
bool NavParser::decodeGalileo(const uint8_t *const buf, Ephemeris *eph)
{
    // TODO
}

#include "UBLOX/eph.h"


bool EphConverter::convertUBX(const ublox::RXM_SFRBX_t& msg, Eph& eph)
{
    eph.sat = msg.svId;
    using namespace ublox;
    switch (msg.gnssId)
    {
    case GnssID_GPS:     return decodeGPS((uint8_t*)msg.dwrd, eph);
    case GnssID_Galileo: return decodeGalileo((uint8_t*)msg.dwrd, eph);
    }
}

#include <cstring>
static unsigned int   U4(const unsigned char *p)
{
  unsigned int u;
  memcpy(&u,p,4);
  return u;
}

extern void setbitu(unsigned char *buff, int pos, int len, unsigned int data)
{
    unsigned int mask=1u<<(len-1);
    int i;
    if (len<=0||32<len) return;
    for (i=pos;i<pos+len;i++,mask>>=1) {
        if (data&mask) buff[i/8]|=1u<<(7-i%8); else buff[i/8]&=~(1u<<(7-i%8));
    }
}

bool EphConverter::decodeGPS(const uint8_t * const buf, Eph &eph)
{
    // Chop parity bits and extra two bits of padding off
    //
    // MSB--------------|---------------------------------------------------|---------------LSB
    // |  Pad (2-bits)  |                    Data (24-bits)                 | Parity (6 bits) |
    // |----------------|---------------------------------------------------|-----------------|
    //

    uint8_t no_parity[30];
    const uint8_t* p = buf;
    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 3; j++)
            no_parity[3*i+j] = (p[j] << 2) & 0xFFF | (p[j+1] >> 6);
        p += 4;
    }
    uint32_t tow = getBitsPage<0,17,1>(no_parity);
    eph.alert_flag  = getBitsPage<17,1,1>(no_parity);
    eph.anti_spoof  = getBitsPage<18,1,1>(no_parity);
    int8_t subframe_id = getBitsPage<19,3,1>(no_parity);

    uint8_t subframe_id2 = getBits<19,3,BitOrder::BE>(no_parity+3);
    uint32_t tow2 = (getBits<0, 16, BitOrder::BE>(no_parity+3));

    printSubframe(subframe_id, no_parity);

    switch (subframe_id)
    {
        case 0: return false;
        case 1: return decodeGPSSubframe1(no_parity, eph);
        case 2: return decodeGPSSubframe2(no_parity, eph);
        case 3: return decodeGPSSubframe3(no_parity, eph);
        case 4: return decodeGPSSubframe4(no_parity, eph);
        case 5: return decodeGPSSubframe5(no_parity, eph);
        case 6: return false;
        case 7: return false;
        default: break; // not supported
    }
}

bool EphConverter::decodeGalileo(const uint8_t * const buf, Eph &eph)
{
    uint8_t part1 = getBits<0,1>(buf);
    uint8_t page1 = getBits<1,1>(buf);
    uint8_t part2 = getBits<16,1>(buf);
    uint8_t page2 = getBits<17,1>(buf);

    // skip alert page
    if (page1 == 1 || page2 == 1) return false;

    // Test even-odd parts
    if (part1!=0||part2!=1)  return false;

    uint8_t type = getBits<2,6>(buf);
    if (type > 6) return false; // unsupported messages

    uint8_t page_type[5];
    uint8_t iod[4];

    int i = 0;
    page_type[0] = getBits<6>(buf, i);  i += 6+2+88;
    eph.week = getBits<12>(buf, i);     i += 12;
    eph.tow = getBits<20>(buf, i);

    i = 128;
    page_type[1] = getBits<6>(buf, i);
    iod[0] = getBits<10>(buf, i);



}

bool EphConverter::decodeGPSSubframe1(const uint8_t * const buf, Eph &eph)
{
    eph.tow = getBits<31,17>(buf);
    eph.alert_flag = getBits<48,1>(buf);
    eph.anti_spoof = getBits<49,1>(buf);
    eph.week = getBits<61,10>(buf);
    eph.code_on_L2 = getBits<71,2>(buf);
    eph.ura = getBits<73,4>(buf);
    eph.health = getBits<77,6>(buf);
    uint16_t iodc_MSB = getBits<83,2>(buf);
    eph.L2_P_data_flag = getBits<91,1>(buf);
    eph.tgd = getBits<197,8>(buf) * P2_31;
    uint16_t iodc_LSB = getBits<211,8>(buf);
    eph.iodc = (iodc_MSB << 8 | iodc_LSB); // May not be right
    eph.toc = getBits<219,16>(buf) * 16.0;
    eph.af2 = getBits<241,8>(buf) * P2_55;
    eph.af1 = getBits<249,16>(buf) * P2_43;
    eph.af0 = getBits<271,22>(buf) * P2_31;

    eph.got_subframe1 = true;
    eph.iode1 = (uint8_t)eph.week;
}


#include <stdio.h>
bool EphConverter::decodeGPSSubframe2(const uint8_t * const buf, Eph &eph)
{
    eph.iode = getBitsPage<0,8,2>(buf);
    eph.crs = getSignedBitsPage<8,16,2>(buf) * P2_5;
    eph.delta_n = getSignedBitsPage<0,16,3>(buf) * P2_43 * PI;
    eph.m0 = getSignedBitsPage<16,32,3>(buf) * P2_31 * PI;
    eph.cuc = getSignedBitsPage<0,16,5>(buf) * P2_29;
    eph.ecc = getBitsPage<16,32,5>(buf) * P2_33;
    eph.cus = getSignedBitsPage<0,16,7>(buf) * P2_29;
    eph.sqrta = getBitsPage<16,32,7>(buf) *P2_19;
    eph.toe = getBitsPage<0,16,9>(buf)*16.0;

    eph.got_subframe2 = true;
    eph.iode2 = getBitsPage<0,8,2>(buf);
}


bool EphConverter::decodeGPSSubframe3(const uint8_t * const buf, Eph &eph)
{
    eph.cic = getSignedBitsPage<0,16,2>(buf) * P2_29;
    eph.omega0 = getSignedBitsPage<16,32,2>(buf) * P2_31 * PI;
    eph.cis = getSignedBitsPage<0,16,4>(buf) * P2_29;
    eph.i0 = getSignedBitsPage<16,32,4>(buf) * P2_31 * PI;
    eph.crc = getSignedBitsPage<0,16,6>(buf) * P2_5;
    eph.w = getSignedBitsPage<16,32,6>(buf) * P2_31 * PI;
    eph.omegadot = getSignedBitsPage<0,24,8>(buf) * P2_43 * PI;
    eph.iode = getBitsPage<0,8,9>(buf);
    eph.idot = getBitsPage<8,14,9>(buf) * PI * P2_43;

    eph.got_subframe3 = true;
    eph.iode3 = eph.iode;
}

bool EphConverter::decodeGPSSubframe4(const uint8_t * const buf, Eph &eph)
{
    return false;
}

bool EphConverter::decodeGPSSubframe5(const uint8_t * const buf, Eph &eph)
{
    return false;
}

void EphConverter::printSubframe(int frame, const uint8_t *buf)
{
    printf("const uint8_t sf%d_example[30] = {", frame);
    for (int i = 0; i < 30; i++)
    {
        if (i % 3 == 0)
           printf("\n\t");
        printf("0x%x, ", buf[i]);
    }
    printf("\n};\n");
    fflush(stdout);
}

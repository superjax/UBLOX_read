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

TEST (Eph, getBitsAlignedLittleEndian)
{
    union
    {
        uint32_t words[1];
        uint8_t bytes[4];
    } thing;

    for (int i = 0; i < 1; i++)
    {
        thing.words[i] = i * 0x01234567;
    }

    EXPECT_EQ((getBits<0, 16>(thing.bytes)), (thing.bytes[0] << 8 | thing.bytes[1]));
    EXPECT_EQ((getBits<16,16>(thing.bytes)), (thing.bytes[2] << 8 | thing.bytes[3]));
}

TEST (Eph, getBitsAlignedBigEndian)
{
    union
    {
        uint32_t words[1];
        uint8_t bytes[4];
    } thing;

    for (int i = 0; i < 1; i++)
    {
        thing.words[i] = i * 0x01234567;
    }

    EXPECT_EQ((getBits<0, 16,BitOrder::BE>(thing.bytes)), (thing.bytes[1] << 8 | thing.bytes[0]));
    EXPECT_EQ((getBits<16,16,BitOrder::BE>(thing.bytes)), (thing.bytes[3] << 8 | thing.bytes[2]));
}

TEST (Eph, getBytes)
{
    union
    {
        uint32_t words[4];
        uint8_t bytes[16];
    } thing;

    for (int i = 0; i < 4; i++)
    {
        thing.words[i] = i * 0x01234567;
    }

    EXPECT_EQ((getBits<0,8>(thing.bytes)), thing.bytes[0]);
    EXPECT_EQ((getBits<8,8>(thing.bytes)), thing.bytes[1]);
    EXPECT_EQ((getBits<16,8>(thing.bytes)), thing.bytes[2]);
    EXPECT_EQ((getBits<24,8>(thing.bytes)), thing.bytes[3]);

    EXPECT_EQ((getBits<32,8,BitOrder::LE>(thing.bytes)), thing.bytes[4]);
    EXPECT_EQ((getBits<40,8,BitOrder::LE>(thing.bytes)), thing.bytes[5]);
    EXPECT_EQ((getBits<48,8,BitOrder::LE>(thing.bytes)), thing.bytes[6]);
    EXPECT_EQ((getBits<56,8,BitOrder::LE>(thing.bytes)), thing.bytes[7]);

    EXPECT_EQ((getBits<64,8,BitOrder::BE>(thing.bytes)), thing.bytes[8]);
    EXPECT_EQ((getBits<72,8,BitOrder::BE>(thing.bytes)), thing.bytes[9]);
    EXPECT_EQ((getBits<80,8,BitOrder::BE>(thing.bytes)), thing.bytes[10]);
    EXPECT_EQ((getBits<88,8,BitOrder::BE>(thing.bytes)), thing.bytes[11]);

    EXPECT_EQ((getBits<96,8>(thing.bytes)), thing.bytes[12]);
    EXPECT_EQ((getBits<104,8>(thing.bytes)), thing.bytes[13]);
    EXPECT_EQ((getBits<112,8>(thing.bytes)), thing.bytes[14]);
    EXPECT_EQ((getBits<120,8>(thing.bytes)), thing.bytes[15]);
}

TEST (Eph, getBytesMisAligned)
{
    union
    {
        uint32_t words[4];
        uint8_t bytes[16];
    } thing;

    for (int i = 0; i < 4; i++)
    {
        thing.words[i] = i * 0x01234567;
    }

    EXPECT_EQ((getBits<10,3>(thing.bytes)), (thing.bytes[1] >> 8) & 0x07);

    EXPECT_EQ((getBits<34,3>(thing.bytes)), (thing.words[1] >> 2) & 0x07);
}

TEST (Eph, getBitsMisAligned)
{
    union
    {
        uint32_t word;
        uint8_t bytes[4];
    } thing;

    thing.bytes[0] = 0x01;
    thing.bytes[1] = 0x23;
    thing.bytes[2] = 0x45;
    thing.bytes[3] = 0x67;

    EXPECT_EQ((getBits<4,8,BitOrder::BE>(thing.bytes)), 0x12);
    EXPECT_EQ((getBits<4,16,BitOrder::BE>(thing.bytes)), 0x1234);
    EXPECT_EQ((getBits<12,8,BitOrder::BE>(thing.bytes)), 0x34);
    EXPECT_EQ((getBits<12,16,BitOrder::BE>(thing.bytes)), 0x3456);
    EXPECT_EQ((getBits<16,16,BitOrder::BE>(thing.bytes)), 0x4567);
    EXPECT_EQ((getBits<4,24,BitOrder::BE>(thing.bytes)), 0x123456);
    EXPECT_EQ((getBits<4,12,BitOrder::BE>(thing.bytes)), 0x123);
    EXPECT_EQ((getBits<12,12,BitOrder::BE>(thing.bytes)), 0x345);
}

/* extract unsigned/signed bits ------------------------------------------------
* extract unsigned/signed bits from byte data
* args   : unsigned char *buff I byte data
*          int    pos    I      bit position from start of data (bits)
*          int    len    I      bit length (bits) (len<=32)
* return : extracted unsigned/signed bits
*-----------------------------------------------------------------------------*/
static unsigned int getbitu(const unsigned char *buff, int pos, int len)
{
    unsigned int bits=0;
    int i;
    for (i=pos;i<pos+len;i++) bits=(bits<<1)+((buff[i/8]>>(7-i%8))&1u);
    return bits;
}
static int getbits(const unsigned char *buff, int pos, int len)
{
    unsigned int bits=getbitu(buff,pos,len);
    if (len<=0||32<=len||!(bits&(1u<<(len-1)))) return (int)bits;
    return (int)(bits|(~0u<<len)); /* extend sign */
}

TEST (Eph, getBitsRTKLib)
{
    union
    {
        uint32_t word;
        uint8_t bytes[4];
    } thing;

    thing.bytes[0] = 0x01;
    thing.bytes[1] = 0x23;
    thing.bytes[2] = 0x45;
    thing.bytes[3] = 0x67;

    EXPECT_EQ((getBits<4,8,BitOrder::BE>(thing.bytes)), getbitu(thing.bytes, 4,8));
    EXPECT_EQ((getBits<12,12,BitOrder::BE>(thing.bytes)), getbitu(thing.bytes, 12,12));
}

#define P2_5        0.03125             /* 2^-5 */
#define P2_6        0.015625            /* 2^-6 */
#define P2_11       4.882812500000000E-04 /* 2^-11 */
#define P2_15       3.051757812500000E-05 /* 2^-15 */
#define P2_17       7.629394531250000E-06 /* 2^-17 */
#define P2_19       1.907348632812500E-06 /* 2^-19 */
#define P2_20       9.536743164062500E-07 /* 2^-20 */
#define P2_21       4.768371582031250E-07 /* 2^-21 */
#define P2_23       1.192092895507810E-07 /* 2^-23 */
#define P2_24       5.960464477539063E-08 /* 2^-24 */
#define P2_27       7.450580596923828E-09 /* 2^-27 */
#define P2_29       1.862645149230957E-09 /* 2^-29 */
#define P2_30       9.313225746154785E-10 /* 2^-30 */
#define P2_31       4.656612873077393E-10 /* 2^-31 */
#define P2_32       2.328306436538696E-10 /* 2^-32 */
#define P2_33       1.164153218269348E-10 /* 2^-33 */
#define P2_35       2.910383045673370E-11 /* 2^-35 */
#define P2_38       3.637978807091710E-12 /* 2^-38 */
#define P2_39       1.818989403545856E-12 /* 2^-39 */
#define P2_40       9.094947017729280E-13 /* 2^-40 */
#define P2_43       1.136868377216160E-13 /* 2^-43 */
#define P2_48       3.552713678800501E-15 /* 2^-48 */
#define P2_50       8.881784197001252E-16 /* 2^-50 */
#define P2_55       2.775557561562891E-17 /* 2^-55 */
#define SC2RAD      3.1415926535898     /* semi-circle to radian (IS-GPS) */
static int decode_subfrm2(const unsigned char *buff, Eph *eph)
{
    double sqrtA;
    int i=48;

    eph->iode=getbitu(buff,i, 8);              i+= 8;
    eph->crs =getbits(buff,i,16)*P2_5;         i+=16;
    eph->delta_n=getbits(buff,i,16)*P2_43*SC2RAD; i+=16;
    eph->m0  =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
    eph->cuc =getbits(buff,i,16)*P2_29;        i+=16;
    eph->ecc   =getbitu(buff,i,32)*P2_33;        i+=32;
    eph->cus =getbits(buff,i,16)*P2_29;        i+=16;
    eph->sqrta    =getbitu(buff,i,32)*P2_19;        i+=32;
    eph->toe=getbitu(buff,i,16)*16.0;         i+=16;
//    eph->fit =getbitu(buff,i, 1)?0.0:4.0; /* 0:4hr,1:>4hr */
    return 2;
}

static int decode_subfrm3(const unsigned char *buff, Eph *eph)
{
    double tow,toc;
    int i=48,iode;

//    trace(4,"decode_subfrm3:\n");
//    trace(5,"decode_subfrm3: buff="); traceb(5,buff,30);

    eph->cic =getbits(buff,i,16)*P2_29;        i+=16;
    eph->omega0=getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
    eph->cis =getbits(buff,i,16)*P2_29;        i+=16;
    eph->i0  =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
    eph->crc =getbits(buff,i,16)*P2_5;         i+=16;
    eph->w =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
    eph->omegadot=getbits(buff,i,24)*P2_43*SC2RAD; i+=24;
    eph->iode3     =getbitu(buff,i, 8);              i+= 8;
    eph->idot=getbits(buff,i,14)*P2_43*SC2RAD;

    /* check iode and iodc consistency */
//    if (iode!=eph->iode||iode!=(eph->iodc&0xFF)) return 0;

    /* adjustment for week handover */
//    tow=time2gpst(eph->ttr,&eph->week);
//    toc=time2gpst(eph->toc,NULL);
//    if      (eph->toes<tow-302400.0) {eph->week++; tow-=604800.0;}
//    else if (eph->toes>tow+302400.0) {eph->week--; tow+=604800.0;}
//    eph->toe=gpst2time(eph->week,eph->toes);
//    eph->toc=gpst2time(eph->week,toc);
//    eph->ttr=gpst2time(eph->week,tow);

	return 3;
}

TEST (Eph, parseSubframe2)
{
	const uint8_t sf2_example[30] = {
		0xc4, 0x3b, 0x0,
		0xcf, 0xad, 0x2a,
		0xcb, 0x8f, 0xfc,
		0x3b, 0x7c, 0xea,
		0x60, 0x24, 0x16,
		0x2a, 0x3e, 0xec,
		0xf1, 0x1e, 0x5a,
		0x39, 0xf7, 0xb0,
		0x22, 0x3, 0xa6,
		0xf0, 0x4d, 0xf8,
	};

	Eph eph;
	Eph eph_rtklib;

	EphConverter conv;
	conv.decodeGPSSubframe2(sf2_example, eph);
	decode_subfrm2(sf2_example, &eph_rtklib);

    EXPECT_FLOAT_EQ(eph.iode, eph_rtklib.iode);
    EXPECT_FLOAT_EQ(eph.crs, eph_rtklib.crs);
    EXPECT_FLOAT_EQ(eph.delta_n, eph_rtklib.delta_n);
    EXPECT_FLOAT_EQ(eph.m0, eph_rtklib.m0);
    EXPECT_FLOAT_EQ(eph.cuc, eph_rtklib.cuc);
    EXPECT_FLOAT_EQ(eph.ecc, eph_rtklib.ecc);
    EXPECT_FLOAT_EQ(eph.cus, eph_rtklib.cus);
    EXPECT_FLOAT_EQ(eph.sqrta, eph_rtklib.sqrta);
    EXPECT_FLOAT_EQ(eph.toe, eph_rtklib.toe);
}

TEST (Eph, parseSubframe3)
{
	const uint8_t sf3_example[30] = {
		0xc4, 0x3b, 0x0,
		0x2c, 0x31, 0x2e,
		0x1a, 0xad, 0x9c,
		0x7f, 0xf9, 0x7e,
		0x6f, 0xdb, 0xcc,
		0x57, 0x24, 0xf8,
		0x17, 0x6e, 0x2e,
		0x16, 0x5, 0x96,
		0xfd, 0xe1, 0x6a,
		0x22, 0x60, 0x38,
	};

	Eph eph;
	Eph eph_rtklib;

	EphConverter conv;
	conv.decodeGPSSubframe3(sf3_example, eph);
	decode_subfrm3(sf3_example, &eph_rtklib);

    EXPECT_FLOAT_EQ(eph.cic, eph_rtklib.cic);
    EXPECT_FLOAT_EQ(eph.omega0, eph_rtklib.omega0);
    EXPECT_FLOAT_EQ(eph.cis, eph_rtklib.cis);
    EXPECT_FLOAT_EQ(eph.i0, eph_rtklib.i0);
    EXPECT_FLOAT_EQ(eph.crc, eph_rtklib.crc);
    EXPECT_FLOAT_EQ(eph.w, eph_rtklib.w);
    EXPECT_FLOAT_EQ(eph.omega0, eph_rtklib.omega0);
    EXPECT_FLOAT_EQ(eph.iode3, eph_rtklib.iode3);
    EXPECT_FLOAT_EQ(eph.idot, eph_rtklib.idot);
}

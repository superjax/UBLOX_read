#include <vector>

#include "UBLOX/eph.h"


bool EphConverter::convertUBX(const ublox::RXM_SFRBX_t& msg, std::vector<Eph>& eph_vec)
{
    Eph* eph = nullptr;
    for (int i = 0; i < eph_vec.size(); i++)
    {
        if (eph_vec[i].sat == msg.svId)
            eph = &eph_vec[i];
    }
    if (!eph)
    {
        eph_vec.emplace_back();
        eph = &eph_vec.back();
    }

    eph->sat = msg.svId;
    using namespace ublox;
    switch (msg.gnssId)
    {
    case GnssID_GPS:     return decodeGPS((uint8_t*)msg.dwrd, eph);
//    case GnssID_Galileo: return decodeGalileo((uint8_t*)msg.dwrd, eph);
    }
}

#include <time.h>
#include <ctime>
#include <cmath>

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
extern void setbits(unsigned char *buff, int pos, int len, int data)
{
    if (data<0) data|=1<<(len-1); else data&=~(1<<(len-1)); /* set sign bit */
    setbitu(buff,pos,len,(unsigned int)data);
}

extern unsigned int getbitu(const unsigned char *buff, int pos, int len)
{
    unsigned int bits=0;
    int i;
    for (i=pos;i<pos+len;i++) bits=(bits<<1)+((buff[i/8]>>(7-i%8))&1u);
    return bits;
}
extern int getbits(const unsigned char *buff, int pos, int len)
{
    unsigned int bits=getbitu(buff,pos,len);
    if (len<=0||32<=len||!(bits&(1u<<(len-1)))) return (int)bits;
    return (int)(bits|(~0u<<len)); /* extend sign */
}



bool EphConverter::decodeGPSSubframe1(const uint8_t *const buff, Eph *eph)
{
    double tow;
    int i=48,week,tgd;
    tow = getbitu(buff,24,17)*6.0;
    week = getbitu(buff,i,10);                  i+=10;
    eph->code_on_L2 = getbitu(buff,i, 2);       i+= 2;
    eph->ura = getbitu(buff,i, 4);              i+= 4;
    eph->health = getbitu(buff,i, 6);           i+= 6;
    uint8_t iodc0 = getbitu(buff,i, 2);         i+= 2;
    eph->L2_P_data_flag = getbitu(buff,i, 1);   i+= 1+87;
    tgd = getbits(buff,i, 8);                   i+= 8;
    eph->iode = getbitu(buff,i, 8);         i+= 8;
    eph->toc = getbitu(buff,i,16)*16.0;         i+=16;
    eph->af2 = getbits(buff,i, 8)*P2_55;        i+= 8;
    eph->af1 = getbits(buff,i,16)*P2_43;        i+=16;
    eph->af0 = getbits(buff,i,22)*P2_31;

    eph->tgd = tgd == -128?0.0 : tgd*P2_31; /* ref [4] */
    eph->iodc = (iodc0 << 8) + eph->iode;
    eph->week = week;

    eph->iode1 = eph->iode;
    eph->got_subframe1 = true;
//    eph->ttr=gpst2time(eph->week,tow);

    return true;
}

bool EphConverter::decodeGPSSubframe2(const unsigned char *buff, Eph *eph)
{
    int i=48;
    eph->iode = getbitu(buff,i, 8);                 i+= 8;
    eph->crs = getbits(buff,i,16)*P2_5;             i+=16;
    eph->delta_n = getbits(buff,i,16)*P2_43*PI;     i+=16;
    eph->m0  = getbits(buff,i,32)*P2_31*PI;         i+=32;
    eph->cuc = getbits(buff,i,16)*P2_29;            i+=16;
    eph->ecc = getbitu(buff,i,32)*P2_33;            i+=32;
    eph->cus = getbits(buff,i,16)*P2_29;            i+=16;
    eph->sqrta = getbitu(buff,i,32)*P2_19;          i+=32;
    eph->toe = getbitu(buff,i,16)*16.0;             i+=16;
    eph->fit_interval_flag = getbitu(buff,i, 1)?0.0:4.0; /* 0:4hr,1:>4hr */

    eph->iode2 = eph->iode;
    eph->got_subframe2 = true;

    return 2;
}


bool EphConverter::decodeGPSSubframe3(const unsigned char *buff, Eph *eph)
{
    int i=48;
    eph->cic = getbits(buff,i,16)*P2_29;            i+=16;
    eph->omega0 = getbits(buff,i,32)*P2_31*PI;      i+=32;
    eph->cis = getbits(buff,i,16)*P2_29;            i+=16;
    eph->i0  = getbits(buff,i,32)*P2_31*PI;         i+=32;
    eph->crc = getbits(buff,i,16)*P2_5;             i+=16;
    eph->w = getbits(buff,i,32)*P2_31*PI;           i+=32;
    eph->omegadot = getbits(buff,i,24)*P2_43*PI;    i+=24;
    eph->iode3 = getbitu(buff,i, 8);                i+= 8;
    eph->idot = getbits(buff,i,14)*P2_43*PI;

    eph->iode3 = eph->iode;
    eph->got_subframe3 = true;

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


bool EphConverter::decodeGPS(const uint8_t *buf, Eph* eph)
{
    unsigned int words[10];
    int i,id;
    const uint8_t *p=buf;

    for (i=0;i<10;i++,p+=4)
        words[i]=U4(p)>>6; /* 24 bits without parity */

    id=(words[1]>>2)&7;
    if (id<1||5<id)
    {
        return -1;
    }

    uint8_t subfrm[30];
    for (i=0;i<10;i++) {
        setbitu(subfrm,i*24,24,words[i]);
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
        return true;

    return 0;
}


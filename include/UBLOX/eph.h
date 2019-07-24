#pragma once

#include <cassert>
#include <vector>
#include <functional>
#include <type_traits>
#include "UBLOX/parsers/ubx_defs.h"
#include "UBLOX/utctime.h"

template <size_t len>
struct MinimumTypeHelper {
    typedef
    typename std::conditional<len == 0 , void,
    typename std::conditional<len <= 8 , uint8_t,
    typename std::conditional<len <= 16, uint16_t,
    typename std::conditional<len <= 32, uint32_t,
    typename std::conditional<len <= 64, uint64_t,
    void>::type>::type>::type>::type>::type type;
};

template <size_t len>
struct SignedMinimumTypeHelper {
    typedef
    typename std::conditional<len == 0 , void,
    typename std::conditional<len <= 8 , int8_t,
    typename std::conditional<len <= 16, int16_t,
    typename std::conditional<len <= 32, int32_t,
    typename std::conditional<len <= 64, int64_t,
    void>::type>::type>::type>::type>::type type;
};

class Ephemeris
{
public:
    UTCTime  toe;                //!< reference time ephemeris (UTC Time)                                           [s]
    UTCTime  toc;                //!< reference time (clock)   (UTC Time)                                           [s]

    uint8_t  sat;                //!< Sat ID
    uint16_t prn;                //!< GPS PRN number (helps with debugging)
    uint32_t tow;                //!< time of week in subframe1; the time of the leading bit edge of subframe 2     [s]
    uint16_t iodc;               //!< 10 bit issue of data (clock); 8 LSB bits will match the iode                  []
    uint8_t  iode;               //!< 8 bit  issue of data (ephemeris)                                              []
    uint16_t week;               //!< 10 bit gps week 0-1023 (user must account for week rollover )                 [week]
    uint8_t  health;             //!< 6 bit health parameter; 0 if healthy; unhealth othersize                      [0=healthy]
    uint8_t  alert_flag;         //!< 1 = URA may be worse than indicated                                           [0,1]
    uint8_t  anti_spoof;         //!< anti-spoof flag from 0=off; 1=on                                              [0,1]
    uint8_t  code_on_L2;         //!< 0=reserved; 1=P code on L2; 2=C/A on L2                                       [0,1,2]
    uint8_t  ura;                //!< User Range Accuracy lookup code; 0 is excellent; 15 is use at own risk        [0-15]; see p. 83 GPSICD200C
    uint8_t  L2_P_data_flag;     //!< flag indicating if P is on L2 1=true                                          [0,1]
    uint8_t  fit_interval_flag;  //!< fit interval flag (four hour interval or longer) 0=4 fours; 1=greater         [0,1]
    uint16_t age_of_data_offset; //!< age of data offset                                                            [s]
    double   tgd[4];                //!< group delay                                                                   [s]
    double   af2;                //!< polynomial clock correction coefficient (rate of clock drift)                 [s/s^2]
    double   af1;                //!< polynomial clock correction coefficient (clock drift)                         [s/s]
    double   af0;                //!< polynomial clock correction coefficient (clock bias)                          [s]
    double   m0;                 //!< mean anomaly at reference time                                                [rad]
    double   delta_n;            //!< mean motion difference from computed value                                    [rad/s]
    double   ecc;                //!< eccentricity                                                                  []
    double   sqrta;              //!< square root of the semi-major axis                                            [m^(1/2)]
    double   omega0;             //!< longitude of ascending node of orbit plane at weekly epoch                    [rad]
    double   i0;                 //!< inclination angle at reference time                                           [rad]
    double   w;                  //!< argument of perigee                                                           [rad]
    double   omegadot;           //!< rate of right ascension                                                       [rad/s]
    double   idot;               //!< rate of inclination angle                                                     [rad/s]
    double   cuc;                //!< amplitude of the cosine harmonic correction term to the argument of latitude  [rad]
    double   cus;                //!< amplitude of the sine harmonic correction term to the argument of latitude    [rad]
    double   crc;                //!< amplitude of the cosine harmonic correction term to the orbit radius          [m]
    double   crs;                //!< amplitude of the sine harmonic correction term to the orbit radius            [m]
    double   cic;                //!< amplitude of the cosine harmonic correction term to the angle of inclination  [rad]
    double   cis;                //!< amplitude of the sine harmonic correction term to the angle of inclination    [rad]

    // Synchronization Variables
    uint8_t iode1 = 0;
    uint8_t iode2 = 0;
    uint8_t iode3 = 0;
    bool got_subframe1 = false;
    bool got_subframe2 = false;
    bool got_subframe3 = false;
};

class GlonassEphemeris /* GLONASS broadcast ephemeris type */
{        
public:
    int sat;            /* satellite number */
    int iode;           /* IODE (0-6 bit of tb field) */
    int frq;            /* satellite frequency number */
    int svh,sva,age;    /* satellite health, accuracy, age of operation */
    UTCTime toe;        /* epoch of epherides (UTC) */
    UTCTime tof;        /* message frame time (UTC) */
    double pos[3];      /* satellite position (ecef) (m) */
    double vel[3];      /* satellite velocity (ecef) (m/s) */
    double acc[3];      /* satellite acceleration (ecef) (m/s^2) */
    double taun,gamn;   /* SV clock bias (s)/relative freq bias */
    double dtaun;       /* delay between L1 and L2 (s) */

    bool got_string1 = false;
    bool got_string2 = false;
    bool got_string3 = false;
    bool got_string4 = false;
};


class NavConverter
{
    std::vector<Ephemeris> eph_;
    std::vector<GlonassEphemeris> geph_;
public:
    static constexpr double PI =  3.1415926535897932;
    static constexpr double P2_5 =  0.03125;               /* 2^-5 */
    static constexpr double P2_11 = 4.882812500000000E-04;  /* 2^-11 */
    static constexpr double P2_19 = 1.907348632812500E-06;  /* 2^-19 */
    static constexpr double P2_20 = 9.536743164062500E-07; /* 2^-20 */
    static constexpr double P2_29 = 1.862645149230957E-09; /* 2^-29 */
    static constexpr double P2_30 = 9.313225746154785E-10; /* 2^-30 */
    static constexpr double P2_31 = 4.656612873077393E-10; /* 2^-31 */
    static constexpr double P2_32 = 2.328306436538696E-10; /* 2^-32 */
    static constexpr double P2_33 = 1.164153218269348E-10; /* 2^-33 */
    static constexpr double P2_34 = 5.820766091346740E-11; /* 2^-34 */
    static constexpr double P2_40 = 9.094947017729280E-13;  /* 2^-40 */
    static constexpr double P2_43 = 1.136868377216160E-13;  /* 2^-43 */
    static constexpr double P2_46 = 1.421085471520200E-14; /* 2^-46 */
    static constexpr double P2_55 = 2.775557561562891E-17; /* 2^-55 */
    static constexpr double P2_59 = 1.734723475976810E-18; /* 2^-59 */
    bool convertUBX(const ublox::RXM_SFRBX_t& msg);

    bool decodeGPS(const uint8_t *const buf, Ephemeris* eph);
    bool decodeGPSSubframe1(const uint8_t *const buf, Ephemeris* eph);
    bool decodeGPSSubframe2(const uint8_t *const buf, Ephemeris* eph);
    bool decodeGPSSubframe3(const uint8_t *const buf, Ephemeris* eph);
    bool decodeGPSSubframe4(const uint8_t *const buf, Ephemeris* eph);
    bool decodeGPSSubframe5(const uint8_t *const buf, Ephemeris* eph);

    bool decodeGalileo(const uint8_t *const buf, Ephemeris* eph);
    bool decodeBeidou(const uint8_t *const buf, Ephemeris* eph);

    bool decodeGlonass(const ublox::RXM_SFRBX_t &msg, GlonassEphemeris &eph);
    bool decodeGlonassString(const unsigned char *buff, GlonassEphemeris *geph);
    // bool decodeGlonassString2(const unsigned char *buff, GlonassEphemeris *geph);
    // bool decodeGlonassString3(const unsigned char *buff, GlonassEphemeris *geph);
    // bool decodeGlonassString4(const unsigned char *buff, GlonassEphemeris *geph);
    // bool decodeGlonassString5(const unsigned char *buff, GlonassEphemeris *geph);

    void printSubframe(int frame, const uint8_t *buf);

    typedef std::function<void(const Ephemeris& eph)> eph_cb;
    typedef std::function<void(const GlonassEphemeris& eph)> geph_cb;

    std::vector<eph_cb> eph_callbacks;
    std::vector<geph_cb> geph_callbacks;

    void registerCallback(eph_cb cb);
    void registerCallback(geph_cb cb);
};



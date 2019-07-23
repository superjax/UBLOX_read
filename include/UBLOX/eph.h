#pragma once

#include <cassert>
#include <vector>
#include <type_traits>
#include "UBLOX/parsers/ubx_defs.h"

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

class Eph
{
public:
    uint8_t sat;                 //!< Sat ID
    uint16_t prn;                //!< GPS PRN number (helps with debugging)
    uint32_t tow;                //!< time of week in subframe1; the time of the leading bit edge of subframe 2     [s]
    uint16_t iodc;               //!< 10 bit issue of data (clock); 8 LSB bits will match the iode                  []
    uint8_t  iode;               //!< 8 bit  issue of data (ephemeris)                                              []
    uint32_t toe;                //!< reference time ephemeris (0-604800)                                           [s]
    uint32_t toc;                //!< reference time (clock)   (0-604800)                                           [s]
    uint16_t week;               //!< 10 bit gps week 0-1023 (user must account for week rollover )                 [week]
    uint8_t  health;             //!< 6 bit health parameter; 0 if healthy; unhealth othersize                      [0=healthy]
    uint8_t  alert_flag;         //!< 1 = URA may be worse than indicated                                           [0,1]
    uint8_t  anti_spoof;         //!< anti-spoof flag from 0=off; 1=on                                              [0,1]
    uint8_t  code_on_L2;         //!< 0=reserved; 1=P code on L2; 2=C/A on L2                                       [0,1,2]
    uint8_t  ura;                //!< User Range Accuracy lookup code; 0 is excellent; 15 is use at own risk        [0-15]; see p. 83 GPSICD200C
    uint8_t  L2_P_data_flag;     //!< flag indicating if P is on L2 1=true                                          [0,1]
    uint8_t  fit_interval_flag;  //!< fit interval flag (four hour interval or longer) 0=4 fours; 1=greater         [0,1]
    uint16_t age_of_data_offset; //!< age of data offset                                                            [s]
    double   tgd;                //!< group delay                                                                   [s]
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

    uint8_t iode1;
    uint8_t iode2;
    uint8_t iode3;
    bool got_subframe1;
    bool got_subframe2;
    bool got_subframe3;
};

class EphConverter
{
public:
    static constexpr double PI =  3.1415926535897932;
    static constexpr double P2_5 =  0.03125;               /* 2^-5 */
    static constexpr double P2_19 = 1.907348632812500E-06; /* 2^-19 */
    static constexpr double P2_29 = 1.862645149230957E-09; /* 2^-29 */
    static constexpr double P2_31 = 4.656612873077393E-10; /* 2^-31 */
    static constexpr double P2_33 = 1.164153218269348E-10; /* 2^-33 */
    static constexpr double P2_43 = 1.136868377216160E-13; /* 2^-43 */
    static constexpr double P2_55 = 2.775557561562891E-17; /* 2^-55 */
    bool convertUBX(const ublox::RXM_SFRBX_t& msg, std::vector<Eph>& eph_vec);
    bool decodeGPS(const uint8_t *const buf, Eph* eph);
    bool decodeGalileo(const uint8_t *const buf, Eph& eph);
    bool decodeGPSSubframe1(const uint8_t *const buf, Eph* eph);
    bool decodeGPSSubframe2(const uint8_t *const buf, Eph* eph);
    bool decodeGPSSubframe3(const uint8_t *const buf, Eph* eph);
    bool decodeGPSSubframe4(const uint8_t *const buf, Eph* eph);
    bool decodeGPSSubframe5(const uint8_t *const buf, Eph* eph);


    void printSubframe(int frame, const uint8_t* buf);
};




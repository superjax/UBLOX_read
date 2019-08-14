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

#pragma once

#include <cassert>
#include <functional>
#include <type_traits>
#include <vector>

#include "UBLOX/parsers/ubx.h"
#include "UBLOX/parsers/ubx_defs.h"
#include "UBLOX/utctime.h"

class EphBase
{
public:
    uint8_t gnssID;
    uint8_t sat;

    std::string Type() const
    {
        switch (gnssID)
        {
        case ublox::GnssID_GPS:
            return "GPS";
        case ublox::GnssID_Galileo:
            return "Galileo";
        case ublox::GnssID_Glonass:
            return "Glonass";
        case ublox::GnssID_Qzss:
            return "Qzss";
        case ublox::GnssID_Beidou:
            return "Beidou";
        default:
            return "Unknown";
        }
    }
};

/* GPS, Galileo, QZSS, Beidou broadcast ephemeris type */
class Ephemeris : public EphBase
{
public:
    static constexpr double GPS_FREQL1 = 1.57542e9;         // (Hz)
    static constexpr double GPS_FREQL2 = 1.22760e9;         // (Hz)
    static constexpr double GALILEO_FREQL5a = 1.17645e9;    // (Hz)
    static constexpr double BEIDOU_FREQ_B1 = 1.561098e9;    // (Hz)
    static constexpr double BEIDOU_FREQ_B1_2 = 1.589742e9;  // (Hz)
    static constexpr double BEIDOU_FREQ_B2 = 1.20714e9;     // (Hz)

    UTCTime toe;  // reference time ephemeris (UTC Time) [s]
    UTCTime toc;  // reference time (clock)   (UTC Time) [s]

    uint32_t tow;   // time of week in subframe1; the time of the leading bit edge of subframe 2 [s]
    uint16_t iodc;  // 10 bit issue of data (clock); 8 LSB bits will match the iode []
    uint8_t iode;   // 8 bit  issue of data (ephemeris) []
    uint16_t week;  // 10 bit gps week 0-1023 (user must account for week rollover ) [week]
    uint32_t toes;  // Time of ephemeris (seconds part)
    uint32_t tocs;  // Time of clock (seconds part)
    uint8_t health;      // 6 bit health parameter; 0 if healthy; unhealth othersize [0=healthy]
    uint8_t alert_flag;  // 1 = URA may be worse than indicated [0,1]
    uint8_t anti_spoof;  // anti-spoof flag from 0=off; 1=on [0,1]
    uint8_t code_on_L2;  // 0=reserved; 1=P code on L2; 2=C/A on L2 [0,1,2]
    uint8_t ura;         // User Range Accuracy lookup code; 0 is excellent; 15 is use at own risk
                         // [0-15]; see p. 83 GPSICD200C
    uint8_t L2_P_data_flag;       // flag indicating if P is on L2 1=true [0,1]
    uint8_t fit_interval_flag;    // fit interval flag (four hour interval or longer) 0=4 fours;
                                  // 1=greater         [0,1]
    uint16_t age_of_data_offset;  // age of data offset [s]
    double tgd[4];                // group delay [s]
    double af2;       // polynomial clock correction coefficient (rate of clock drift) [s/s^2]
    double af1;       // polynomial clock correction coefficient (clock drift) [s/s]
    double af0;       // polynomial clock correction coefficient (clock bias) [s]
    double m0;        // mean anomaly at reference time [rad]
    double delta_n;   // mean motion difference from computed value [rad/s]
    double ecc;       // eccentricity []
    double sqrta;     // square root of the semi-major axis [m^(1/2)]
    double omega0;    // longitude of ascending node of orbit plane at weekly epoch [rad]
    double i0;        // inclination angle at reference time [rad]
    double w;         // argument of perigee [rad]
    double omegadot;  // rate of right ascension [rad/s]
    double idot;      // rate of inclination angle [rad/s]
    double cuc;  // amplitude of the cosine harmonic correction term to the argument of latitude
                 // [rad]
    double cus;  // amplitude of the sine harmonic correction term to the argument of latitude [rad]
    double crc;  // amplitude of the cosine harmonic correction term to the orbit radius [m]
    double crs;  // amplitude of the sine harmonic correction term to the orbit radius [m]
    double cic;  // amplitude of the cosine harmonic correction term to the angle of inclination
                 // [rad]
    double cis;  // amplitude of the sine harmonic correction term to the angle of inclination [rad]

    // Synchronization Variables
    uint8_t iode1 = 0;
    uint8_t iode2 = 0;
    uint8_t iode3 = 0;
    bool got_subframe1 = false;
    bool got_subframe2 = false;
    bool got_subframe3 = false;
};

/* GLONASS broadcast ephemeris type */
class GlonassEphemeris : public EphBase
{
public:
    static constexpr double FREQ1_GLO = 1.60200E9;   // GLONASS G1 base frequency (Hz)
    static constexpr double DFRQ1_GLO = 0.56250E6;   // GLONASS G1 bias frequency (Hz/n)
    static constexpr double FREQ2_GLO = 1.24600E9;   // GLONASS G2 base frequency (Hz)
    static constexpr double DFRQ2_GLO = 0.43750E6;   // GLONASS G2 bias frequency (Hz/n)
    static constexpr double FREQ3_GLO = 1.202025E9;  // GLONASS G3 frequency (Hz)
    int iode;                                        // IODE (0-6 bit of tb field)
    int frq;                                         // satellite frequency number
    int svh, sva, age;                               // satellite health, accuracy, age of operation
    UTCTime toe;                                     // epoch of epherides (UTC)
    UTCTime tof;                                     // message frame time (UTC)
    double pos[3];                                   // satellite position (ecef) (m)
    double vel[3];                                   // satellite velocity (ecef) (m/s)
    double acc[3];                                   // satellite acceleration (ecef) (m/s^2)
    double taun, gamn;                               // SV clock bias (s)/relative freq bias
    double dtaun;                                    // delay between L1 and L2 (s)

    // Synchronization Variables
    bool got_frame1 = false;
    bool got_frame2 = false;
    bool got_frame3 = false;
    bool got_frame4 = false;
    int time_mark = 0;
};

class NavParser : public ublox::UBXListener
{
public:
    static constexpr double PI = 3.1415926535897932;
    static constexpr double P2_5 = 0.03125;                 // 2^-5
    static constexpr double P2_11 = 4.882812500000000E-04;  // 2^-11
    static constexpr double P2_19 = 1.907348632812500E-06;  // 2^-19
    static constexpr double P2_20 = 9.536743164062500E-07;  // 2^-20
    static constexpr double P2_29 = 1.862645149230957E-09;  // 2^-29
    static constexpr double P2_30 = 9.313225746154785E-10;  // 2^-30
    static constexpr double P2_31 = 4.656612873077393E-10;  // 2^-31
    static constexpr double P2_32 = 2.328306436538696E-10;  // 2^-32
    static constexpr double P2_33 = 1.164153218269348E-10;  // 2^-33
    static constexpr double P2_34 = 5.820766091346740E-11;  // 2^-34
    static constexpr double P2_40 = 9.094947017729280E-13;  // 2^-40
    static constexpr double P2_43 = 1.136868377216160E-13;  // 2^-43
    static constexpr double P2_46 = 1.421085471520200E-14;  // 2^-46
    static constexpr double P2_55 = 2.775557561562891E-17;  // 2^-55
    static constexpr double P2_59 = 1.734723475976810E-18;  // 2^-59

    NavParser();
    void got_ubx(const uint8_t cls, const uint8_t id, const ublox::UBX_message_t &msg) override
    {
        assert(cls == ublox::CLASS_RXM && id == ublox::RXM_SFRBX);
        convertUBX(msg.RXM_SFRBX);
    }

    // Main entry point for the parser.  Feed it SFRBX messages, and it'll call the appropriate
    // callbacks when they are ready
    bool convertUBX(const ublox::RXM_SFRBX_t &msg);

    // Decodes GPS messages (called by convertUBX, but here for convenience)
    bool decodeGPS(const uint8_t *const buf, Ephemeris *eph);

    // Decodes GLONASS messages (called by convertUBX, but here for convenience)
    bool decodeGlonass(const ublox::RXM_SFRBX_t &msg, GlonassEphemeris &eph);
    bool decodeGalileo(const uint8_t *const buf, Ephemeris *eph);
    bool decodeBeidou(const uint8_t *const buf, Ephemeris *eph);

    typedef std::function<void(const Ephemeris &eph)> eph_cb;
    typedef std::function<void(const GlonassEphemeris &eph)> geph_cb;
    void registerCallback(eph_cb cb);
    void registerCallback(geph_cb cb);

    // GLONASS time is seeded with GPS time, so this function is here to help in the case that
    // not GPS messages are being received, but GLONASS is.
    void setGPSTime(const UTCTime &t) { GPS_time_ = t; }

private:
    bool decodeGPSSubframe1(const uint8_t *const buf, Ephemeris *eph);
    bool decodeGPSSubframe2(const uint8_t *const buf, Ephemeris *eph);
    bool decodeGPSSubframe3(const uint8_t *const buf, Ephemeris *eph);
    bool decodeGPSSubframe4(const uint8_t *const buf, Ephemeris *eph);
    bool decodeGPSSubframe5(const uint8_t *const buf, Ephemeris *eph);

    bool decodeGlonassString(const unsigned char *buff, GlonassEphemeris *geph);
    bool decodeGlonassFrame1(const unsigned char *buff, GlonassEphemeris *geph);
    bool decodeGlonassFrame2(const unsigned char *buff, GlonassEphemeris *geph);
    bool decodeGlonassFrame3(const unsigned char *buff, GlonassEphemeris *geph);
    bool decodeGlonassFrame4(const unsigned char *buff, GlonassEphemeris *geph);
    bool decodeGlonassFrame5(const unsigned char *buff, GlonassEphemeris *geph);

    bool decode_gal_inav(const unsigned char *buff, Ephemeris *eph);
    bool decodeGalileoSubframe0(const uint8_t *const buf, Ephemeris *eph);
    bool decodeGalileoSubframe1(const uint8_t *const buf, Ephemeris *eph);
    bool decodeGalileoSubframe2(const uint8_t *const buf, Ephemeris *eph);
    bool decodeGalileoSubframe3(const uint8_t *const buf, Ephemeris *eph);
    bool decodeGalileoSubframe4(const uint8_t *const buf, Ephemeris *eph);
    bool decodeGalileoSubframe5(const uint8_t *const buf, Ephemeris *eph);

    std::vector<eph_cb> eph_callbacks;
    std::vector<geph_cb> geph_callbacks;

    std::vector<Ephemeris> eph_;
    std::vector<GlonassEphemeris> geph_;

    UTCTime GPS_time_ = {0, 0};
    UTCTime prev_gal_toe = {0, 0};
    UTCTime prev_gal_tof = {0, 0};
};

#include <iostream>

inline std::ostream &operator<<(std::ostream &os, const Ephemeris &eph)
{
    os << "Constellation: " << eph.Type() << "\n";
    os << "Sat ID: " << (int)eph.sat << "\n\n";
    os << "toe " << eph.toe << " (" << eph.toe.sec << ", " << eph.toe.nsec << ")" << "\n";
    os << "toc " << eph.toc << " (" << eph.toc.sec << ", " << eph.toc.nsec << ")" << "\n";
    os << "tow " << (int)eph.tow << "\n";
    os << "iodc " << (int)eph.iodc << "\n";
    os << "iode " << (int)eph.iode << "\n";
    os << "week " << (int)eph.week << "\n";
    os << "toes " << (int)eph.toes << "\n";
    os << "tocs " << (int)eph.tocs << "\n";
    os << "health " << (int)eph.health << "\n";
    os << "alert_flag " << (int)eph.alert_flag << "\n";
    os << "anti_spoof " << (int)eph.anti_spoof << "\n";
    os << "code_on_L2 " << (int)eph.code_on_L2 << "\n";
    os << "ura " << (int)eph.ura << "\n";
    os << "L2_P_data_flag " << (int)eph.L2_P_data_flag << "\n";
    os << "fit_interval_flag " << (int)eph.fit_interval_flag << "\n";
    os << "age_of_data_offset " << (int)eph.age_of_data_offset << "\n";
    os << "tgd[0] " << eph.tgd[0] << "\n";
    os << "tgd[1] " << eph.tgd[1] << "\n";
    os << "tgd[2] " << eph.tgd[2] << "\n";
    os << "tgd[3] " << eph.tgd[3] << "\n";
    os << "af2 " << eph.af2 << "\n";
    os << "af1 " << eph.af1 << "\n";
    os << "af0 " << eph.af0 << "\n";
    os << "m0 " << eph.m0 << "\n";
    os << "delta_n " << eph.delta_n << "\n";
    os << "ecc " << eph.ecc << "\n";
    os << "sqrta " << eph.sqrta << "\n";
    os << "omega0 " << eph.omega0 << "\n";
    os << "i0 " << eph.i0 << "\n";
    os << "w " << eph.w << "\n";
    os << "omegadot " << eph.omegadot << "\n";
    os << "idot " << eph.idot << "\n";
    os << "cuc " << eph.cuc << "\n";
    os << "cus " << eph.cus << "\n";
    os << "crc " << eph.crc << "\n";
    os << "crs " << eph.crs << "\n";
    os << "cic " << eph.cic << "\n";
    os << "cis " << eph.cis << "\n";
}

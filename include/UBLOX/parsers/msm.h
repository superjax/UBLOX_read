#include <cstdint>
#include <type_traits>

#include "UBLOX/utctime.h"

namespace rtcm
{

enum {
    GPS,
    Galileo,
    Glonass,
    Beidou
};

struct MSM
{
    static constexpr double CLIGHT = 299792458.0;
    static constexpr double RANGE_MS = (CLIGHT*0.001);
    static constexpr double P2_10 = 0.0009765625;           // 2^-10
    static constexpr double P2_24 = 5.960464477539063E-08;  // 2^-24
    static constexpr double P2_29 = 1.862645149230957E-09;  // 2^-29
    static constexpr double P2_31 = 4.656612873077393E-10;  // 2^-31
    static constexpr double P2_34 =  5.820766091346740E-11; // 2^-34

    uint8_t* selfArray()
    {
        return (uint8_t*)this;
    }

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

    template<int len>
    typename MinimumTypeHelper<len>::type getBits(int start)
    {
        constexpr typename MinimumTypeHelper<len>::type mask = (len == 64) ? 0xFFFFFFFF : (1ul << len) - 1ul;
        uint8_t *p = selfArray() + start % 8;
        int i = 0;
        typename MinimumTypeHelper<len+8>::type ret = *p;
        while (i < len)
        {
            ret |= (*(++p) << (i+8));
            i += 8;
        }
        return ((ret >> (start % 8)) & mask);
    }

    void decodeTime(int constellation)
    {
        switch (constellation)
        {
        case GPS:
        case Galileo:
        {
            uint32_t tow = getBits<30>(48);
            t = UTCTime::fromGPS(UTCTime::now().GpsWeek(), tow);
            break;
        }
        case Glonass:
        {
            int dow = getBits<3>(48);
            int tod_ms = getBits<27>(51);
//            t = UTCTime::fromGlonass(dow, tod_ms);
            break;
        }
//        case Beidou:
//        {
//            uint32_t tow = getBits<30>(48);
//            t = UTCTime::fromBeidou(UTCTime::now().BeidouWeek(), tow);
//            break;
//        }
        }
    }

    virtual int decode() = 0;

    int decodeHeader(int constellation)
    {
        int i = 36;
        staid = getBits<12>(i); i += 12;
        decodeTime(constellation); i+=30;
        sync = getBits<1>(i); i+=1;
        iod = getBits<3>(i); i+=3;
        time_s = getBits<7>(i); i+=7;
        clk_str = getBits<2>(i); i+=2;
        clk_ext = getBits<2>(i); i+=2;
        smooth = getBits<1>(i); i+=1;
        tint_s = getBits<3>(i); i+=3;
        for (int j = 1; j <= 64; j++)
        {
            uint8_t mask = getBits<1>(i); i+=1;
            if (mask)
                sats[nsat++] = j;
        }
        for (int j = 1; j <= 32; j++)
        {
            uint8_t mask = getBits<1>(i); i+=1;
            if (mask)
                sigs[nsig++] = j;
        }
        /// TODO: check length of RTCM message
        if (nsat*nsig > 64)
            return -1;

        int ncell = 0;
        for (int j = 0; j < nsat*nsig; j++)
        {
            cellmask[j] = getBits<1>(i); i+=1;
            if (cellmask[j])
                ncell++;
        }
        return ncell;
    }
    UTCTime t;                      // time of measurement (in UTC)
    uint16_t staid;                 // Station Id
    uint8_t sync;                   //
    uint8_t iod;                    // Issue of data station
    uint8_t time_s;                 // cumulative session transmitting time
    uint8_t clk_str;                // clock steering indicato
    uint8_t clk_ext;                // external clock indicator
    uint8_t smooth;                 // divergence free smoothing indicator
    uint8_t tint_s;                 // soothing interval
    size_t nsig;                    // number of signals
    size_t nsat;                    // number of satellites
    uint8_t sigs[32];               // signals
    uint8_t sats[64];               // satellites
    uint64_t cellmask[64];          // cellmask
};

// 1074 GPS MSM4 Full GNSS Pseudoranges and PhaseRanges plus CNR
// 1084 GLONASS MSM4
// 1094 Galileo MSM4
// 1124 BeiDou MSM4
template <int Constellation>
struct MSM4 : MSM
{
    int decode() override
    {
        int ncell = decodeHeader(Constellation);
        tow = this->gnss_epoch_time * 0.001;

        if (ncell < 0)
            return -1;

        int i = 24 + 169*nsat*nsig;

        // Satellite data
        for (int j = 0; j < nsat; j++)
        {
            int rng = getBits<8>(i); i+=8;
            if (rng != 255)
                range[j] = rng * RANGE_MS;
        }
        for (int j = 0; j < nsat; j++)
        {
            int rng_m = getBits<10>(i); i+=10;
            if (range[j] != 0.0)
                range[j] += rng_m * P2_10 * RANGE_MS;
        }

        // Signal Data
        for (int j = 0; j < ncell; j++)
        {
            int pr = getBits<15>(i); i+=15;
            if (pr != -16384)
                pseudorange[j] = pr * P2_24 * RANGE_MS;
        }

        for (int j = 0; j < ncell; j++)
        {
            int cp = getBits<22>(i); i+=22;
            if (cp != -2097152)
                phaserange[j] = cp * P2_29 * RANGE_MS;
        }
        for (int j = 0; j < ncell; j++)
        {
            lock[j] = getBits<4>(i); i+=4;
        }
        for (int j = 0; j < ncell; j++)
        {
            half[j] = getBits<1>(i); i+=1;
        }
        for (int j = 0; j < ncell; j++)
        {
            cnr = getBits<6>(i)*1.0; i+=6;
        }
    }

    double tow;
    double range[64];
    double pseudorange[64];
    double phaserange[64];
    uint8_t lock[64];
    uint8_t half[64];
    double cnr[64];
};


// 1075 GPS MSM5 Full GNSS Pseudoranges, PhaseRanges, PhaseRangeRate and CNR
// 1085 GLONASS MSM5
// 1095 Galileo MSM5
// 1125 BeiDou MSM5
template <int Constellation>
struct MSM5 : public MSM
{
    int decode() override
    {
        int ncell = decodeHeader(Constellation);

        if (ncell < 0)
            return -1;

        int i = 24 + 169*nsat*nsig;

        // Satellite data
        for (int j = 0; j < nsat; j++)
        {
            int rng = getBits<8>(i); i+=8;
            if (rng != 255)
                range[j] = rng * RANGE_MS;
        }
        for (int j = 0; j < nsat; j++)
        {
            extended_info[j] = getBits<4>(i); i+=4;
        }
        for (int j = 0; j < nsat; j++)
        {
            int rng_m = getBits<10>(i); i+=10;
            if (range[j] != 0.0)
                range[j] += rng_m * P2_10 * RANGE_MS;
        }
        for (int j = 0; j < nsat; j++)
        {
            int rate = getBits<14>(i); i+= 14;
            if (rate != -8192)
                phaserangerates[j] = rate * 1.0;
        }

        // Signal Data
        for (int j = 0; j < ncell; j++)
        {
            int pr = getBits<15>(i); i+=15;
            if (pr != -16384)
                pseudorange[j] = pr * P2_24 * RANGE_MS;
        }

        for (int j = 0; j < ncell; j++)
        {
            int cp = getBits<22>(i); i+=22;
            if (cp != -2097152)
                phaserange[j] = cp * P2_29 * RANGE_MS;
        }
        for (int j = 0; j < ncell; j++)
        {
            lock[j] = getBits<4>(i); i+=4;
        }
        for (int j = 0; j < ncell; j++)
        {
            half[j] = getBits<1>(i); i+=1;
        }
        for (int j = 0; j < ncell; j++)
        {
            cnr[j] = getBits<6>(i)*1.0; i+=6;
        }
        for (int j = 0; j < ncell; j++)
        {
            int rrv = getBits<15>(i); i+=15;
            if (rrv != -16384)
                fine_phaserangerates[j] = rrv * 0.0001;
        }
    }

    double tow;
    uint8_t extended_info[64];
    double range[64];
    double pseudorange[64];
    double phaserange[64];
    double phaserangerates[64];
    double fine_phaserangerates[64];
    uint8_t lock[64];
    uint8_t half[64];
    double cnr[64];
};

// 1077 GPS MSM7 Full GNSS Pseudoranges, PhaseRanges, PhaseRangeRate and CNR (high resolution)
// 1087 GLONASS MSM7
// 1097 Galileo MSM7
// 1127 BeiDou MSM7
template <int Constellation>
struct MSM7 : public MSM
{
    int decode() override
    {
        int ncell = decodeHeader(Constellation);
        if (ncell < 0)
            return -1;

        int i = 24 + 169*nsat*nsig;

        // Satellite data
        for (int j = 0; j < nsat; j++)
        {
            int rng = getBits<8>(i); i+=8;
            if (rng != 255)
                range[j] = rng * RANGE_MS;
        }
        for (int j = 0; j < nsat; j++)
        {
            extended_info[j] = getBits<4>(i); i+=4;
        }
        for (int j = 0; j < nsat; j++)
        {
            int rng_m = getBits<10>(i); i+=10;
            if (range[j] != 0.0)
                range[j] += rng_m * P2_10 * RANGE_MS;
        }
        for (int j = 0; j < nsat; j++)
        {
            int rate = getBits<14>(i); i+= 14;
            if (rate != -8192)
                phaserangerates[j] = rate * 1.0;
        }

        // Signal Data
        for (int j = 0; j < ncell; j++)
        {
            int pr = getBits<20>(i); i+=20;
            if (pr != -524288)
                pseudorange[j] = pr * P2_29 * RANGE_MS;
        }

        for (int j = 0; j < ncell; j++)
        {
            int cp = getBits<24>(i); i+=24;
            if (cp != -8388608)
                phaserange[j] = cp * P2_31 * RANGE_MS;
        }
        for (int j = 0; j < ncell; j++)
        {
            lock[j] = getBits<10>(i); i+=10;
        }
        for (int j = 0; j < ncell; j++)
        {
            half[j] = getBits<1>(i); i+=1;
        }
        for (int j = 0; j < ncell; j++)
        {
            cnr = getBits<10>(i)*0.0625; i+=10;
        }
        for (int j = 0; j < ncell; j++)
        {
            int rrv = getBits<15>(i); i+=15;
            if (rrv != -16384)
                fine_phaserangerates[j] = rrv * 0.0001;
        }
    }

    double tow;
    uint8_t extended_info[64];
    double range[64];
    double pseudorange[64];
    double phaserange[64];
    double phaserangerates[64];
    double fine_phaserangerates[64];
    uint16_t lock[64];
    uint8_t half[64];
    double cnr[64];
};
}

// 1230 GLONASS code-phase biases

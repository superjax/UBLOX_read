#pragma once

#include <vector>
#include <Eigen/Core>

#include "UBLOX/utctime.h"

#include "UBLOX/parsers/ubx_defs.h"

struct Obs
{
    enum{
        LLI_HALFC = 0x01,
        LLI_SLIP = 0x02,
        LLI_HALFA = 0x04,
        LLI_HALFS = 0x08
    };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    UTCTime t;
    uint8_t sat_idx; // index in sats_ SatVec
    uint8_t gnssId;
    uint8_t sigId;
    uint8_t satId;
    uint16_t lockTime; // ms
    uint8_t rcv;
    uint8_t SNR;
    uint8_t LLI; // loss-of-lock indicator
    uint8_t code;
    uint8_t covP; // psuedorange cov
    uint8_t covD; // Doppler cov
    uint8_t covL; // carrier phase cov
    Eigen::Vector3d z; // [prange, doppler, cphase]
    bool half_cyc; // half-cycle error.  See https://portal.u-blox.com/s/question/0D52p00008HKCdLCAX/what-do-the-halfcyc-and-subhalfcyc-bits-trkstat-bitfield-ubxrxmrawx-message-stand-for


    Obs();
    bool operator < (const Obs& other);
};

void convertUBX(const ublox::RXM_RAWX_t& msg, std::vector<Obs, Eigen::aligned_allocator<Obs>>& obsVec);

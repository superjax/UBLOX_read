#include "UBLOX/obs.h"

Obs::Obs()
{
    sat_idx = -1;
}
bool Obs::operator <(const Obs& other)
{
    return sat_idx < other.sat_idx;
}

void convertUBX(const ublox::RXM_RAWX_t& msg, std::vector<Obs, Eigen::aligned_allocator<Obs>>& obsVec)
{
    using namespace ublox;
    UTCTime t = UTCTime::fromGPS(msg.week, msg.rcvTow);
    obsVec.resize(msg.numMeas);
    for (int i = 0; i < msg.numMeas; i++)
    {
        obsVec[i].t = t;
        obsVec[i].satId = msg.meas[i].svId;
        obsVec[i].gnssId = msg.meas[i].gnssId;
        obsVec[i].sigId = msg.meas[i].sigId;
        obsVec[i].lockTime = msg.meas[i].locktime;
        obsVec[i].SNR = msg.meas[i].cno*4;

        obsVec[i].z[0] = msg.meas[i].prMeas;
        obsVec[i].z[1] = msg.meas[i].doMeas;
        obsVec[i].z[2] = msg.meas[i].cpMeas;

        if (msg.meas[i].cpMeas != 0.0
            && (msg.meas[i].trkStat & RXM_RAWX_t::trkStat_HalfCyc | RXM_RAWX_t::trkStat_subHalfCyc))
        {
            obsVec[i].LLI =  Obs::LLI_HALFC;
        }
        else
        {
            obsVec[i].LLI = 0;
        }

        obsVec[i].covL = msg.meas[i].doStdev*msg.meas[i].doStdev;
        obsVec[i].covP = msg.meas[i].prStdev*msg.meas[i].prStdev;
        obsVec[i].covL = msg.meas[i].cpStdev*msg.meas[i].cpStdev;
    }
}

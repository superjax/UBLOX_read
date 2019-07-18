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
    UTCTime t = UTCTime::fromGPS(msg.week, msg.rcvTow);
    obsVec.resize(msg.numMeas);
    for (int i = 0; i < msg.numMeas; i++)
    {
        obsVec[i].t = t;
        obsVec[i].z[0] = msg.meas[i].prMeas;
        obsVec[i].z[1] = msg.meas[i].doMeas;
        obsVec[i].z[2] = msg.meas[i].cpMeas;
    }
//        obsVec[i].
//    }

}

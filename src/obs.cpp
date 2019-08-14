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

#include "UBLOX/obs.h"

Obs::Obs() { sat_idx = -1; }
bool Obs::operator<(const Obs& other) { return sat_idx < other.sat_idx; }

void convertUBX(const ublox::RXM_RAWX_t& msg,
                std::vector<Obs, Eigen::aligned_allocator<Obs>>& obsVec)
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
        obsVec[i].SNR = msg.meas[i].cno * 4;

        obsVec[i].z[0] = msg.meas[i].prMeas;
        obsVec[i].z[1] = msg.meas[i].doMeas;
        obsVec[i].z[2] = msg.meas[i].cpMeas;

        if (msg.meas[i].cpMeas != 0.0 &&
            (msg.meas[i].trkStat & RXM_RAWX_t::trkStat_HalfCyc | RXM_RAWX_t::trkStat_subHalfCyc))
        {
            obsVec[i].LLI = Obs::LLI_HALFC;
        }
        else
        {
            obsVec[i].LLI = 0;
        }

        obsVec[i].covL = msg.meas[i].doStdev * msg.meas[i].doStdev;
        obsVec[i].covP = msg.meas[i].prStdev * msg.meas[i].prStdev;
        obsVec[i].covL = msg.meas[i].cpStdev * msg.meas[i].cpStdev;
    }
}

/* Copyright (c) 2019 James Jackson, Matt Rydalch
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

#include <signal.h>
#include <fstream>
#include <string>

#include "UBLOX/ublox.h"

#include "UBLOX/async_comm_adapter.h"
#include "logger.h"

bool stop = false;
void inthand(int signum)
{
    stop = true;
}

struct UBX_Callback_Handler : public ublox::UBXListener
{
    UBX_Callback_Handler()
    {
        subscribe(ublox::CLASS_NAV, ublox::NAV_PVT);
        subscribe(ublox::CLASS_RXM, ublox::RXM_RAWX);
    }
    void got_ubx(const uint8_t cls, const uint8_t id, const ublox::UBX_message_t& msg)
    {
        std::cout << "GOT UBX MESSAGE" << std::endl;
        if (cls == ublox::CLASS_NAV && id == ublox::NAV_PVT)
            pvt_callback(msg.NAV_PVT);
        else if (cls == ublox::CLASS_RXM && id == ublox::RXM_RAWX)
            rawx_callback(msg.RXM_RAWX);
    }

    void pvt_callback(const ublox::NAV_PVT_t& msg)
    {
        printf("t: %d %d/%d, %d:%d:%d.%d, lla: %.3f, %.3f, %.3f, vel: %2.3f, %2.3f, %2.3f\n",
               msg.year, msg.month, msg.day, msg.hour, msg.min, msg.sec, msg.nano / 1000000,
               msg.lat * 1e-7, msg.lon * 1e-7, msg.height * 1e-3, msg.velN * 1e-3, msg.velE * 1e-3,
               msg.velD * 1e-3);
        printf("tow: %d\n", msg.iTOW);
        fflush(stdout);  // Will now print everything in the stdout buffer
    }

    void rawx_callback(const ublox::RXM_RAWX_t& msg)
    {
        for (int i = 0; i < msg.numMeas; ++i)
        {
            printf("week: %d, tow: %.3f gnss_id: %d, sat_id: %d, P %.3f, L %.3f, D %.3f\n",
                   msg.week, msg.rcvTow, msg.meas[i].gnssId, msg.meas[i].svId, msg.meas[i].prMeas,
                   msg.meas[i].cpMeas, msg.meas[i].doMeas);
            fflush(stdout);
        }
    }
};

void eph_callback(const Ephemeris& eph)
{
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
    std::cout << "GPS sat: " << (int)eph.sat;
    std::cout << ", now = " << UTCTime::now();
    std::cout << ", toe = " << eph.toe;
    std::cout << ", tof = " << eph.toe << std::endl;
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
}

void geph_callback(const GlonassEphemeris& geph)
{
    std::cout << "*************************************************************\n";
    std::cout << "Glonass sat:" << (int)geph.sat;
    std::cout << ", now = " << UTCTime::now();
    std::cout << ", toe = " << geph.toe;
    std::cout << ", tof = " << geph.tof << std::endl;
    std::cout << "*************************************************************\n";
}

int main(int argc, char** argv)
{
    // Create a UBLOX instance

    std::string log_file = "";
    if (argc > 1)
    {
        log_file = argv[1];
    }
    else
    {
        std::cerr << "Must supply a log file to read" << std::endl;
        return 1;
    }

    Logger log(log_file, Logger::Type::READ);
    ublox::UBLOX ublox(log);

    UBX_Callback_Handler ubx_handler;
    ublox.registerUBXListener(&ubx_handler);

    // look for Ctrl+c and quit
    signal(SIGINT, inthand);

    // Connect Callbacks to Satellite Ephemeris Reception
    ublox.registerEphCallback(eph_callback);
    ublox.registerGephCallback(geph_callback);

    log.play();

    return 0;
}

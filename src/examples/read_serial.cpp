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
#include <set>
#include <string>

#include "UBLOX/ublox.h"

#include "UBLOX/async_comm_adapter.h"
#include "logger.h"

bool stop = false;
void inthand(int signum)
{
    stop = true;
}

std::set<int> found_gps_sats;
std::set<int> found_gal_sats;
std::set<int> found_glo_sats;

struct UBX_Callback_Handler : public ublox::UBXListener
{
    UBX_Callback_Handler()
    {
        subscribe(ublox::CLASS_NAV, ublox::NAV_PVT);
        subscribe(ublox::CLASS_RXM, ublox::RXM_RAWX);
    }
    void got_ubx(const uint8_t cls, const uint8_t id, const ublox::UBX_message_t& msg)
    {
        // if (cls == ublox::CLASS_NAV && id == ublox::NAV_PVT)
        //     pvt_callback(msg.NAV_PVT);
        // else if (cls == ublox::CLASS_RXM && id == ublox::RXM_RAWX)
        //     rawx_callback(msg.RXM_RAWX);
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
    std::cout << eph.Type() << " sat: " << (int)eph.sat;
    std::cout << ", now = " << UTCTime::now();
    std::cout << ", toe = " << eph.toe;
    std::cout << ", tof = " << eph.toe << std::endl;
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";

    if (eph.gnssID == ublox::GnssID_GPS)
    {
        found_gps_sats.insert(eph.sat);

        std::cout << "Total GPS: " << found_gps_sats.size() << ": ";
        for (auto& gps : found_gps_sats) std::cout << gps << ", ";
        std::cout << std::endl;
    }
    else if (eph.gnssID == ublox::GnssID_Galileo)
    {
        found_gal_sats.insert(eph.sat);

        std::cout << "Total GAL: " << found_gal_sats.size() << ": ";
        for (auto& gal : found_gal_sats) std::cout << gal << ", ";
        std::cout << std::endl;
    }
}

void geph_callback(const GlonassEphemeris& geph)
{
    std::cout << "*************************************************************\n";
    std::cout << "Glonass sat:" << (int)geph.sat;
    std::cout << ", now = " << UTCTime::now();
    std::cout << ", toe = " << geph.toe;
    std::cout << ", tof = " << geph.tof << std::endl;
    std::cout << "*************************************************************\n";
    found_glo_sats.insert(geph.sat);
    std::cout << "Total GLONASS: " << found_glo_sats.size() << ": ";
    for (auto& glo : found_glo_sats) std::cout << glo << ", ";
    std::cout << std::endl;
}

int main(int argc, char** argv)
{
    std::string port = "/dev/ttyACM0";
    if (argc > 1)
    {
        port = argv[1];
    }

    std::string log_file = "";
    if (argc > 2)
    {
        log_file = argv[2];
    }

    ac_adapter::Serial ser(port, 3000000);
    ser.init();
    ublox::UBLOX ublox(ser);

    UBX_Callback_Handler ubx_handler;
    ublox.registerUBXListener(&ubx_handler);

    Logger* logger = nullptr;
    if (!log_file.empty())
    {
        logger = new Logger(log_file, Logger::Type::WRITE);
        ser.add_listener(logger);
    }

    // look for Ctrl+c and quit
    signal(SIGINT, inthand);

    // Connect Callbacks to Satellite Ephemeris Reception
    ublox.registerEphCallback(eph_callback);
    ublox.registerGephCallback(geph_callback);

    while (!stop)
    {
        sleep(1);
    }

    if (logger)
    {
        delete logger;
    }

    return 0;
}

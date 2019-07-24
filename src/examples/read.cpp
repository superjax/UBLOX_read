#include <signal.h>
#include <fstream>

#include "UBLOX/ublox.h"

#include "UBLOX/eph.h"

bool stop = false;
void inthand(int signum)
{
    stop = true;
}

void pvt_callback(uint8_t cls, uint8_t type, const ublox::UBX_message_t& in_msg)
{
    const ublox::NAV_PVT_t& msg(in_msg.NAV_PVT);
    printf("t: %d %d/%d, %d:%d:%d.%d, lla: %.3f, %.3f, %.3f, vel: %2.3f, %2.3f, %2.3f\n",
           msg.year, msg.month, msg.day,
           msg.hour, msg.min, msg.sec, msg.nano/1000000,
           msg.lat*1e-7, msg.lon*1e-7, msg.height*1e-3,
           msg.velN*1e-3, msg.velE*1e-3, msg.velD*1e-3);
    printf("tow: %d\n", msg.iTOW);
    fflush(stdout); // Will now print everything in the stdout buffer

}

NavConverter conv;

void sfrbx_callback(uint8_t cls, uint8_t type, const ublox::UBX_message_t& in_msg)
{
    conv.convertUBX(in_msg.RXM_SFRBX);
}

void eph_callback(const Ephemeris& eph)
{
    std::cout << "GPS\n";
    std::cout << "now = " << UTCTime::now() << std::endl;
    std::cout << "toe = " << eph.toe << std::endl;
    std::cout << "tof = " << eph.toe << std::endl;
    int debug = 1;
}

void geph_callback(const GlonassEphemeris& geph)
{
    std::cout << "Glonass\n";
    std::cout << "now = " << UTCTime::now() << std::endl;
    std::cout << "toe = " << geph.toe << std::endl;
    std::cout << "tof = " << geph.toe << std::endl;
    int debug = 1;
}

int main(int argc, char**argv)
{
    // Create a UBLOX instance

    std::string port = "/dev/ttyACM1";
    if(argc > 1)
        port = argv[1];
    ublox::UBLOX ublox(port);

    // look for Ctrl+C and quit
    signal(SIGINT, inthand);

    // Connect a callback to the PVT message
    ublox.registerUBXCallback(ublox::CLASS_NAV, ublox::NAV_PVT, &pvt_callback);
    ublox.registerUBXCallback(ublox::CLASS_RXM, ublox::RXM_SFRBX, &sfrbx_callback);

    conv.registerCallback(geph_callback);
    conv.registerCallback(eph_callback);

    ublox.readFile("/home/superjax/ublox_test/ublox.raw");
//    ublox.initLogFile("/tmp/ublox.raw");

    // while (!stop)
    // {
//        ublox.ubx_.enable_message(ublox::CLASS_RXM, ublox::RXM_RAWX, 1);
//        sleep(1);
    // }

    std::cout << "\nquitting" << std::endl;
    return 0;
}

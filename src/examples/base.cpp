#include <signal.h>

#include "UBLOX/ublox.h"

int i = 1;

bool stop = false;
void inthand(int signum)
{
    stop = true;
}

void pvt_callback(uint8_t cls, uint8_t type, const ublox::UBX_message_t& in_msg)
{
    const ublox::NAV_PVT_t& msg(in_msg.NAV_PVT);
    printf("%d t: %d.%d, lla: %.3f, %.3f, %.3f, vel: %2.3f, %2.3f, %2.3f\n",
           i,
           msg.min, msg.sec,
           msg.lat*1e-7, msg.lon*1e-7, msg.height*1e-3,
           msg.velN*1e-3, msg.velE*1e-3, msg.velD*1e-3);
    fflush(stdout); // Will now print everything in the stdout buffer
    i++;
}

int main(int argc, char** argv)
{
    // Create a UBLOX instance

    std::string port = "/dev/ttyACM0";
    if(argc > 1)
        port = argv[1];
    ublox::UBLOX ublox(port);
    ublox.initBase("localhost", 16140, "localhost", 16145);

    // look for Ctrl+C and quit
    signal(SIGINT, inthand);

    // Connect a callback to the PVT message
    ublox.registerUBXCallback(ublox::CLASS_NAV, ublox::NAV_PVT, &pvt_callback);
    while (!stop)
    {
    }

    std::cout << "\nquitting" << std::endl;
    return 0;
}

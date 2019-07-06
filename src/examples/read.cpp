#include <signal.h>
#include <fstream>

#include "UBLOX/ublox.h"

bool stop = false;
void inthand(int signum)
{
    stop = true;
}

void pvt_callback(uint8_t cls, uint8_t type, const UBX::UBX_message_t& in_msg)
{
    const UBX::NAV_PVT_t& msg(in_msg.NAV_PVT);
    printf("t: %d.%d, lla: %.3f, %.3f, %.3f, vel: %2.3f, %2.3f, %2.3f\n",
           msg.min, msg.sec,
           msg.lat*1e-7, msg.lon*1e-7, msg.height*1e-3,
           msg.velN*1e-3, msg.velE*1e-3, msg.velD*1e-3);
    fflush(stdout); // Will now print everything in the stdout buffer

}

int main(int argc, char**argv)
{
//    // Create a UBLOX instance

//    std::string port = "/dev/ttyACM0";
//    if(argc > 1)
//        port = argv[1];
//    UBLOX ublox(port);

//    // look for Ctrl+C and quit
//    signal(SIGINT, inthand);

//    // Connect a callback to the PVT message
//    ublox.registerUBXCallback(UBX::CLASS_NAV, UBX::NAV_PVT, &pvt_callback);

//    while (!stop)
//    {
//    }

//    std::cout << "\nquitting" << std::endl;
//    return 0;

    std::ifstream file("/home/superjax/Downloads/RTCM3.bin",  std::ifstream::binary);
    file.seekg(0, file.end);
    uint32_t len = file.tellg();
    file.seekg (0, file.beg);
    char* buffer = new char [len];
    file.read(buffer, len);

    RTCM rtcm;
    for (int i = 0; i < len; i++)
    {
        rtcm.read_cb(buffer[i]);
    }
}

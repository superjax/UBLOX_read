#include <signal.h>

#include "UBLOX/ublox.h"

int i = 1;

bool stop = false;
void inthand(int signum)
{
    stop = true;
}

void relposned_callback(uint8_t cls, uint8_t type, const UBX::UBX_message_t& in_msg)
{
    int RTK_flag;
    const UBX::NAV_RELPOSNED_t& msg(in_msg.NAV_RELPOSNED);
    if (msg.flags && 0b000000010)
    {
        printf("RTK");
        RTK_flag = 1;
    }
    else
    {
        printf("No RTK \n");
        RTK_flag = 0;
    }
    if (RTK_flag == 1)
    {
        if (msg.flags && 0b000100000)
            printf(", Moving Base");
        if (msg.flags && 0b000001000)
            printf(" , Floating \n");
        else if (msg.flags && 0b000010000)
            printf(" , Fixed \n");
        printf("%d relative t: %d, NED: %d, %d, %d, Distance: %d\n",
               i,
               msg.iTow/1000,
               msg.relPosN, msg.relPosE, msg.relPosD, msg.relPosLength);
    }

    fflush(stdout); // Will now print everything in the stdout buffer
    i++;
}

int main(int argc, char** argv)
{
    // Create a UBLOX instance

    std::string port = "/dev/ttyACM0";
    if(argc > 1)
        port = argv[1];
    UBLOX ublox(UBLOX::ROVER, port);

    // look for Ctrl+C and quit
    signal(SIGINT, inthand);

    // Connect a callback to the RELPOSNED message
    ublox.registerUBXCallback(UBX::CLASS_NAV, UBX::NAV_RELPOSNED, &relposned_callback);

    while (!stop)
    {
    }

    std::cout << "\nquitting" << std::endl;
    return 0;
}

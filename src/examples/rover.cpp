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
#include <iostream>

#include "UBLOX/ublox.h"

#include "UBLOX/async_comm_adapter.h"

using namespace std;

bool stop = false;
void inthand(int signum)
{
    stop = true;
}

struct UBX_Callback_Handler : public ublox::UBXListener
{
    UBX_Callback_Handler() { subscribe(ublox::CLASS_NAV, ublox::NAV_RELPOSNED); }
    void got_ubx(const uint8_t cls, const uint8_t id, const ublox::UBX_message_t& msg)
    {
        if (cls == ublox::CLASS_NAV && id == ublox::NAV_RELPOSNED)
            relposned_callback(cls, id, msg.NAV_RELPOSNED);
    }

    void relposned_callback(uint8_t cls, uint8_t type, const ublox::NAV_RELPOSNED_t& msg)
    {
        int RTK_flag;
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
            if (msg.flags && 0b000000100)
                printf("valid relative position components \n");
            printf("tow: %d relNED: %d, %d, %d, Distance: %d\n", msg.iTow / 1000, msg.relPosN,
                   msg.relPosE, msg.relPosD, msg.relPosLength);
        }
        fflush(stdout);  // Will now print everything in the stdout buffer
    }
};

int main(int argc, char** argv)
{
    // Create a UBLOX instance

    std::string port = "/dev/ttyACM1";
    if (argc > 1)
        port = argv[1];

    // Create the ublox object
    ac_adapter::Serial ser(port, 921600);
    ser.init();
    ublox::UBLOX ublox(ser);

    // Connect a callback to the RELPOSNED message
    UBX_Callback_Handler ubx_handler;
    ublox.registerUBXListener(&ubx_handler);

    // Connect to the UDP RTCM stream
    ac_adapter::UDP udp("localhost", 16145, "localhost", 16140);
    if (!udp.init())
        throw std::runtime_error("Failed to initialize Rover receive UDP");
    ublox.config_rover(&udp);

    // look for Ctrl+C and quit
    signal(SIGINT, inthand);

    while (!stop)
    {
        sleep(1);
    }

    std::cout << "\nquitting" << std::endl;
    return 0;
}

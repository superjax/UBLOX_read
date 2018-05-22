#include "ublox.h"
#include "iostream"


void echo(uint8_t byte)
{
    std::printf("Received byte: %d\n", byte);  
}

int main()
{
    
    // open serial port
//    async_comm::Serial serial("/dev/ttyUSB0", 115200);
//    serial.register_receive_callback(&echo);
    
//    if (!serial.init())
//    {
//      std::printf("Failed to initialize serial port\n");
//      return 2;
//    }
    
//    while(1) {}
    UBLOX ubx;
    ubx.init();	
    while(1) {}
}

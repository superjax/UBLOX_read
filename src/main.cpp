#include <iostream>

#include "ublox.h"

#include <async_comm/udp.h>

#include <cstdint>
#include <cstring>

#include <chrono>
#include <thread>
#include <vector>

void callback(const uint8_t* buf, size_t len)
{
  for (size_t i = 0; i < len; i++)
  {
    std::cout << buf[i];
  }
}

int udp(int rover, int message)
{
    /////////udp communication////

    // send message from base to rover
    if (rover == 0)
    {
        // open UDP port
        async_comm::UDP udp2("localhost", 14625, "localhost", 14620);
        udp2.register_receive_callback(&callback);

        if (!udp2.init())
        {
          std::cout << "Failed to initialize UDP port 2" << std::endl;
          return 1;
        }
        char message1[] = "RTCM = %d", message;
        udp2.send_bytes((uint8_t*) message1, std::strlen(message1));

        // wait for all bytes to be received
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        //std::cout << std::endl << std::flush;

        std::cout.flush();

        udp2.close();
    }
    else  //send message from rover to base
    {

        // open UDP port
        async_comm::UDP udp1("localhost", 14620, "localhost", 14625);
        udp1.register_receive_callback(&callback);

        if (!udp1.init())
        {
          std::cout << "Failed to initialize UDP port 1" << std::endl;
          return 1;
        }
        // send message the other direction
        char message2[] = "Hello world 2!";
        udp1.send_bytes((uint8_t*) message2, std::strlen(message2));

        // wait for all bytes to be received
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        //std::cout << std::endl << std::flush;

        std::cout.flush();

        // close UDP ports
        udp1.close();


    }

    return 0;

}

int main(int argc, char** argv)
{
    //This is used to select the port and is used later as well
  std::string port = "/dev/ttyACM0";
  int rover = 0;
  if (argc > 1)
  {
    rover = 1;
    port = argv[1];
  }
  std::cout << "opening " << port << std::endl;
  UBLOX ubx(port);
  ubx.init(rover);

  double lla[3];
  float uvw[3];
  std::map<uint8_t, std::string> fix_types =
  {{UBLOX::FIX_TYPE_NO_FIX, "NO_FIX"},
   {UBLOX::FIX_TYPE_DEAD_RECKONING, "DEAD_RECKONING"},
   {UBLOX::FIX_TYPE_2D, "2D"},
   {UBLOX::FIX_TYPE_3D, "3D"},
   {UBLOX::FIX_TYPE_GPS_AND_DEAD_RECKONING, "GPS_AND_DEAD_RECKONING"},
   {UBLOX::FIX_TYPE_TIME_ONLY, "TIME_ONLY"}};

  while(1)
  {
    if (ubx.new_data())
    {
      uint8_t fix_type;
      uint32_t t_ms;
      ubx.read(lla, uvw, fix_type, t_ms);
      printf("%s - lla: %f, %f, %f - uvw: %f, %f, %f\n",
             fix_types[fix_type].c_str(),
             lla[0], lla[1], lla[2],
          uvw[0], uvw[1], uvw[2]);
      std::cout.flush();
//      int RTCM = ubx.get_RTCM();
//      udp(rover, RTCM);
    }
  }
}

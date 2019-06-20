#include <iostream>

#include "ublox.h"

#include <async_comm/udp.h>

#include <cstdint>
#include <cstring>

#include <chrono>
#include <thread>
#include <vector>

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
    }
    ubx.udp(rover);
  }
}
